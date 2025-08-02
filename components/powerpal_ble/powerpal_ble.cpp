#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

#include <nvs_flash.h>
#include <nvs.h>
#include <vector>
#include <cstdio>

#ifdef USE_ESP32
namespace esphome {
namespace powerpal_ble {

static const char *const TAG = "powerpal_ble";


void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "POWERPAL");
  LOG_SENSOR(" ", "Battery", this->battery_);
  LOG_SENSOR(" ", "Power", this->power_sensor_);
  LOG_SENSOR(" ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR(" ", "Total Energy", this->energy_sensor_);
  }

void Powerpal::setup() {
  this->authenticated_ = false;
  this->pulse_multiplier_ = ((seconds_in_minute * this->reading_batch_size_[0]) / (this->pulses_per_kwh_ / kw_to_w_conversion));
  
    // ——— NVS init & load ———
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  err = nvs_open("powerpal", NVS_READWRITE, &this->nvs_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed (%d)", err);
  } else {
    this->nvs_ok_ = true;
    this->nvs_queue_ = xQueueCreate(1, sizeof(NVSCommitData));
    if (this->nvs_queue_)
      xTaskCreate(&Powerpal::nvs_commit_task, "pp_nvs", 4096, this, 1, &this->nvs_task_);
    uint64_t stored = 0;
    err = nvs_get_u64(this->nvs_handle_, "daily", &stored);
    if (err == ESP_OK) {
      this->daily_pulses_ = stored;
      ESP_LOGI(TAG, "Loaded daily_pulses: %llu", stored);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGI(TAG, "No stored daily_pulses; starting at zero");
    } else {
      ESP_LOGE(TAG, "Error reading daily_pulses (%d)", err);
    }

    uint64_t stored_total = 0;
    esp_err_t err = nvs_get_u64(this->nvs_handle_, "total", &stored_total);
    if (err == ESP_OK) {
      this->total_pulses_ = stored_total;
      ESP_LOGI(TAG, "Loaded total_pulses: %llu", stored_total);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGI(TAG, "No stored total_pulses; starting at zero");
    } else {
      ESP_LOGE(TAG, "Error reading total_pulses (%d)", err);
    }

    ESP_LOGI(TAG, "After setup, total_pulses_ = %llu", this->total_pulses_);

  }
  
  
  
  ESP_LOGI(TAG, "pulse_multiplier_: %f", this->pulse_multiplier_);
  ESP_LOGI(TAG, "Loaded persisted daily_pulses: %llu", this->daily_pulses_);

  // ——— set sensor metadata defaults ———
  if (this->energy_sensor_) {
    this->energy_sensor_->set_device_class("energy");
    this->energy_sensor_->set_state_class(sensor::STATE_CLASS_TOTAL_INCREASING);
    this->energy_sensor_->set_unit_of_measurement("kWh");
  }
  if (this->daily_energy_sensor_) {
    this->daily_energy_sensor_->set_device_class("energy");
    this->daily_energy_sensor_->set_state_class(sensor::STATE_CLASS_TOTAL_INCREASING);
    this->daily_energy_sensor_->set_unit_of_measurement("kWh");
  }

}



std::string Powerpal::pkt_to_hex_(const uint8_t *data, uint16_t len) {
  char buf[64];
  memset(buf, 0, 64);
  for (int i = 0; i < len; i++)
    sprintf(&buf[i * 2], "%02x", data[i]);
  std::string ret = buf;
  return ret;
}


void Powerpal::decode_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
}

void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Battery: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length == 1) {
    this->battery_->publish_state(data[0]);
  }
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  if (length < 6) {
    ESP_LOGW(TAG, "parse_measurement_: packet too short (%hu)", length);
    return;
  }

  // 1) Build UNIX timestamp from bytes [0..3]
  uint32_t t32 = uint32_t(data[0]) |
                 (uint32_t(data[1]) << 8) |
                 (uint32_t(data[2]) << 16) |
                 (uint32_t(data[3]) << 24);
  time_t unix_time = time_t(t32);

  // 2) Determine day-of-year for rollover
  int today;
#ifdef USE_TIME
  if (this->time_.has_value()) {
    auto *time_comp = *this->time_;
    auto now = time_comp->now();
    if (now.is_valid()) {
      today = now.day_of_year;
    } else {
      struct tm *tm_info = ::localtime(&unix_time);
      today = tm_info->tm_yday + 1;
    }
  } else {
    struct tm *tm_info = ::localtime(&unix_time);
    today = tm_info->tm_yday + 1;
  }
#else
  struct tm *tm_info = ::localtime(&unix_time);
  today = tm_info->tm_yday + 1;
#endif

  // 3) First-measurement vs midnight-rollover
  bool force_commit = false;
  if (this->day_of_last_measurement_ == 0) {
    this->day_of_last_measurement_ = today;
  } else if (this->day_of_last_measurement_ != today) {
    this->day_of_last_measurement_ = today;
    this->daily_pulses_ = 0;
    force_commit = true;
  }

  // 4) Read pulse count for this interval
  uint16_t pulses = uint16_t(data[4]) | (uint16_t(data[5]) << 8);

  // 5) Instantaneous power (W) using actual elapsed time
  static uint32_t last_timestamp_s = 0;
  uint32_t interval_s = last_timestamp_s ? (t32 - last_timestamp_s) : 0;
  last_timestamp_s = t32;
  if (interval_s == 0) {
    ESP_LOGD(TAG, "Skipping power calc on first measurement after reboot or rollover");
  } else {
    float energy_kwh = float(pulses) / float(this->pulses_per_kwh_);
    float power_w = (energy_kwh * 3600.0f * 1000.0f) / float(interval_s);
    if (this->power_sensor_)
      this->power_sensor_->publish_state(power_w);
  }

  // 6) Cost for this interval
  float cost = (float(pulses) / this->pulses_per_kwh_) * this->energy_cost_;
  if (this->cost_sensor_)
    this->cost_sensor_->publish_state(cost);

  // 7) Raw pulses
  if (this->pulses_sensor_)
    this->pulses_sensor_->publish_state(pulses);

  // 8) Watt-hours for this interval
  float wh = (float(pulses) / this->pulses_per_kwh_) * 1000.0f;
  uint32_t wh_int = (uint32_t) roundf(wh);
  if (this->watt_hours_sensor_)
    this->watt_hours_sensor_->publish_state((int)wh_int);

  // 9) Timestamp
  if (this->timestamp_sensor_)
    this->timestamp_sensor_->publish_state((long)unix_time);

  // 10 & 11) Accumulate totals then queue commit via scheduled task
  this->total_pulses_ += pulses;
  float total_kwh = this->total_pulses_ / this->pulses_per_kwh_;
  if (this->energy_sensor_)
    this->energy_sensor_->publish_state(total_kwh);

  this->daily_pulses_ += pulses;
  float daily_kwh = this->daily_pulses_ / this->pulses_per_kwh_;
  if (this->daily_energy_sensor_)
    this->daily_energy_sensor_->publish_state(daily_kwh);

  PowerpalMeasurement measurement{pulses, unix_time, wh_int, cost};
  this->stored_measurements_.push_back(measurement);
  ESP_LOGD(TAG, "Buffered reading: ts=%ld pulses=%u (pending=%zu)", (long) unix_time, pulses,
           this->stored_measurements_.size());

  this->schedule_commit_(force_commit);

  if (this->daily_pulses_sensor_)
    this->daily_pulses_sensor_->publish_state(this->daily_pulses_);
}

void Powerpal::send_pending_readings_() {
  if (this->stored_measurements_.empty()) {
    ESP_LOGD(TAG, "No pending readings to upload");
    return;
  }
  if (this->http_request_ == nullptr) {
    ESP_LOGW(TAG, "HTTP request component not configured; skipping upload");
    return;
  }

  ESP_LOGI(TAG, "Uploading %zu reading(s) to Powerpal", this->stored_measurements_.size());

  std::string payload = "[";
  for (size_t i = 0; i < this->stored_measurements_.size(); ++i) {
    const auto &m = this->stored_measurements_[i];
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"cost\":%.11f,\"is_peak\":false,\"pulses\":%u,\"timestamp\":%ld,\"watt_hours\":%u}",
             m.cost, m.pulses, (long)m.timestamp, m.watt_hours);
    if (i != 0)
      payload += ", ";
    payload += buf;
  }
  payload += "]";

  char url[256];
  snprintf(url, sizeof(url),
           "https://readings.powerpal.net/api/v1/meter_reading/%s",
           this->powerpal_device_id_.c_str());

  std::vector<http_request::Header> headers{
      http_request::Header{"Authorization", this->powerpal_apikey_},
      http_request::Header{"Content-Type", "application/json"},
  };

  auto *req = this->http_request_->send(http_request::HTTPMethod::HTTP_POST, url, headers, payload);
  if (req != nullptr) {
    req->on_response([](http_request::Response *resp) {
      ESP_LOGI(TAG, "Powerpal upload completed with status %d", resp->status_code);
    });
    req->on_error([](http_request::Error *error) {
      ESP_LOGW(TAG, "Powerpal upload failed (status %d): %s", error->status_code,
               error->message.c_str());
    });
  } else {
    ESP_LOGW(TAG, "Failed to initiate HTTP request to Powerpal API");
  }
}

void Powerpal::schedule_commit_(bool force) {
  if (!this->nvs_ok_)
    return;

  App.schedule([this, force]() {
    uint32_t now_s = millis() / 1000;
    bool time_ok = (now_s - this->last_commit_ts_) >= COMMIT_INTERVAL_S;
    bool thresh_ok = (this->total_pulses_ - this->last_pulses_for_threshold_) >= PULSE_THRESHOLD;
    if (force || time_ok || thresh_ok) {
      ESP_LOGD(TAG, "NVS THROTTLED commit #%u at %us: total=%llu daily=%llu",
               this->nvsc_commit_count_ + 1, now_s,
               this->total_pulses_, this->daily_pulses_);
      if (this->nvs_queue_) {
        NVSCommitData data{this->total_pulses_, this->daily_pulses_};
        if (xQueueSend(this->nvs_queue_, &data, 0) != pdTRUE) {
          ESP_LOGW(TAG, "NVS queue full; commit skipped");
        }
      }
      this->send_pending_readings_();
      this->stored_measurements_.clear();
      this->last_commit_ts_ = now_s;
      this->last_pulses_for_threshold_ = this->total_pulses_;
      ++this->nvsc_commit_count_;
    }
  });
}



void Powerpal::nvs_commit_task(void *param) {
  auto *self = static_cast<Powerpal *>(param);
  NVSCommitData data;
  for (;;) {
    if (xQueueReceive(self->nvs_queue_, &data, portMAX_DELAY) == pdTRUE) {
      nvs_set_u64(self->nvs_handle_, "total", data.total);
      nvs_set_u64(self->nvs_handle_, "daily", data.daily);
      esp_err_t err = nvs_commit(self->nvs_handle_);
      if (err != ESP_OK)
        ESP_LOGE(TAG, "NVS commit failed (%d)", err);
    }
  }
}

std::string Powerpal::uuid_to_device_id_(const uint8_t *data, uint16_t length) {
  const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  std::string device_id;
  for (int i = length-1; i >= 0; i--) {
    device_id.append(hexmap[(data[i] & 0xF0) >> 4]);
    device_id.append(hexmap[data[i] & 0x0F]);
  }
  return device_id;
}

std::string Powerpal::serial_to_apikey_(const uint8_t *data, uint16_t length) {
  const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  std::string api_key;
  for (int i = 0; i < length; i++) {
    if ( i == 4 || i == 6 || i == 8 || i == 10 ) {
      api_key.append("-");
    }
    api_key.append(hexmap[(data[i] & 0xF0) >> 4]);
    api_key.append(hexmap[data[i] & 0x0F]);
  }
  return api_key;
}


void Powerpal::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_DISCONNECT_EVT: {
      this->authenticated_ = false;
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGI(TAG, "POWERPAL: services discovered, looking up characteristic handles…");

      // Pairing Code
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID)) {
        this->pairing_code_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → pairing_code handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  ! pairing_code characteristic not found");
      }

      // Reading Batch Size
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID)) {
        this->reading_batch_size_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → reading_batch_size handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  ! reading_batch_size characteristic not found");
      }

      // Measurement
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID)) {
        this->measurement_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → measurement handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  ! measurement characteristic not found");
      }

      // (optional) UUID & serial if you need them:
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_UUID_UUID)) {
        this->uuid_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → uuid handle = 0x%02x", ch->handle);
      }
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_SERIAL_UUID)) {
        this->serial_number_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → serial handle = 0x%02x", ch->handle);
      }

      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent_->address_str().c_str());
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        break;
      }
      // reading batch size
      if (param->read.handle == this->reading_batch_size_char_handle_) {
        ESP_LOGD(TAG, "Received reading_batch_size read event");
        this->decode_(param->read.value, param->read.value_len);
        if (param->read.value_len == 4) {
          if (param->read.value[0] != this->reading_batch_size_[0]) {
            // reading batch size needs changing, so write
            auto status =
                esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->reading_batch_size_char_handle_, sizeof(this->reading_batch_size_),
                                         this->reading_batch_size_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (status) {
              ESP_LOGW(TAG, "Error sending write request for batch_size, status=%d", status);
            }
          } else {
            // reading batch size is set correctly so subscribe to measurement notifications
            auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                            this->measurement_char_handle_);
            if (status) {
              ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                       this->parent_->address_str().c_str(), status);
            }
          }
        } else {
          // error, length should be 4
        }
        break;
      }

      // battery
      if (param->read.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Received battery read event");
        this->parse_battery_(param->read.value, param->read.value_len);
        break;
      }

      // firmware
      if (param->read.handle == this->firmware_char_handle_) {
        ESP_LOGD(TAG, "Received firmware read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // led sensitivity
      if (param->read.handle == this->led_sensitivity_char_handle_) {
        ESP_LOGD(TAG, "Received led sensitivity read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // serial number (device id)
      if (param->read.handle == this->serial_number_char_handle_) {
        ESP_LOGI(TAG, "Received serial_number read event");
        this->powerpal_device_id_ = this->uuid_to_device_id_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal device id: %s", this->powerpal_device_id_.c_str());

        break;
      }

      // uuid (API key)
      if (param->read.handle == this->uuid_char_handle_) {
        ESP_LOGI(TAG, "Received uuid read event");
        this->powerpal_apikey_ = this->serial_to_apikey_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal apikey: %s", this->powerpal_apikey_.c_str());

        break;
      }

      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_WRITE_CHAR_EVT (Write confirmed)", this->parent_->address_str().c_str());
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error writing value to char at handle %d, status=%d", param->write.handle, param->write.status);
        break;
      }

      if (param->write.handle == this->pairing_code_char_handle_ && !this->authenticated_) {
        this->authenticated_ = true;

        auto read_reading_batch_size_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->reading_batch_size_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_reading_batch_size_status) {
          ESP_LOGW(TAG, "Error sending read request for reading batch size, status=%d", read_reading_batch_size_status);
        }

        if (!this->powerpal_apikey_.length()) {
          // read uuid (apikey)
          auto read_uuid_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                            this->uuid_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_uuid_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal uuid, status=%d", read_uuid_status);
          }
        }
        if (!this->powerpal_device_id_.length()) {
          // read serial number (device id)
          auto read_serial_number_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                            this->serial_number_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_serial_number_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal serial number, status=%d", read_serial_number_status);
          }
        }

        if (this->battery_ != nullptr) {
          // read battery
          auto read_battery_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                             this->battery_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_battery_status) {
            ESP_LOGW(TAG, "Error sending read request for battery, status=%d", read_battery_status);
          }
          // Enable notifications for battery
          auto notify_battery_status = esp_ble_gattc_register_for_notify(
              this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), this->battery_char_handle_);
          if (notify_battery_status) {
            ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                     this->parent_->address_str().c_str(), notify_battery_status);
          }
        }

        // read firmware version
        auto read_firmware_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->firmware_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_firmware_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_firmware_status);
        }

        // read led sensitivity
        auto read_led_sensitivity_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->led_sensitivity_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_led_sensitivity_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_led_sensitivity_status);
        }

        break;
      }
      if (param->write.handle == this->reading_batch_size_char_handle_) {
        // reading batch size is now set correctly so subscribe to measurement notifications
        auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                        this->measurement_char_handle_);
        if (status) {
          ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                   this->parent_->address_str().c_str(), status);
        }
        break;
      }

      ESP_LOGW(TAG, "[%s] Missed all handle matches: %d",
               this->parent_->address_str().c_str(), param->write.handle);
      break;
    }  // ESP_GATTC_WRITE_CHAR_EVT

    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[%s] Received Notification", this->parent_->address_str().c_str());

      // battery
      if (param->notify.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Received battery notify event");
        this->parse_battery_(param->notify.value, param->notify.value_len);
        break;
      }

      // measurement
      if (param->notify.handle == this->measurement_char_handle_) {
        ESP_LOGD(TAG, "Received measurement notify event");
        this->parse_measurement_(param->notify.value, param->notify.value_len);
        break;
      }
      break;  // registerForNotify
    }
    default:
      break;
  }
}

void Powerpal::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    // This event is sent once authentication has completed
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "[%s] Writing pairing code to Powerpal", this->parent_->address_str().c_str());
        auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                               this->pairing_code_char_handle_, sizeof(this->pairing_code_),
                                               this->pairing_code_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (status) {
          ESP_LOGW(TAG, "Error sending write request for pairing_code, status=%d", status);
        }
      }
      break;
    }
    default:
      break;
  }
}

}  // namespace powerpal_ble
}  // namespace esphome

#endif
