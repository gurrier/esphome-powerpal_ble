#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <nvs_flash.h>
#include <nvs.h>

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

void Powerpal::reset_connection_state_() {
  this->authenticated_ = false;
  this->pending_subscription_ = false;
  this->subscription_in_progress_ = false;
  this->subscription_retry_scheduled_ = false;

  this->pairing_code_char_handle_ = 0;
  this->reading_batch_size_char_handle_ = 0;
  this->measurement_char_handle_ = 0;
  this->battery_char_handle_ = 0;
  this->led_sensitivity_char_handle_ = 0;
  this->firmware_char_handle_ = 0;
  this->uuid_char_handle_ = 0;
  this->serial_number_char_handle_ = 0;

  this->stored_measurements_count_ = 0;
  this->stored_measurements_.clear();
  this->last_measurement_timestamp_s_ = 0;
  this->reconnect_pending_ = false;
  this->client_connected_ = false;
}

void Powerpal::on_connect() {
  ESP_LOGI(TAG, "[%s] Connected to Powerpal GATT server", this->parent_->address_str());
  this->client_connected_ = true;
  this->pending_subscription_ = true;
  this->subscription_in_progress_ = false;
  this->subscription_retry_scheduled_ = false;
  this->reconnect_pending_ = false;
  this->stored_measurements_.clear();
  this->stored_measurements_count_ = 0;
  this->last_measurement_timestamp_s_ = 0;
  this->authenticated_ = false;

  this->set_timeout(1000, [this]() { this->request_subscription_("post-connect"); });
}

void Powerpal::on_disconnect() {
  ESP_LOGW(TAG, "[%s] Disconnected from Powerpal GATT server", this->parent_->address_str());
  this->reset_connection_state_();

  if (!this->reconnect_pending_) {
    this->reconnect_pending_ = true;
    this->set_timeout(10000, [this]() {
      this->reconnect_pending_ = false;
      if (this->parent_ == nullptr)
        return;
      if (this->client_connected_) {
        ESP_LOGD(TAG, "[%s] Reconnect timer fired but client already connected", this->parent_->address_str());
        return;
      }
      ESP_LOGI(TAG, "[%s] Attempting BLE reconnect", this->parent_->address_str());
      this->pending_subscription_ = true;
      this->parent_->connect();
    });
  }
}

void Powerpal::setup() {
  this->authenticated_ = false;
  if (this->pulses_per_kwh_ <= 0.0f) {
    ESP_LOGW(TAG, "Invalid pulses_per_kwh configured (%.3f); defaulting to 1.0", this->pulses_per_kwh_);
    this->pulses_per_kwh_ = 1.0f;
  }
  this->pulse_multiplier_ =
      ((seconds_in_minute * this->reading_batch_size_[0]) / (this->pulses_per_kwh_ / kw_to_w_conversion));
  
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
  if (this->nvs_ok_) {
    // Anchor commit threshold and publish restored energy values so Home Assistant statistics
    // continue from the persisted counters even before the first measurement arrives.
    this->last_commit_ts_ = millis() / 1000;
    this->last_pulses_for_threshold_ = this->total_pulses_;
    if (this->energy_sensor_) {
      this->energy_sensor_->publish_state(this->total_pulses_ / this->pulses_per_kwh_);
    }
    if (this->daily_energy_sensor_) {
      this->daily_energy_sensor_->publish_state(this->daily_pulses_ / this->pulses_per_kwh_);
    }
  }

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

  this->reset_connection_state_();
}



std::string Powerpal::pkt_to_hex_(const uint8_t *data, uint16_t len) {
  if (data == nullptr || len == 0)
    return {};

  static constexpr char HEXMAP[] = "0123456789abcdef";
  std::string ret;
  ret.reserve(static_cast<size_t>(len) * 2);
  for (uint16_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    ret.push_back(HEXMAP[(byte >> 4) & 0x0F]);
    ret.push_back(HEXMAP[byte & 0x0F]);
  }
  return ret;
}


void Powerpal::decode_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
}

void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Battery: DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  if (length != 1) {
    ESP_LOGW(TAG, "Unexpected battery payload length %hu", length);
    return;
  }

  if (this->battery_ == nullptr) {
    ESP_LOGV(TAG, "Battery data received but no battery sensor configured");
    return;
  }

  this->battery_->publish_state(data[0]);
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  if (length < 6) {
    ESP_LOGW(TAG, "parse_measurement_: packet too short (%hu)", length);
    return;
  }

  if (this->pulses_per_kwh_ <= 0.0f) {
    ESP_LOGW(TAG, "pulses_per_kwh must be greater than zero; skipping measurement");
    return;
  }

  // 1) Build UNIX timestamp from bytes [0..3]
  uint32_t t32 = uint32_t(data[0]) |
                 (uint32_t(data[1]) << 8) |
                 (uint32_t(data[2]) << 16) |
                 (uint32_t(data[3]) << 24);
  time_t unix_time = time_t(t32);

  // 2) Determine day-of-year for rollover
  int today = 0;
  bool have_valid_day = false;
#ifdef USE_TIME
  if (this->time_.has_value() && *this->time_ != nullptr) {
    auto *time_comp = *this->time_;
    auto now = time_comp->now();
    if (now.is_valid()) {
      today = now.day_of_year;
      have_valid_day = true;
    } else {
      ESP_LOGV(TAG, "RTC time invalid, using measurement timestamp");
    }
  }
#endif
  if (!have_valid_day) {
    struct tm *tm_info = ::localtime(&unix_time);
    if (tm_info == nullptr) {
      ESP_LOGW(TAG, "localtime conversion failed for timestamp %u", static_cast<unsigned>(t32));
      return;
    }
    today = tm_info->tm_yday + 1;
  }

  // 3) First-measurement vs midnight-rollover
  if (this->day_of_last_measurement_ == 0) {
    this->day_of_last_measurement_ = today;
  } else if (this->day_of_last_measurement_ != today) {
    this->day_of_last_measurement_ = today;
    this->daily_pulses_ = 0;
    if (this->nvs_ok_) {
      ESP_LOGD(TAG, "NVS rollover commit at day change, resetting daily_pulses");
      nvs_erase_key(this->nvs_handle_, "daily");
      nvs_commit(this->nvs_handle_);
      this->last_commit_ts_ = millis() / 1000;
      this->last_pulses_for_threshold_ = this->total_pulses_;
      ++this->nvsc_commit_count_;
    }
  }

  // 4) Read pulse count for this interval
  uint16_t pulses = uint16_t(data[4]) | (uint16_t(data[5]) << 8);

  // 5) Instantaneous power (W) using actual elapsed time
  uint32_t interval_s = this->last_measurement_timestamp_s_ ? (t32 - this->last_measurement_timestamp_s_) : 0;
  this->last_measurement_timestamp_s_ = t32;
  if (interval_s == 0) {
    ESP_LOGD(TAG, "Skipping power calc on first measurement after reboot or rollover");
  } else {
    float energy_kwh = float(pulses) / float(this->pulses_per_kwh_);
    float power_w = (energy_kwh * 3600.0f * 1000.0f) / float(interval_s);
    if (this->power_sensor_)
      this->power_sensor_->publish_state(power_w);
  }

  // 6) Cost for this interval
  if (this->cost_sensor_) {
    float cost = (float(pulses) / this->pulses_per_kwh_) * this->energy_cost_;
    this->cost_sensor_->publish_state(cost);
  }

  // 7) Raw pulses
  if (this->pulses_sensor_)
    this->pulses_sensor_->publish_state(pulses);

  // 8) Watt-hours for this interval
  float wh = (float(pulses) / this->pulses_per_kwh_) * 1000.0f;
  if (this->watt_hours_sensor_)
    this->watt_hours_sensor_->publish_state((int)roundf(wh));

  // 9) Timestamp
  if (this->timestamp_sensor_)
    this->timestamp_sensor_->publish_state((long)unix_time);

  // 10 & 11) Accumulate and throttled NVS commit for Total & Daily Energy
  this->total_pulses_ += pulses;
  float total_kwh = this->total_pulses_ / this->pulses_per_kwh_;
  if (this->energy_sensor_)
    this->energy_sensor_->publish_state(total_kwh);

  this->daily_pulses_ += pulses;
  float daily_kwh = this->daily_pulses_ / this->pulses_per_kwh_;
  if (this->daily_energy_sensor_)
    this->daily_energy_sensor_->publish_state(daily_kwh);

  if (this->nvs_ok_) {
    uint32_t now_s = millis() / 1000;
    bool time_ok = (now_s - this->last_commit_ts_) >= COMMIT_INTERVAL_S;
    bool thresh_ok = (this->total_pulses_ - this->last_pulses_for_threshold_) >= PULSE_THRESHOLD;
    if (time_ok || thresh_ok) {
      ESP_LOGD(TAG, "NVS THROTTLED commit #%u at %us: total=%llu daily=%llu",
               ++this->nvsc_commit_count_, now_s,
               this->total_pulses_, this->daily_pulses_);
      nvs_set_u64(this->nvs_handle_, "total", this->total_pulses_);
      nvs_set_u64(this->nvs_handle_, "daily", this->daily_pulses_);
      esp_err_t err = nvs_commit(this->nvs_handle_);
      if (err != ESP_OK)
        ESP_LOGE(TAG, "NVS commit failed (%d)", err);
      this->last_commit_ts_ = now_s;
      this->last_pulses_for_threshold_ = this->total_pulses_;
    }
  }

  if (this->daily_pulses_sensor_)
    this->daily_pulses_sensor_->publish_state(this->daily_pulses_);
}



std::string Powerpal::uuid_to_device_id_(const uint8_t *data, uint16_t length) {
  if (data == nullptr || length == 0)
    return {};

  static constexpr char HEXMAP[] = "0123456789abcdef";
  std::string device_id;
  device_id.reserve(static_cast<size_t>(length) * 2);
  for (int i = static_cast<int>(length) - 1; i >= 0; i--) {
    uint8_t byte = data[i];
    device_id.push_back(HEXMAP[(byte & 0xF0) >> 4]);
    device_id.push_back(HEXMAP[byte & 0x0F]);
  }
  return device_id;
}

std::string Powerpal::serial_to_apikey_(const uint8_t *data, uint16_t length) {
  if (data == nullptr || length == 0)
    return {};

  static constexpr char HEXMAP[] = "0123456789abcdef";
  std::string api_key;
  api_key.reserve(static_cast<size_t>(length) * 2 + 4);
  for (uint16_t i = 0; i < length; i++) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {
      api_key.push_back('-');
    }
    uint8_t byte = data[i];
    api_key.push_back(HEXMAP[(byte & 0xF0) >> 4]);
    api_key.push_back(HEXMAP[byte & 0x0F]);
  }
  return api_key;
}


void Powerpal::request_subscription_(const char *trigger_reason) {
  if (!this->pending_subscription_)
    return;

  if (this->subscription_in_progress_) {
    ESP_LOGV(TAG, "[%s] Subscription already in progress, ignoring trigger '%s'", this->parent_->address_str(), trigger_reason);
    return;
  }

  if (this->pairing_code_char_handle_ == 0 || this->reading_batch_size_char_handle_ == 0 || this->measurement_char_handle_ == 0) {
    ESP_LOGD(TAG, "[%s] GATT handles not ready, waiting to subscribe (%s)", this->parent_->address_str(), trigger_reason);
    if (!this->subscription_retry_scheduled_) {
      this->subscription_retry_scheduled_ = true;
      this->set_timeout(500, [this]() {
        this->subscription_retry_scheduled_ = false;
        this->request_subscription_("wait-handles");
      });
    }
    return;
  }

  ESP_LOGI(TAG, "[%s] Writing pairing code to resume notifications (%s)", this->parent_->address_str(), trigger_reason);
  auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->pairing_code_char_handle_, sizeof(this->pairing_code_),
                                         this->pairing_code_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "[%s] Failed to submit pairing write (%s), status=%d", this->parent_->address_str(), trigger_reason, status);
    if (!this->subscription_retry_scheduled_) {
      this->subscription_retry_scheduled_ = true;
      this->set_timeout(2000, [this]() {
        this->subscription_retry_scheduled_ = false;
        this->request_subscription_("retry");
      });
    }
    return;
  }

  this->subscription_in_progress_ = true;
}


void Powerpal::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGD(TAG, "[%s] ESP_GATTC_OPEN_EVT", this->parent_->address_str());
        this->on_connect();
      } else {
        ESP_LOGW(TAG, "[%s] ESP_GATTC_OPEN_EVT failed, status=%d", this->parent_->address_str(),
                 param->open.status);
        this->reset_connection_state_();
      }
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] ESP_GATTC_DISCONNECT_EVT", this->parent_->address_str());
      this->on_disconnect();
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

      this->pending_subscription_ = true;
      this->request_subscription_("service discovery");
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent_->address_str());
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
                       this->parent_->address_str(), status);
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

      // serialNumber
      if (param->read.handle == this->serial_number_char_handle_) {
        ESP_LOGI(TAG, "Received serial_number read event");
        this->powerpal_device_id_ = this->uuid_to_device_id_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal device id: %s", this->powerpal_device_id_.c_str());

        break;
      }

      // uuid
      if (param->read.handle == this->uuid_char_handle_) {
        ESP_LOGI(TAG, "Received uuid read event");
        this->powerpal_apikey_ = this->serial_to_apikey_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal apikey: %s", this->powerpal_apikey_.c_str());

        break;
      }

      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_WRITE_CHAR_EVT (Write confirmed)", this->parent_->address_str());

      if (param->write.handle == this->pairing_code_char_handle_) {
        this->subscription_in_progress_ = false;
        if (param->write.status != ESP_GATT_OK) {
          ESP_LOGW(TAG, "Error writing pairing code at handle %d, status=%d", param->write.handle, param->write.status);
          this->pending_subscription_ = true;
          if (!this->subscription_retry_scheduled_) {
            this->subscription_retry_scheduled_ = true;
            this->set_timeout(2000, [this]() {
              this->subscription_retry_scheduled_ = false;
              this->request_subscription_("retry-after-fail");
            });
          }
          break;
        }

        this->authenticated_ = true;
        this->pending_subscription_ = false;
        this->subscription_retry_scheduled_ = false;

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
                     this->parent_->address_str(), notify_battery_status);
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

      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error writing value to char at handle %d, status=%d", param->write.handle, param->write.status);
        break;
      }

      if (param->write.handle == this->reading_batch_size_char_handle_) {
        // reading batch size is now set correctly so subscribe to measurement notifications
        auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                        this->measurement_char_handle_);
        if (status) {
          ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                   this->parent_->address_str(), status);
        }
        break;
      }

      ESP_LOGW(TAG, "[%s] Missed all handle matches: %d",
               this->parent_->address_str(), param->write.handle);
      break;
    }  // ESP_GATTC_WRITE_CHAR_EVT
    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[%s] Received Notification", this->parent_->address_str());

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
        ESP_LOGI(TAG, "[%s] Authentication completed", this->parent_->address_str());
        this->pending_subscription_ = true;
        this->subscription_in_progress_ = false;
        this->subscription_retry_scheduled_ = false;
        this->request_subscription_("auth-complete");
      } else {
        ESP_LOGW(TAG, "[%s] Authentication failed, reason=0x%02x", this->parent_->address_str(),
                 param->ble_security.auth_cmpl.fail_reason);
        this->pending_subscription_ = false;
        this->subscription_in_progress_ = false;
        this->subscription_retry_scheduled_ = false;
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
