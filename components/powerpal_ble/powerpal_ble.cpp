#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <nvs.h>
#include <nvs_flash.h>

#ifdef USE_ESP32
namespace esphome {
namespace powerpal_ble {

static const char *const TAG = "powerpal_ble";

void Powerpal::setup() {
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  nvs_open("powerpal", NVS_READWRITE, &this->nvs_handle_);

  // Load persisted daily_pulses_
  uint64_t stored = 0;
  if (nvs_get_u64(this->nvs_handle_, "daily", &stored) == ESP_OK) {
    this->daily_pulses_ = stored;
    ESP_LOGI(TAG, "Restored daily_pulses_ from prefs: %llu", stored);
  } else {
    ESP_LOGI(TAG, "No persisted daily_pulses_ found, starting from 0");
  }

  this->authenticated_ = false;
  this->pulse_multiplier_ = ((seconds_in_minute * this->reading_batch_size_[0]) / (this->pulses_per_kwh_ / kw_to_w_conversion));
  ESP_LOGI(TAG, "pulse_multiplier_: %f", this->pulse_multiplier_);
}

void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "POWERPAL BLE");
  LOG_SENSOR("  ", "Battery", this->battery_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  LOG_SENSOR("  ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR("  ", "Total Energy", this->energy_sensor_);
}


std::string Powerpal::pkt_to_hex_(const uint8_t *data, uint16_t len) {
  // unchanged helper
  char buf[64]; memset(buf, 0, sizeof(buf));
  for (int i = 0; i < len; i++) sprintf(&buf[i * 2], "%02x", data[i]);
  return std::string(buf);
}


void Powerpal::decode_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
}

void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  // unchanged
  if (length == 1 && this->battery_)
    this->battery_->publish_state(data[0]);
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  if (length < 6) return;
  // decode timestamp & pulses
  uint32_t unix_time = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
  uint16_t pulses = data[4] | (data[5] << 8);
  float avg_watts = pulses * this->pulse_multiplier_;

  // update sensors
  if (this->power_sensor_) this->power_sensor_->publish_state(avg_watts);
  if (this->pulses_sensor_) this->pulses_sensor_->publish_state(pulses);
  // ... other sensors unchanged ...

  // accumulate daily pulses
  this->daily_pulses_ += pulses;
  if (this->daily_pulses_sensor_) this->daily_pulses_sensor_->publish_state(this->daily_pulses_);

  // persist updated daily_pulses_
  esp_err_t err = nvs_set_u64(this->nvs_handle_, "daily", this->daily_pulses_);
  if (err == ESP_OK) {
    nvs_commit(this->nvs_handle_);
    ESP_LOGI(TAG, "Saved daily_pulses_ to prefs: %llu", this->daily_pulses_);
  } else {
    ESP_LOGW(TAG, "Failed to save daily_pulses_ to prefs: %d", err);
  }
}

std::string Powerpal::uuid_to_device_id_(const uint8_t *data, uint16_t length) {
  // unchanged
  const char *hexmap = "0123456789abcdef";
  std::string id;
  for (int i = length - 1; i >= 0; --i) {
    id += hexmap[(data[i] >> 4) & 0xF];
    id += hexmap[data[i] & 0xF];
  }
  return id;
}

std::string Powerpal::serial_to_apikey_(const uint8_t *data, uint16_t length) {
  // unchanged
  const char *hexmap = "0123456789abcdef";
  std::string key;
  for (int i = 0; i < (int)length; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) key += '-';
    key += hexmap[(data[i] >> 4) & 0xF];
    key += hexmap[data[i] & 0xF];
  }
  return key;
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

      // serialNumber
      if (param->read.handle == this->serial_number_char_handle_) {
        ESP_LOGI(TAG, "Received uuid read event");
        this->powerpal_device_id_ = this->uuid_to_device_id_(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal device id: %s", this->powerpal_device_id_.c_str());

        break;
      }

      // uuid
      if (param->read.handle == this->uuid_char_handle_) {
        ESP_LOGI(TAG, "Received serial_number read event");
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
