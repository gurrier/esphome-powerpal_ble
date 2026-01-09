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

// … setup() and other functions remain unchanged …

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

    // … other cases remain unchanged …

    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent_->address_str());
      // … rest of function unchanged …
      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_WRITE_CHAR_EVT (Write confirmed)", this->parent_->address_str());
      // … rest of function unchanged …
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[%s] Received Notification", this->parent_->address_str());
      // … rest of function unchanged …
      break;
    }
    default:
      break;
  }
}

void Powerpal::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
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
