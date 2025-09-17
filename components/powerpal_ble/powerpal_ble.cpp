#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <time.h>
#include <type_traits>
#include <utility>
#include <esp_http_client.h>
#include <nvs.h>
#include <nvs_flash.h>
#include "esp_crt_bundle.h"

#ifdef USE_ESP32
namespace esphome {
namespace powerpal_ble {

namespace {

template<typename...>
using void_t = void;

template<typename T, typename = void>
struct has_connected_member : std::false_type {};

template<typename T>
struct has_connected_member<T, void_t<decltype(std::declval<const T &>().connected)>> : std::true_type {};

template<typename Client>
bool client_connected(Client *client, std::true_type) {
  return client->connected;
}

template<typename Client>
bool client_connected(Client *client, std::false_type) {
  return client->is_connected();
}

}  // namespace

static const char *const TAG = "powerpal_ble";
static constexpr uint32_t POWERPAL_RECONNECT_DELAY_MS = 10000;
static constexpr uint32_t POWERPAL_HANDSHAKE_RETRY_MS = 500;
static constexpr uint32_t POWERPAL_NOTIFY_RETRY_MS = 1000;

Powerpal::~Powerpal() {
  if (this->nvs_ok_) {
    nvs_close(this->nvs_handle_);
    this->nvs_ok_ = false;
  }
}

void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "POWERPAL");
  LOG_SENSOR(" ", "Battery", this->battery_);
  LOG_SENSOR(" ", "Power", this->power_sensor_);
  LOG_SENSOR(" ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR(" ", "Total Energy", this->energy_sensor_);
  LOG_SENSOR(" ", "Cost", this->cost_sensor_);
  LOG_SENSOR(" ", "Pulses", this->pulses_sensor_);
  LOG_SENSOR(" ", "Watt Hours", this->watt_hours_sensor_);
  LOG_SENSOR(" ", "Timestamp", this->timestamp_sensor_);
  LOG_SENSOR(" ", "Daily Pulses", this->daily_pulses_sensor_);
  }

void Powerpal::setup() {
  this->clear_connection_state_();
  this->reconnect_scheduled_ = false;

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
    err = nvs_get_u64(this->nvs_handle_, "total", &stored_total);
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

bool Powerpal::is_parent_connected_() const {
  if (this->parent_ == nullptr)
    return false;

  return client_connected(this->parent_, has_connected_member<ble_client::BLEClient>{});
}

void Powerpal::clear_connection_state_() {
  this->authenticated_ = false;
  this->service_discovered_ = false;
  this->auth_complete_ = false;
  this->measurement_notify_registered_ = false;
  this->pairing_code_write_inflight_ = false;
  this->handshake_in_progress_ = false;

  this->pairing_code_char_handle_ = 0;
  this->reading_batch_size_char_handle_ = 0;
  this->measurement_char_handle_ = 0;
  this->battery_char_handle_ = 0;
  this->led_sensitivity_char_handle_ = 0;
  this->firmware_char_handle_ = 0;
  this->uuid_char_handle_ = 0;
  this->serial_number_char_handle_ = 0;
}

void Powerpal::schedule_reconnect_() {
  if (this->parent_ == nullptr)
    return;

  if (this->reconnect_scheduled_) {
    ESP_LOGV(TAG, "[%s] Reconnect already scheduled", this->parent_->address_str().c_str());
    return;
  }

  this->reconnect_scheduled_ = true;
  this->set_timeout("powerpal_ble_reconnect", POWERPAL_RECONNECT_DELAY_MS, [this]() {
    this->reconnect_scheduled_ = false;
    if (this->parent_ == nullptr)
      return;

    if (this->is_parent_connected_()) {
      ESP_LOGV(TAG, "[%s] Reconnect timer fired but already connected", this->parent_->address_str().c_str());
      return;
    }

    ESP_LOGI(TAG, "[%s] Attempting BLE reconnect", this->parent_->address_str().c_str());
    this->parent_->connect();
  });
}

void Powerpal::attempt_subscription_(uint32_t delay_ms) {
  this->set_timeout("powerpal_ble_handshake", delay_ms, [this]() {
    if (this->parent_ == nullptr)
      return;

    if (!this->is_parent_connected_()) {
      ESP_LOGV(TAG, "[%s] Not connected, waiting before attempting resubscribe",
               this->parent_->address_str().c_str());
      return;
    }

    if (this->measurement_notify_registered_) {
      ESP_LOGV(TAG, "[%s] Measurement notifications already active",
               this->parent_->address_str().c_str());
      return;
    }

    if (!this->service_discovered_) {
      ESP_LOGV(TAG, "[%s] Waiting for service discovery", this->parent_->address_str().c_str());
      this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
      return;
    }

    if (!this->auth_complete_) {
      ESP_LOGV(TAG, "[%s] Waiting for authentication", this->parent_->address_str().c_str());
      this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
      return;
    }

    if (this->handshake_in_progress_) {
      ESP_LOGV(TAG, "[%s] Handshake already in progress", this->parent_->address_str().c_str());
      this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
      return;
    }

    if (this->pairing_code_char_handle_ == 0 || this->reading_batch_size_char_handle_ == 0 ||
        this->measurement_char_handle_ == 0) {
      ESP_LOGW(TAG, "[%s] Characteristic handles missing, retrying discovery",
               this->parent_->address_str().c_str());
      this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
      return;
    }

    esp_err_t status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                this->pairing_code_char_handle_, sizeof(this->pairing_code_),
                                                this->pairing_code_, ESP_GATT_WRITE_TYPE_RSP,
                                                ESP_GATT_AUTH_REQ_NONE);
    if (status != ESP_OK) {
      ESP_LOGW(TAG, "[%s] Failed to write pairing code (%d), will retry",
               this->parent_->address_str().c_str(), status);
      this->handshake_in_progress_ = false;
      this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS * 2);
      return;
    }

    ESP_LOGI(TAG, "[%s] Sent pairing code to resume notifications",
             this->parent_->address_str().c_str());
    this->handshake_in_progress_ = true;
    this->pairing_code_write_inflight_ = true;
  });
}

bool Powerpal::register_for_measurement_notifications_() {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Measurement subscription requested but BLE client is unavailable");
    return false;
  }

  if (this->measurement_char_handle_ == 0) {
    ESP_LOGW(TAG, "[%s] Measurement handle not available for notification subscription",
             this->parent_->address_str().c_str());
    return false;
  }

  if (this->measurement_notify_registered_) {
    return true;
  }

  auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                  this->measurement_char_handle_);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
             this->parent_->address_str().c_str(), status);
    this->handshake_in_progress_ = false;
    this->measurement_notify_registered_ = false;
    this->attempt_subscription_(POWERPAL_NOTIFY_RETRY_MS);
    return false;
  }

  ESP_LOGI(TAG, "[%s] Measurement notifications enabled", this->parent_->address_str().c_str());
  this->measurement_notify_registered_ = true;
  this->handshake_in_progress_ = false;
  return true;
}

void Powerpal::handle_connect_() {
  if (this->parent_ == nullptr)
    return;

  ESP_LOGI(TAG, "[%s] Connected to Powerpal", this->parent_->address_str().c_str());
  this->auth_complete_ = false;
  this->service_discovered_ = false;
  this->measurement_notify_registered_ = false;
  this->handshake_in_progress_ = false;
  this->pairing_code_write_inflight_ = false;
  this->reconnect_scheduled_ = false;
  this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
}

void Powerpal::handle_disconnect_() {
  if (this->parent_ == nullptr)
    return;

  ESP_LOGW(TAG, "[%s] Disconnected from Powerpal", this->parent_->address_str().c_str());
  this->clear_connection_state_();
  this->schedule_reconnect_();
}

void Powerpal::on_connect() {
  this->handle_connect_();
}

void Powerpal::on_disconnect() {
  this->handle_disconnect_();
}


std::string Powerpal::pkt_to_hex_(const uint8_t *data, uint16_t len) {
  std::string ret;
  ret.reserve(static_cast<size_t>(len) * 2);
  for (uint16_t i = 0; i < len; i++) {
    char byte_buf[3];
    snprintf(byte_buf, sizeof(byte_buf), "%02x", data[i]);
    ret.append(byte_buf, 2);
  }
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
  auto calc_today = [&]() -> int {
    struct tm tm_info;
    if (localtime_r(&unix_time, &tm_info) == nullptr) {
      ESP_LOGE(TAG, "localtime_r failed");
      return 0;
    }
    return tm_info.tm_yday + 1;
  };

  int today;
#ifdef USE_TIME
  if (this->time_.has_value()) {
    auto *time_comp = *this->time_;
    auto now = time_comp->now();
    if (now.is_valid()) {
      today = now.day_of_year;
    } else {
      today = calc_today();
    }
  } else {
    today = calc_today();
  }
#else
  today = calc_today();
#endif

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
      ++this->nvs_commit_count_;
    }
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
  float cost = 0.0f;
  if (this->energy_cost_ > 0.0f) {
    cost = (float(pulses) / this->pulses_per_kwh_) * this->energy_cost_;
    if (this->cost_sensor_)
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
               ++this->nvs_commit_count_, now_s,
               this->total_pulses_, this->daily_pulses_);
      nvs_set_u64(this->nvs_handle_, "total", this->total_pulses_);
      nvs_set_u64(this->nvs_handle_, "daily", this->daily_pulses_);
      esp_err_t nvs_err = nvs_commit(this->nvs_handle_);
      if (nvs_err != ESP_OK)
        ESP_LOGE(TAG, "NVS commit failed (%d)", nvs_err);
      this->last_commit_ts_ = now_s;
      this->last_pulses_for_threshold_ = this->total_pulses_;
    }
  }

  if (this->daily_pulses_sensor_)
    this->daily_pulses_sensor_->publish_state(this->daily_pulses_);

  if (this->powerpal_cloud_uploader_) {
    if (this->powerpal_device_id_.length() && this->powerpal_apikey_.length()) {
      ESP_LOGD(TAG, "Attempting upload to Powerpal");
      this->upload_reading_(t32, pulses, cost, wh);
    } else {
      ESP_LOGD(TAG, "Skipping upload: missing device ID or API key");
    }
  }
}

void Powerpal::upload_reading_(uint32_t timestamp, uint16_t pulses, float cost, float watt_hours) {
  char url[128];
  snprintf(url, sizeof(url), "https://readings.powerpal.net/api/v1/meter_reading/%s", this->powerpal_device_id_.c_str());

  char payload[256];
  snprintf(payload, sizeof(payload),
           "[ {\"cost\":%.2f, \"is_peak\": false, \"pulses\":%u, \"timestamp\":%lu, \"watt_hours\":%.0f} ]",
           cost, pulses, static_cast<unsigned long>(timestamp), watt_hours);

  ESP_LOGD(TAG, "Upload URL: %s", url);
  ESP_LOGD(TAG, "Upload JSON: %s", payload);
  esp_http_client_config_t config = {};
  config.url = url;
  config.timeout_ms = 5000;
  config.crt_bundle_attach = esp_crt_bundle_attach;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    ESP_LOGW(TAG, "Failed to initialise HTTP client");
    return;
  }
  esp_http_client_set_method(client, HTTP_METHOD_POST);
  esp_http_client_set_header(client, "Authorization", this->powerpal_apikey_.c_str());
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, payload, strlen(payload));

  esp_err_t err = esp_http_client_perform(client);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Upload to Powerpal failed: %s", esp_err_to_name(err));
  } else {
    int status = esp_http_client_get_status_code(client);
    if (status < 200 || status >= 300) {
      ESP_LOGW(TAG, "Upload to Powerpal failed, status: %d", status);
    } else {
      ESP_LOGV(TAG, "Uploaded reading to Powerpal (status %d)", status);
    }
  }
  esp_http_client_cleanup(client);
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
    case ESP_GATTC_CONNECT_EVT: {
      if (this->parent_ != nullptr) {
        ESP_LOGI(TAG, "[%s] ESP_GATTC_CONNECT_EVT", this->parent_->address_str().c_str());
        this->handle_connect_();
      } else {
        ESP_LOGW(TAG, "ESP_GATTC_CONNECT_EVT received without an active parent");
      }
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      if (this->parent_ != nullptr) {
        ESP_LOGW(TAG, "[%s] ESP_GATTC_DISCONNECT_EVT", this->parent_->address_str().c_str());
        this->handle_disconnect_();
      } else {
        ESP_LOGW(TAG, "ESP_GATTC_DISCONNECT_EVT received without an active parent");
        this->clear_connection_state_();
      }
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

      this->service_discovered_ = true;
      this->measurement_notify_registered_ = false;
      this->handshake_in_progress_ = false;
      this->pairing_code_write_inflight_ = false;
      this->attempt_subscription_(0);
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent_->address_str().c_str());
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        if (param->read.handle == this->reading_batch_size_char_handle_) {
          this->handshake_in_progress_ = false;
          this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
        }
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
              this->handshake_in_progress_ = false;
              this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS * 2);
            }
          } else {
            // reading batch size is set correctly so subscribe to measurement notifications
            if (!this->register_for_measurement_notifications_()) {
              this->handshake_in_progress_ = false;
            }
          }
        } else {
          ESP_LOGW(TAG, "Unexpected reading_batch_size length (%d)", param->read.value_len);
          this->handshake_in_progress_ = false;
          this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS);
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
        this->pairing_code_write_inflight_ = false;

        auto read_reading_batch_size_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->reading_batch_size_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_reading_batch_size_status) {
          ESP_LOGW(TAG, "Error sending read request for reading batch size, status=%d", read_reading_batch_size_status);
          this->handshake_in_progress_ = false;
          this->attempt_subscription_(POWERPAL_HANDSHAKE_RETRY_MS * 2);
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
        this->register_for_measurement_notifications_();
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
      this->auth_complete_ = param->ble_security.auth_cmpl.success;
      if (this->auth_complete_) {
        ESP_LOGI(TAG, "[%s] BLE authentication complete", this->parent_->address_str().c_str());
        this->attempt_subscription_(0);
      } else {
        ESP_LOGW(TAG, "[%s] BLE authentication failed", this->parent_->address_str().c_str());
        this->handshake_in_progress_ = false;
        this->schedule_reconnect_();
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
