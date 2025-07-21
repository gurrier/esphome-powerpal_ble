// powerpal_ble.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#else
#include <ctime>
#endif

#ifdef USE_ESP32

#include <esp_gattc_api.h>
#include <Preferences.h>

namespace esphome {
namespace powerpal_ble {

namespace espbt = esphome::esp32_ble_tracker;

struct PowerpalMeasurement {
  uint16_t pulses;
  time_t timestamp;
  uint32_t watt_hours;
  float cost;
};

static const espbt::ESPBTUUID POWERPAL_SERVICE_UUID =
    espbt::ESPBTUUID::from_raw("59DAABCD-12F4-25A6-7D4F-55961DCE4205");
// ... other UUIDs ...
static const uint8_t seconds_in_minute = 60;
static const float kw_to_w_conversion = 1000.0;

class Powerpal : public esphome::ble_client::BLEClientNode, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

 protected:
#ifdef USE_TIME
  optional<time::RealTimeClock *> time_{};
#endif
  uint16_t day_of_last_measurement_{0};

  Preferences prefs_;  // For persisting daily pulses

  uint8_t pairing_code_[4];
  uint8_t reading_batch_size_[4] = {0x01, 0x00, 0x00, 0x00};
  float pulses_per_kwh_;
  float pulse_multiplier_;
  uint64_t total_pulses_{0};
  uint64_t daily_pulses_{0};

  sensor::Sensor *battery_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *daily_energy_sensor_{nullptr};
  sensor::Sensor *cost_sensor_{nullptr};
  sensor::Sensor *pulses_sensor_{nullptr};
  sensor::Sensor *daily_pulses_sensor_{nullptr};
  sensor::Sensor *watt_hours_sensor_{nullptr};
  sensor::Sensor *timestamp_sensor_{nullptr};

  // Internal parsing and helpers
  std::string pkt_to_hex_(const uint8_t *data, uint16_t len);
  void decode_(const uint8_t *data, uint16_t length);
  void parse_battery_(const uint8_t *data, uint16_t length);
  void parse_measurement_(const uint8_t *data, uint16_t length);
  std::string uuid_to_device_id_(const uint8_t *data, uint16_t length);
  std::string serial_to_apikey_(const uint8_t *data, uint16_t length);
};

}  // namespace powerpal_ble
}  // namespace esphome

#endif  // USE_ESP32 

#endif
