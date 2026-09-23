import json
import logging
import subprocess
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, ble_client, text_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_BATTERY_LEVEL,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TIMESTAMP,
    ENTITY_CATEGORY_DIAGNOSTIC,
    DEVICE_CLASS_ENERGY,
    CONF_ENERGY,
    CONF_POWER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS,
    UNIT_WATT,
    UNIT_PERCENT,
    CONF_TIME_ID,
)

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@gurrier"]
DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["text_sensor"]

powerpal_ble_ns = cg.esphome_ns.namespace("powerpal_ble")
Powerpal = powerpal_ble_ns.class_("Powerpal", ble_client.BLEClientNode, cg.Component)

CONF_PAIRING_CODE = "pairing_code"
CONF_NOTIFICATION_INTERVAL = "notification_interval"
CONF_PULSES_PER_KWH = "pulses_per_kwh"
CONF_COST_PER_KWH = "cost_per_kwh"
CONF_POWERPAL_DEVICE_ID = "powerpal_device_id"
CONF_POWERPAL_APIKEY = "powerpal_apikey"
CONF_DAILY_ENERGY = "daily_energy"
CONF_WATT_HOURS = "watt_hours"
CONF_TIME_STAMP = "timestamp"
CONF_PULSES = "pulses"
CONF_COST = "cost"
CONF_DAILY_PULSES = "daily_pulses"
CONF_LED_SENSITIVITY = "led_sensitivity"
CONF_VERSION = "version"
CONF_STALE_RESTART_AFTER = "stale_restart_after"
CONF_WATCHDOG_RESTART_COUNT = "watchdog_restart_count"
CONF_WATCHDOG_LAST_REASON = "watchdog_last_reason"


def _component_version():
    """Version of this component: manifest.json, plus the git commit when known."""
    base = Path(__file__).resolve().parent
    try:
        version = json.loads((base / "manifest.json").read_text())["version"]
    except (OSError, ValueError, KeyError):
        version = "unknown"

    # Only trust git if this folder is inside its own checkout. Otherwise (e.g. a
    # copy dropped into a config directory that is itself a git repo) it would
    # report an unrelated repository's commit.
    def git(*args):
        return subprocess.run(
            ["git", "-C", str(base), *args],
            capture_output=True, text=True, timeout=5, check=True,
        ).stdout.strip()

    try:
        if Path(git("rev-parse", "--show-toplevel")).resolve() == base.parent.parent:
            sha = git("rev-parse", "--short", "HEAD")
            if sha:
                version = f"{version} ({sha})"
    except (OSError, subprocess.SubprocessError):
        pass
    return version


def _validate(config):
    if CONF_DAILY_ENERGY in config and CONF_TIME_ID not in config:
        _LOGGER.warning(
            "Using daily_energy without a time_id means relying on your Powerpal's RTC for packet times, which is not recommended. "
            "Please consider adding a time component to your ESPHome yaml, and its time_id to your powerpal_ble component."
        )
    return config


def powerpal_deviceid(value):
    value = cv.string_strict(value)
    if len(value) != 8:
        raise cv.Invalid(f"{CONF_POWERPAL_DEVICE_ID} must be 8 digits")
    try:
        int(value, 16)
    except ValueError:
        raise cv.Invalid(
            f"{CONF_POWERPAL_DEVICE_ID} must only be a string of hexadecimal values"
        )
    return value


def powerpal_apikey(value):
    value = cv.string_strict(value)
    parts = value.split("-")
    if len(parts) != 5:
        raise cv.Invalid("UUID must consist of 5 - (hyphen) separated parts")
    parts_int = []
    if (
        len(parts[0]) != 8
        or len(parts[1]) != 4
        or len(parts[2]) != 4
        or len(parts[3]) != 4
        or len(parts[4]) != 12
    ):
        raise cv.Invalid("UUID must be of format XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX")
    for part in parts:
        try:
            parts_int.append(int(part, 16))
        except ValueError:
            raise cv.Invalid("UUID parts must be hexadecimal values from 00 to FF")

    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Powerpal),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_DAILY_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                # Resets to 0 at midnight. HA's docs are explicit that this is exactly
                # what TOTAL_INCREASING is for: a decreasing reading is interpreted as
                # the start of a new meter cycle, not an error. (MEASUREMENT, tried
                # earlier, is invalid for device_class=energy; no state_class at all,
                # tried after that, needlessly threw away long-term statistics.)
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_WATT_HOURS): sensor.sensor_schema(),
            cv.Optional(CONF_PULSES): sensor.sensor_schema(),
            cv.Optional(CONF_DAILY_PULSES): sensor.sensor_schema(),
            # A plain numeric sensor can't hold this exactly: ESPHome's Sensor::publish_state()
            # takes a float, whose 24-bit mantissa only represents integers exactly up to ~16.7M
            # -- a unix timestamp (~1.79 billion) rounds to the nearest multiple of 128, silently
            # merging any two readings less than ~64s apart. A text sensor with an ISO 8601 string
            # has no such limit, and HA renders device_class: timestamp as a real datetime.
            cv.Optional(CONF_TIME_STAMP): text_sensor.text_sensor_schema(
                device_class=DEVICE_CLASS_TIMESTAMP,
            ),
            cv.Optional(CONF_COST): sensor.sensor_schema(
                # A float has ~7 significant decimal digits total, regardless of magnitude,
                # so anything past single-digit decimals here would just be display noise.
                accuracy_decimals=6
            ),
            cv.Required(CONF_PAIRING_CODE): cv.int_range(min=1, max=999999),
            cv.Required(CONF_NOTIFICATION_INTERVAL): cv.int_range(min=1, max=60),
            cv.Required(CONF_PULSES_PER_KWH): cv.float_range(min=1),
            cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                device_class=DEVICE_CLASS_BATTERY,
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LED_SENSITIVITY): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                icon="mdi:led-on",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VERSION): text_sensor.text_sensor_schema(
                icon="mdi:tag-outline",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # Opt-in self-healing: if set, restart the whole device after this long without a
            # measurement. Field evidence is that the ESP32's BLE stack can wedge in a way a
            # same-process reconnect never recovers from, while WiFi/the API stay fully
            # responsive -- see the comment on stale_restart_after_s_ in powerpal_ble.h. Unset
            # by default so nobody's device reboots itself without asking for it.
            cv.Optional(CONF_STALE_RESTART_AFTER): cv.positive_time_period_seconds,
            cv.Optional(CONF_WATCHDOG_RESTART_COUNT): sensor.sensor_schema(
                icon="mdi:restart-alert",
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_WATCHDOG_LAST_REASON): text_sensor.text_sensor_schema(
                icon="mdi:restart-alert",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_COST_PER_KWH): cv.float_range(min=0),
            cv.Optional(
                CONF_POWERPAL_DEVICE_ID
            ): powerpal_deviceid,  # deviceid (optional) # if not configured, will grab from device
            cv.Optional(
                CONF_POWERPAL_APIKEY
            ): powerpal_apikey,  # apikey (optional) # if not configured, will grab from device
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    if CONF_POWER in config:
        sens = await sensor.new_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))

    if CONF_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY])
        cg.add(var.set_energy_sensor(sens))

    if CONF_DAILY_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_DAILY_ENERGY])
        cg.add(var.set_daily_energy_sensor(sens))

    if CONF_PULSES in config:
        sens = await sensor.new_sensor(config[CONF_PULSES])
        cg.add(var.set_pulses_sensor(sens))

    if CONF_DAILY_PULSES in config:
        sens = await sensor.new_sensor(config[CONF_DAILY_PULSES])
        cg.add(var.set_daily_pulses_sensor(sens))
    if CONF_WATT_HOURS in config:
        sens = await sensor.new_sensor(config[CONF_WATT_HOURS])
        cg.add(var.set_watt_hours(sens))

    if CONF_TIME_STAMP in config:
        sens = await text_sensor.new_text_sensor(config[CONF_TIME_STAMP])
        cg.add(var.set_timestamp(sens))

    if CONF_COST in config:
        sens = await sensor.new_sensor(config[CONF_COST])
        cg.add(var.set_cost_sensor(sens))

    if CONF_PAIRING_CODE in config:
        cg.add(var.set_pairing_code(config[CONF_PAIRING_CODE]))

    if CONF_NOTIFICATION_INTERVAL in config:
        cg.add(var.set_notification_interval(config[CONF_NOTIFICATION_INTERVAL]))

    if CONF_PULSES_PER_KWH in config:
        cg.add(var.set_pulses_per_kwh(config[CONF_PULSES_PER_KWH]))

    if CONF_BATTERY_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(var.set_battery(sens))

    if CONF_LED_SENSITIVITY in config:
        sens = await sensor.new_sensor(config[CONF_LED_SENSITIVITY])
        cg.add(var.set_led_sensitivity(sens))

    cg.add(var.set_version(_component_version()))
    if CONF_VERSION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_VERSION])
        cg.add(var.set_version_sensor(sens))

    if CONF_STALE_RESTART_AFTER in config:
        cg.add(var.set_stale_restart_after(config[CONF_STALE_RESTART_AFTER]))

    if CONF_WATCHDOG_RESTART_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_WATCHDOG_RESTART_COUNT])
        cg.add(var.set_watchdog_restart_count_sensor(sens))

    if CONF_WATCHDOG_LAST_REASON in config:
        sens = await text_sensor.new_text_sensor(config[CONF_WATCHDOG_LAST_REASON])
        cg.add(var.set_watchdog_last_reason_sensor(sens))

    if CONF_COST_PER_KWH in config:
        cg.add(var.set_energy_cost(config[CONF_COST_PER_KWH]))

    if CONF_POWERPAL_DEVICE_ID in config:
        cg.add(var.set_device_id(config[CONF_POWERPAL_DEVICE_ID]))

    if CONF_POWERPAL_APIKEY in config:
        cg.add(var.set_apikey(config[CONF_POWERPAL_APIKEY]))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(time_))
