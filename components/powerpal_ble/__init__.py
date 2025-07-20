MULTI_CONF = True
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, sensor, time
from esphome.const import CONF_ID
from esphome.components.ble_client import register_ble_node

DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["sensor"]

powerpal_ble_ns = cg.esphome_ns.namespace("powerpal_ble")
Powerpal = powerpal_ble_ns.class_("Powerpal", cg.Component, ble_client.BLEClientNode)

CONF_BLE_CLIENT_ID = "ble_client_id"
CONF_PAIRING_CODE = "pairing_code"
CONF_NOTIFICATION_INTERVAL = "notification_interval"
CONF_PULSES_PER_KWH = "pulses_per_kwh"
CONF_TIME_ID = "time_id"

CONF_BATTERY = "battery"
CONF_POWER_SENSOR = "power_sensor"
CONF_ENERGY_SENSOR = "energy_sensor"
CONF_DAILY_ENERGY_SENSOR = "daily_energy_sensor"
CONF_COST_SENSOR = "cost_sensor"
CONF_PULSES_SENSOR = "pulses_sensor"
CONF_WATT_HOURS = "watt_hours"
CONF_TIMESTAMP = "timestamp"
CONF_DAILY_PULSES_SENSOR = "daily_pulses_sensor"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Powerpal),
    cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
    cv.Required(CONF_PAIRING_CODE): cv.uint32_t,
    cv.Required(CONF_NOTIFICATION_INTERVAL): cv.uint8_t,
    cv.Required(CONF_PULSES_PER_KWH): cv.float_,
    cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
    cv.Optional(CONF_BATTERY): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_POWER_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_ENERGY_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_DAILY_ENERGY_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_COST_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_PULSES_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_WATT_HOURS): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_TIMESTAMP): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_DAILY_PULSES_SENSOR): cv.use_id(sensor.Sensor),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # BLE client
    client = await cg.get_variable(config[CONF_BLE_CLIENT_ID])
    cg.add(var.set_ble_client(client))

    # pairing, interval, etc.
    cg.add(var.set_pairing_code(config[CONF_PAIRING_CODE]))
    cg.add(var.set_notification_interval(config[CONF_NOTIFICATION_INTERVAL]))
    cg.add(var.set_pulses_per_kwh(config[CONF_PULSES_PER_KWH]))

    # time
    time_obj = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(time_obj))

    # Optional sensors — **each** goes through get_variable()
    if CONF_BATTERY in config:
        bat = await cg.get_variable(config[CONF_BATTERY])
        cg.add(var.set_battery(bat))
    if CONF_POWER_SENSOR in config:
        p = await cg.get_variable(config[CONF_POWER_SENSOR])
        cg.add(var.set_power_sensor(p))
    if CONF_ENERGY_SENSOR in config:
        e = await cg.get_variable(config[CONF_ENERGY_SENSOR])
        cg.add(var.set_energy_sensor(e))
    if CONF_DAILY_ENERGY_SENSOR in config:
        de = await cg.get_variable(config[CONF_DAILY_ENERGY_SENSOR])
        cg.add(var.set_daily_energy_sensor(de))
    if CONF_COST_SENSOR in config:
        c = await cg.get_variable(config[CONF_COST_SENSOR])
        cg.add(var.set_cost_sensor(c))
    if CONF_PULSES_SENSOR in config:
        ps = await cg.get_variable(config[CONF_PULSES_SENSOR])
        cg.add(var.set_pulses_sensor(ps))
    if CONF_WATT_HOURS in config:
        wh = await cg.get_variable(config[CONF_WATT_HOURS])
        cg.add(var.set_watt_hours(wh))
    if CONF_TIMESTAMP in config:
        ts = await cg.get_variable(config[CONF_TIMESTAMP])
        cg.add(var.set_timestamp(ts))
    if CONF_DAILY_PULSES_SENSOR in config:
        dps = await cg.get_variable(config[CONF_DAILY_PULSES_SENSOR])
        cg.add(var.set_daily_pulses_sensor(dps))
