import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_MODE,
)

from . import OBD2BLEClient, CONF_OBD2BLE_ID, obd2_ble_ns

DEPENDENCIES = ['obd2_ble']

CONF_CODE = "code"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
    cv.GenerateID(CONF_OBD2BLE_ID): cv.use_id(OBD2BLEClient),
    cv.Required(CONF_NAME): cv.string,
    cv.Required(CONF_MODE): cv.string,
    cv.Required(CONF_CODE): cv.string,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await binary_sensor.register_binary_sensor(var, config)
    parent = await cg.get_variable(config[CONF_OBD2BLE_ID])
    cg.add(parent.add_task_for_binary_sensor(var, config[CONF_MODE], config[CONF_CODE]))

