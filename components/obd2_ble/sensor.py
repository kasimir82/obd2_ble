import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_MODE,
)

from . import OBD2BLEClient, CONF_OBD2BLE_ID, obd2_ble_ns

DEPENDENCIES = ['obd2_ble']

CONF_CANID = 'can_id'
CONF_PID = 'pid'

CONFIG_SCHEMA = sensor.sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(sensor.Sensor),
    cv.GenerateID(CONF_OBD2BLE_ID): cv.use_id(OBD2BLEClient),
    cv.Required(CONF_NAME): cv.string,
    cv.Optional(CONF_CANID): cv.string,
    cv.Required(CONF_MODE): cv.string,
    cv.Required(CONF_PID): cv.string,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    parent = await cg.get_variable(config[CONF_OBD2BLE_ID])
    can_id = ''
    if CONF_CANID in config:
        can_id = config[CONF_CANID]
    cg.add(parent.add_task_for_sensor(var, can_id, config[CONF_MODE], config[CONF_PID]))


