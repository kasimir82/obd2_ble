import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import (
    CONF_ID,
    CONF_SERVICE_UUID,
    CONF_CHARACTERISTIC_UUID,
)

DEPENDENCIES = ['ble_client']

obd2_ble_ns = cg.esphome_ns.namespace('obd2_ble')
OBD2BLEClient = obd2_ble_ns.class_(
    'OBD2BLEClient', ble_client.BLEClientNode, cg.Component
)

CONF_OBD2BLE_ID = "obd2_ble_id"
CONF_READ = "read"
CONF_WRITE = "write"
CONF_NOTIFY = "notify"
CONF_INIT_COMMANDS = "init_commands"
CONF_COMMAND_DELAY = "command_delay"
CONF_COMMAND_WAIT = "command_wait"
CONF_RESPONSE_WAIT = "response_wait"
CONF_PUBLISH_DELAY = "publish_delay"
CONF_DISCONNECT_DELAY = "disconnect_delay"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OBD2BLEClient),
    cv.Required(CONF_SERVICE_UUID): cv.string,
    cv.Required(CONF_CHARACTERISTIC_UUID): cv.Schema({
        cv.Required(CONF_READ): cv.string,
        cv.Required(CONF_WRITE): cv.string,
        cv.Required(CONF_NOTIFY): cv.string,
    }),
    cv.Optional(CONF_INIT_COMMANDS, default="ATZ ATE0 ATL0 ATS0 ATH1 ATSP0 ATDPN 0100"): cv.string,
    cv.Optional(CONF_COMMAND_DELAY, default=1000): cv.int_,
    cv.Optional(CONF_COMMAND_WAIT, default=5000): cv.int_,
    cv.Optional(CONF_RESPONSE_WAIT, default=10000): cv.int_,
    cv.Optional(CONF_PUBLISH_DELAY, default=500): cv.int_,
    cv.Optional(CONF_DISCONNECT_DELAY, default=600000): cv.int_,
}).extend(ble_client.BLE_CLIENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    cg.add(var.set_service_uuid(config[CONF_SERVICE_UUID]))
    cg.add(var.set_read_char_uuid(config[CONF_CHARACTERISTIC_UUID][CONF_READ]))
    cg.add(var.set_write_char_uuid(config[CONF_CHARACTERISTIC_UUID][CONF_WRITE]))
    cg.add(var.set_notify_char_uuid(config[CONF_CHARACTERISTIC_UUID][CONF_NOTIFY]))
    cg.add(var.set_init_commands(config[CONF_INIT_COMMANDS].split()))
    cg.add(var.set_command_delay(config[CONF_COMMAND_DELAY]))
    cg.add(var.set_command_wait(config[CONF_COMMAND_WAIT]))
    cg.add(var.set_response_wait(config[CONF_RESPONSE_WAIT]))
    cg.add(var.set_publish_delay(config[CONF_PUBLISH_DELAY]))
    cg.add(var.set_disconnect_delay(config[CONF_DISCONNECT_DELAY]))

