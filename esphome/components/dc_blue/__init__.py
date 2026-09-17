from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32
import esphome.config_validation as cv
from esphome.const import CONF_DATA_PIN, CONF_ID, CONF_INVERTED, CONF_TRIGGER_PIN
from esphome.cpp_helpers import gpio_pin_expression

from .constants import CONF_CLEAR_PERIOD, CONF_SYMBOL_PERIOD, CONF_TRIGGER_PERIOD

CODEOWNERS = ["@jpmeijers"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["binary_sensor", "cover"]

dc_blue_component_ns = cg.esphome_ns.namespace("dc_blue")
DcBlueComponent = dc_blue_component_ns.class_("DcBlueComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DcBlueComponent),
        cv.Required(CONF_DATA_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_TRIGGER_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_SYMBOL_PERIOD, default=970): cv.int_range(min=100, max=10000),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
        cv.Optional(CONF_TRIGGER_PERIOD, default=1000): cv.int_range(min=100, max=10000),
        cv.Optional(CONF_CLEAR_PERIOD, default=1000): cv.int_range(min=100, max=10000),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # ESPHome 2026.9+ excludes GPTimer by default. Older versions without this
    # helper already include the driver, so retain compatibility with them.
    if hasattr(esp32, "include_builtin_idf_component"):
        esp32.include_builtin_idf_component("esp_driver_gptimer")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    data_pin = await gpio_pin_expression(config[CONF_DATA_PIN])
    cg.add(var.set_data_pin(data_pin))
    trigger_pin = await gpio_pin_expression(config[CONF_TRIGGER_PIN])
    cg.add(var.set_trigger_pin(trigger_pin))

    cg.add(var.set_symbol_period(config[CONF_SYMBOL_PERIOD]))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_trigger_period(config[CONF_TRIGGER_PERIOD]))
    cg.add(var.set_clear_period(config[CONF_CLEAR_PERIOD]))
