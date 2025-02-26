import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.components.dc_blue import DcBlueComponent
from esphome.components.dc_blue.constants import (
    CONF_DC_BLUE_ID,
    CONF_LIGHT,
    CONF_BATTERY,
)
import esphome.config_validation as cv

DEPENDENCIES = ["dc_blue", "binary_sensor"]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_LIGHT): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_BATTERY): binary_sensor.binary_sensor_schema(),
    }
).extend(
    {
        cv.GenerateID(CONF_DC_BLUE_ID): cv.use_id(DcBlueComponent),
    }
)


async def to_code(config):
    platform = await cg.get_variable(config[CONF_DC_BLUE_ID])

    if light_config := config.get(CONF_LIGHT):
        sens = await binary_sensor.new_binary_sensor(light_config)
        cg.add(platform.set_light_binary_sensor(sens))
    
    if battery_config := config.get(CONF_BATTERY):
        sens = await binary_sensor.new_binary_sensor(battery_config)
        cg.add(platform.set_battery_binary_sensor(sens))
