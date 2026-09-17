import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv

from . import DcBlueComponent
from .constants import CONF_AC_POWER, CONF_DC_BLUE_ID, CONF_LIGHT

DEPENDENCIES = ["dc_blue"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DC_BLUE_ID): cv.use_id(DcBlueComponent),
        cv.Optional(CONF_LIGHT): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_AC_POWER): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_DC_BLUE_ID])

    if light_config := config.get(CONF_LIGHT):
        sensor = await binary_sensor.new_binary_sensor(light_config)
        cg.add(parent.set_light_binary_sensor(sensor))

    if ac_power_config := config.get(CONF_AC_POWER):
        sensor = await binary_sensor.new_binary_sensor(ac_power_config)
        cg.add(parent.set_ac_power_binary_sensor(sensor))
