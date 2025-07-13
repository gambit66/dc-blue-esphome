import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.components.dc_blue import dc_blue_component_ns, DcBlueComponent
from esphome.components.dc_blue.constants import (
    CONF_DC_BLUE_ID,
)

import esphome.config_validation as cv

DEPENDENCIES = ["dc_blue", "cover"]

DcBlueCover = dc_blue_component_ns.class_("DcBlueCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.cover_schema(DcBlueCover).extend(
    {
        cv.GenerateID(CONF_DC_BLUE_ID): cv.use_id(DcBlueComponent),
    }
)


async def to_code(config):
    platform = await cg.get_variable(config[CONF_DC_BLUE_ID])
    
    sens = platform.create_garage_cover_sensor()
    await cover.register_cover(sens, config)
