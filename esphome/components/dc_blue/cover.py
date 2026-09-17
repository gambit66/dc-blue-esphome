import esphome.codegen as cg
from esphome.components import cover
import esphome.config_validation as cv

from . import DcBlueComponent, dc_blue_component_ns
from .constants import CONF_DC_BLUE_ID

DEPENDENCIES = ["dc_blue"]

DcBlueCover = dc_blue_component_ns.class_("DcBlueCover", cover.Cover)

CONFIG_SCHEMA = cover.cover_schema(DcBlueCover).extend(
    {cv.GenerateID(CONF_DC_BLUE_ID): cv.use_id(DcBlueComponent)}
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_DC_BLUE_ID])
    sensor = await cover.new_cover(config)
    cg.add(parent.set_garage_cover_sensor(sensor))
