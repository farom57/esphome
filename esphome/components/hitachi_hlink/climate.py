import esphome.codegen as cg
from esphome.components import binary_sensor, climate, sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_BEEPER,
    CONF_FILTER,
    CONF_ID,
    CONF_OUTDOOR_TEMPERATURE,
    CONF_TIMEOUT,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_TEMPERATURE,
    ICON_AIR_FILTER,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

CODEOWNERS = ["@farom57"]
DEPENDENCIES = ["climate", "uart", "sensor", "binary_sensor"]

hitachi_hlink_ns = cg.esphome_ns.namespace("hitachi_hlink")

HitachiClimate = hitachi_hlink_ns.class_(
    "HitachiClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(HitachiClimate),
            cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FILTER): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                icon=ICON_AIR_FILTER,
            ),
            cv.Optional(CONF_BEEPER, default=False): cv.boolean,
            cv.Optional(CONF_TIMEOUT, default="500ms"): cv.time_period,
        }
    )
    .extend(cv.polling_component_schema("30s"))
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)
    cg.add(var.set_beeper_feedback(config[CONF_BEEPER]))
    cg.add(var.set_response_timeout(config[CONF_TIMEOUT].total_milliseconds))
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))
    if CONF_FILTER in config:
        sens_filter = await binary_sensor.new_binary_sensor(config[CONF_FILTER])
        cg.add(var.set_filter_sensor(sens_filter))
