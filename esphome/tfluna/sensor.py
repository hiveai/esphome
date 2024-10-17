import esphome.codegen as cg
import esphome.config_validation as cv
import math
from esphome.components import i2c, sensor
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_DISTANCE,
    CONF_SIGNAL_STRENGTH,
    CONF_TEMPERATURE,
    CONF_TRIGGER_PIN,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_METER,
 )

DEPENDENCIES = ["i2c"]

tfluna_ns = cg.esphome_ns.namespace("tfluna")
TFLunaSensor = tfluna_ns.class_(
    "TFLunaSensor", cg.Component, i2c.I2CDevice
)

CONF_ENABLED = "enabled"
CONF_AMP_THRESHOLD = "amplitude_threshold"
CONF_DUMMY_DIST = "dummy_distance"
CONF_MINIMUM_DISTANCE= "minimum_distance"
CONF_MAXIMUM_DISTANCE = "maximum_distance"
CONF_SILENCE = "silence"
CONF_OUTPUT_FREQ = "output_frequency"
CONF_POWER_SAVING_FREQ = "power_saving_frequency"
CONF_ON_OFF_MODE = "on_off_mode"
CONF_ON_OFF_DISTANCE = "on_off_distance"
CONF_ON_OFF_ZONE = "on_off_zone"
CONF_ON_OFF_DELAY1 = "on_off_delay1"
CONF_ON_OFF_DELAY2 = "on_off_delay2"
CONF_LOW_SAMPLE_RATE_PERIOD = "low_sample_rate_period"
CONF_LOW_SAMPLE_RATE_FRAMES = "low_sample_rate_frames"

def validate_freq(value):
    value = cv.int_(value)
    if value != 0 and (value > 250 or value < 0 or math.floor(500 / math.floor(500 / value)) != value):
        raise cv.Invalid(f"{value} hz is not a supported frequency.")
    return value

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TFLunaSensor),
            cv.Required(CONF_TRIGGER_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_DISTANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SIGNAL_STRENGTH): sensor.sensor_schema(
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENABLED, default=True): cv.boolean,
            cv.Optional(CONF_AMP_THRESHOLD, default=100): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_DUMMY_DIST, default=0): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_MINIMUM_DISTANCE, default=0): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_MAXIMUM_DISTANCE, default=1000): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_SILENCE, default=False): cv.boolean,
            cv.Optional(CONF_OUTPUT_FREQ, default=100): validate_freq,
            cv.Optional(CONF_POWER_SAVING_FREQ, default=0): cv.int_range(min=0, max=10),
            cv.Optional(CONF_ON_OFF_MODE, default=0): cv.int_range(min=0, max=2),
            cv.Optional(CONF_ON_OFF_DISTANCE, default=0): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_ON_OFF_ZONE, default=0): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_ON_OFF_DELAY1, default=0): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_ON_OFF_DELAY2, default=0): cv.int_range(min=0, max=0xffff),
            cv.Optional(CONF_LOW_SAMPLE_RATE_PERIOD, default=0): cv.int_range(min=0, max=0xffffffff),
            cv.Optional(CONF_LOW_SAMPLE_RATE_FRAMES, default=0): cv.int_range(min=0, max=0xffffffff),
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x10))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    trigger_pin = await cg.gpio_pin_expression(config[CONF_TRIGGER_PIN])
    cg.add(var.set_trigger_pin(trigger_pin))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    if CONF_SIGNAL_STRENGTH in config:
        sens = await sensor.new_sensor(config[CONF_SIGNAL_STRENGTH])
        cg.add(var.set_signal_strength_sensor(sens))
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    cg.add(var.set_enabled(config[CONF_ENABLED]))
    cg.add(var.set_amplitude(config[CONF_AMP_THRESHOLD], config[CONF_DUMMY_DIST]))
    cg.add(var.set_distance_limit(config[CONF_MINIMUM_DISTANCE], config[CONF_MAXIMUM_DISTANCE], config[CONF_SILENCE]))
    cg.add(var.set_output_frequency(config[CONF_OUTPUT_FREQ]))
    cg.add(var.set_power_saving_frequency(config[CONF_POWER_SAVING_FREQ]))
    cg.add(var.set_on_off_mode(config[CONF_ON_OFF_MODE], config[CONF_ON_OFF_DISTANCE], config[CONF_ON_OFF_ZONE], 
        config[CONF_ON_OFF_DELAY1], config[CONF_ON_OFF_DELAY2]))
    cg.add(var.set_low_sample_rate(config[CONF_LOW_SAMPLE_RATE_PERIOD], config[CONF_LOW_SAMPLE_RATE_FRAMES]))
