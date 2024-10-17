#include "tfluna_sensor.h"
#include "esphome/core/log.h"
#include <cinttypes>

// Support for the Benewake TF-Luna LIDAR distance sensor

namespace esphome {
namespace tfluna {

static const uint8_t TFLUNA_TRIGGER_CMD[] = {0x5a,0x04,0x04,0x00};
static const uint8_t TFLUNA_SOFT_RESET_CMD[] = {0x5a,0x04,0x02,0x00};
static uint8_t TFLUNA_AMPLITUDE_THRESHOLD_CMD[] = {0x5a,0x07,0x22,0x00,0x00,0x00,0x00};
static uint8_t TFLUNA_DISTANCE_LIMIT_CMD[] = {0x5a,0x09,0x3A,0x00,0x00,0x00,0x00,0x00,0x00};
static uint8_t TFLUNA_READ_ID_CMD[] = {0x5a,0x05,0x3f,0x00,0x00};
static uint8_t TFLUNA_OUTPUT_FREQUENCY_CMD[] = {0x5a,0x06,0x03,0x01,0x00,0x00};
static uint8_t TFLUNA_POWER_SAVING_FREQUENCY_CMD[] = {0x5a,0x06,0x35,0x00,0x00,0x00};
static uint8_t TFLUNA_ON_OFF_MODE_CMD[] = {0x5a,0x0d,0x3b,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static uint8_t TFLUNA_LOW_SAMPLE_RATE_CMD[] = {0x5a,0x0c,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static const uint8_t TFLUNA_DATA_REGISTER = 0x00;
static const uint8_t TFLUNA_DATA_LENGTH = 6;
static const uint8_t TFLUNA_VERSION_REGISTER = 0x0a;
static const uint8_t TFLUNA_VERSION_LENGTH = 3;
static const uint8_t TFLUNA_SN_REGISTER = 0x10;
static const uint8_t TFLUNA_SN_LENGTH = 14;
static const uint8_t TFLUNA_ENABLED_REGISTER = 0x25;
static const uint8_t TFLUNA_AMPLITUDE_THRESHOLD_REGISTER = 0x2a;
static const uint8_t TFLUNA_DUMMY_DISTANCE_REGISTER = 0x2c;
static const uint8_t TFLUNA_MINIMUM_DISTANCE_REGISTER = 0x2e;
static const uint8_t TFLUNA_MAXIMUM_DISTANCE_REGISTER = 0x30;

static const uint16_t TFLUNA_OVER_EXPOSED_VALUE = UINT16_MAX;
static bool isSetup = false;

static unsigned char sn_save[TFLUNA_SN_LENGTH + 1];
static uint8_t ver_save[TFLUNA_VERSION_LENGTH];
static bool enabled_save = false;
static uint16_t amplitude_threshold_save = UINT16_MAX;
static uint16_t dummy_distance_save = UINT16_MAX;
static uint16_t minimum_distance_save = UINT16_MAX;
static uint16_t maximum_distance_save = UINT16_MAX;
static bool silence_save = false;
static uint16_t output_frequency_save = UINT16_MAX;
static uint8_t power_saving_frequency_save = UINT8_MAX;
static uint8_t on_off_mode_save = UINT8_MAX;
static uint16_t on_off_distance_save = UINT16_MAX;
static uint16_t on_off_zone_save = UINT16_MAX;
static uint16_t on_off_delay1_save = UINT16_MAX;
static uint16_t on_off_delay2_save = UINT16_MAX;
static uint32_t low_sample_rate_period_save = UINT32_MAX;
static uint32_t low_sample_rate_frames_save = UINT32_MAX;

void TFLunaSensor::dump_config() {
  LOG_SENSOR("", "TF-Luna", this->distance_sensor_);
  if (this->signal_strength_sensor_ != nullptr) {
    LOG_SENSOR("", "TF-Luna", this->signal_strength_sensor_);
  }
  if (this->temperature_sensor_ != nullptr) {
    LOG_SENSOR("", "TF-Luna", this->temperature_sensor_);
  }
  ESP_LOGCONFIG(TAG, "Serial Number: %s", sn_save);
  ESP_LOGCONFIG(TAG, "Firmware Version: %u.%u.%u", ver_save[0], ver_save[1], ver_save[2]);
  ESP_LOGCONFIG(TAG, "Enabled: %s", enabled_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "Output Frequency: %uhz", output_frequency_);
  ESP_LOGCONFIG(TAG, "Power Saving Frequency: %uhz", power_saving_frequency_);
  ESP_LOGCONFIG(TAG, "Amplitude Threshold: %u", amplitude_threshold_);
  ESP_LOGCONFIG(TAG, "Dummy Distance: %ucm", dummy_distance_);
  ESP_LOGCONFIG(TAG, "Minimum Distance: %ucm", minimum_distance_);
  ESP_LOGCONFIG(TAG, "Maximum Distance: %ucm", maximum_distance_);
  ESP_LOGCONFIG(TAG, "Silence: %s", silence_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "On/Off Mode: %u", on_off_mode_);
  ESP_LOGCONFIG(TAG, "On/Off Distance: %ucm", on_off_distance_);
  ESP_LOGCONFIG(TAG, "On/Off Zone: %ucm", on_off_zone_);
  ESP_LOGCONFIG(TAG, "On/Off Delay 1: %ums", on_off_delay1_);
  ESP_LOGCONFIG(TAG, "On/Off Delay 2: %ums", on_off_delay2_);
  ESP_LOGCONFIG(TAG, "Low Sample Rate Period: %us", low_sample_rate_period_);
  ESP_LOGCONFIG(TAG, "Low Sample Rate Frames: %u", low_sample_rate_frames_);
  LOG_PIN("Trigger Pin: ", this->trigger_pin_);
  if (setup_warn_cnt_ > 0)
    ESP_LOGD(TAG, "Setup Warning Count: %u", setup_warn_cnt_);
  if (run_warn_cnt_ > 0)
    ESP_LOGW(TAG, "Running Warning Count: %u", run_warn_cnt_);
  if (this->status_has_warning())
    ESP_LOGW(TAG, "Has Warning!");
  if (this->status_has_error())
    ESP_LOGE(TAG, "Has Error!");
  if (this->is_failed())
    ESP_LOGE(TAG, "Failed!");
  else if (this->is_ready())
    ESP_LOGI(TAG, "Ready");
  LOG_I2C_DEVICE(this);
}

void TFLunaSensor::setup() {
  this->trigger_pin_->setup();
  this->read_registers();
  isSetup = true;
  this->configure_sensor();
  setup_warn_cnt_ = run_warn_cnt_;
  run_warn_cnt_ = 0;
}

void TFLunaSensor::loop() {
  if (!enabled_ || !this->trigger_pin_->digital_read()) {
    return;
  }

  uint8_t data[TFLUNA_DATA_LENGTH];
  if (this->read_register(TFLUNA_DATA_REGISTER, data, TFLUNA_DATA_LENGTH, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna failed on read");
    this->status_set_warning();
    run_warn_cnt_++;
    return;
  }

  uint16_t distance = (data[0] | data[1] << 8);
  uint16_t strength = (data[2] | data[3] << 8);
  uint16_t temperature_raw = (data[4] | data[5] << 8);
  float temperature = (temperature_raw / 100.0f);
  //ESP_LOGD(TAG, "Got distance=%ucm strength=%u temperature=%.2f°C", distance, strength, temperature);

  if (this->distance_sensor_ != nullptr) {
    if (strength < amplitude_threshold_) {
      ESP_LOGW(TAG, "Distance measurement underexposure");
      this->distance_sensor_->publish_state(NAN);
    } else if (strength == TFLUNA_OVER_EXPOSED_VALUE) {
      ESP_LOGW(TAG, "Distance measurement overexposure");
      this->distance_sensor_->publish_state(NAN);
    } else {
      this->distance_sensor_->publish_state(distance / 100.0f);
    }
  }

  if (this->signal_strength_sensor_ != nullptr) {
    this->signal_strength_sensor_->publish_state(strength);
  }
  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }

  this->status_clear_warning();
}

void TFLunaSensor::on_shutdown() {
  send_reset();
}

void TFLunaSensor::trigger_read() {
  if (this->write(TFLUNA_TRIGGER_CMD, sizeof(TFLUNA_TRIGGER_CMD)) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna failed on trigger command");
    this->status_set_warning();
    run_warn_cnt_++;
  } else {
    ESP_LOGD(TAG, "Sent read trigger");
  }
}

void TFLunaSensor::send_reset() {
  if (this->write(TFLUNA_SOFT_RESET_CMD, sizeof(TFLUNA_SOFT_RESET_CMD)) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna failed on soft reset command");
    this->status_set_warning();
    run_warn_cnt_++;
  } else {
    ESP_LOGD(TAG, "Sent soft reset");
  }
}

void TFLunaSensor::read_registers() {
  const uint8_t TFLUNA_BUFFER_OFFSET = 0x25;
  uint8_t buff[TFLUNA_BUFFER_LENGTH];
  if (this->read_register(TFLUNA_VERSION_REGISTER, buff, TFLUNA_VERSION_LENGTH, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna for version failed on read");
    this->status_set_warning();
    run_warn_cnt_++;
  } else {
    ver_save[0] = buff[2];
    ver_save[1] = buff[1];
    ver_save[2] = buff[0];
  }
  if (this->read_register(TFLUNA_SN_REGISTER, buff, TFLUNA_SN_LENGTH, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna for serial failed on read");
    this->status_set_warning();
    run_warn_cnt_++;
  } else {
    for (uint8_t i = 0; i < TFLUNA_SN_LENGTH; i++)
      sn_save[i] = buff[i];
    sn_save[TFLUNA_SN_LENGTH] = 0x00;
  }
  if (this->read_register(TFLUNA_BUFFER_OFFSET, buff, TFLUNA_BUFFER_LENGTH - 2, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna for registers failed on read");
    this->status_set_warning();
    run_warn_cnt_++;
  } else {
    this->status_clear_warning();
    enabled_save = (buff[TFLUNA_ENABLED_REGISTER - TFLUNA_BUFFER_OFFSET] > 0);
    amplitude_threshold_save = buff[TFLUNA_AMPLITUDE_THRESHOLD_REGISTER - TFLUNA_BUFFER_OFFSET] | 
      buff[TFLUNA_AMPLITUDE_THRESHOLD_REGISTER - TFLUNA_BUFFER_OFFSET + 1] << 8;
    dummy_distance_save = buff[TFLUNA_DUMMY_DISTANCE_REGISTER - TFLUNA_BUFFER_OFFSET] | 
      buff[TFLUNA_DUMMY_DISTANCE_REGISTER - TFLUNA_BUFFER_OFFSET + 1] << 8;
    minimum_distance_save = buff[TFLUNA_MINIMUM_DISTANCE_REGISTER - TFLUNA_BUFFER_OFFSET] | 
      buff[TFLUNA_MINIMUM_DISTANCE_REGISTER - TFLUNA_BUFFER_OFFSET + 1] << 8;
    maximum_distance_save = buff[TFLUNA_MAXIMUM_DISTANCE_REGISTER - TFLUNA_BUFFER_OFFSET] | 
      buff[TFLUNA_MAXIMUM_DISTANCE_REGISTER - TFLUNA_BUFFER_OFFSET + 1] << 8;
    
    if (read_config(TFLUNA_OUTPUT_FREQUENCY_CMD[2], TFLUNA_OUTPUT_FREQUENCY_CMD[1], buff) > 4)
      output_frequency_save = buff[3] | buff[4] << 8;
    if (read_config(TFLUNA_POWER_SAVING_FREQUENCY_CMD[2], TFLUNA_POWER_SAVING_FREQUENCY_CMD[1], buff) > 4)
      power_saving_frequency_save = buff[3] | buff[4] << 8;
    if (read_config(TFLUNA_ON_OFF_MODE_CMD[2], TFLUNA_ON_OFF_MODE_CMD[1], buff) > 11) {
      on_off_mode_save = buff[3];
      on_off_distance_save = buff[4] | buff[5] << 8;
      on_off_zone_save = buff[6] | buff[7] << 8;
      on_off_delay1_save = buff[8] | buff[9] << 8;
      on_off_delay2_save = buff[10] | buff[11] << 8;
    }
    if (read_config(TFLUNA_LOW_SAMPLE_RATE_CMD[2], TFLUNA_LOW_SAMPLE_RATE_CMD[1], buff) > 10) {
      low_sample_rate_period_save = buff[3] | buff[4] <<  8 | buff[5] << 16 | buff[6] << 24;
      low_sample_rate_frames_save = buff[7] | buff[8] <<  8 | buff[9] << 16 | buff[10] << 24;
    }
  }
}

uint8_t TFLunaSensor::read_config(uint8_t id, uint8_t data_len, uint8_t (&buff)[TFLUNA_BUFFER_LENGTH]) {
  TFLUNA_READ_ID_CMD[3] = id;
  if (this->write(TFLUNA_READ_ID_CMD, sizeof(TFLUNA_READ_ID_CMD)) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with TF-Luna failed on read request command");
    this->status_set_warning();
    run_warn_cnt_++;
    return 0;
  }
  
  uint8_t len = data_len;
  if (len > TFLUNA_BUFFER_LENGTH)
    len = TFLUNA_BUFFER_LENGTH;
  ESP_LOGD(TAG, "Sent read request for 0x%02X", id);
  if (this->read(buff, len) == i2c::ERROR_OK && 
      buff[0] == TFLUNA_READ_ID_CMD[0] && buff[2] == id) {
    if (len > buff[1])
      len = buff[1];
    ESP_LOGD(TAG, "Data received: %u bytes", len);
    return len;
  }
  
  return 0;
}

void TFLunaSensor::configure_sensor() {
  if (!isSetup)
    return;
  if (enabled_ != enabled_save) {
    uint8_t data[] = { 0x00 };
    if (enabled_)
      data[0] = 0x01;
    if (this->write_register(TFLUNA_ENABLED_REGISTER, data, 1, 1) == i2c::ERROR_OK) {
      ESP_LOGD(TAG, "Set enabled: %s", enabled_ ? "true" : "false");
      this->status_clear_warning();
      enabled_save = enabled_;
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on enabled write");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
  this->configure_amplitude();
  this->configure_distance_limit();
  this->configure_output_frequency();
  this->configure_power_saving_frequency();
  this->configure_on_off_mode();
  this->configure_low_sample_rate();
}

void TFLunaSensor::configure_amplitude() {
  if (enabled_ && isSetup && (amplitude_threshold_save != amplitude_threshold_ || 
      dummy_distance_save != dummy_distance_)) {
    TFLUNA_AMPLITUDE_THRESHOLD_CMD[3] = amplitude_threshold_ / 10;
    TFLUNA_AMPLITUDE_THRESHOLD_CMD[4] = dummy_distance_ & 0xff;
    TFLUNA_AMPLITUDE_THRESHOLD_CMD[5] = (dummy_distance_ >> 8) & 0xff;
    if (this->write(TFLUNA_AMPLITUDE_THRESHOLD_CMD, sizeof(TFLUNA_AMPLITUDE_THRESHOLD_CMD)) == i2c::ERROR_OK) {
      amplitude_threshold_save = amplitude_threshold_;
      dummy_distance_save = dummy_distance_;
      ESP_LOGD(TAG, "Set amplitude threshold: %u", amplitude_threshold_);
      ESP_LOGD(TAG, "Set dummy distance: %u", dummy_distance_);
      this->status_clear_warning();
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on write for amplitude threshold");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
}

void TFLunaSensor::configure_distance_limit() {
  if (enabled_ && isSetup && (minimum_distance_save != minimum_distance_ || 
      maximum_distance_save != maximum_distance_ || silence_save != silence_)) {
    TFLUNA_DISTANCE_LIMIT_CMD[3] = minimum_distance_ & 0xff;
    TFLUNA_DISTANCE_LIMIT_CMD[4] = (minimum_distance_ >> 8) & 0xff;
    TFLUNA_DISTANCE_LIMIT_CMD[5] = maximum_distance_ & 0xff;
    TFLUNA_DISTANCE_LIMIT_CMD[6] = (maximum_distance_ >> 8) & 0xff;
    if (silence_) TFLUNA_DISTANCE_LIMIT_CMD[7] = 0x01;
    else TFLUNA_DISTANCE_LIMIT_CMD[7] = 0x00;
    if (this->write(TFLUNA_DISTANCE_LIMIT_CMD, sizeof(TFLUNA_DISTANCE_LIMIT_CMD)) == i2c::ERROR_OK) {
      minimum_distance_save = minimum_distance_;
      maximum_distance_save = maximum_distance_;
      silence_save != silence_;
      ESP_LOGD(TAG, "Set minimum distance: %u", minimum_distance_);
      ESP_LOGD(TAG, "Set maximum distance: %u", maximum_distance_);
      ESP_LOGD(TAG, "Set silence: %s", silence_ ? "true" : "false");
      this->status_clear_warning();
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on write for distance limit");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
}

void TFLunaSensor::configure_output_frequency() {
  if (enabled_ && isSetup && output_frequency_save != output_frequency_) {
    TFLUNA_OUTPUT_FREQUENCY_CMD[3] = output_frequency_ & 0xff;
    TFLUNA_OUTPUT_FREQUENCY_CMD[4] = (output_frequency_ >> 8) & 0xff;
    if (this->write(TFLUNA_OUTPUT_FREQUENCY_CMD, sizeof(TFLUNA_OUTPUT_FREQUENCY_CMD)) == i2c::ERROR_OK) {
      output_frequency_save = output_frequency_;
      ESP_LOGD(TAG, "Set output frequency: %u", output_frequency_);
      this->status_clear_warning();
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on write for output frequency");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
}

void TFLunaSensor::configure_power_saving_frequency() {
  if (enabled_ && isSetup && power_saving_frequency_save != power_saving_frequency_) {
    TFLUNA_POWER_SAVING_FREQUENCY_CMD[3] = power_saving_frequency_;
    if (this->write(TFLUNA_POWER_SAVING_FREQUENCY_CMD, sizeof(TFLUNA_POWER_SAVING_FREQUENCY_CMD)) == i2c::ERROR_OK) {
      power_saving_frequency_save = power_saving_frequency_;
      ESP_LOGD(TAG, "Set power saving frequency: %u", power_saving_frequency_);
      this->status_clear_warning();
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on write for power saving frequency");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
}

void TFLunaSensor::configure_on_off_mode() {
  if (enabled_ && isSetup && (on_off_mode_save != on_off_mode_ ||
      on_off_distance_save != on_off_distance_ || on_off_zone_save != on_off_zone_ || 
      on_off_delay1_save != on_off_delay1_ || on_off_delay2_save != on_off_delay2_)) {
    TFLUNA_ON_OFF_MODE_CMD[3] = on_off_mode_;
    TFLUNA_ON_OFF_MODE_CMD[4] = on_off_distance_ & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[5] = (on_off_distance_ >> 8) & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[6] = on_off_zone_ & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[7] = (on_off_zone_ >> 8) & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[8] = on_off_delay1_ & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[9] = (on_off_delay1_ >> 8) & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[10] = on_off_delay2_ & 0xff;
    TFLUNA_ON_OFF_MODE_CMD[11] = (on_off_delay2_ >> 8) & 0xff;
    if (this->write(TFLUNA_ON_OFF_MODE_CMD, sizeof(TFLUNA_ON_OFF_MODE_CMD)) == i2c::ERROR_OK) {
      on_off_mode_save = on_off_mode_;
      on_off_distance_save = on_off_distance_;
      on_off_zone_save = on_off_zone_;
      on_off_delay1_save = on_off_delay1_;
      on_off_delay2_save = on_off_delay2_;
      ESP_LOGD(TAG, "Set on/off mode: %u", on_off_mode_);
      ESP_LOGD(TAG, "Set on/off distance: %u", on_off_distance_);
      ESP_LOGD(TAG, "Set on/off zone: %u", on_off_zone_);
      ESP_LOGD(TAG, "Set on/off delay1: %u", on_off_delay1_);
      ESP_LOGD(TAG, "Set on/off delay2: %u", on_off_delay2_);
      this->status_clear_warning();
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on write for on/off mode");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
}

void TFLunaSensor::configure_low_sample_rate() {
  if (enabled_ && isSetup && (low_sample_rate_period_save != low_sample_rate_period_ || 
      low_sample_rate_frames_save != low_sample_rate_frames_)) {
    TFLUNA_LOW_SAMPLE_RATE_CMD[3]  = (low_sample_rate_period_ >>  0) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[4]  = (low_sample_rate_period_ >>  8) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[5]  = (low_sample_rate_period_ >> 16) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[6]  = (low_sample_rate_period_ >> 24) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[7]  = (low_sample_rate_frames_ >>  0) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[8]  = (low_sample_rate_frames_ >>  8) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[9]  = (low_sample_rate_frames_ >> 16) & 0xff;
    TFLUNA_LOW_SAMPLE_RATE_CMD[10] = (low_sample_rate_frames_ >> 24) & 0xff;
    if (this->write(TFLUNA_LOW_SAMPLE_RATE_CMD, sizeof(TFLUNA_LOW_SAMPLE_RATE_CMD)) == i2c::ERROR_OK) {
      low_sample_rate_period_save = low_sample_rate_period_;
      low_sample_rate_frames_save = low_sample_rate_frames_;
      ESP_LOGD(TAG, "Set low sample rate period: %u", low_sample_rate_period_);
      ESP_LOGD(TAG, "Set low sample rate frames: %u", low_sample_rate_frames_);
      this->status_clear_warning();
    } else {
      ESP_LOGE(TAG, "Communication with TF-Luna failed on write for low sample rate");
      this->status_set_warning();
      run_warn_cnt_++;
    }
  }
}

}  // namespace tfluna
}  // namespace esphome
