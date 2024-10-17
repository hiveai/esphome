#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <cinttypes>

// Support for the Benewake TF-Luna LIDAR distance sensor

namespace esphome {
namespace tfluna {

constexpr static const char *const TAG = "tfluna";
static const uint8_t TFLUNA_BUFFER_LENGTH = 14;

class TFLunaSensor : public Component, public i2c::I2CDevice {
 public:
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_signal_strength_sensor(sensor::Sensor *signal_strength_sensor) { signal_strength_sensor_ = signal_strength_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_trigger_pin(GPIOPin *trigger_pin) { trigger_pin_ = trigger_pin; }
  
  bool get_enabled() { return enabled_; }
  void set_enabled(bool value) {
    enabled_ = value;
    this->configure_sensor();
  }
  
  unsigned int get_amplitude_threshold() { return (unsigned int) amplitude_threshold_; }
  unsigned int get_dummy_distance() { return (unsigned int) dummy_distance_; }
  void set_amplitude(unsigned int threshold, unsigned int dummy_distance) {
    if (threshold > UINT16_MAX) ESP_LOGW(TAG, "threshold out of range");
    else if (dummy_distance > UINT16_MAX) ESP_LOGW(TAG, "dummy_distance out of range");
    else {
      amplitude_threshold_ = (uint16_t) threshold;
      dummy_distance_ = (uint16_t) dummy_distance;
      this->configure_amplitude();
    }
  }
  
  unsigned int get_minimum_distance() { return (unsigned int) minimum_distance_; }
  unsigned int get_maximum_distance() { return (unsigned int) maximum_distance_; }
  bool get_silence() { return silence_; }
  void set_distance_limit(unsigned int minimum, unsigned int maximum, bool silence) {
    if (minimum > UINT16_MAX) ESP_LOGW(TAG, "minimum_distance out of range");
    else if (maximum > UINT16_MAX) ESP_LOGW(TAG, "maximum_distance out of range");
    else {
      minimum_distance_ = (uint16_t) minimum;
      maximum_distance_ = (uint16_t) maximum;
      silence_ = silence;
      this->configure_distance_limit();
    }
  }

  unsigned int get_output_frequency() { return (unsigned int) output_frequency_; }
  void set_output_frequency(unsigned int output_frequency) {
    if (output_frequency == 0 || (output_frequency <= 250 && 
        (500u / (500u / output_frequency)) == output_frequency)) {
      output_frequency_ = (uint16_t) output_frequency;
      if (output_frequency > 0 && power_saving_frequency_ != 0) {
        this->set_power_saving_frequency(0);
      }
      this->configure_output_frequency();
    } else {
      ESP_LOGW(TAG, "output_frequency out of range");
    }
  }
  
  unsigned int get_power_saving_frequency() { return (unsigned int) power_saving_frequency_; }
  void set_power_saving_frequency(unsigned int power_saving_frequency) {
    if (power_saving_frequency <= 10) {
      power_saving_frequency_ = (uint8_t) power_saving_frequency;
      if (power_saving_frequency > 0 && output_frequency_ != 0) {
        this->set_output_frequency(0);
      }
      this->configure_power_saving_frequency();
    } else {
      ESP_LOGW(TAG, "power_saving_frequency out of range");
    }
  }
  
  unsigned int get_on_off_mode() { return (unsigned int) on_off_mode_; }
  unsigned int get_on_off_distance() { return (unsigned int) on_off_distance_; }
  unsigned int get_on_off_zone() { return (unsigned int) on_off_zone_; }
  unsigned int get_on_off_delay1() { return (unsigned int) on_off_delay1_; }
  unsigned int get_on_off_delay2() { return (unsigned int) on_off_delay2_; }
  void set_on_off_mode(unsigned int mode, unsigned int distance, 
      unsigned int zone, unsigned int delay1, unsigned int delay2) {
    if (mode > 2) ESP_LOGW(TAG, "mode out of range");
    else if (distance > UINT16_MAX) ESP_LOGW(TAG, "distance out of range");
    else if (zone > UINT16_MAX) ESP_LOGW(TAG, "zone out of range");
    else if (delay1 > UINT16_MAX) ESP_LOGW(TAG, "delay1 out of range");
    else if (delay2 > UINT16_MAX) ESP_LOGW(TAG, "delay2 out of range");
    else {
      on_off_mode_ = (uint8_t) mode;
      on_off_distance_ = (uint16_t) distance;
      on_off_zone_ = (uint16_t) zone;
      on_off_delay1_ = (uint16_t) delay1;
      on_off_delay2_ = (uint16_t) delay2;
      this->configure_on_off_mode();
    }
  }
  
  unsigned int get_low_sample_rate_period() { return (unsigned int) low_sample_rate_period_; }
  unsigned int get_low_sample_rate_frames() { return (unsigned int) low_sample_rate_frames_; }
  void set_low_sample_rate(unsigned int period, unsigned int frames) {
    if (period > UINT32_MAX) ESP_LOGW(TAG, "period out of range");
    else if (frames > UINT32_MAX) ESP_LOGW(TAG, "frames out of range");
    else {
      low_sample_rate_period_ = (uint32_t) period;
      low_sample_rate_frames_ = (uint32_t) frames;
      this->configure_low_sample_rate();
    }
  }
  
  void set_parameter(std::string name, unsigned int value) {
    if (name == "enabled") {
      if (value != 0 && value != 1) ESP_LOGW(TAG, "enabled must be 0 or 1");
      else this->set_enabled(value == 1U ? true : false);
    }
    else if (name == "amplitude_threshold") this->set_amplitude(value, (unsigned int) dummy_distance_);
    else if (name == "dummy_distance") this->set_amplitude((unsigned int) amplitude_threshold_, value);
    else if (name == "minimum_distance") this->set_distance_limit(value, (unsigned int) maximum_distance_, silence_);
    else if (name == "maximum_distance") this->set_distance_limit((unsigned int) minimum_distance_, value, silence_);
    else if (name == "silence") {
      if (value != 0 && value != 1) ESP_LOGW(TAG, "silence must be 0 or 1");
      else this->set_distance_limit((unsigned int) minimum_distance_, (unsigned int) maximum_distance_, value == 1U ? true : false);
    }
    else if (name == "output_frequency") this->set_output_frequency(value);
    else if (name == "power_saving_frequency") this->set_power_saving_frequency(value);
    else if (name == "low_sample_rate_period") this->set_low_sample_rate(value, (unsigned int) low_sample_rate_frames_);
    else if (name == "low_sample_rate_frames") this->set_low_sample_rate((unsigned int) low_sample_rate_period_, value);
    else
      ESP_LOGW(TAG, "Allowed parameters are: enabled, amplitude_threshold, dummy_distance, "
      "minimum_distance, maximum_distance, silence, output_frequency, power_saving_frequency, "
      "low_sample_rate_period and low_sample_rate_frames");
}

  void dump_config() override;
  void setup() override;
  void loop() override;
  void on_shutdown() override;
  void trigger_read();
  void send_reset();
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *signal_strength_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  GPIOPin *trigger_pin_;
  bool enabled_;
  uint16_t amplitude_threshold_;
  uint16_t dummy_distance_;
  uint16_t minimum_distance_;
  uint16_t maximum_distance_;
  bool silence_;
  uint16_t output_frequency_;
  uint8_t power_saving_frequency_;
  uint8_t on_off_mode_;
  uint16_t on_off_distance_;
  uint16_t on_off_zone_;
  uint16_t on_off_delay1_;
  uint16_t on_off_delay2_;
  uint32_t low_sample_rate_period_;
  uint32_t low_sample_rate_frames_;
  uint16_t run_warn_cnt_;
  uint16_t setup_warn_cnt_;

  
  void read_registers();
  uint8_t read_config(uint8_t, uint8_t, uint8_t (&)[TFLUNA_BUFFER_LENGTH]);
  void configure_sensor();
  void configure_amplitude();
  void configure_distance_limit();
  void configure_output_frequency();
  void configure_power_saving_frequency();
  void configure_on_off_mode();
  void configure_low_sample_rate();
};

}  // namespace tfluna
}  // namespace esphome