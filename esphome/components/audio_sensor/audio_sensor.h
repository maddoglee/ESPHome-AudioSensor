#pragma once

#include "esphome.h"
#include "arduinoFFT.h"

#include "esphome/core/component.h"
#include "esphome/components/i2s_audio/i2s_audio.h"

#define SAMPLES 1024
#define OCTAVES 9

const float aweighting[] = {-39.4, -26.2, -16.1, -8.6, -3.2, 0.0, 1.2, 1.0, -1.1};

namespace esphome {
namespace audio_sensor {

class AudioSensorComponent : public Component, public CustomAPIDevice {
 public:
  AudioSensorComponent(i2s::I2SInput *i2s_audio_input) : i2s_audio_input_(i2s_audio_input) {}

  void setup() override {
    // This will be called by App.setup()
    ESP_LOGI("AudioSensor", "Audio Sensor setup");
    this->register_service(&AudioSensorComponent::detect_washing_machine, "detect_washing_machine");
    this->register_service(&AudioSensorComponent::detect_fire_alarm, "detect_fire_alarm");
    this->register_service(&AudioSensorComponent::detect_dishwasher, "detect_dishwasher");
  }

}  // namespace audio_sensor
}  // namespace esphome

  void loop() override {
    // This will be called by App.loop()
    static int32_t samples[SAMPLES];
    size_t num_bytes_read = 0;
    int samples_read = this->i2s_audio_input_->read(samples, SAMPLES * sizeof(int32_t), &num_bytes_read);

    if (samples_read <= 0) return;

    for (size_t i = 0; i < SAMPLES; i++) {
      real[i] = (samples[i] >> 16) / 10.0;
      imag[i] = 0.0;
    }

    fft.Windowing(FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
    fft.Compute(FFT_FORWARD);
    calculateEnergy(real, imag, SAMPLES);
    sumEnergy(real, energy, 1, OCTAVES);
    float loudness = calculateLoudness(energy, aweighting, OCTAVES, 1.0);

    unsigned int peak = (int)floor(fft.MajorPeak());

    if (detectFrequency(&washingMachine, 6, peak, 116, 233, true)) {
      ESP_LOGI("AudioSensor", "Detected Washing Machine");
      this->publish_state_("washing_machine", true);
    }

    if (detectFrequency(&fireAlarm, 5, peak, 153, 164, true)) {
      ESP_LOGI("AudioSensor", "Detected Fire Alarm");
      this->publish_state_("fire_alarm", true);
    }

    if (detectFrequency(&dishWasher, 6, peak, 229, 91, true)) {
      ESP_LOGI("AudioSensor", "Detected Dishwasher");
      this->publish_state_("dishwasher", true);
    }
  }

  void detect_washing_machine() {
    ESP_LOGI("AudioSensor", "Service call: detect_washing_machine");
    // Logic to handle washing machine detection
  }

  void detect_fire_alarm() {
    ESP_LOGI("AudioSensor", "Service call: detect_fire_alarm");
    // Logic to handle fire alarm detection
  }

  void detect_dishwasher() {
    ESP_LOGI("AudioSensor", "Service call: detect_dishwasher");
    // Logic to handle dishwasher detection
  }

 protected:
  i2s::I2SInput *i2s_audio_input_;
  float real[SAMPLES];
  float imag[SAMPLES];
  arduinoFFT fft = arduinoFFT(real, imag, SAMPLES, SAMPLES);
  float energy[OCTAVES];
  unsigned int washingMachine = 0;
  unsigned int dishWasher = 0;
  unsigned int fireAlarm = 0;

  void publish_state_(const char *state, bool value) {
    this->publish_state(state, value ? "ON" : "OFF");
  }
  
  void calculateEnergy(float *vReal, float *vImag, uint16_t samples) {
    for (uint16_t i = 0; i < samples; i++) {
      vReal[i] = sq(vReal[i]) + sq(vImag[i]);
      vImag[i] = 0.0;
    }
  }

  void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves) {
    int bin = bin_size;
    for (int octave = 0; octave < num_octaves; octave++) {
      float sum = 0.0;
      for (int i = 0; i < bin_size; i++) {
        sum += bins[bin++];
      }
      energies[octave] = sum;
      bin_size *= 2;
    }
  }

  float decibel(float v) {
    return 10.0 * log(v) / log(10);
  }

  float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale) {
    float sum = 0.0;
    for (int i = 0; i < num_octaves; i++) {
      float energy = scale * energies[i];
      sum += energy * pow(10, weights[i] / 10.0);
      energies[i] = decibel(energy);
    }
    return decibel(sum);
  }

  bool detectFrequency(unsigned int *mem, unsigned int minMatch, double peak, unsigned int bin1, unsigned int bin2, bool wide) {
    *mem = *mem << 1;
    if (peak == bin1 || peak == bin2 || (wide && (peak == bin1 + 1 || peak == bin1 - 1 || peak == bin2 + 1 || peak == bin2 - 1))) {
      *mem |= 1;
    }
    
    if (countSetBits(*mem) >= minMatch) {
      return true;
    }
    return false;
  }

  unsigned int countSetBits(unsigned int n) {
    unsigned int count = 0;
    while (n) {
      count += n & 1;
      n >>= 1;
    }
    return count;
  }
};
