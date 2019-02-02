#include <nest/sensors.hpp>

void Sensors::update() {
  if (millis() - _last_read < SENSOR_UPDATE_RATE_MS)
    return;
  _sht_0_valid = sht_read(SHT_0_ADDR, &_sht_0_temp, &_sht_0_rh);
  _sht_1_valid = sht_read(SHT_1_ADDR, &_sht_1_temp, &_sht_1_rh);
  uint16_t brightness = adc_read();
  _brightness_perc = (((float)brightness) * 100) / 4096;
  _last_read = millis();

  post_wifi();
}

bool Sensors::has_temp() { return _sht_1_valid || _sht_0_valid; }

bool Sensors::has_rh() { return _sht_1_valid || _sht_0_valid; }

double Sensors::get_temp() {
  if (_sht_1_valid && _sht_0_valid)
    return (_sht_1_temp + _sht_0_temp) / 2;
  if (_sht_1_valid)
    return _sht_1_temp;
  if (_sht_0_valid)
    return _sht_0_temp;
  return -1;
}

double Sensors::get_rh() {
  if (_sht_1_valid && _sht_0_valid)
    return (_sht_1_rh + _sht_0_rh) / 2;
  if (_sht_1_valid)
    return _sht_1_rh;
  if (_sht_0_valid)
    return _sht_0_rh;
  return -1;
}

float Sensors::get_brightness() { return _brightness_perc; }

void Sensors::post_wifi() {
  if (has_temp())
    wifi_fsm.send_temperature(get_temp());
  if (has_rh())
    wifi_fsm.send_rh(get_rh());
  wifi_fsm.send_brightness(get_brightness());
  wifi_fsm.send_uptime(millis());
}
