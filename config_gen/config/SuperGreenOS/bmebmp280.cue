package config

modules bmebmp280: _I2C_MODULE & {
  array_len: len(_i2c_conf)
}

modules bmebmp280 fields "\(k)_present": _INT8 & _HTTP & {
  default: 0
} for k, v in _i2c_conf

modules bmebmp280 fields "\(k)_temp": _INT8 & _HTTP & {
  default: 0
  temp_sensor: 0x4+k
  helper: "BMP280 temperature sensor on sensor port #\(k)"
} for k, v in _i2c_conf

modules bmebmp280 fields "\(k)_humi": _INT8 & _HTTP & {
  default: 0
  humi_sensor: 0x4+k
  helper: "BMP280 humidity sensor on sensor port #\(k)"
} for k, v in _i2c_conf

modules bmebmp280 fields "\(k)_pressure": _INT32 & _HTTP & {
  default: 0
  pres_sensor: 0x4+k
  helper: "BMP280 pressure sensor on sensor port #\(k)"
} for k, v in _i2c_conf