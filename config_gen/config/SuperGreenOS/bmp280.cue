package config

modules bmp280: _I2C_MODULE & {
  init: false
  array_len: len(_i2c_conf)
}

modules bmp280 fields "\(k)_present": _INT8 & _HTTP & {
  default: 0
} for k, v in _i2c_conf

