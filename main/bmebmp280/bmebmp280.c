/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Inspired by https://github.com/epvuc/esp-fume-detector/blob/master/main/bme280_sup.c
 *
 */

#include <stdlib.h>
#include "bmebmp280.h"
#include "driver/i2c.h"

#include "../core/kv/kv.h"
#include "../core/log/log.h"
#include "../core/i2c/i2c.h"
#include "../box/box.h"
#include "../core/time/utils.h"

#include "bme280_driver.h"

#define SUCCESS 0
#define FAIL -1
#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void user_delay_ms(uint32_t ms);

struct bme280_dev bme280;

void init_bmebmp280(int i2cId) {
  ESP_LOGI(SGO_LOG_EVENT, "@BMEBMP280 Initializing bmp280 i2c device %d", i2cId);
}

/*
 *
 */
int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
  int32_t iError = 0;

  esp_err_t espRc;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write(cmd, reg_data, cnt, true);
  i2c_master_stop(cmd);

  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
  ESP_LOGD(SGO_LOG_EVENT, "@BMEBMP280 in user_i2c_write, i2c_master_cmd_begin returns %d", espRc);

  if (espRc == ESP_OK) {
    iError = SUCCESS;
  } else {
    iError = FAIL;
  }
  i2c_cmd_link_delete(cmd);

  return (int8_t)iError;
}

/*
 *
 */
int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
  int32_t iError = 0;
  esp_err_t espRc;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

  if (cnt > 1) {
    i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
  ESP_LOGD(SGO_LOG_EVENT, "@BMEBMP280 in user_i2c_read, i2c_master_cmd_begin returns %d", espRc);

  if (espRc == ESP_OK) {
    iError = SUCCESS;
  } else {
    iError = FAIL;
  }

  i2c_cmd_link_delete(cmd);

  return (int8_t)iError;
}

void user_delay_ms(uint32_t ms) {
  vTaskDelay(ms/portTICK_PERIOD_MS);
}

void loop_bmebmp280(int i2cId) {
  ESP_LOGD(SGO_LOG_EVENT, "@BMEBMP280 in loop_bmebmp280, i2c device %d", i2cId);

  start_i2c(i2cId);

  {
    int8_t result_bme280_init;
    uint8_t settings_sel;

    bme280.dev_id = BME280_I2C_ADDR_PRIM;
    bme280.chip_id = BME280_CHIP_ID;
    bme280.intf = BME280_I2C_INTF;
    bme280.read = (void *)user_i2c_read;
    bme280.write = (void *)user_i2c_write;
    bme280.delay_ms = (void *)user_delay_ms;

    ESP_LOGD(SGO_LOG_EVENT, "@BMEBMP280 in loop_bmebmp280, calling bme280_init");
    result_bme280_init = bme280_init(&bme280);

    if (result_bme280_init == BME280_OK) {
      ESP_LOGI(SGO_LOG_EVENT, "@BMEBMP280 Successfully initialized BME280!");
    } else if (result_bme280_init > 0) {
      ESP_LOGW(SGO_LOG_EVENT, "@BMEBMP280 initialization warning: %d", result_bme280_init);
    } else {
      switch (result_bme280_init) {
        case BME280_E_NULL_PTR:
          ESP_LOGE(SGO_LOG_EVENT, "@BMEBMP280 initialization error: NULL_PTR (%d)", result_bme280_init);
          break;
        case BME280_E_DEV_NOT_FOUND:
          ESP_LOGE(SGO_LOG_EVENT, "@BMEBMP280 initialization error: BME280_E_DEV_NOT_FOUND (%d)", result_bme280_init);
          break;
        case BME280_E_INVALID_LEN:
          ESP_LOGE(SGO_LOG_EVENT, "@BMEBMP280 initialization error: BME280_E_INVALID_LEN (%d)", result_bme280_init);
          break;
        case BME280_E_COMM_FAIL:
          ESP_LOGE(SGO_LOG_EVENT, "@BMEBMP280 initialization error: BME280_E_COMM_FAIL (%d)", result_bme280_init);
          break;
        case BME280_E_SLEEP_MODE_FAIL:
          ESP_LOGE(SGO_LOG_EVENT, "@BMEBMP280 initialization error: BME280_E_SLEEP_MODE_FAIL (%d)", result_bme280_init);
          break;
        case BME280_E_NVM_COPY_FAILED:
          ESP_LOGE(SGO_LOG_EVENT, "@BMEBMP280 initialization error: NVM_COPY_FAILED (%d)", result_bme280_init);
          break;
      }
      return;
    }

    bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
    bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
    bme280.settings.filter = BME280_FILTER_COEFF_16;
    bme280.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    int8_t rslt_set_sensor_settings;
    rslt_set_sensor_settings = bme280_set_sensor_settings(settings_sel, &bme280);
    ESP_LOGD(SGO_LOG_EVENT, "@BMEBMP280 bme280 settings config result %d", rslt_set_sensor_settings);

    if (rslt_set_sensor_settings != BME280_OK) {
      return;
    }

    int8_t rslt_bme280_set_sensor_mode;
    char msgbuf[128];
    struct bme280_data comp_data;

    rslt_bme280_set_sensor_mode = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);
    if (rslt_bme280_set_sensor_mode != BME280_OK) {
      return;
    }

    bme280.delay_ms(15000);

    int8_t rslt_bme280_get_sensor_data;
    rslt_bme280_get_sensor_data = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280);
    if (rslt_bme280_get_sensor_data != BME280_OK) {
      ESP_LOGD(SGO_LOG_EVENT, "@BMEBMP280 bme280_get_sensor_data() returns %d", rslt_bme280_get_sensor_data);
      return;
    }

    snprintf(msgbuf, sizeof(msgbuf), "Temperature:%.02f C Pressure:%0.2f hPa Humidity:%0.2f %%",
        comp_data.temperature,
        comp_data.pressure/100,
        comp_data.humidity);

    set_bmebmp280_present(i2cId, 1);
    set_bmebmp280_temp(i2cId, comp_data.temperature);
    set_bmebmp280_humi(i2cId, comp_data.humidity);
    set_bmebmp280_pressure(i2cId, comp_data.pressure/100);

    ESP_LOGI(SGO_LOG_EVENT, "@BMEBMP280 %s", msgbuf);
  }

  vTaskDelay(500 / portTICK_RATE_MS);

  stop_i2c(i2cId);
}