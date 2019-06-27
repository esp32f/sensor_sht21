#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include "macros.h"


// I2C slave address
#define SHT21_ADDR 0x40
// Register map
#define SHT21_CMD_TEMP_NO_HOLD        0xF3
#define SHT21_CMD_RH_NO_HOLD          0xF5
#define SHT21_CMD_WRITE_USER_REGISTER 0xE6
#define SHT21_CMD_READ_USER_REGISTER  0xE7


esp_err_t i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed) {
  i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = scl,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = clk_speed,
  };
  ERET( i2c_param_config(port, &config) );
  ERET( i2c_driver_install(port, config.mode, 0, 0, 0) );
  return ESP_OK;
}


esp_err_t sht21_register(i2c_port_t port, uint8_t *ans) {
  uint8_t addr = SHT21_ADDR;
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  ERET( i2c_master_start(h) );
  ERET( i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true) );
  ERET( i2c_master_write_byte(h, SHT21_CMD_READ_USER_REGISTER, true) );
  ERET( i2c_master_start(h) );
  ERET( i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_READ, true) );
  ERET( i2c_master_read_byte(h, ans, I2C_MASTER_LAST_NACK) );
  ERET( i2c_master_stop(h) );
  ERET( i2c_master_cmd_begin(port, h, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(h);
  return ESP_OK;
}


esp_err_t sht21_cmd_bytes(i2c_port_t port, uint8_t cmd, uint8_t *buff) {
  uint8_t addr = SHT21_ADDR;
  i2c_cmd_handle_t h = i2c_cmd_link_create();
  ERET( i2c_master_start(h) );
  ERET( i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true) );
  ERET( i2c_master_write_byte(h, cmd, true) );
  ERET( i2c_master_stop(h) );
  ERET( i2c_master_cmd_begin(port, h, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(h);
  vTaskDelay(50 / portTICK_RATE_MS);
  h = i2c_cmd_link_create();
  ERET( i2c_master_start(h) );
  ERET( i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_READ, false) );
  ERET( i2c_master_stop(h) );
  ERET( i2c_master_cmd_begin(port, h, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(h);
  vTaskDelay(50 / portTICK_RATE_MS);
  h = i2c_cmd_link_create();
  ERET( i2c_master_start(h) );
  ERET( i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_READ, true) );
  ERET( i2c_master_read(h, buff, 2, I2C_MASTER_ACK) );
  ERET( i2c_master_read_byte(h, buff+2, I2C_MASTER_LAST_NACK) );
  ERET( i2c_master_stop(h) );
  ERET( i2c_master_cmd_begin(port, h, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(h);
  return ESP_OK;
}


esp_err_t sht21_rh(i2c_port_t port, float *ans) {
  uint8_t buff[3];
  ERET( sht21_cmd_bytes(port, SHT21_CMD_RH_NO_HOLD, buff) );
  int16_t intv = (buff[0] << 8) | (buff[1] && 0xFC);
  *ans = -6 + 125 * (intv / 65536.0);
  return ESP_OK;
}


esp_err_t sht21_temp(i2c_port_t port, float *ans) {
  uint8_t buff[3];
  ERET( sht21_cmd_bytes(port, SHT21_CMD_TEMP_NO_HOLD, buff) );
  int16_t intv = (buff[0] << 8) | (buff[1] && 0xFC);
  *ans = -46.25 + 175.72 * (intv / 65536.0);
  return ESP_OK;
}


void app_main() {
  float rh, temp;
  uint8_t reg;
  i2c_port_t port = I2C_NUM_0;
  printf("- Initialize I2C master\n");
  ERETV( i2c_init(port, GPIO_NUM_18, GPIO_NUM_19, 100000) );
  ERETV( sht21_register(port, &reg) );
  printf("Register = %X\n", reg);
  ERETV( sht21_rh(port, &rh) );
  printf("RH = %f\n", rh);
  ERETV( sht21_temp(port, &temp) );
  printf("Temp = %f\n", temp);
}
