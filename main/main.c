#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include "macros.h"


// I2C slave address
#define SHT21_ADDR 0x40
// Register map
#define SHT21_CMD_TEMP_HOLD     0xE3
#define SHT21_CMD_TEMP_NO_HOLD  0xF3
#define SHT21_CMD_RH_HOLD       0xE5
#define SHT21_CMD_RH_NO_HOLD    0xF5


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


esp_err_t sht21_read_register(i2c_port_t port, uint8_t *ans) {
  uint8_t addr = SHT21_ADDR;
  printf("- Write Read user register command\n");
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true) );
  ERET( i2c_master_write_byte(cmd, 0xE7, true) );
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true) );
  ERET( i2c_master_read_byte(cmd, ans, I2C_MASTER_LAST_NACK) );
  ERET( i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}


esp_err_t sht21_rh_bytes(i2c_port_t port, uint8_t *buff) {
  uint8_t addr = SHT21_ADDR;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true) );
  ERET( i2c_master_write_byte(cmd, SHT21_CMD_RH_NO_HOLD, true) );
  ERET( i2c_master_stop(cmd) );
  ERET( i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(cmd);
  vTaskDelay(100 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true) );
  ERET( i2c_master_read(cmd, buff, 2, I2C_MASTER_ACK) );
  ERET( i2c_master_read_byte(cmd, buff+2, I2C_MASTER_LAST_NACK) );
  ERET( i2c_master_stop(cmd) );
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}


esp_err_t sht21_rh(i2c_port_t port, float *ans) {
  printf("- Read RH bytes from SHT21\n");
  uint8_t buff[3];
  ERET( sht21_rh_bytes(port, buff) );
  int16_t intv = (buff[0] << 8) | (buff[1] && 0xFC);
  printf("buff = %x %x %x\n", buff[0], buff[1], buff[2]);
  *ans = -6 + 125*(intv / 65536.0);
  // *ans = (0.0026812744140625* (intv & 0xFFFC)) - 46.85;
  return ESP_OK;
}


void app_main() {
  float rh;
  i2c_port_t port = I2C_NUM_0;
  printf("- Initialize I2C master\n");
  ERETV( i2c_init(port, GPIO_NUM_18, GPIO_NUM_19, 100000) );
  for(int i=0; i<10; i++) {
    ERETV( sht21_rh(port, &rh) );
  }
  printf("RH = %f\n", rh);
}
