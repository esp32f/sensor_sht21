#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include "macros.h"


// I2C slave address
#define SHT21_ADDR 0x40
// Register map
#define SHT21_CMD_TEMP_WAIT     0xE3
#define SHT21_CMD_TEMP_NO_WAIT  0xF3
#define SHT21_CMD_RH_WAIT       0xE5
#define SHT21_CMD_RH_NO_WAIT    0xF5


esp_err_t i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed) {
  i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = scl,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = clk_speed,
  };
  i2c_param_config(port, &config);
  return i2c_driver_install(port, config.mode, 0, 0, 0);
}


esp_err_t i2c_read(i2c_port_t port, uint8_t addr, uint8_t *buff, size_t size) {
  if (size == 0) return ESP_OK;
  // create a command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // send start bit
  i2c_master_start(cmd);
  // write slave address, set last bit for read, and check ACK from slave
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  // read n-1 bytes with ACK
  if (size > 1) i2c_master_read(cmd, buff, size - 1, I2C_MASTER_ACK);
  // read last byte with NACK
  i2c_master_read_byte(cmd, buff + size - 1, I2C_MASTER_NACK);
  // send stop bit
  i2c_master_stop(cmd);
  // execute created command
  esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  // delete command link
  i2c_cmd_link_delete(cmd);
  return ret;
}


esp_err_t i2c_write(i2c_port_t port, uint8_t addr, uint8_t *data, size_t size) {
  if (size == 0) return ESP_OK;
  // create a command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // send start bit
  i2c_master_start(cmd);
  // write slave address, set last bit for write, and check ACK from slave
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  // write all bytes, and check ACK from slave
  i2c_master_write(cmd, data, size, ACK_CHECK_EN);
  // send stop bit
  i2c_master_stop(cmd);
  // execute created command
  esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  // delete command link
  i2c_cmd_link_delete(cmd);
  return ret;
}


esp_err_t sht21_rh_int(i2c_port_t port, uint16_t *ans) {
  uint8_t addr = SHT21_ADDR;
  // write RH wait command
  uint8_t buff[3] = {SHT21_CMD_RH_WAIT};
  esp_err_t ret = i2c_write(port, addr, buff, 1);
  if (ret != ESP_OK) return ret;
  // read RH data + checksum byte
  ret = i2c_read(port, addr, buff, sizeof(buff));
  if (ret != ESP_OK) return ret;
  *ans = (((uint16_t) buff[0]) << 8) | buff[1];
  return ESP_OK;
}


esp_err_t sht21_temp_int(i2c_port_t port, uint16_t *ans) {
  uint8_t addr = SHT21_ADDR;
  // write temp wait command
  uint8_t buff[3] = {SHT21_CMD_TEMP_WAIT};
  esp_err_t ret = i2c_write(port, addr, buff, 1);
  // read temp data + checksum byte
  ret = i2c_read(port, addr, buff, 3);
  if (ret != ESP_OK) return ret;
  *ans = (((uint16_t) buff[0])<<8) | buff[1];
  return ESP_OK;
}


esp_err_t sht21_rh(i2c_port_t port, float *ans) {
    uint16_t intv;
    esp_err_t ret = sht21_rh_int(port, &intv);
    *ans = (0.0019073486328125 * (intv & 0xFFFC) ) - 6; 
    return ret;
}


esp_err_t sht21_temp(i2c_port_t port, float *ans) {
  uint16_t intv;
  esp_err_t ret = sht21_temp_int(port, &intv);
  *ans = (0.0026812744140625* (intv & 0xFFFC)) - 46.85;
  return ret;
}


void app_main() {
}
