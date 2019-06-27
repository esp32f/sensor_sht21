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


esp_err_t i2c_read(i2c_port_t port, uint8_t addr, uint8_t *buff, size_t size) {
  if (size == 0) return ESP_OK;
  // create a command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // send start bit
  ERET( i2c_master_start(cmd) );
  // write slave address, set last bit for read, and check ACK from slave
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN) );
  // read n-1 bytes with ACK
  if (size > 1) ERET( i2c_master_read(cmd, buff, size - 1, I2C_MASTER_ACK) );
  // read last byte with NACK
  ERET( i2c_master_read_byte(cmd, buff + size - 1, I2C_MASTER_NACK) );
  // send stop bit
  ERET( i2c_master_stop(cmd) );
  // execute created command
  ERET( i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) );
  // delete command link
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}


esp_err_t i2c_write(i2c_port_t port, uint8_t addr, uint8_t *data, size_t size) {
  if (size == 0) return ESP_OK;
  // create a command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // send start bit
  ERET( i2c_master_start(cmd) );
  // write slave address, set last bit for write, and check ACK from slave
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN) );
  // write all bytes, and check ACK from slave
  ERET( i2c_master_write(cmd, data, size, ACK_CHECK_EN) );
  // send stop bit
  ERET( i2c_master_stop(cmd) );
  // execute created command
  ERET( i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) );
  // delete command link
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}


esp_err_t sht21_rh_int(i2c_port_t port, uint16_t *ans) {
  uint8_t addr = SHT21_ADDR;
  printf("- Write RH no wait command\n");
  static uint8_t buff[3] = {SHT21_CMD_RH_NO_WAIT};
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, SHT21_CMD_RH_NO_WAIT, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(100 / portTICK_RATE_MS);
  printf("- Read RH data\n");
  i2c_cmd_link_create(cmd);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, buff+0, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, buff+1, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, buff+2, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  printf("- Save to float\n");
  // ERET( i2c_write(port, addr, buff, 1) );
  // printf("- Read RH data + checksum byte\n");
  // ERET( i2c_read(port, addr, buff, sizeof(buff)) );
  *ans = (((uint16_t) buff[0]) << 8) | buff[1];
  return ESP_OK;
}


esp_err_t sht21_temp_int(i2c_port_t port, uint16_t *ans) {
  uint8_t addr = SHT21_ADDR;
  printf("- Write Temp wait command\n");
  uint8_t buff[3] = {SHT21_CMD_TEMP_WAIT};
  ERET( i2c_write(port, addr, buff, 1) );
  printf("- Read temp data + checksum byte\n");
  ERET( i2c_read(port, addr, buff, 3) );
  *ans = (((uint16_t) buff[0])<<8) | buff[1];
  return ESP_OK;
}


esp_err_t sht21_temp(i2c_port_t port, float *ans) {
  uint16_t intv;
  printf("- Read Temp int from SHT21\n");
  ERET( sht21_temp_int(port, &intv) );
  *ans = (0.0026812744140625* (intv & 0xFFFC)) - 46.85;
  return ESP_OK;
}


esp_err_t sht21_rh_nohold(i2c_port_t port, uint8_t* ans) {
  uint8_t addr = SHT21_ADDR;
  printf("- Send RH no hold command\n");
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true) );
  ERET( i2c_master_write_byte(cmd, 0xF5, true) );
  ERET( i2c_master_stop(cmd) );
  ERET( i2c_master_cmd_begin(port, cmd, 100 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(cmd);
  printf("- Wait for 100ms\n");
  vTaskDelay(100 / portTICK_RATE_MS);
  printf("- Read RH 3 bytes\n");
  cmd = i2c_cmd_link_create();
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true) );
  ERET( i2c_master_read(cmd, ans, 2, I2C_MASTER_ACK) );
  ERET( i2c_master_read_byte(cmd, ans+2, I2C_MASTER_LAST_NACK) );
  ERET( i2c_master_stop(cmd) );
  ERET( i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(cmd);
  return ESP_OK;
}


esp_err_t sht21_rh_hold(i2c_port_t port, uint8_t* ans) {
  uint8_t addr = SHT21_ADDR;
  printf("- Send RH hold command\n");
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true) );
  ERET( i2c_master_write_byte(cmd, 0xE5, true) );
  ERET( i2c_master_start(cmd) );
  ERET( i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true) );
  ERET( i2c_master_read(cmd, ans, 2, I2C_MASTER_ACK) );
  ERET( i2c_master_read_byte(cmd, ans+2, I2C_MASTER_LAST_NACK) );
  ERET( i2c_master_stop(cmd) );
  ERET( i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS) );
  i2c_cmd_link_delete(cmd);
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


esp_err_t sht21_rh(i2c_port_t port, float *ans) {
    uint16_t intv[2];
    printf("- Read RH int from SHT21\n");
    ERET( sht21_rh_nohold(port, (uint8_t*)&intv) );
    *ans = (0.0019073486328125 * (intv[0] & 0xFFFC) ) - 6; 
    return ESP_OK;
}


void app_main() {
  // uint8_t reg;
  uint8_t tbuff[4] = {0, 0, 0, 0};
  float rh, temp;
  i2c_port_t port = I2C_NUM_0;
  printf("- Initialize I2C master\n");
  ERETV( i2c_init(port, GPIO_NUM_18, GPIO_NUM_19, 100000) );
  // ERETV( sht21_read_register(port, &reg) );
  // printf("User register = %x\n", reg);
  ERETV( sht21_rh(port, &rh) );
  printf("data read complete\n");
  printf("RH = %f\n", rh);
  return;
  printf("- Read RH from SHT21\n");
  ERETV( sht21_rh(port, &rh) );
  printf("- Read temp from SHT21\n");
  ERETV( sht21_temp(port, &temp) );
  printf("RH = %f, Temp = %f\n", rh, temp);
}
