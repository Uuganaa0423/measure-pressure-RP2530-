#include "XGZP6847D.h"
#include "hardware/gpio.h"
/**
 * Initialize I2C interface for pressure sensor
 */
 void init_i2c_pressure_sensor(void)
 {
     i2c_init(I2C_PORT, I2C_FREQ);
     gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
     gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
     gpio_pull_up(I2C_SDA_PIN);
     gpio_pull_up(I2C_SCL_PIN);
 }
 
 /**
  * Write raw command byte to measurement-command register (0x30)
  */
 bool write_measurement_command(uint8_t command)
 {
     uint8_t buffer[2] = {MEASUREMENT_COMMAND_REG, command};
     int written = i2c_write_blocking(I2C_PORT, PRESSURE_SENSOR_ADDR, buffer, sizeof(buffer), false);
     return written == sizeof(buffer);
 }
 
 /**
  * Read a single sensor register
  */
 bool read_sensor_register(uint8_t reg_addr, uint8_t *value)
 {
     if (!value) {
         return false;
     }
 
     int ret = i2c_write_blocking(I2C_PORT, PRESSURE_SENSOR_ADDR, &reg_addr, 1, true);
     if (ret != 1) {
         return false;
     }
 
     ret = i2c_read_blocking(I2C_PORT, PRESSURE_SENSOR_ADDR, value, 1, false);
     return ret == 1;
 }
 
 /**
  * Start a pressure/temperature conversion and wait for SCO to clear
  */
 bool trigger_measurement(uint32_t timeout_ms)
 {
     if (!write_measurement_command(DEFAULT_MEAS_COMMAND)) {
         printf("Failed to write measurement command (0x%02X)\n", DEFAULT_MEAS_COMMAND);
         return false;
     }
 
     absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
     uint8_t status = 0;
 
     do {
         if (!read_sensor_register(MEASUREMENT_COMMAND_REG, &status)) {
             printf("Failed to read measurement command register\n");
             return false;
         }
 
         if ((status & MEAS_SCO_BIT) == 0) {
             // Conversion finished
             return true;
         }
 
         sleep_ms(1);
     } while (!time_reached(deadline));
 
     printf("Measurement timeout: SCO bit stayed high. Last status=0x%02X\n", status);
     return false;
 }
 
 /**
  * Read pressure from XGZP6847D sensor
  * @return 24-bit pressure value (raw sensor reading)
  */
 int32_t read_pressure_raw(void)
 {
     uint8_t msb = 0, csb = 0, lsb = 0;
 
     if (!read_sensor_register(PRESSURE_MSB_REG, &msb) ||
         !read_sensor_register(PRESSURE_CSB_REG, &csb) ||
         !read_sensor_register(PRESSURE_LSB_REG, &lsb)) {
         printf("Failed to read pressure registers\n");
         return 0;
     }
 
     int32_t pressure = ((int32_t)msb << 16) | ((int32_t)csb << 8) | (int32_t)lsb;
 
     // Handle sign extension for 24-bit signed value
     if (pressure & 0x00800000) {
         pressure |= 0xFF000000;
     }
 
     return pressure;
 }
 
 /**
  * Dump pressure/temperature registers for debugging
  */
 void dump_sensor_data_registers(void)
 {
     uint8_t value = 0;
 
     printf("--- Sensor Register Snapshot ---\n");
     for (uint8_t addr = PRESSURE_MSB_REG; addr <= PRESSURE_LSB_REG; addr++) {
         if (read_sensor_register(addr, &value)) {
             printf("Reg 0x%02X = 0x%02X\n", addr, value);
         } else {
             printf("Reg 0x%02X read failed\n", addr);
         }
     }
 
     if (read_sensor_register(MEASUREMENT_COMMAND_REG, &value)) {
         printf("Reg 0x%02X (CMD) = 0x%02X\n", MEASUREMENT_COMMAND_REG, value);
     } else {
         printf("Reg 0x%02X (CMD) read failed\n", MEASUREMENT_COMMAND_REG);
     }
     printf("--------------------------------\n");
 }
 
 /**
  * Convert raw pressure reading to Pascal (Pa)
  * Using k-factor = 16 for conversion
  * @param raw_value Raw 24-bit pressure value from sensor
  * @return Pressure in Pascal
  */
 float convert_to_pascal(int32_t raw_value)
 {
     // XGZP6847D conversion: Pressure (Pa) = raw_value / k
     // k-factor = 16
     const float k_factor = 16.0f;
     return (float)raw_value / k_factor;
 }
 
 
 /**
  * Monitor pressure readings via USB UART
  */
 int32_t get_pressure(void)
 {
     if (!trigger_measurement(50)) {
         printf("Measurement trigger failed\n");
         return 0;
     }
 
     dump_sensor_data_registers();
 
     int32_t raw_pressure = read_pressure_raw();
     float pressure_pa = convert_to_pascal(raw_pressure);
 
     printf("Pressure - Raw: %ld (0x%06lX), Pa: %.2f\n",
             raw_pressure, raw_pressure & 0xFFFFFF, pressure_pa);
    return pressure_pa;        
 }