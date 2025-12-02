#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// XGZP6847D Pressure Sensor Configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define I2C_FREQ 100000  // 100 kHz
#define PRESSURE_SENSOR_ADDR 0x6D

// Sensor register map (from datasheet)
#define PRESSURE_MSB_REG 0x06  // Pressure Data out <23:16>
#define PRESSURE_CSB_REG 0x07  // Pressure Data out <15:8>
#define PRESSURE_LSB_REG 0x08  // Pressure Data out <7:0>
#define MEASUREMENT_COMMAND_REG 0x30

// Measurement-command bit-field helpers (see datasheet)
#define MEAS_SLEEP_TIME_SHIFT 4
#define MEAS_SLEEP_TIME_MASK  0xF0
#define MEAS_GAIN_SHIFT       3
#define MEAS_GAIN_MASK        0x38         // Gain_P<5:3>
#define MEAS_SCO_BIT          (1u << 3)    // Start of conversion flag
#define MEAS_CTRL_MASK        0x07         // Measurement_control<2:0>

// Sleep-time presets (active in sleep-mode conversion)
#define MEAS_SLEEP_0_MS       0x0
#define MEAS_SLEEP_62MS       0x1
#define MEAS_SLEEP_125MS      0x2
#define MEAS_SLEEP_250MS      0x3
#define MEAS_SLEEP_500MS      0x4
#define MEAS_SLEEP_1S         0xF

// Gain presets (table depends on datasheet, default to unity)
#define MEAS_GAIN_X1          0x0
#define MEAS_GAIN_X2          0x1
#define MEAS_GAIN_X4          0x2
#define MEAS_GAIN_X8          0x3

// Measurement-control field (per datasheet description)
#define MEAS_CTRL_PRESSURE_ONLY            0x001
#define MEAS_CTRL_COMBINED_SINGLE_SHOT     0x002  // Temperature + pressure, one shot
#define MEAS_CTRL_COMBINED_SLEEP_MODE      0x003  // Periodic combined conversion (sleep mode)
#define MEAS_CTRL_RESERVED                 0x007  // Example placeholder

// Helper macros to build command fields
#define MEAS_SLEEP_FIELD(code)   (((code) & 0x0F) << MEAS_SLEEP_TIME_SHIFT)
#define MEAS_GAIN_FIELD(code)    (((code) & 0x07) << MEAS_GAIN_SHIFT)
#define MEAS_CTRL_FIELD(code)    ((code) & MEAS_CTRL_MASK)

// Default command: immediate combined temperature + pressure conversion
#define DEFAULT_SLEEP_CODE      MEAS_SLEEP_0_MS          // Not used for single-shot
#define DEFAULT_GAIN_CODE       MEAS_GAIN_X1
#define DEFAULT_MEAS_CONTROL    MEAS_CTRL_COMBINED_SINGLE_SHOT
#define DEFAULT_MEAS_COMMAND    (MEAS_SLEEP_FIELD(DEFAULT_SLEEP_CODE) | \
                                 MEAS_GAIN_FIELD(DEFAULT_GAIN_CODE)   | \
                                 MEAS_CTRL_FIELD(DEFAULT_MEAS_CONTROL) | \
                                 MEAS_SCO_BIT)

void init_i2c_pressure_sensor(void);
bool write_measurement_command(uint8_t command);
bool read_sensor_register(uint8_t reg_addr, uint8_t *value);
bool trigger_measurement(uint32_t timeout_ms);
int32_t read_pressure_raw(void);
void dump_sensor_data_registers(void);
float convert_to_pascal(int32_t raw_value);
int32_t get_pressure(void);