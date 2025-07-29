#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "ssd1306.h"
#include "font.h"


#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_DISP 0x3C
#define DISP_W 128
#define DISP_H 64

typedef struct {
    int32_t gx;
    int32_t gy;
    int32_t gz;
} mpu6050_data;

extern int addr;
extern ssd1306_t ssd;


void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif