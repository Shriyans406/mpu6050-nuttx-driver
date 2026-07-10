/****************************************************************************
 * include/nuttx/sensors/mpu6050.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MPU6050_H
#define __INCLUDE_NUTTX_SENSORS_MPU6050_H

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/* MPU6050 I2C addresses */
#define MPU6050_ADDR_LOW   0x68
#define MPU6050_ADDR_HIGH  0x69

/* Register addresses */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75

/* Full Scale Range Options */
#define MPU6050_ACCEL_FS_2G     0
#define MPU6050_ACCEL_FS_4G     1
#define MPU6050_ACCEL_FS_8G     2
#define MPU6050_ACCEL_FS_16G    3

#define MPU6050_GYRO_FS_250DPS  0
#define MPU6050_GYRO_FS_500DPS  1
#define MPU6050_GYRO_FS_1000DPS 2
#define MPU6050_GYRO_FS_2000DPS 3

/* Data structure */
struct mpu6050_data_s
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
};

#ifdef __cplusplus
extern "C"
{
#endif

int mpu6050_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_MPU6050_H */
