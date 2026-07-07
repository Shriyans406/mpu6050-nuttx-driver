/****************************************************************************
 * include/nuttx/sensors/mpu6050.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPU6050 I2C addresses */

#define MPU6050_ADDR_LOW   0x68
#define MPU6050_ADDR_HIGH  0x69

/* Register addresses */

#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1a
#define MPU6050_GYRO_CONFIG     0x1b
#define MPU6050_ACCEL_CONFIG    0x1c
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3a
#define MPU6050_ACCEL_XOUT_H    0x3b
#define MPU6050_ACCEL_XOUT_L    0x3c
#define MPU6050_ACCEL_YOUT_H    0x3d
#define MPU6050_ACCEL_YOUT_L    0x3e
#define MPU6050_ACCEL_ZOUT_H    0x3f
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_PWR_MGMT_1      0x6b
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: mpu6050_register
 *
 * Description:
 *   Register the MPU6050 6-axis IMU sensor device with the NuttX uORB
 *   sensor framework.
 *
 * Input Parameters:
 *   devno - Device number for sensor registration (e.g. 0)
 *   i2c   - Pointer to the I2C master interface
 *   addr  - I2C slave address of the MPU6050 device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpu6050_register(int devno, FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_MPU6050_H */
