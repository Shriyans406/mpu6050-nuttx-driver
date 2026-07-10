/****************************************************************************
 * apps/examples/sensor_fusion/sensor_fusion_main.c
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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <nuttx/sensors/sensor.h>
#include "Fusion/Fusion.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_LOW_MASK    0xff00
#define REG_HIGH_MASK   0x00ff
#define MPU6050_FS_SEL  32.8f
#define MPU6050_AFS_SEL 4096.0f

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu6050_imu_msg
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read_mpu6050_legacy
 *
 * Description:
 *   Read sensor data from legacy MPU6050 character driver (/dev/imu0).
 ****************************************************************************/

static void read_mpu6050_legacy(int fd, FAR struct sensor_accel *acc_data,
                                FAR struct sensor_gyro *gyro_data)
{
  struct mpu6050_imu_msg raw_imu;
  int16_t raw_data[7];
  int ret;

  memset(&raw_imu, 0, sizeof(raw_imu));

  ret = read(fd, &raw_data, sizeof(raw_data));
  if (ret != sizeof(raw_data))
    {
      printf("Failed to read legacy MPU6050 sensor data: %d\n", ret);
    }
  else
    {
      raw_imu.acc_x  = ((raw_data[0] & REG_HIGH_MASK) << 8) +
                       ((raw_data[0] & REG_LOW_MASK) >> 8);
      raw_imu.acc_y  = ((raw_data[1] & REG_HIGH_MASK) << 8) +
                       ((raw_data[1] & REG_LOW_MASK) >> 8);
      raw_imu.acc_z  = ((raw_data[2] & REG_HIGH_MASK) << 8) +
                       ((raw_data[2] & REG_LOW_MASK) >> 8);
      raw_imu.gyro_x = ((raw_data[4] & REG_HIGH_MASK) << 8) +
                       ((raw_data[4] & REG_LOW_MASK) >> 8);
      raw_imu.gyro_y = ((raw_data[5] & REG_HIGH_MASK) << 8) +
                       ((raw_data[5] & REG_LOW_MASK) >> 8);
      raw_imu.gyro_z = ((raw_data[6] & REG_HIGH_MASK) << 8) +
                       ((raw_data[6] & REG_LOW_MASK) >> 8);
    }

  acc_data->x = raw_imu.acc_x / MPU6050_AFS_SEL;
  acc_data->y = raw_imu.acc_y / MPU6050_AFS_SEL;
  acc_data->z = raw_imu.acc_z / MPU6050_AFS_SEL;

  /* Gyro data in deg/s */

  gyro_data->x = raw_imu.gyro_x / MPU6050_FS_SEL;
  gyro_data->y = raw_imu.gyro_y / MPU6050_FS_SEL;
  gyro_data->z = raw_imu.gyro_z / MPU6050_FS_SEL;
}

/****************************************************************************
 * Name: read_mpu6050_uorb
 *
 * Description:
 *   Read sensor data from modern MPU6050 lower-half uORB driver nodes.
 ****************************************************************************/

static int read_mpu6050_uorb(int fd_accel, int fd_gyro,
                             FAR struct sensor_accel *acc_data,
                             FAR struct sensor_gyro *gyro_data)
{
  struct sensor_accel acc_msg;
  struct sensor_gyro gyro_msg;
  int ret;

  ret = read(fd_accel, &acc_msg, sizeof(acc_msg));
  if (ret != sizeof(acc_msg))
    {
      return -EIO;
    }

  ret = read(fd_gyro, &gyro_msg, sizeof(gyro_msg));
  if (ret != sizeof(gyro_msg))
    {
      return -EIO;
    }

  /* uORB accelerometer data is in m/s^2. Convert to g (9.80665 m/s^2)
   * for consistency with the Fusion AHRS input format.
   */

  acc_data->x = acc_msg.x / 9.80665f;
  acc_data->y = acc_msg.y / 9.80665f;
  acc_data->z = acc_msg.z / 9.80665f;

  /* uORB gyro is in rad/s. Convert to deg/s for Fusion library. */

  gyro_data->x = gyro_msg.x * (180.0f / M_PI_F);
  gyro_data->y = gyro_msg.y * (180.0f / M_PI_F);
  gyro_data->z = gyro_msg.z * (180.0f / M_PI_F);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * sensor_fusion_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct sensor_accel imu_acc_data;
  struct sensor_gyro imu_gyro_data;
  FusionVector accelerometer;
  FusionVector gyroscope;
  FusionEuler euler;
  FusionAhrs ahrs;
  int iterations = CONFIG_EXAMPLES_SENSOR_FUSION_SAMPLES;
  float acq_period = CONFIG_EXAMPLES_SENSOR_FUSION_SAMPLE_RATE / 1000.0f;
  int fd_accel = -1;
  int fd_gyro = -1;
  int fd_legacy = -1;
  bool use_uorb = false;
  int i;

  FusionAhrsInitialise(&ahrs);

  printf("Sensor Fusion example\n");
  printf("Sample Rate: %.2f Hz\n", 1.0f / acq_period);

  /* Try opening modern MPU6050 uORB lower-half driver nodes first */

  fd_accel = open("/dev/uorb/sensor_accel0", O_RDONLY);
  if (fd_accel < 0)
    {
      fd_accel = open("/dev/sensor_accel0", O_RDONLY);
    }

  fd_gyro = open("/dev/uorb/sensor_gyro0", O_RDONLY);
  if (fd_gyro < 0)
    {
      fd_gyro = open("/dev/sensor_gyro0", O_RDONLY);
    }

  if (fd_accel >= 0 && fd_gyro >= 0)
    {
      use_uorb = true;
      printf("Using uORB MPU6050 driver (/dev/uorb/sensor_accel0)\n");
    }
  else
    {
      if (fd_accel >= 0)
        {
          close(fd_accel);
        }

      if (fd_gyro >= 0)
        {
          close(fd_gyro);
        }

      /* Fall back to legacy MPU6050 character driver (/dev/imu0) */

      fd_legacy = open("/dev/imu0", O_RDONLY);
      if (fd_legacy < 0)
        {
          printf("Failed to open MPU6050 sensor (/dev/uorb or /dev/imu0)\n");
          return EXIT_FAILURE;
        }

      printf("Using legacy MPU6050 driver (/dev/imu0)\n");
    }

  for (i = 0; i < iterations; i++)
    {
      if (use_uorb)
        {
          if (read_mpu6050_uorb(fd_accel, fd_gyro,
                                &imu_acc_data, &imu_gyro_data) < 0)
            {
              printf("Failed to read uORB sensor data\n");
            }
        }
      else
        {
          read_mpu6050_legacy(fd_legacy, &imu_acc_data, &imu_gyro_data);
        }

      accelerometer.axis.x = imu_acc_data.x;
      accelerometer.axis.y = imu_acc_data.y;
      accelerometer.axis.z = imu_acc_data.z;

      gyroscope.axis.x = imu_gyro_data.x;
      gyroscope.axis.y = imu_gyro_data.y;
      gyroscope.axis.z = imu_gyro_data.z;

      FusionAhrsUpdateNoMagnetometer(&ahrs,
                                     gyroscope,
                                     accelerometer,
                                     acq_period);
      euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

      printf("Yaw: %.3f | Pitch: %.3f | Roll: %.3f\n",
             euler.angle.yaw, euler.angle.pitch, euler.angle.roll);
      usleep(CONFIG_EXAMPLES_SENSOR_FUSION_SAMPLE_RATE * 1000);
    }

  if (use_uorb)
    {
      close(fd_accel);
      close(fd_gyro);
    }
  else
    {
      close(fd_legacy);
    }

  return EXIT_SUCCESS;
}
