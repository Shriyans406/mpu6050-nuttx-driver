/****************************************************************************
 * examples/mpu6050_test/mpu6050_test_main.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/sensors/mpu6050.h>

/**
 * Name: main
 *
 * Description:
 *   Read MPU6050 6-axis IMU sensor (accel + gyro + temp)
 */

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  struct mpu6050_data_s data;
  int read_count = 1;
  int i;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double temp;

  if (argc > 1)
    {
      read_count = atoi(argv[1]);
    }

  if (read_count < 1 || read_count > 100)
    {
      read_count = 1;
    }

  fd = open("/dev/mpu6050", O_RDONLY);
  if (fd < 0)
    {
      printf("MPU6050: ERROR - Cannot open /dev/mpu6050: %d\n", errno);
      return EXIT_FAILURE;
    }

  printf("MPU6050: Successfully opened /dev/mpu6050\n");
  printf("MPU6050: Reading sensor %d time(s)...\n\n", read_count);

  for (i = 0; i < read_count; i++)
    {
      ret = read(fd, &data, sizeof(struct mpu6050_data_s));
      if (ret < 0)
        {
          printf("MPU6050: ERROR - Read failed: %d\n", errno);
          close(fd);
          return EXIT_FAILURE;
        }

      /* Convert raw values to physical units
       * Accel: ±2g range, LSB = 16384 LSB/g -> value/16384 = g
       * Gyro: ±250°/s range, LSB = 131 LSB/°/s -> value/131 = °/s
       * Temp: formula from datasheet: (value/340.0) + 36.53 = °C
       */

      accel_x = data.accel_x / 16384.0;  /* in g */
      accel_y = data.accel_y / 16384.0;
      accel_z = data.accel_z / 16384.0;

      gyro_x = data.gyro_x / 131.0;      /* in °/s */
      gyro_y = data.gyro_y / 131.0;
      gyro_z = data.gyro_z / 131.0;

      temp = (data.temp / 340.0) + 36.53; /* in °C */

      printf("Reading %d:\n", i + 1);
      printf("  Accelerometer (g):\n");
      printf("    X: %7.3f  (raw: %6d)\n", accel_x, data.accel_x);
      printf("    Y: %7.3f  (raw: %6d)\n", accel_y, data.accel_y);
      printf("    Z: %7.3f  (raw: %6d)\n", accel_z, data.accel_z);

      printf("  Gyroscope (°/s):\n");
      printf("    X: %7.1f   (raw: %6d)\n", gyro_x, data.gyro_x);
      printf("    Y: %7.1f   (raw: %6d)\n", gyro_y, data.gyro_y);
      printf("    Z: %7.1f   (raw: %6d)\n", gyro_z, data.gyro_z);

      printf("  Temperature: %6.2f°C (raw: %d)\n", temp, data.temp);
      printf("\n");

      if (i < read_count - 1)
        {
          sleep(1);
        }
    }

  close(fd);

  printf("MPU6050: Test completed successfully!\n");
  return EXIT_SUCCESS;
}
