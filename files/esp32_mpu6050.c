/****************************************************************************
 * boards/xtensa/esp32/esp32-devkitc-v4/src/esp32_mpu6050.c
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

#include <debug.h>
#include <syslog.h>

#include <nuttx/sensors/mpu6050.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_SENSORS_MPU6050

/**
 * Name: esp32_mpu6050_initialize
 *
 * Description:
 *   Initialize MPU6050 on ESP32-DevKit-C
 *
 * Hardware:
 *   - I2C0: GPIO 21 (SDA), GPIO 22 (SCL)
 *   - MPU6050 AD0 pin to GND (I2C address = 0x68)
 *   - Pull-up resistors: 4.7kΩ on SDA and SCL
 */

int esp32_mpu6050_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = esp32_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "MPU6050: Failed to get I2C0\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "MPU6050: I2C0 initialized\n");

  ret = mpu6050_register("/dev/mpu6050", i2c, 0x68);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MPU6050: Registration failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "MPU6050: Driver registered at /dev/mpu6050\n");
  return OK;
}

#endif /* CONFIG_SENSORS_MPU6050 */
