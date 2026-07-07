/****************************************************************************
 * drivers/sensors/mpu6050.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mpu6050.h>

#ifdef CONFIG_SENSORS_MPU6050

#define MPU6050_I2C_FREQUENCY  400000  /* 400 kHz */

/* Device private data */
struct mpu6050_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  struct mpu6050_data_s data;
  uint8_t accel_fs;
  uint8_t gyro_fs;
};

/* Forward declarations */
static int     mpu6050_open(FAR struct file *filep);
static int     mpu6050_close(FAR struct file *filep);
static ssize_t mpu6050_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static int     mpu6050_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

static const struct file_operations g_mpu6050_fops =
{
  .open  = mpu6050_open,
  .close = mpu6050_close,
  .read  = mpu6050_read,
  .ioctl = mpu6050_ioctl,
};

/**
 * Name: mpu6050_write_reg
 *
 * Description:
 *   Write a single byte to an MPU6050 register
 */

static int mpu6050_write_reg(FAR struct mpu6050_dev_s *priv,
                              uint8_t regaddr, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  buffer[0] = regaddr;
  buffer[1] = value;

  msg.frequency = MPU6050_I2C_FREQUENCY;
  msg.addr = priv->addr;
  msg.flags = 0;
  msg.buffer = buffer;
  msg.length = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return ret < 0 ? ret : OK;
}

/**
 * Name: mpu6050_read_reg
 *
 * Description:
 *   Read a single byte from an MPU6050 register
 */

static int mpu6050_read_reg(FAR struct mpu6050_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *value)
{
  struct i2c_msg_s msgs[2];
  int ret;

  msgs[0].frequency = MPU6050_I2C_FREQUENCY;
  msgs[0].addr = priv->addr;
  msgs[0].flags = 0;
  msgs[0].buffer = &regaddr;
  msgs[0].length = 1;

  msgs[1].frequency = MPU6050_I2C_FREQUENCY;
  msgs[1].addr = priv->addr;
  msgs[1].flags = I2C_M_READ;
  msgs[1].buffer = value;
  msgs[1].length = 1;

  ret = I2C_TRANSFER(priv->i2c, msgs, 2);
  return ret < 0 ? ret : OK;
}

/**
 * Name: mpu6050_read_data
 *
 * Description:
 *   Read all sensor data (accel, gyro, temp) from MPU6050
 */

static int mpu6050_read_data(FAR struct mpu6050_dev_s *priv)
{
  struct i2c_msg_s msgs[2];
  uint8_t regaddr = MPU6050_ACCEL_XOUT_H;
  uint8_t buffer[14];  /* 14 bytes: accel(6) + temp(2) + gyro(6) */
  int16_t raw_accel_x, raw_accel_y, raw_accel_z;
  int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
  int16_t raw_temp;
  int ret;

  /* Setup I2C messages for read */
  msgs[0].frequency = MPU6050_I2C_FREQUENCY;
  msgs[0].addr = priv->addr;
  msgs[0].flags = 0;
  msgs[0].buffer = &regaddr;
  msgs[0].length = 1;

  msgs[1].frequency = MPU6050_I2C_FREQUENCY;
  msgs[1].addr = priv->addr;
  msgs[1].flags = I2C_M_READ;
  msgs[1].buffer = buffer;
  msgs[1].length = 14;

  ret = I2C_TRANSFER(priv->i2c, msgs, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MPU6050: I2C transfer failed: %d\n", ret);
      return ret;
    }

  /* Parse accelerometer data (MSB first, big-endian) */
  raw_accel_x = (buffer[0] << 8) | buffer[1];
  raw_accel_y = (buffer[2] << 8) | buffer[3];
  raw_accel_z = (buffer[4] << 8) | buffer[5];

  /* Parse temperature data */
  raw_temp = (buffer[6] << 8) | buffer[7];

  /* Parse gyroscope data */
  raw_gyro_x = (buffer[8] << 8) | buffer[9];
  raw_gyro_y = (buffer[10] << 8) | buffer[11];
  raw_gyro_z = (buffer[12] << 8) | buffer[13];

  /* Store raw values (user will convert using full scale range) */
  priv->data.accel_x = raw_accel_x;
  priv->data.accel_y = raw_accel_y;
  priv->data.accel_z = raw_accel_z;
  priv->data.gyro_x = raw_gyro_x;
  priv->data.gyro_y = raw_gyro_y;
  priv->data.gyro_z = raw_gyro_z;
  priv->data.temp = raw_temp;

  syslog(LOG_DEBUG, "MPU6050: Accel X=%d Y=%d Z=%d Gyro X=%d Y=%d Z=%d\n",
         raw_accel_x, raw_accel_y, raw_accel_z,
         raw_gyro_x, raw_gyro_y, raw_gyro_z);

  return OK;
}

/**
 * Name: mpu6050_open
 */

static int mpu6050_open(FAR struct file *filep)
{
  return OK;
}

/**
 * Name: mpu6050_close
 */

static int mpu6050_close(FAR struct file *filep)
{
  return OK;
}

/**
 * Name: mpu6050_read
 */

static ssize_t mpu6050_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu6050_dev_s *priv = inode->i_private;
  int ret;

  if (buflen < sizeof(struct mpu6050_data_s))
    {
      return -EINVAL;
    }

  ret = mpu6050_read_data(priv);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, &priv->data, sizeof(struct mpu6050_data_s));
  return sizeof(struct mpu6050_data_s);
}

/**
 * Name: mpu6050_ioctl
 */

static int mpu6050_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/**
 * Name: mpu6050_register
 */

int mpu6050_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c,
                     uint8_t addr)
{
  FAR struct mpu6050_dev_s *priv;
  uint8_t whoami;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  priv = (FAR struct mpu6050_dev_s *)kmm_malloc(sizeof(struct mpu6050_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(struct mpu6050_dev_s));
  priv->i2c = i2c;
  priv->addr = addr;

  /* Verify MPU6050 is present */
  ret = mpu6050_read_reg(priv, MPU6050_WHO_AM_I, &whoami);
  if (ret < 0 || whoami != 0x68)
    {
      syslog(LOG_ERR, "MPU6050: WHO_AM_I = 0x%02x (expected 0x68)\n", whoami);
      kmm_free(priv);
      return -ENODEV;
    }

  syslog(LOG_INFO, "MPU6050: Found at 0x%02x\n", addr);

  /* Initialize MPU6050 */
  mpu6050_write_reg(priv, MPU6050_PWR_MGMT_1, 0x00);  /* Wake up */
  mpu6050_write_reg(priv, MPU6050_SMPLRT_DIV, 9);     /* 100 Hz */
  mpu6050_write_reg(priv, MPU6050_ACCEL_CONFIG, 0x00); /* ±2g */
  mpu6050_write_reg(priv, MPU6050_GYRO_CONFIG, 0x00);  /* ±250°/s */

  /* Register character device */
  ret = register_driver(devpath, &g_mpu6050_fops, 0666, priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MPU6050: Failed to register: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  syslog(LOG_INFO, "MPU6050: Driver registered at %s\n", devpath);
  return OK;
}

#endif /* CONFIG_SENSORS_MPU6050 */
