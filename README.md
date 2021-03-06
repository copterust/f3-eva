# copterust/f3-eva

Drone firmware written in [rust](https://github.com/rust-lang/rust).

(See [fcfs](https://github.com/copterust/fcfs-rtfm) as well).

[![Build Status](https://travis-ci.org/copterust/f3-eva.svg?branch=master)](https://travis-ci.org/copterust/f3-eva)

# Installation

We use rust nightly with target thumbv7em-none-eabi, travis uses both stable and nightly.

# Hardware

Officially supported devices are:

## Brains

* board -- [stm32f303](https://aliexpress.com/item/STM32F303-STM32F303RCT6/32861629069.html?spm=a2g0s.9042311.0.0.437a33edq27JIF)

* range scope --  [GY-530 VL53L0X](https://aliexpress.com/item/GY-530-VL53L0X-laser-range-finder-ToF-distance-measurement-Flight-time-range-sensor-module/32840503781.html?spm=a2g0s.9042311.0.0.437a33edq27JIF)

* gyroscope & acceleormeter --[MPU9250](https://ru.aliexpress.com/item/SPI-IIC-MPU9250-MPU-9250-MPU-9250-9-Axis-Attitude-Gyro-Accelerator-Magnetometer-Sensor-Module-MPU9250/32663479975.html?spm=a2g0s.9042311.0.0.437a33edq27JIF)

* pressure sensor -- [GY-BMP280-3.3](https://www.banggood.com/GY-BMP280-3_3-High-Precision-Atmospheric-Pressure-Sensor-Module-For-Arduino-p-1111135.html?rmmds=myorder&cur_warehouse=CN)

## Body

### Hexocopter

### Quadrocopter

## Doc

Refer to [docs](docs/PLATFORMS.md)
