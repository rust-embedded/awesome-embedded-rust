# Embedded Rust

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

This is a curated list of resources related to embedded and low-level programming in the programming language Rust, including a list of useful crates.

[<img src="https://rawgit.com/rust-embedded/awesome-embedded-rust/master/rust-embedded-logo-256x256.png" align="right" width="256">](http://www.rust-embedded.org)

## Table of contents

* [Community](#community)
* [Books, blogs and training materials](#books-blogs-and-training-materials)
* [Tools](#tools)
* [Device crates](#device-crates)
    * [ARM](#arm)
    * [Nordic](#nordic)
    * [NXP](#nxp)
    * [STMicroelectronics](#stmicroelectronics)
    * [MSP430](#msp430)
* [HAL implementation crates](#hal-implementation-crates)
    * [OS](#os)
    * [Nordic](#nordic-1)
    * [NXP](#nxp-1)
    * [STMicroelectronics](#stmicroelectronics-1)
    * [Texas Instruments](#texas-instruments)
* [Board support crates](#board-support-crates)
    * [Nordic](#nordic-2)
    * [NXP](#nxp-2)
    * [STMicroelectronics](#stmicroelectronics-2)
* [Driver crates](#driver-crates)
    * [WIP](#wip)
* [no-std crates](#no-std-crates)
* [Rust forks](#rust-forks)
    - [AVR](#avr)
* [License](#license)

## Community

In 2018 Mozilla asked for creation of an embedded workgroup to help drive adoption in the Rust ecosystem.

- [Embedded WG](https://github.com/rust-lang-nursery/embedded-wg/), including newsletters with progress updates.

- [embedded.rs](https://t.me/embedded_rs) Telegram chat about Rust for microcontrollers in Russian language.

## Books, blogs and training materials

-   [Discovery](https://japaric.github.io/discovery/) by @japaric — this book is an introductory course on microcontroller-based embedded systems that uses Rust as the teaching language.
-   [Exploring Rust on Teensy](https://branan.github.io/teensy/) by @branan — Beginner set of articles on getting into embedded dev in Rust.
-   [Pragmatic Bare Metal Rust](http://www.hashmismatch.net/pragmatic-bare-metal-rust/) A starter article about starting Rust development on STM32 microcontrollers (cubeMX + FFI).
-   [Using Rust in an Embedded Project: A Simple Example](https://spin.atomicobject.com/2016/07/08/rust-embedded-project-example/#.V3-os-6qlZw.hackernews) Article and some links on setting up Rust cross-compiling.
-   [Robigalia](https://robigalia.org) IoT operating system in Rust running on secure seL4 microkernel.
-   [Tock](https://www.tockos.org) An embedded operating system designed for running multiple concurrent, mutually distrustful applications on low-memory and low-power microcontrollers
-   [intermezzOS](http://intermezzos.github.io) A small teaching operating system in Rust. A book with some explanations is also included.
-   [Raspberry Pi Bare Metal Programming with Rust](https://medium.com/@thiagopnts/raspberry-pi-bare-metal-programming-with-rust-a6f145e84024) A starter article on OSdev with Rust on RPi (cross-compiler setup and a very basic LED-blinking kernel).
-   [Fearless concurrency](http://blog.japaric.io/fearless-concurrency/) by @japaric — How to easily develop Rust programs for pretty much any ARM Cortex-M microcontroller with memory-safe concurrency.
-   [RTFM v2](http://blog.japaric.io/rtfm-v2/) Real-Time For the Masses — Cortex-M programming framework for building concurrent applications.
-   [cortex-m rtfm](https://github.com/japaric/cortex-m-rtfm) RTFM framework for ARM Cortex-M microcontrollers
-   [msp430 rtfm](https://github.com/japaric/msp430-rtfm) RTFM framework for MSP430 MCUs
-   [FreeRTOS.rs](https://github.com/hashmismatch/freertos.rs) Rust interface for the FreeRTOS API

## Tools

-   [xargo](https://github.com/japaric/xargo) Rust package manager with support for non-default std libraries — build rust runtime for your own embedded system.
-   [svd2rust](https://github.com/japaric/svd2rust) Generate Rust structs with register mappings from SVD files.
-   [μtest](https://github.com/japaric/utest) unit testing for microcontrollers and other no-std systems.
-   [bindgen](https://crates.io/crates/bindgen) Automatically generates Rust FFI bindings to C and C++ libraries.
-   [cortex-m semihosting](https://github.com/japaric/cortex-m-semihosting) Semihosting for ARM Cortex-M processors

## Device crates

Register definition for microcontroller families. Usually generated using [`svd2rust`].

[`svd2rust`]: https://crates.io/crates/svd2rust

*NOTE* You may be able to find even more device crates by searching for the
[`svd2rust`][svd2rust-kw] keyword on crates.io!

[svd2rust-kw]: https://crates.io/keywords/svd2rust

### ARM

- [`cortex-m`](https://github.com/japaric/cortex-m) Low level access to Cortex-M processors

### Nordic

- [`nrf51`](https://crates.io/crates/nrf51) Peripheral access API for nRF51 microcontrollers (generated using svd2rust)
- [`nrf51822` playground](https://github.com/japaric/nrf51822) A crate to play with the nrf51822 module

### NXP

- [`lpc82x`](https://crates.io/crates/lpc82x)
- [`mkw41z`](https://crates.io/crates/mkw41z)

### STMicroelectronics

- [`stm32f042`](https://crates.io/crates/stm32f042)
- [`stm32f103xx`](https://crates.io/crates/stm32f103xx)
- [`stm32f30x`](https://crates.io/crates/stm32f30x)
- [`stm32l151`](https://crates.io/crates/stm32l151)

### MSP430
-   [`msp430g2553`](https://github.com/japaric/msp430g2553) Peripheral access API for MSP430G2553 microcontrollers (generated using svd2rust)
-   [rust on msp](https://github.com/japaric/rust_on_msp) Simple blinking LED example that runs on MSP430.
-   [msp430 quickstart](https://github.com/japaric/msp430-quickstart) some examples for msp430

## HAL implementation crates

Implementations of [`embedded-hal`] for microcontroller families and systems running some OS.

[`embedded-hal`]: https://crates.io/crates/embedded-hal

*NOTE* You may be able to find even more HAL implementation crates by searching for the
[`embedded-hal-impl`] and [`embedded-hal`][embedded-hal-kw] keywords on crates.io!

[`embedded-hal-impl`]: https://crates.io/keywords/embedded-hal-impl
[embedded-hal-kw]: https://crates.io/keywords/embedded-hal

### OS

- [`linux-embedded-hal`] for embedded Linux systems like the Raspberry Pi.

[`linux-embedded-hal`]: https://crates.io/crates/linux-embedded-hal

### Nordic

- [`nrf51-hal`](https://crates.io/crates/nrf51-hal)

### NXP

Also check the list of [NXP board support crates][nxp-bsc]!

[nxp-bsc]: #nxp-1

- [`lpc82x-hal`](https://github.com/braun-robotics/rust-lpc82x-hal)

- [`mkw41z-hal`](https://crates.io/crates/mkw41z-hal)

### STMicroelectronics

Also check the list of [STMicroelectronics board support crates][stm-bsc]!

[stm-bsc]: #stmicroelectronics-2

- [`stm32f042-hal`](https://crates.io/crates/stm32f042-hal)
  - Has examples that can run on boards like the [Nucleo-F042K6] and similar boards

[Nucleo-F042K6]: http://www.st.com/en/evaluation-tools/nucleo-f042k6.html

- [`stm32f103xx-hal`](https://github.com/japaric/stm32f103xx-hal)
  - Has examples that can run on boards like the [Blue pill], [Nucleo-F103RB] and similar boards

[Nucleo-F103RB]: http://www.st.com/en/evaluation-tools/nucleo-f103rb.html

- [`stm32f30x-hal`](https://crates.io/crates/stm32f30x-hal)

- [`stm32l151-hal`](https://crates.io/crates/stm32l151-hal)

### Texas Instruments

- [`tm4c123x-hal`](https://github.com/thejpster/tm4c123x-hal)

[Blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill

## Board support crates

Crates tailored for specific development boards.

[STM32F3DISCOVERY]: http://www.st.com/en/evaluation-tools/stm32f3discovery.html

### Nordic

- [`microbit`](https://crates.io/crates/microbit) - [micro:bit]

[micro:bit]: http://microbit.org/

### NXP

- [`frdm-kw41z`](https://crates.io/crates/frdm-kw41z) - [FRDM-KW41Z]

[FRDM-KW41Z]: https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/kinetis-cortex-m-mcus/w-serieswireless-conn.m0-plus-m4/freedom-development-kit-for-kinetis-kw41z-31z-21z-mcus:FRDM-KW41Z

### STMicroelectronics

- [`nucleo-f042k6`](https://github.com/therealprof/nucleo-f042k6.git) - [Nucleo-F042K6]
- [`nucleo-f103rb`](https://github.com/therealprof/nucleo-f103rb.git) - [Nucleo-F103RB]
- [`f3`](https://crates.io/crates/f3) - [STM32F3DISCOVERY]

## Driver crates

Platform agnostic crates to interface external components. These crates use the [`embedded-hal`]
interface to support [all the devices and systems that implement the `embedded-hal`
traits][hal-impl].

[hal-impl]: #hal-implementation-crates

The list below contains drivers developed as part of the [Weekly Driver initiative][wd] and that
have achieved the "released" status (published on crates.io + documentation / short blog post).

[wd]: https://github.com/rust-lang-nursery/embedded-wg/issues/39

1. [L3GD20] - SPI - Gyroscope - [Intro blog post][1&2]
2. [LSM303DLHC] - I2C - Accelerometer + compass (magnetometer) - [Intro blog post][1&2]
3. [MCP3008] - SPI - 8 channel 10-bit ADC - [Intro blog post][3]
4. [ENC28J60] - SPI - Ethernet controller - [Intro blog post][4]
5. [MCP3425] - I2C - 16-bit ADC - [Intro blog post][5]
6. [SGP30] - I2C - Gas sensor - [Intro blog post][6]
7. [HTS221] - I2C - Humidity and temperature sensor - [Intro blog post][7]

[L3GD20]: https://crates.io/crates/l3gd20
[LSM303DLHC]: https://crates.io/crates/lsm303dlhc
[1&2]: http://blog.japaric.io/wd-1-2-l3gd20-lsm303dlhc-madgwick/

[MCP3008]: https://crates.io/crates/adc-mcp3008
[3]: http://pramode.in/2018/02/24/an-introduction-to-writing-embedded-hal-based-drivers-in-rust/

[ENC28J60]: https://crates.io/crates/enc28j60
[4]: http://blog.japaric.io/wd-4-enc28j60/

[MCP3425]: https://crates.io/crates/mcp3425
[5]: https://blog.dbrgn.ch/2018/3/13/rust-mcp3425-driver/
[SGP30]: https://crates.io/crates/sgp30
[6]: https://blog.dbrgn.ch/2018/4/1/rust-sgp30-driver/

[HTS221]: https://crates.io/crates/hts221
[7]: https://medium.com/@pdanielgallagher/hts221-humidity-and-temperature-sensor-88056ea9e5fa

*NOTE* You may be able to find even more driver crates by searching for the [`embedded-hal-driver`]
keyword on crates.io!

[`embedded-hal-driver`]: https://crates.io/keywords/embedded-hal-driver

### WIP

Work in progress drivers. Help the authors make these crates awesome!

- [MFRC522] - SPI - RFID tag reader/writer
- [MPU9250] - SPI - Accelerometer + gyroscope + compass
- [motor-driver] - Motor drivers: L298N, TB6612FNG, etc.
- [MAG3110] - I2C - Magnetometer
- [SI5351] - I2C - clock generator
- [SI7021] - I2C - Humidity and temperature sensor
- [MAX7219] - SPI - LED display driver
- [DS3231] - I2C - real time clock
- [BH1750] - I2C - ambient light sensor (lux meter)
- [SHT2x] - I2C - temperature / humidity sensors
- [INA260] - I2C - power monitor
- [SSD1306] - I2C - OLED display driver
- [ILI9341] - SPI - TFT LCD display
- [HD44780] - Parallel port - LCD controller
- [MCP9808] - I2C - Temperature sensor
- [MMA7660FC] - I2C - 3-axis accelerometer
- [AXP209] - I2C - Power management unit
- [DS3234] - SPI - Real time clock
- [PCD8544] - SPI - 48x84 pixels matrix LCD controller
- [HC-SR04] - DIO - Ultrasound sensor
- [AFE4400] - SPI - Pulse oximeter
- [SX1278] - SPI - Long range (LoRa) transceiver
- [RFM69] - SPI - ISM radio transceiver
- [LS010B7DH01] - SPI - Memory LCD
- [MAX31855] - SPI - Thermocouple digital converter
- [SHT3x] - I2C - Temperature / humidity sensors

[MFRC522]: https://github.com/japaric/mfrc522
[MPU9250]: https://github.com/japaric/mpu9250
[motor-driver]: https://github.com/japaric/motor-driver
[MAG3110]: https://github.com/therealprof/mag3110
[SI5351]: https://github.com/ilya-epifanov/si5351
[SI7021]: https://github.com/wose/si7021
[MAX7219]: https://github.com/maikelwever/max7219
[DS3231]: https://github.com/wose/ds3231
[BH1750]: https://github.com/wose/bh1750
[SHT2x]: https://github.com/dbrgn/sht2x-rs
[INA260]: https://crates.io/crates/ina260
[SSD1306]: https://github.com/jamwaffles/ssd1306
[ILI9341]: https://github.com/yuri91/ili9341-rs
[HD44780]: http://github.com/kunerd/clerk
[MCP9808]: https://crates.io/crates/mcp9808
[MMA7660FC]: https://github.com/rahul-thakoor/mma7660fc
[PCD8544]: https://github.com/pcein/pcd8544
[AXP209]: https://github.com/RandomInsano/axp209-rs
[DS3234]: https://github.com/rust-lang-nursery/embedded-wg/issues/39#issuecomment-375262785
[HC-SR04]: https://github.com/nordmoen/hc-sr04
[AFE4400]: https://github.com/ReeceStevens/afe4400
[SX1278]: https://github.com/susu/sx1278
[RFM69]: https://github.com/lolzballs/rfm69
[LS010B7DH01]: https://github.com/byronwasti/ls010b7dh01
[MAX31855]: https://github.com/mbacch/max31855
[SHT3x]: https://github.com/miek/sht3x-rs

## no-std crates

[`#![no_std]` crates][no-std-category] designed to run on resource constrained devices.

*whoops* this list is currently empty. Want to help us write it? Leave a comment on issue [#2].

- [bit_field](https://crates.io/crates/bit_field): manipulating bitfields and bitarrays

[no-std-category]: https://crates.io/categories/no-std
[#2]: https://github.com/rust-embedded/awesome-embedded-rust/issues/2

## Rust forks

### AVR
-  [AVR Rust](https://github.com/avr-rust/rust) Fork of Rust with AVR support.

## License

This list is licensed under

- CC0 1.0 Universal License ([LICENSE-CC0](LICENSE-CC0) or
  https://creativecommons.org/publicdomain/zero/1.0/legalcode)
