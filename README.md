# Embedded Rust

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

This is a curated list of resources related to embedded and low-level programming in the programming language Rust, including a list of useful crates.

[<img src="https://rawgit.com/rust-embedded/awesome-embedded-rust/master/rust-embedded-logo-256x256.png" align="right" width="256">](http://www.rust-embedded.org)

This project is developed and maintained by the [Resources team][team].

## Table of contents

* [Community](#community)
* [Books, blogs and training materials](#books-blogs-and-training-materials)
* [Tools](#tools)
* [Device crates](#device-crates)
    * [Nordic](#nordic)
    * [NXP](#nxp)
    * [STMicroelectronics](#stmicroelectronics)
    * [Texas Instruments](#texas-instruments)
    * [MSP430](#msp430)
* [HAL implementation crates](#hal-implementation-crates)
    * [OS](#os)
    * [Nordic](#nordic-1)
    * [NXP](#nxp-1)
    * [STMicroelectronics](#stmicroelectronics-1)
    * [Texas Instruments](#texas-instruments-1)
    * [Espressif](#espressif)
* [Architecture support crates](#architecture-support-crates)
    * [ARM](#arm)
* [Board support crates](#board-support-crates)
    * [Nordic](#nordic-2)
    * [NXP](#nxp-2)
    * [STMicroelectronics](#stmicroelectronics-2)
* [Driver crates](#driver-crates)
    * [WIP](#wip)
* [no-std crates](#no-std-crates)
* [Rust forks](#rust-forks)
    - [AVR](#avr)
* [Firmware projects](#firmware-projects)
* [License](#license)

## Community

In 2018 Rust community has created an embedded workgroup to help drive adoption in the Rust ecosystem.

- [Embedded WG](https://github.com/rust-lang-nursery/embedded-wg/), including newsletters with progress updates.

- You can usually find the members of the embedded WG on the `#rust-embedded` channel (server: `irc.mozilla.org`).

- [embedded.rs](https://t.me/embedded_rs) Telegram chat about Rust for microcontrollers in Russian language.

## Books, blogs and training materials

-   [Discovery](https://rust-embedded.github.io/discovery) by @rust-embedded — this book is an introductory course on microcontroller-based embedded systems that uses Rust as the teaching language. Original author: @japaric
-   [Cortex-M Quickstart](https://docs.rs/cortex-m-quickstart/0.3.1/cortex_m_quickstart/) by @japaric – a template and introduction to embedded Rust, suitable for developers familiar to embedded development but new to embedded Rust.
-   [Exploring Rust on Teensy](https://branan.github.io/teensy/) by @branan — Beginner set of articles on getting into embedded dev in Rust.
-   [Pragmatic Bare Metal Rust](http://www.hashmismatch.net/pragmatic-bare-metal-rust/) A starter article about starting Rust development on STM32 microcontrollers (cubeMX + FFI).
-   [Using Rust in an Embedded Project: A Simple Example](https://spin.atomicobject.com/2016/07/08/rust-embedded-project-example/#.V3-os-6qlZw.hackernews) Article and some links on setting up Rust cross-compiling.
-   [Robigalia](https://robigalia.org) IoT operating system in Rust running on secure seL4 microkernel.
-   [Tock](https://www.tockos.org) An embedded operating system designed for running multiple concurrent, mutually distrustful applications on low-memory and low-power microcontrollers
-   [intermezzOS](http://intermezzos.github.io) A small teaching operating system in Rust. A book with some explanations is also included.
-   Raspberry Pi Bare Metal Programming with Rust
    -   [32-bit Version (most Pi1 and Pi2 variants)](https://medium.com/@thiagopnts/raspberry-pi-bare-metal-programming-with-rust-a6f145e84024) A starter article on OSdev with Rust on RPi (cross-compiler setup and a very basic LED-blinking kernel).
    -   [64-bit Version (Pi2 Model B v1.2 and all Pi3)](https://github.com/andre-richter/rust-raspi3-tutorial) A growing collection of tutorials, from simple booting to interfacing components like UARTs or random number generators; Features a painless cross-toolchain setup.
-   [Fearless concurrency](http://blog.japaric.io/fearless-concurrency/) by @japaric — How to easily develop Rust programs for pretty much any ARM Cortex-M microcontroller with memory-safe concurrency.
-   [RTFM v2](http://blog.japaric.io/rtfm-v2/) Real-Time For the Masses — Cortex-M programming framework for building concurrent applications.
-   [cortex-m rtfm](https://github.com/japaric/cortex-m-rtfm) RTFM framework for ARM Cortex-M microcontrollers
-   [msp430 rtfm](https://github.com/japaric/msp430-rtfm) RTFM framework for MSP430 MCUs
-   [FreeRTOS.rs](https://github.com/hashmismatch/freertos.rs) Rust interface for the FreeRTOS API
-   [MicroRust](https://droogmic.github.io/microrust/) Introductory book for embedded development in Rust on the micro:bit.

## Tools

-   [xargo](https://github.com/japaric/xargo) Rust package manager with support for non-default std libraries — build rust runtime for your own embedded system.
-   [svd2rust](https://github.com/japaric/svd2rust) Generate Rust structs with register mappings from SVD files.
-   [μtest](https://github.com/japaric/utest) unit testing for microcontrollers and other no-std systems.
-   [bindgen](https://crates.io/crates/bindgen) Automatically generates Rust FFI bindings to C and C++ libraries. - ![crates.io](https://img.shields.io/crates/v/bindgen.svg)
-   [cortex-m semihosting](https://github.com/japaric/cortex-m-semihosting) Semihosting for ARM Cortex-M processors
-   [bobbin-cli](https://github.com/bobbin-rs/bobbin-cli) A Rust command line tool to simplify embedded development and deployment.

## Device crates

Register definition for microcontroller families. Usually generated using [`svd2rust`]. - ![crates.io](https://img.shields.io/crates/v/svd2rust.svg)

[`svd2rust`]: https://crates.io/crates/svd2rust

*NOTE* You may be able to find even more device crates by searching for the
[`svd2rust`][svd2rust-kw] keyword on crates.io!

[svd2rust-kw]: https://crates.io/keywords/svd2rust

### Microchip

- [`atsamd21`](https://github.com/wez/atsamd21-rs) Peripheral access API for Microchip (formerly Atmel) SAMD21 microcontrollers.  This git repo hosts both the device crate and the hal.

### Nordic

- [`nrf51`](https://crates.io/crates/nrf51) Peripheral access API for nRF51 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf51.svg)

### NXP

- [`lpc82x`](https://crates.io/crates/lpc82x) - ![crates.io](https://img.shields.io/crates/v/lpc82x.svg)
- [`mkw41z`](https://crates.io/crates/mkw41z) - ![crates.io](https://img.shields.io/crates/v/mkw41z.svg)
- [`k64`](https://crates.io/crates/k64) - ![crates.io](https://img.shields.io/crates/v/k64.svg)

### STMicroelectronics

- [`stm32f042`](https://crates.io/crates/stm32f042) - ![crates.io](https://img.shields.io/crates/v/stm32f042.svg)
- [`stm32f103xx`](https://crates.io/crates/stm32f103xx) Peripheral access API for STM32F103XX microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/stm32f103xx.svg)
- [`stm32f100xx`](https://github.com/japaric/stm32f100xx) Peripheral access API for STM32F100XX microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/stm32f100xx.svg)
- [`stm32f30x`](https://crates.io/crates/stm32f30x) Peripheral access API for STM32F30X microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/stm32f30x.svg)
- [`stm32f429`](https://crates.io/crates/stm32f429) Peripheral access API for STM32F429 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/stm32f429.svg)
- [`stm32f469xx`](https://crates.io/crates/stm32f469xx) Peripheral access API for STM32f469XX microcontrollers (generated using svd2rust) -![crates.io](https://img.shields.io/crates/v/stm32f469xx.svg)
- [`stm32l151`](https://crates.io/crates/stm32l151) - ![crates.io](https://img.shields.io/crates/v/stm32l151.svg)
* [`stm32-rs`](https://github.com/adamgreig/stm32-rs) Peripheral access API for most STM32 microcontrollers (generated using svd2rust): [stm32f0](https://crates.io/crates/stm32f0), [stm32f1](https://crates.io/crates/stm32f1), [stm32f2](https://crates.io/crates/stm32f2), [stm32f3](https://crates.io/crates/stm32f3), [stm32f4](https://crates.io/crates/stm32f4), [stm32f7](https://crates.io/crates/stm32f7), [stm32l0](https://crates.io/crates/stm32l0), [stm32l1](https://crates.io/crates/stm32l1), [stm32l4](https://crates.io/crates/stm32l4), [stm32h7](https://crates.io/crates/stm32h7)

### Texas Instruments

-   [`tm4c123x`](https://crates.io/crates/tm4c123x) Peripheral access API for TM4C123x microcontrollers (generated using svd2rust)
-   [`tm4c129x`](https://crates.io/crates/tm4c129x) Peripheral access API for TM4C129x microcontrollers (generated using svd2rust)

### MSP430

-   [`msp430g2553`](https://github.com/japaric/msp430g2553) Peripheral access API for MSP430G2553 microcontrollers (generated using svd2rust)
-   [rust on msp](https://github.com/japaric/rust_on_msp) Simple blinking LED example that runs on MSP430.
-   [msp430 quickstart](https://github.com/japaric/msp430-quickstart) some examples for msp430

## HAL implementation crates

Implementations of [`embedded-hal`] for microcontroller families and systems running some OS. - ![crates.io](https://img.shields.io/crates/v/embedded-hal.svg)

[`embedded-hal`]: https://crates.io/crates/embedded-hal

*NOTE* You may be able to find even more HAL implementation crates by searching for the
[`embedded-hal-impl`] and [`embedded-hal`][embedded-hal-kw] keywords on crates.io!

[`embedded-hal-impl`]: https://crates.io/keywords/embedded-hal-impl
[embedded-hal-kw]: https://crates.io/keywords/embedded-hal

### OS

- [`linux-embedded-hal`] for embedded Linux systems like the Raspberry Pi. - ![crates.io](https://img.shields.io/crates/v/linux-embedded-hal.svg)

[`linux-embedded-hal`]: https://crates.io/crates/linux-embedded-hal

### Nordic

- [`nrf51-hal`](https://crates.io/crates/nrf51-hal) - ![crates.io](https://img.shields.io/crates/v/nrf51-hal.svg)

### NXP

Also check the list of [NXP board support crates][nxp-bsc]!

[nxp-bsc]: #nxp-1

- [`lpc82x-hal`](https://crates.io/crates/lpc82x-hal) - [![crates.io](https://img.shields.io/crates/v/lpc82x-hal.svg)](https://crates.io/crates/lpc82x-hal)

- [`mkw41z-hal`](https://crates.io/crates/mkw41z-hal) - ![crates.io](https://img.shields.io/crates/v/mkw41z-hal.svg)

### STMicroelectronics

Also check the list of [STMicroelectronics board support crates][stm-bsc]!

[stm-bsc]: #stmicroelectronics-2

- [`stm32f042-hal`](https://crates.io/crates/stm32f042-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f042-hal.svg)
  - Has examples that can run on boards like the [Nucleo-F042K6] and similar boards

[Nucleo-F042K6]: http://www.st.com/en/evaluation-tools/nucleo-f042k6.html

- [`stm32f103xx-hal`](https://github.com/japaric/stm32f103xx-hal)
  - Has examples that can run on boards like the [Blue pill], [Nucleo-F103RB] and similar boards

[Nucleo-F103RB]: http://www.st.com/en/evaluation-tools/nucleo-f103rb.html

- [`stm32f30x-hal`](https://crates.io/crates/stm32f30x-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f30x-hal.svg)

- [`stm32f429-hal`](https://crates.io/crates/stm32f429-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f429-hal.svg)

- [`stm32l151-hal`](https://crates.io/crates/stm32l151-hal) - ![crates.io](https://img.shields.io/crates/v/stm32l151-hal.svg)

- [`stm32l432xx-hal`](https://crates.io/crates/stm32l432xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32l432xx-hal.svg)
   - Has examples that can run on boards like the [Nucleo-L432KC] and similar boards

[Nucleo-L432KC]: https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-l432kc.html

### Texas Instruments

- [`tm4c123x-hal`](https://github.com/thejpster/tm4c123x-hal)

[Blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill

### Espressif

- [`esp8266-hal`](https://github.com/emosenkis/esp8266-hal) ![crates.io](https://img.shields.io/crates/v/esp8266-hal.svg) (not supported by rustc, so must be built with [mrustc](https://github.com/thepowersgang/mrustc), typically via the [esp-rs](https://github.com/emosenkis/esp-rs) build script)

## Architecture support crates

Crates tailored for general CPU architectures.

### ARM

- [`cortex-m`](https://github.com/japaric/cortex-m) Low level access to Cortex-M processors - ![crates.io](https://img.shields.io/crates/v/cortex-m.svg)
- [`cortex-a`](https://github.com/andre-richter/cortex-a) Low level access to Cortex-A processors (early state) - ![crates.io](https://img.shields.io/crates/v/cortex-a.svg)

## Board support crates

Crates tailored for specific boards.

[STM32F3DISCOVERY]: http://www.st.com/en/evaluation-tools/stm32f3discovery.html

### Nordic

- [`microbit`](https://crates.io/crates/microbit) - [micro:bit] - ![crates.io](https://img.shields.io/crates/v/microbit.svg)

[micro:bit]: http://microbit.org/

### NXP

- [`frdm-kw41z`](https://crates.io/crates/frdm-kw41z) - [FRDM-KW41Z] - ![crates.io](https://img.shields.io/crates/v/frdm-kw41z.svg)

[FRDM-KW41Z]: https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/kinetis-cortex-m-mcus/w-serieswireless-conn.m0-plus-m4/freedom-development-kit-for-kinetis-kw41z-31z-21z-mcus:FRDM-KW41Z

### STMicroelectronics

- [`nucleo-f042k6`](https://github.com/therealprof/nucleo-f042k6.git) - [Nucleo-F042K6]
- [`nucleo-f103rb`](https://github.com/therealprof/nucleo-f103rb.git) - [Nucleo-F103RB]
- [`f3`](https://crates.io/crates/f3) Board Support Crate for the [STM32F3DISCOVERY] - ![crates.io](https://img.shields.io/crates/v/f3.svg)
- [`blue-pill`](https://github.com/japaric/blue-pill) Board Support Crate for [Blue Pill].

### Texas Instruments

- [`stellaris-launchpad`](https://crates.io/crates/stellaris-launchpad) - For the Texas Instruments Stellaris Launchpad and Tiva-C Launchpad ![crates.io](https://img.shields.io/crates/v/stellaris-launchpad.svg)
- [`monotron`](https://github.com/thejpster/monotron) - A 1980s home-computer style application for the Texas Instruments Stellaris Launchpad. PS/2 keyboard input, text output on a bit-bashed 800x600 VGA signal. Uses [menu], [vga-framebuffer] and [pc-keyboard].

### Special Purpose

- [`betafpv-f3`](https://github.com/JoshMcguigan/betafpv-f3) - For the BetaFPV F3 drone flight controller

## Driver crates

Platform agnostic crates to interface external components. These crates use the [`embedded-hal`]
interface to support [all the devices and systems that implement the `embedded-hal`
traits][hal-impl].

[hal-impl]: #hal-implementation-crates

The list below contains drivers developed as part of the [Weekly Driver initiative][wd] and that
have achieved the "released" status (published on crates.io + documentation / short blog post).

[wd]: https://github.com/rust-lang-nursery/embedded-wg/issues/39

1. [L3GD20] - SPI - Gyroscope - [Intro blog post][1&2] - ![crates.io](https://img.shields.io/crates/v/l3gd20.svg)
2. [LSM303DLHC] - I2C - Accelerometer + compass (magnetometer) - [Intro blog post][1&2] - ![crates.io](https://img.shields.io/crates/v/lsm303dlhc.svg)
3. [MCP3008] - SPI - 8 channel 10-bit ADC - [Intro blog post][3] - ![crates.io](https://img.shields.io/crates/v/adc-mcp3008.svg)
4. [ENC28J60] - SPI - Ethernet controller - [Intro blog post][4] - ![crates.io](https://img.shields.io/crates/v/enc28j60.svg)
5. [MCP3425] - I2C - 16-bit ADC - [Intro blog post][5] - ![crates.io](https://img.shields.io/crates/v/mcp3425.svg)
6. [SGP30] - I2C - Gas sensor - [Intro blog post][6] - ![crates.io](https://img.shields.io/crates/v/sgp30.svg)
7. [HTS221] - I2C - Humidity and temperature sensor - [Intro blog post][7] - ![crates.io](https://img.shields.io/crates/v/hts221.svg)
8. [SSD1306] - I2C/SPI - OLED display controller - [Intro blog post][8] - ![crates.io](https://img.shields.io/crates/v/ssd1306.svg)
9. [MMA7660FC] - I2C - 3-axis accelerometer - [Intro blog post][9]
10. [shift-register-driver] - GPIO - Shift register - [Intro blog post][10] - ![crates.io](https://img.shields.io/crates/v/shift-register-driver.svg)
11. [TSL256X] - I2C - Light Intensity Sensor - [Intro blog post][11] - ![crates.io](https://img.shields.io/crates/v/tsl256x.svg)

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

[SSD1306]: https://crates.io/crates/ssd1306
[8]: https://wapl.es/electronics/rust/2018/04/30/ssd1306-driver.html

[MMA7660FC]: https://crates.io/crates/mma7660fc
[9]: https://rahul-thakoor.github.io/an-i2c-rust-driver-for-mma7660fc-based-3-axis-digital-accelerometer/

[shift-register-driver]: https://crates.io/crates/shift-register-driver
[10]: https://www.joshmcguigan.com/blog/shift-register-driver/

[TSL256X]: https://crates.io/crates/tsl256x
[11]: https://www.joshmcguigan.com/blog/tsl256x-light-intensity-sensor-driver/

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
- [INA260] - I2C - power monitor - ![crates.io](https://img.shields.io/crates/v/ina260.svg)
- [ILI9341] - SPI - TFT LCD display
- [HD44780] - Parallel port - LCD controller
- [HD44780-driver] - GPIO - LCD controller - ![crates.io](https://img.shields.io/crates/v/hd44780-driver.svg)
- [MCP9808] - I2C - Temperature sensor - ![crates.io](https://img.shields.io/crates/v/mcp9808.svg)
- [AXP209] - I2C - Power management unit
- [DS3234] - SPI - Real time clock
- [PCD8544] - SPI - 48x84 pixels matrix LCD controller
- [PCD8544_rich] - SPI - Rich driver for 48x84 pixels matrix LCD controller  - ![crates.io](https://img.shields.io/crates/v/pcd8544.svg)
- [HC-SR04] - DIO - Ultrasound sensor
- [AFE4400] - SPI - Pulse oximeter
- [SX1278] - SPI - Long range (LoRa) transceiver
- [RFM69] - SPI - ISM radio transceiver
- [LS010B7DH01] - SPI - Memory LCD
- [MAX31855] - SPI - Thermocouple digital converter
- [MAX31865] - SPI - RTD to Digital converter - ![crates.io](https://img.shields.io/crates/v/max31865.svg)
- [SHT3x] - I2C - Temperature / humidity sensors
- [SX1509] - I2C - IO Expander / Keypad driver
- [NRF24L01] - SPI - 2.4 GHz wireless communication
- [embedded-nrf24l01] - SPI+GPIO - 2.4 GHz radio
- [stm32-eth] - MCU - Ethernet
- [bme680] - I2C - Temperature / humidity / gas / pressure sensor - ![crates.io](https://img.shields.io/crates/v/bme680.svg)
- [w5500] - SPI - Ethernet Module with hardwired protocols : TCP, UDP, ICMP, IPv4, ARP, IGMP, PPPoE - ![crates.io](https://img.shields.io/crates/v/w5500.svg)
- [OneWire] - 1wire - OneWire protocol implementation with drivers for devices such as [DS18B20](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf) - ![crates.io](https://img.shields.io/crates/v/onewire.svg)
- [SSD1322] - SPI - Graphical OLED display controller - ![crates.io](https://img.shields.io/crates/v/ssd1322.svg)
- [GridEYE] - I2C - Rust driver for Grid-EYE / Panasonic AMG88(33)
- [EEPROM24x] - I2C - 24x series serial EEPROM driver ![crates.io](https://img.shields.io/crates/v/eeprom24x.svg)
- [DS1307] - I2C - Real-time clock driver - ![crates.io](https://img.shields.io/crates/v/ds1307.svg)
- [PCF857x] - I2C - I/O expanders: PCF8574, PCF8574A, PCF8575 ![crates.io](https://img.shields.io/crates/v/pcf857x.svg)
- [eink-waveshare] - SPI - driver for E-Paper Modules from Waveshare
- [BlueNRG] - SPI - driver for BlueNRG-MS Bluetooth module. ![crates.io](https://img.shields.io/crates/v/bluenrg.svg)

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
[ILI9341]: https://github.com/yuri91/ili9341-rs
[HD44780]: http://github.com/kunerd/clerk
[HD44780-driver]: https://crates.io/crates/hd44780-driver
[MCP9808]: https://crates.io/crates/mcp9808
[PCD8544]: https://github.com/pcein/pcd8544
[PCD8544_rich]: https://crates.io/crates/pcd8544
[AXP209]: https://github.com/RandomInsano/axp209-rs
[DS3234]: https://github.com/rust-lang-nursery/embedded-wg/issues/39#issuecomment-375262785
[HC-SR04]: https://github.com/nordmoen/hc-sr04
[AFE4400]: https://github.com/ReeceStevens/afe4400
[SX1278]: https://github.com/susu/sx1278
[RFM69]: https://github.com/lolzballs/rfm69
[LS010B7DH01]: https://github.com/byronwasti/ls010b7dh01
[MAX31855]: https://github.com/mbacch/max31855
[MAX31865]: https://crates.io/crates/max31865
[SHT3x]: https://github.com/miek/sht3x-rs
[SX1509]: https://github.com/wez/sx1509
[NRF24L01]: https://github.com/maikelwever/nrf24l01
[embedded-nrf24l01]: https://crates.io/crates/embedded-nrf24l01
[stm32-eth]: https://github.com/astro/stm32-eth
[bme680]: https://github.com/marcelbuesing/bme680
[w5500]: https://crates.io/crates/w5500
[OneWire]: https://crates.io/crates/onewire
[SSD1322]: https://crates.io/crates/ssd1322
[GridEYE]: https://github.com/uwearzt/grideye
[EEPROM24x]: https://crates.io/crates/eeprom24x
[DS1307]: https://crates.io/crates/ds1307
[PCF857x]: https://crates.io/crates/pcf857x
[eink-waveshare]: https://crates.io/crates/eink_waveshare_rs
[BlueNRG]: https://crates.io/crates/bluenrg

## no-std crates

[`#![no_std]` crates][no-std-category] designed to run on resource constrained devices.

- [bit_field](https://crates.io/crates/bit_field): manipulating bitfields and bitarrays - ![crates.io](https://img.shields.io/crates/v/bit_field.svg)
- [heapless](https://crates.io/crates/heapless): provides `Vec`, `String`, `LinearMap`, `RingBuffer` backed by fixed-size buffers  - ![crates.io](https://img.shields.io/crates/v/heapless.svg)
- [intrusive-collections](https://crates.io/crates/intrusive-collections): intrusive (non-allocating) singly/doubly linked lists and red-black trees - ![crates.io](https://img.shields.io/crates/v/intrusive-collections.svg)
- [managed](https://crates.io/crates/managed): provides `ManagedSlice`, `ManagedMap` backed by either their std counterparts or fixed-size buffers for `#![no_std]`. - ![crates.io](https://img.shields.io/crates/v/managed.svg)
- [nalgebra](https://crates.io/crates/nalgebra): general-purpose and low-dimensional linear algebra library - ![crates.io](https://img.shields.io/crates/v/nalgebra.svg)
- [smoltcp](https://github.com/m-labs/smoltcp): a small TCP/IP stack that runs without `alloc`
- [embedded-graphics](https://crates.io/crates/embedded-graphics): 2D drawing library for any size display - ![crates.io](https://img.shields.io/crates/v/embedded-graphics.svg)
- [scroll](https://crates.io/crates/scroll): extensible and endian-aware Read/Write traits for generic containers - ![crates.io](https://img.shields.io/crates/v/scroll.svg)
- [vga-framebuffer]: A VGA signal generator and font renderer for VGA-less microcontrollers. Used by [Monotron](https://github.com/thejpster/monotron) to generate 48 by 36 character display using 3 SPI peripherals and a timer.
- [menu]: A basic command-line interface library. Has nested menus and basic help functionality.
- [pc-keyboard]: A PS/2 keyboard protocol driver. Transport (bit-banging or SPI) agnostic, but can convert Set 2 Scancodes into Unicode.
- [console-traits](https://github.com/thejpster/console-traits): Describes a basic text console. Used by [menu] and implemented by [vga-framebuffer].
- [register-rs](https://github.com/rust-osdev/register-rs): Unified interface for MMIO and CPU registers. Provides type-safe bitfield manipulation. - ![crates.io](https://img.shields.io/crates/v/register.svg)

[pc-keyboard]: https://github.com/thejpster/pc-keyboard
[vga-framebuffer]: https://github.com/thejpster/vga-framebuffer-rs
[menu]: https://github.com/thejpster/menu

### WIP

Work in progress crates. Help the authors make these crates awesome!

- [light-cli](https://github.com/rudihorn/light-cli): a lightweight heapless cli interface
- [bluetooth-hci](https://crates.io/crates/bluetooth-hci): device-independent Bluetooth Host-Controller Interface implementation.

[no-std-category]: https://crates.io/categories/no-std

## Rust forks

### AVR
-  [AVR Rust](https://github.com/avr-rust/rust) Fork of Rust with AVR support.

## Firmware projects
- [anne-key](https://github.com/ah-/anne-key): Alternate keyboard firmware for the Obins ANNE Pro

## License

This list is licensed under

- CC0 1.0 Universal License ([LICENSE-CC0](LICENSE-CC0) or
  https://creativecommons.org/publicdomain/zero/1.0/legalcode)

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], the maintainer of this crate, the [HAL team][team], promises
to intervene to uphold that code of conduct.

[CoC]: CODE_OF_CONDUCT.md
[team]: https://github.com/rust-embedded/wg#the-resources-team
