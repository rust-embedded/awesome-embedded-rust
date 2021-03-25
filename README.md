# Embedded Rust

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

This is a curated list of resources related to embedded and low-level programming in the programming language Rust, including a list of useful crates.

[<img src="https://rawgit.com/rust-embedded/awesome-embedded-rust/master/rust-embedded-logo-256x256.png" align="right" width="256">](http://www.rust-embedded.org)

This project is developed and maintained by the [Resources team][team].

**Don't see something you want or need here?** Add it to the [Not Yet Awesome Embedded Rust](https://github.com/rust-embedded/not-yet-awesome-embedded-rust) list!

## Table of contents

- [Embedded Rust](#embedded-rust)
  - [Table of contents](#table-of-contents)
  - [Community](#community)
    - [Community Chat Rooms](#community-chat-rooms)
  - [Books, blogs and training materials](#books-blogs-and-training-materials)
  - [Tools](#tools)
  - [Real-time](#real-time)
    - [Real-time Operating System (RTOS)](#real-time-operating-system-rtos)
    - [Real-time tools](#real-time-tools)
  - [Peripheral Access Crates](#peripheral-access-crates)
    - [Microchip](#microchip)
    - [Nordic](#nordic)
    - [NXP](#nxp)
    - [SiFive](#sifive)
    - [Silicon Labs](#silicon-labs)
    - [STMicroelectronics](#stmicroelectronics)
    - [Texas Instruments](#texas-instruments)
    - [MSP430](#msp430)
    - [Ambiq Micro](#ambiq-micro)
    - [GigaDevice](#gigadevice)
    - [XMC](#xmc)
  - [HAL implementation crates](#hal-implementation-crates)
    - [OS](#os)
    - [Microchip](#microchip-1)
    - [Nordic](#nordic-1)
    - [NXP](#nxp-1)
    - [SiFive](#sifive-1)
    - [STMicroelectronics](#stmicroelectronics-1)
    - [Texas Instruments](#texas-instruments-1)
    - [MSP430](#msp430-1)
    - [Espressif](#espressif)
    - [Silicon Labs](#silicon-labs-1)
    - [XMC](#xmc-1)
    - [GigaDevice](#gigadevice-1)
  - [Architecture support crates](#architecture-support-crates)
    - [ARM](#arm)
    - [RISC-V](#risc-v)
    - [MIPS](#mips)
  - [Board support crates](#board-support-crates)
    - [1BitSquared](#1bitsquared)
    - [Adafruit](#adafruit)
    - [Arduino](#arduino)
    - [Nordic](#nordic-2)
    - [NXP](#nxp-2)
    - [Sparkfun](#sparkfun)
    - [SeeedStudio](#seeedstudio)
    - [SiFive](#sifive-2)
    - [Sipeed](#sipeed)
    - [Sony](#sony)
    - [STMicroelectronics](#stmicroelectronics-2)
    - [Teensy](#teensy)
    - [Texas Instruments](#texas-instruments-2)
    - [Special Purpose](#special-purpose)
  - [Component abstraction crates](#component-abstraction-crates)
  - [Driver crates](#driver-crates)
    - [WIP](#wip)
  - [no-std crates](#no-std-crates)
    - [WIP](#wip-1)
  - [Firmware projects](#firmware-projects)
  - [Old books, blogs and training materials](#old-books-blogs-and-training-materials)
  - [License](#license)
  - [Code of Conduct](#code-of-conduct)

## Community

In 2018 the Rust community created an embedded working group to help drive adoption in the Rust ecosystem.

- [Embedded WG](https://github.com/rust-embedded/wg/), including newsletters with progress updates.

### Community Chat Rooms

- You can usually find community members (including embedded WG members) in the official [`#rust-embedded:matrix.org` Matrix room].
- [embedded.rs] - Telegram chat about Rust for microcontrollers in the Russian language.
- [#esp-rs:matrix.org] - For discussion of using Embedded Rust on Xtensa devices
- [#nrf-rs:matrix.org] - For discussion of using Embedded Rust on Nordic Semiconductor devices
- [#probe-rs:matrix.org] - For discussion of the Probe-rs debugging toolkit
- [#rtic-rs:matrix.org] - For discussion of the Real-Time Interrupt-driven Concurrency framework
- [#rust-embedded-graphics:matrix.org] - For discussion of the [`embedded-graphics`] crate and ecosystem

[#rust-embedded-graphics:matrix.org]: https://matrix.to/#/#rust-embedded-graphics:matrix.org
[#esp-rs:matrix.org]: https://matrix.to/#/#esp-rs:matrix.org
[`#rust-embedded:matrix.org` Matrix room]: https://matrix.to/#/#rust-embedded:matrix.org
[embedded.rs]: https://t.me/embedded_rs
[#rtic-rs:matrix.org]: https://matrix.to/#/#rtic-rs:matrix.org
[#nrf-rs:matrix.org]: https://matrix.to/#/#nrf-rs:matrix.org
[#probe-rs:matrix.org]: https://matrix.to/#/#probe-rs:matrix.org
[`embedded-graphics`]: https://crates.io/crates/embedded-graphics

## Books, blogs and training materials

-   [The Embedded Rust Book](https://rust-embedded.github.io/book/) - An introductory book about using the Rust Programming Language on "Bare Metal" embedded systems, such as Microcontrollers.
-   [Discovery](https://rust-embedded.github.io/discovery) by @rust-embedded — this book is an introductory course on microcontroller-based embedded systems that uses Rust as the teaching language. Original author: @japaric
-   [Cortex-M Quickstart](https://docs.rs/cortex-m-quickstart/0.3.1/cortex_m_quickstart/) by @japaric – a template and introduction to embedded Rust, suitable for developers familiar to embedded development but new to embedded Rust.
- [Writing an OS in rust](https://os.phil-opp.com/) A blog series creating a small operating system in Rust
-   [MicroRust](https://droogmic.github.io/microrust/) Introductory book for embedded development in Rust on the micro:bit.
-   [Physical Computing With Rust](https://rahul-thakoor.github.io/physical-computing-rust/) A (WIP) guide to physical computing with Rust on the Raspberry Pi.
-   [Writing an embedded OS in Rust on the Raspberry Pi](https://github.com/rust-embedded/rust-raspi3-OS-tutorials) A set of tutorials that give a guided, step-by-step tour of how to write a monolithic Operating System kernel for an embedded system from scratch. Runs on the Raspberry Pi 3 and the Raspberry Pi 4.
-   [Writing embedded drivers in Rust isn't that hard](https://hboeving.dev/blog/rust-2c-driver-p1/) A guide to building an embedded-hal driver. [Part 2](https://hboeving.dev/blog/rust-i2c-driver-p2/)
-   [Ferrous Systems' Embedded Training Courses: 2020-current edition](https://github.com/ferrous-systems/embedded-trainings-2020) A hands-on training course for beginner and advanced learners of Embedded Rust, based on Nordic Semiconductor's nRF52840 harware. This training was given at Oxidize Conferences and by [Ferrous Systems] to corporate customers.
-   [Ferrous Systems' Knurling Sessions](https://knurling.ferrous-systems.com/sessions/) are hands-on embedded projects that explore specific concepts using generally available hardware, building full systems and components using microcontrollers, sensors, and actuators.

[Ferrous Systems]: https://ferrous-systems.com

## Tools

-   [xargo](https://github.com/japaric/xargo) Rust package manager with support for non-default std libraries — build rust runtime for your own embedded system.
    - xargo is great but it's in maintenance mode, [cargo-xbuild](https://github.com/rust-osdev/cargo-xbuild) is catching up as intended replacement.
-   [svd2rust](https://github.com/japaric/svd2rust) Generate Rust structs with register mappings from SVD files.
-   [embedded-hal-mock] Mock implementation of `embedded-hal` traits for testing without accessing real hardware. - ![crates.io](https://img.shields.io/crates/v/embedded-hal-mock.svg)
-   [bindgen](https://crates.io/crates/bindgen) Automatically generates Rust FFI bindings to C and C++ libraries. - ![crates.io](https://img.shields.io/crates/v/bindgen.svg)
-   [cortex-m semihosting](https://github.com/japaric/cortex-m-semihosting) Semihosting for ARM Cortex-M processors
-   [bobbin-cli](https://github.com/bobbin-rs/bobbin-cli) A Rust command line tool to simplify embedded development and deployment.
-   [cargo-fel4](https://github.com/maindotrs/cargo-fel4) A Cargo subcommand for working with feL4 projects. - ![crates.io](https://img.shields.io/crates/v/cargo-fel4.svg)
-   [cargo-flash](https://probe.rs/guide/1_tools/cargo-flash/) A small cargo subcommand to download your binary to your target chip. - ![crates.io](https://img.shields.io/crates/v/cargo-flash.svg)
-   [cargo-embed](https://probe.rs/guide/1_tools/cargo-embed/) A superset of cargo-flash with additional useful features like configuration file support, a RTT terminal or a GDB server. - ![crates.io](https://img.shields.io/crates/v/cargo-embed.svg)
-   [cargo-hf2](https://github.com/jacobrosenthal/hf2-rs)  A small cargo subcommand to download cargo builds to Microsoft UF2 bootloaders via HID USB . - ![crates.io](https://img.shields.io/crates/v/cargo-hf2.svg)
-   [uf2](https://github.com/sajattack/uf2conv-rs) Converts binary files to Microsoft's UF2 format for copying over to mass storage device uf2 bootloaders - ![crates.io](https://img.shields.io/crates/v/uf2.svg)
-   [Knurling Tools](https://knurling.ferrous-systems.com/tools/) are developed by [Ferrous Systems] to ease the development process for building, debugging, and testing embedded Rust systems. These tools include:
    - [Probe Run](https://github.com/knurling-rs/probe-run): a cargo runner to flash and run embedded applications just like you would native applications, including backtraces and panicking behavior
    - [defmt](https://github.com/knurling-rs/defmt): a highly efficient logging framework that targets resource-constrained devices, like microcontrollers.
    - [flip-link](https://github.com/knurling-rs/flip-link), a linker wrapper that provides stack overflow protection without an MMU by flipping the standard memory layout of ARM Cortex-M programs
    - [app-template](https://github.com/knurling-rs/app-template), a `cargo-generate` powered project template for quickly setting up new projects using the Knurling Tools.
    - [defmt-test](https://github.com/knurling-rs/defmt-test), an embedded test harness that lets you write and run unit tests as if you were using the built-in `#[test]` attribute, but will run on an embedded target

[embedded-hal-mock]: https://crates.io/crates/embedded-hal-mock

## Real-time

### Real-time Operating System (RTOS)

-   [Drone OS](https://drone-os.github.io) An Embedded Operating System for writing real-time applications in Rust.
-   [FreeRTOS.rs](https://github.com/hashmismatch/freertos.rs) Rust interface for the FreeRTOS API
-   [FreeRTOS-rust](https://github.com/lobaro/FreeRTOS-rust) Rust interface for FreeRTOS with Rust entry point and build support crate.
-   [Tock](https://www.tockos.org) An embedded operating system designed for running multiple concurrent, mutually distrustful applications on low-memory and low-power microcontrollers

### Real-time tools

-   [RTIC v0.5](https://rtic.rs/0.5/book/en/) Real-Time Interrupt-driven Concurrency — A concurrency framework for building real time systems:
    -   [cortex-m rtic](https://github.com/rtic-rs/cortex-m-rtic) RTIC framework for ARM Cortex-M microcontrollers
    -   [msp430 rtfm](https://github.com/japaric/msp430-rtfm) RTFM framework for MSP430 MCUs

## Peripheral Access Crates

Register definition for microcontroller families. Usually generated using [`svd2rust`]. - ![crates.io](https://img.shields.io/crates/v/svd2rust.svg)

Peripheral Access Crates were also called Device Crates.

[`svd2rust`]: https://crates.io/crates/svd2rust

*NOTE* You may be able to find even more peripheral access crates by searching for the
[`svd2rust`][svd2rust-kw] keyword on crates.io!

[svd2rust-kw]: https://crates.io/keywords/svd2rust

### Microchip

- [`atsamd11`](https://github.com/atsamd-rs/atsamd) Peripheral access API for Microchip (formerly Atmel) SAMD11 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`atsamd21`](https://github.com/atsamd-rs/atsamd) Peripheral access API for Microchip (formerly Atmel) SAMD21 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`atsamd51`](https://github.com/atsamd-rs/atsamd) Peripheral access API for Microchip (formerly Atmel) SAMD51 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`atsame54`](https://github.com/atsamd-rs/atsamd) Peripheral access API for Microchip (formerly Atmel) SAME54 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`avr-device`](https://github.com/Rahix/avr-device) Peripheral access API for Microchip (formerly Atmel) AVR microcontroller family.
- [`sam3x8e`](https://crates.io/crates/sam3x8e) Peripheral access API for Atmel SAMD3X8E microcontrollers (generated using svd2rust)  - ![crates.io](https://img.shields.io/crates/v/sam3x8e.svg)

### Nordic

- [`nrf51`](https://crates.io/crates/nrf51) Peripheral access API for nRF51 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf51.svg)
- [`nrf52810-pac`](https://crates.io/crates/nrf52810-pac) - Peripheral access API for the nRF52810 microcontroller (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf52810-pac.svg)
- [`nrf52811-pac`](https://crates.io/crates/nrf52811-pac) - Peripheral access API for the nRF52811 microcontroller (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf52811-pac.svg)
- [`nrf52832-pac`](https://crates.io/crates/nrf52832-pac) - Peripheral access API for the nRF52832 microcontroller (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf52832-pac.svg)
- [`nrf52833-pac`](https://crates.io/crates/nrf52833-pac) - Peripheral access API for the nRF52833 microcontroller (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf52833-pac.svg)
- [`nrf52840-pac`](https://crates.io/crates/nrf52840-pac) - Peripheral access API for the nRF52840 microcontroller (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf52840-pac.svg)
- [`nrf5340-app-pac`](https://crates.io/crates/nrf5340-app-pac) - Peripheral access API for the nRF5340 application core (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf5340-app-pac.svg)
- [`nrf5340-net-pac`](https://crates.io/crates/nrf5340-net-pac) - Peripheral access API for the nRF5340 network core (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf5340-net-pac.svg)
- [`nrf9160-pac`](https://crates.io/crates/nrf9160-pac) - Peripheral access API for the nRF9160 system-in-package (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/nrf9160-pac.svg)

### NXP

- [`k64`](https://crates.io/crates/k64) - ![crates.io](https://img.shields.io/crates/v/k64.svg)
- [`lpc11uxx`](https://crates.io/crates/lpc11uxx) - ![crates.io](https://img.shields.io/crates/v/lpc11uxx.svg)
- [`lpc55s6x-pac`](https://crates.io/crates/lpc55s6x-pac) - ![crates.io](https://img.shields.io/crates/v/lpc55s6x-pac.svg)
- [`lpc82x-pac`](https://crates.io/crates/lpc82x-pac) - ![crates.io](https://img.shields.io/crates/v/lpc82x-pac.svg)
- [`lpc845-pac`](https://crates.io/crates/lpc845-pac) - ![crates.io](https://img.shields.io/crates/v/lpc845-pac.svg)
- [`mkw41z`](https://crates.io/crates/mkw41z) - ![crates.io](https://img.shields.io/crates/v/mkw41z.svg)
- [`imxrt-ral`](https://github.com/imxrt-rs/imxrt-rs) Register access layer for i.MX RT series. -  ![crates.io](https://img.shields.io/crates/v/imxrt-ral.svg)


### SiFive

- [`e310x`](https://github.com/riscv-rust/e310x) - svd2rust generated interface to SiFive [Freedom E310](https://www.sifive.com/cores/e31) MCUs - ![crates.io](https://img.shields.io/crates/v/e310x.svg)

### Silicon Labs

- [`efm32pg12-pac`](https://crates.io/crates/efm32pg12-pac) - Peripheral access API for Silicon Labs EFM32PG12 microcontrollers - ![crates.io](https://img.shields.io/crates/v/efm32pg12-pac)

### STMicroelectronics

The [`stm32-rs`](https://github.com/stm32-rs/stm32-rs) project has peripheral access APIs for most STM32 microcontrollers (generated using svd2rust):

- [`stm32f0`](https://crates.io/crates/stm32f0) - ![crates.io](https://img.shields.io/crates/v/stm32f0.svg)
- [`stm32f1`](https://crates.io/crates/stm32f1) - ![crates.io](https://img.shields.io/crates/v/stm32f1.svg)
- [`stm32f2`](https://crates.io/crates/stm32f2) - ![crates.io](https://img.shields.io/crates/v/stm32f2.svg)
- [`stm32f3`](https://crates.io/crates/stm32f3) - ![crates.io](https://img.shields.io/crates/v/stm32f3.svg)
- [`stm32f4`](https://crates.io/crates/stm32f4) - ![crates.io](https://img.shields.io/crates/v/stm32f4.svg)
- [`stm32f7`](https://crates.io/crates/stm32f7) - ![crates.io](https://img.shields.io/crates/v/stm32f7.svg)
- [`stm32g0`](https://crates.io/crates/stm32g0) - ![crates.io](https://img.shields.io/crates/v/stm32g0.svg)
- [`stm32g4`](https://crates.io/crates/stm32g4) - ![crates.io](https://img.shields.io/crates/v/stm32g4.svg)
- [`stm32h7`](https://crates.io/crates/stm32h7) - ![crates.io](https://img.shields.io/crates/v/stm32h7.svg)
- [`stm32l0`](https://crates.io/crates/stm32l0) - ![crates.io](https://img.shields.io/crates/v/stm32l0.svg)
- [`stm32l1`](https://crates.io/crates/stm32l1) - ![crates.io](https://img.shields.io/crates/v/stm32l1.svg)
- [`stm32l4`](https://crates.io/crates/stm32l4) - ![crates.io](https://img.shields.io/crates/v/stm32l4.svg)

### Texas Instruments

-   [`tm4c123x`](https://crates.io/crates/tm4c123x) Peripheral access API for TM4C123x microcontrollers (generated using svd2rust)
-   [`tm4c129x`](https://crates.io/crates/tm4c129x) Peripheral access API for TM4C129x microcontrollers (generated using svd2rust)

### MSP430

-   [`msp430g2553`](https://github.com/japaric/msp430g2553) Peripheral access API for MSP430G2553 microcontrollers (generated using svd2rust)
    - [msp430 quickstart](https://github.com/japaric/msp430-quickstart) some examples for msp430
-   [`msp430fr2355`](https://crates.io/crates/msp430fr2355) Peripheral access API for MSP430FR2355 microcontrollers (generated using svd2rust)

### Ambiq Micro

- [`ambiq-apollo1-pac`](https://crates.io/crates/ambiq-apollo1-pac) Peripheral access API for Ambiq Apollo 1 microcontrollers (generated using svd2rust)
- [`ambiq-apollo2-pac`](https://crates.io/crates/ambiq-apollo2-pac) Peripheral access API for Ambiq Apollo 2 microcontrollers (generated using svd2rust)
- [`ambiq-apollo3-pac`](https://crates.io/crates/ambiq-apollo3-pac) Peripheral access API for Ambiq Apollo 3 microcontrollers (generated using svd2rust)
- [`ambiq-apollo3p-pac`](https://crates.io/crates/ambiq-apollo3p-pac) Peripheral access API for Ambiq Apollo 3 Plus microcontrollers (generated using svd2rust)

### GigaDevice

- [`gd32vf103-pac`](https://github.com/riscv-rust/gd32vf103-pac) Peripheral access API for GD32VF103 RISC-V microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/gd32vf103-pac.svg)

### XMC

Peripheral access crates for the different XMC4xxx families of microcontrollers

- [`xmc4100`](https://github.com/xmc-rs/xmc4100) - ![crates.io](https://img.shields.io/crates/v/xmc4100.svg)
- [`xmc4200`](https://github.com/xmc-rs/xmc4200) - ![crates.io](https://img.shields.io/crates/v/xmc4200.svg)
- [`xmc4300`](https://github.com/xmc-rs/xmc4300) - ![crates.io](https://img.shields.io/crates/v/xmc4300.svg)
- [`xmc4400`](https://github.com/xmc-rs/xmc4400) - ![crates.io](https://img.shields.io/crates/v/xmc4400.svg)
- [`xmc4500`](https://github.com/xmc-rs/xmc4500) - ![crates.io](https://img.shields.io/crates/v/xmc4500.svg)
- [`xmc4700`](https://github.com/xmc-rs/xmc4700) - ![crates.io](https://img.shields.io/crates/v/xmc4700.svg)
- [`xmc4800`](https://github.com/xmc-rs/xmc4800) - ![crates.io](https://img.shields.io/crates/v/xmc4800.svg)

## HAL implementation crates

Implementations of [`embedded-hal`] for microcontroller families and systems running some OS. - ![crates.io](https://img.shields.io/crates/v/embedded-hal.svg)

[`embedded-hal`]: https://crates.io/crates/embedded-hal

*NOTE* You may be able to find even more HAL implementation crates by searching for the
[`embedded-hal-impl`] and [`embedded-hal`][embedded-hal-kw] keywords on crates.io!

[`embedded-hal-impl`]: https://crates.io/keywords/embedded-hal-impl
[embedded-hal-kw]: https://crates.io/keywords/embedded-hal

### OS

- [`bitbang-hal`] software protocol implementations for microcontrollers with digital::OutputPin and digital::InputPin
- [`ftdi-embedded-hal`] for FTDI FTx232H chips connected to Linux systems via USB
- [`linux-embedded-hal`] for embedded Linux systems like the Raspberry Pi. - ![crates.io](https://img.shields.io/crates/v/linux-embedded-hal.svg)

[`bitbang-hal`]: https://crates.io/crates/bitbang-hal
[`ftdi-embedded-hal`]: https://github.com/geomatsi/ftdi-embedded-hal
[`linux-embedded-hal`]: https://crates.io/crates/linux-embedded-hal

### Microchip

- [`atsamd-hal`](https://crates.io/crates/atsamd-hal) - HAL for SAMD11, SAMD21, SAMD51 and SAME54 - ![crates.io](https://img.shields.io/crates/v/atsamd-hal.svg)
- [`avr-hal`](https://github.com/Rahix/avr-hal) - HAL for AVR microcontroller family and AVR-based boards

### Nordic

- [`nrf51-hal`](https://crates.io/crates/nrf51-hal) - ![crates.io](https://img.shields.io/crates/v/nrf51-hal.svg)
- [`nrf52810-hal`](https://crates.io/crates/nrf52810-hal) - ![crates.io](https://img.shields.io/crates/v/nrf52810-hal.svg)
- [`nrf52811-hal`](https://crates.io/crates/nrf52811-hal) - ![crates.io](https://img.shields.io/crates/v/nrf52811-hal.svg)
- [`nrf52832-hal`](https://crates.io/crates/nrf52832-hal) - ![crates.io](https://img.shields.io/crates/v/nrf52832-hal.svg)
- [`nrf52833-hal`](https://crates.io/crates/nrf52833-hal) - ![crates.io](https://img.shields.io/crates/v/nrf52833-hal.svg)
- [`nrf52840-hal`](https://crates.io/crates/nrf52840-hal) - ![crates.io](https://img.shields.io/crates/v/nrf52840-hal.svg)
- [`nrf9160-hal`](https://crates.io/crates/nrf9160-hal) - ![crates.io](https://img.shields.io/crates/v/nrf9160-hal.svg)

### NXP

Also check the list of [NXP board support crates][nxp-bsc]!

[nxp-bsc]: #nxp-2

- [`lpc55s6x-hal`](https://crates.io/crates/lpc55s6x-hal) - [![crates.io](https://img.shields.io/crates/v/lpc55s6x-hal.svg)](https://crates.io/crates/lpc55s6x-hal)
- [`lpc8xx-hal`](https://crates.io/crates/lpc8xx-hal) - HAL for lpc82x and lpc845 - [![crates.io](https://img.shields.io/crates/v/lpc8xx-hal.svg)](https://crates.io/crates/lpc8xx-hal)
- [`mkw41z-hal`](https://crates.io/crates/mkw41z-hal) - ![crates.io](https://img.shields.io/crates/v/mkw41z-hal.svg)
- [`imxrt-hal`](https://github.com/imxrt-rs/imxrt-rs) - HAL for i.MX RT series. -  ![crates.io](https://img.shields.io/crates/v/imxrt-hal.svg)


### SiFive

- [`e310x-hal`](https://github.com/riscv-rust/e310x-hal) - HAL for SiFive [Freedom E310](https://www.sifive.com/cores/e31) MCUs - ![crates.io](https://img.shields.io/crates/v/e310x-hal.svg)

### STMicroelectronics

Also check the list of [STMicroelectronics board support crates][stm-bsc]!

[stm-bsc]: #stmicroelectronics-2

- [`stm32f0xx-hal`](https://crates.io/crates/stm32f0xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f0xx-hal.svg)
  - Has examples that can run on boards like the [Nucleo-F042K6] and similar boards

[Nucleo-F042K6]: http://www.st.com/en/evaluation-tools/nucleo-f042k6.html

- [`stm32f1xx-hal`](https://github.com/stm32-rs/stm32f1xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f1xx-hal.svg)
  - Can be run on boards like the [Blue pill], [Nucleo-F103RB] and similar boards

[Blue pill]: https://stm32duinoforum.com/forum/wiki_subdomain/index_title_Blue_Pill.html
[Nucleo-F103RB]: http://www.st.com/en/evaluation-tools/nucleo-f103rb.html

- [`stm32f3xx-hal`](https://crates.io/crates/stm32f3xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f3xx-hal.svg)
- [`stm32f4xx-hal`](https://crates.io/crates/stm32f4xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f4xx-hal.svg)
   - Generic HAL implementation for all MCUs of the stm32f4 series
- [`stm32f7xx-hal`](https://crates.io/crates/stm32f7xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f7xx-hal.svg)
   - Generic HAL implementation for all MCUs of the stm32f7 series
- [`stm32g0xx-hal`](https://crates.io/crates/stm32g0xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32g0xx-hal.svg)
- [`stm32h7xx-hal`](https://crates.io/crates/stm32h7xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32h7xx-hal.svg)
    - HAL implementation for the STMicro STM32H7xx family of microcontrollers
- [`stm32l0xx-hal`](https://crates.io/crates/stm32l0xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32l0xx-hal.svg)
   - HAL implementation for the the STMicro STM32L0xx family of microcontrollers
- [`stm32l1xx-hal`](https://crates.io/crates/stm32l1xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32l1xx-hal.svg)
- [`stm32l151-hal`](https://crates.io/crates/stm32l151-hal) - ![crates.io](https://img.shields.io/crates/v/stm32l151-hal.svg)
- [`stm32l4xx-hal`](https://crates.io/crates/stm32l4xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32l4xx-hal.svg)
   - Generic hal support for stm32l4 devices, has examples that can run on boards like the [Nucleo-L432KC], [Solo], and similar boards
- [`stm32-hal`](https://crates.io/crates/stm32-hal2) - ![crates.io](https://img.shields.io/crates/v/stm32-hal2.svg)
   - HAL implementation for STM32 devices across multiple families, with a focus on newer ones like L4, L5, and H7

[Nucleo-L432KC]: https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-l432kc.html
[Solo]: https://solokeys.com/

### Texas Instruments

- [`tm4c123x-hal`](https://github.com/thejpster/tm4c123x-hal)

### MSP430

- [`msp430fr2x5x-hal`](https://crates.io/crates/msp430fr2x5x-hal)
    - HAL implementation for the MSP430FR2x5x family of microcontrollers

### Espressif

- [`rust-xtensa`](https://github.com/MabezDev/rust-xtensa)
  - rust fork enables projects to be built for the ESP32 and ESP8266. ([quick start repo](https://github.com/MabezDev/xtensa-rust-quickstart)).

### Silicon Labs

- [`tomu-hal`](https://github.com/fudanchii/imtomu-rs)
   - HAL implementation targeted for [Tomu] USB board with EFM32HG309F64 ARMv6-M core. Has support to configure [tomu bootloader] directly from application via `toboot_config` macro.

[Tomu]: https://tomu.im/
[tomu bootloader]: https://github.com/im-tomu/tomu-bootloader

### XMC

- [`xmc1100-hal`](https://github.com/david-sawatzke/xmc1100-hal) - ![crates.io](https://img.shields.io/crates/v/xmc1100-hal.svg)
- [`xmc4-hal`](https://github.com/xmc-rs/xmc4-hal) - ![crates.io](https://img.shields.io/crates/v/xmc4-hal.svg)

### GigaDevice

- [`gd32vf103xx-hal`](https://github.com/riscv-rust/gd32vf103xx-hal) - ![cratex.io](https://img.shields.io/crates/v/gd32vf103xx-hal.svg)
  - HAL for GD32VF103xx microcontrollers
- [`gd32vf103-hal`](https://github.com/luojia65/gd32vf103-hal) - ![crates.io](https://img.shields.io/crates/v/gd32vf103-hal.svg)
  - (WIP) Hardware abstract layer (HAL) for the GD32VF103 RISC-V microcontroller

## Architecture support crates

Crates tailored for general CPU architectures.

### ARM

- [`cortex-a`](https://github.com/andre-richter/cortex-a) Low level access to Cortex-A processors (early state) - ![crates.io](https://img.shields.io/crates/v/cortex-a.svg)
- [`cortex-m`](https://github.com/japaric/cortex-m) Low level access to Cortex-M processors - ![crates.io](https://img.shields.io/crates/v/cortex-m.svg)

### RISC-V

- [`riscv`](https://github.com/rust-embedded/riscv) Low level access to RISC-V processors - ![crates.io](https://img.shields.io/crates/v/riscv.svg)

### MIPS

- [`mips`](https://github.com/Harry-Chen/rust-mips) Low level access to MIPS32 processors - ![crates.io](https://img.shields.io/crates/v/mips.svg)

## Board support crates

Crates tailored for specific boards.

[STM32F3DISCOVERY]: http://www.st.com/en/evaluation-tools/stm32f3discovery.html
[STM32F4DISCOVERY]: https://www.st.com/en/evaluation-tools/stm32f4discovery.html
[STM32F429DISCOVERY]: https://www.st.com/en/evaluation-tools/32f429idiscovery.html

### 1BitSquared

- [`onebitsy`](https://crates.io/crates/onebitsy) - Board support crate for the [1bitsy] STM32F4-based board - ![crates.io](https://img.shields.io/crates/v/onebitsy.svg)

[1bitsy]: https://1bitsy.org/

### Adafruit

- [`metro_m0`](https://crates.io/crates/metro_m0) - Board support for the [Metro M0 board](https://www.adafruit.com/product/3505)
 ![crates.io](https://img.shields.io/crates/v/metro_m0.svg)
- [`metro_m4`](https://crates.io/crates/metro_m4) - Board support for the [Metro M4 board](https://www.adafruit.com/product/3382)
 ![crates.io](https://img.shields.io/crates/v/metro_m4.svg)
- [`pyportal`](https://crates.io/crates/pyportal) - Board support for the [PyPortal board](https://www.adafruit.com/product/4116)
 ![crates.io](https://img.shields.io/crates/v/pyportal.svg)
- [`pygamer`](https://crates.io/crates/pygamer) - Board support for the [PyGamer board](https://www.adafruit.com/product/4242)
 ![crates.io](https://img.shields.io/crates/v/pygamer.svg)
- [`trellis_m4`](https://crates.io/crates/trellis_m4) - Board support for the [NeoTrellis M4 board](https://www.adafruit.com/product/3938)
 ![crates.io](https://img.shields.io/crates/v/trellis_m4.svg)
- [`feather-f405`](https://crates.io/crates/feather-f405) - Board support for the [Feather STM32F405 Express](https://www.adafruit.com/product/4382) ![crates.io](https://img.shields.io/crates/v/feather-f405.svg)
- [`feather_m0`](https://crates.io/crates/feather_m0) - Board support for the [Feather M0 board](https://www.adafruit.com/product/2772) ![crates.io](https://img.shields.io/crates/v/feather_m0.svg)
- [`feather_m4`](https://crates.io/crates/feather_m4) - Board support for the [Feather M4 board](https://www.adafruit.com/product/3857)
 ![crates.io](https://img.shields.io/crates/v/feather_m4.svg)
- [`circuit_playground_express`](https://crates.io/crates/circuit_playground_express) - Board support for the [Circuit Playground Express board](https://www.adafruit.com/product/3333)
 ![crates.io](https://img.shields.io/crates/v/circuit_playground_express.svg)
- [`edgebadge`](https://crates.io/crates/edgebadge) - Board support for the [EdgeBadge board](https://www.adafruit.com/product/4400)
 ![crates.io](https://img.shields.io/crates/v/edgebadge.svg)
- [`gemma_m0`](https://crates.io/crates/gemma_m0) - Board support for the [Gemma M0 board](https://www.adafruit.com/product/3501)
 ![crates.io](https://img.shields.io/crates/v/gemma_m0.svg)
- [`itsybitsy_m0`](https://crates.io/crates/itsybitsy_m0) - Board support for the [ItsyBitsy M0 board](https://www.adafruit.com/product/3727)
 ![crates.io](https://img.shields.io/crates/v/itsybitsy_m0.svg)
- [`itsybitsy_m4`](https://crates.io/crates/itsybitsy_m4) - Board support for the [ItsyBitsy M4 Express board](https://www.adafruit.com/product/3800)
 ![crates.io](https://img.shields.io/crates/v/itsybitsy_m4.svg)
- [`trinket_m0`](https://crates.io/crates/trinket_m0) - Board support for the [Trinket M0 board](https://www.adafruit.com/product/3500)
 ![crates.io](https://img.shields.io/crates/v/trinket_m0.svg)

### Arduino

- [`avr-hal`](https://github.com/Rahix/avr-hal) - Board support crate for several AVR-based boards including the Arduino Uno and the Arduino Leonardo
- [`arduino_mkrvidor4000`](https://crates.io/crates/arduino_mkrvidor4000) - Board support for the [MKR Vidor board](https://store.arduino.cc/usa/mkr-vidor-4000) ![crates.io](https://img.shields.io/crates/v/arduino_mkrvidor4000.svg)
- [`arduino_mkrzero`](https://crates.io/crates/arduino_mkrzero) - Board support for the [mkrzero board](https://store.arduino.cc/arduino-mkrzero) ![crates.io](https://img.shields.io/crates/v/arduino_mkrzero.svg)
- [`arduino_nano33iot`](https://crates.io/crates/arduino_nano33iot) - Board support for the [Arduino Nano 33 IoT](https://store.arduino.cc/usa/nano-33-iot) ![crates.io](https://img.shields.io/crates/v/arduino_nano33iot.svg)

### Nordic

- [`dwm1001`](https://crates.io/crates/dwm1001) - [Decawave DWM1001-DEV] - ![crates.io](https://img.shields.io/crates/v/dwm1001.svg)
- [`microbit`](https://crates.io/crates/microbit) - [micro:bit] - ![crates.io](https://img.shields.io/crates/v/microbit.svg)

[Decawave DWM1001-DEV]: https://www.decawave.com/product/dwm1001-development-board/
[micro:bit]: http://microbit.org/

### NXP

- [`frdm-kw41z`](https://crates.io/crates/frdm-kw41z) - [FRDM-KW41Z] - ![crates.io](https://img.shields.io/crates/v/frdm-kw41z.svg)

[FRDM-KW41Z]: https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/kinetis-cortex-m-mcus/w-serieswireless-conn.m0-plus-m4/freedom-development-kit-for-kinetis-kw41z-31z-21z-mcus:FRDM-KW41Z

### Sparkfun

- [`samd21_mini`](https://crates.io/crates/samd21_mini) - Board support for the [SAMD21 Mini Breakout](https://www.sparkfun.com/products/13664) ![crates.io](https://img.shields.io/crates/v/samd21_mini.svg)

### SeeedStudio

- [`seedstudio-gd32v`](https://github.com/riscv-rust/seedstudio-gd32v) - Board support crate for the [GD32 RISC-V Dev Board](https://www.seeedstudio.com/SeeedStudio-GD32-RISC-V-Dev-Board-p-4302.html)
  ![crates.io](https://img.shields.io/crates/v/seedstudio-gd32v.svg)
  - Contains runnable examples for this board
- [`xiao_m0`](https://crates.io/crates/xiao_m0) - Board support for the [Seeed Studio Seeeduino XIAO](http://wiki.seeedstudio.com/Seeeduino-XIAO/) ![crates.io](https://img.shields.io/crates/v/xiao_m0.svg)

### SiFive

- [`hifive1`](https://github.com/riscv-rust/hifive1) - Board support crate for [HiFive1](https://www.sifive.com/boards/hifive1) and [LoFive](https://hackaday.io/project/26909-lofive) boards - ![crates.io](https://img.shields.io/crates/v/hifive.svg)

### Sipeed

- [`longan-nano`](https://github.com/riscv-rust/longan-nano) - Board support package for the [Longan Nano board](https://www.seeedstudio.com/Sipeed-Longan-Nano-RISC-V-GD32VF103CBT6-Development-Board-p-4205.html)
  ![crates.io](https://img.shields.io/crates/v/longan-nano.svg)
  - Contains runnable examples for this board

### Sony

- [`prussia`](https://github.com/ZirconiumX/prussia) - SDK for the PlayStation 2.

### STMicroelectronics

- [`f3`](https://crates.io/crates/f3) Board Support Crate for the [STM32F3DISCOVERY] - ![crates.io](https://img.shields.io/crates/v/f3.svg)
- [`nucleo-f042k6`](https://github.com/therealprof/nucleo-f042k6.git) - [Nucleo-F042K6]
- [`nucleo-f103rb`](https://github.com/therealprof/nucleo-f103rb.git) - [Nucleo-F103RB]
- [`nucleo-f401re`](https://github.com/jkristell/nucleo-f401re.git) - [Nucleo-F401RE] ![crates.io](https://img.shields.io/crates/v/nucleo-f401re.svg)
- [`solo-bsp`](https://crates.io/crates/solo-bsp) Board Support Crate for [Solo], an open source security token (WIP) - ![crates.io](https://img.shields.io/crates/v/solo-bsp.svg)
- [`stm32f407g-disc`](https://crates.io/crates/stm32f407g-disc) Board Support Crate for the [STM32F4DISCOVERY] (WIP) - ![crates.io](https://img.shields.io/crates/v/stm32f407g-disc.svg)
- [`stm32f429i-disc`](https://crates.io/crates/stm32f429i-disc) Board Support Crate for the [STM32F429DISCOVERY] (WIP) - ![crates.io](https://img.shields.io/crates/v/stm32f429i-disc.svg)
- [`stm32f3-discovery`](https://crates.io/crates/stm32f3-discovery) Board Support Crate for the [STM32F3DISCOVERY] used in [Rust Embedded Discovery book](https://rust-embedded.github.io/discovery/index.html) - ![crates.io](https://img.shields.io/crates/v/stm32f3-discovery)


[Nucleo-F401RE]: https://www.st.com/en/evaluation-tools/nucleo-f401re.html

### Teensy
- [`teensy4-rs`](https://github.com/mciantyre/teensy4-rs) Board Support create for the [Teensy 4.0]

### Texas Instruments

- [`monotron`](https://github.com/thejpster/monotron) - A 1980s home-computer style application for the Texas Instruments Stellaris Launchpad. PS/2 keyboard input, text output on a bit-bashed 800x600 VGA signal. Uses [menu], [vga-framebuffer] and [pc-keyboard].
- [`stellaris-launchpad`](https://crates.io/crates/stellaris-launchpad) - For the Texas Instruments Stellaris Launchpad and Tiva-C Launchpad ![crates.io](https://img.shields.io/crates/v/stellaris-launchpad.svg)

### Special Purpose

- [`betafpv-f3`](https://github.com/JoshMcguigan/betafpv-f3) - For the BetaFPV F3 drone flight controller

## Component abstraction crates

The following crates provide HAL-like abstractions for subcomponents of embedded
devices which go beyond what is available in [`embedded-hal`]:

- [`accelerometer`](https://github.com/NeoBirth/accelerometer.rs) - Generic accelerometer support, including traits and types for taking readings from 2 or 3-axis accelerometers and tracking device orientations - ![crates.io](https://img.shields.io/crates/v/accelerometer.svg)
- [`embedded-graphics`]: 2D drawing library for any size display - ![crates.io](https://img.shields.io/crates/v/embedded-graphics.svg)
- [`radio`](https://github.com/ryankurte/rust-radio) - Generic radio transceiver traits, mocks, and helpers - ![crates.io](https://img.shields.io/crates/v/radio.svg)
- [`smart-leds`](https://github.com/smart-leds-rs): Support for addressable LEDs including WS2812 and APA102
- [`usb-device`](https://github.com/mvirkkunen/usb-device): Abstraction layer between USB peripheral crates & USB class crates - ![crates.io](https://img.shields.io/crates/v/usb-device.svg)
- [`atat`](https://github.com/BlackbirdHQ/atat): Abstraction crate to ease writting AT based driver crates - ![crates.io](https://img.shields.io/crates/v/atat.svg)
- [`embedded-nal`](https://github.com/rust-embedded-community/embedded-nal): An Embedded Network Abstraction Layer - ![crates.io](https://img.shields.io/crates/v/embedded-nal.svg)
- [`embedded-storage`](https://github.com/rust-embedded-community/embedded-storage): An Embedded Storage Abstraction Layer

## Driver crates

Platform agnostic crates to interface external components. These crates use the [`embedded-hal`]
interface to support [all the devices and systems that implement the `embedded-hal`
traits][hal-impl].

[hal-impl]: #hal-implementation-crates

The list below contains drivers developed as part of the [Weekly Driver initiative][wd] and that
have achieved the "released" status (published on crates.io + documentation / short blog post).

[wd]: https://github.com/rust-embedded/wg/issues/39

1. [AD983x] - SPI - AD9833/AD9837 waveform generators / DDS - [Intro blog post][25] - ![crates.io](https://img.shields.io/crates/v/ad983x.svg)
1. [adafruit-alphanum4] - I2C - Driver for [Adafruit 14-segment LED Alphanumeric Backpack][29] based on the ht16k33 chip - ![crates.io](https://img.shields.io/crates/v/adafruit-alphanum4.svg)
1. [ADS1x1x] - I2C - 12/16-bit ADCs like ADS1013, ADS1015, ADS1115, etc. - [Intro blog post][23] - ![crates.io](https://img.shields.io/crates/v/ads1x1x.svg)
1. [ADXL313] - SPI - 3-axis accelerometer - ![crates.io](https://img.shields.io/crates/v/adxl313.svg)
1. [ADXL343] - I2C - 3-axis accelerometer - ![crates.io](https://img.shields.io/crates/v/adxl343.svg)
1. [ADXL355] - SPI - 3-axis accelerometer - [Intro blog post][43] - ![crates.io](https://img.shields.io/crates/v/adxl355.svg)
1. [AHT20] - I2C - Humidity and temperature sensor - [github](https://github.com/chocol4te/aht20) - ![crates.io](https://img.shields.io/crates/v/aht20.svg)
1. [AnyLeaf] - I2C - pH sensor module - [github](https://github.com/AnyLeaf/ph-rust) - ![crates.io](https://img.shields.io/crates/v/anyleaf.svg)
1. [AT86RF212] - SPI - Low power IEEE 802.15.4-2011 ISM RF Transceiver - [Intro blog post][36] - ![crates.io](https://img.shields.io/crates/v/radio-at86rf212.svg)
1. [BlueNRG] - SPI - driver for BlueNRG-MS Bluetooth module - [Intro post][15] ![crates.io](https://img.shields.io/crates/v/bluenrg.svg)
1. [BNO055] - I2C - Bosch Sensortec BNO055 9-axis IMU driver - [Intro post][18] ![crates.io](https://img.shields.io/crates/v/bno055.svg)
1. [dht-sensor] - 1-Wire - DHT11/DHT22 temperature/humidity sensor driver - [github][48] - ![crates.io](https://img.shields.io/crates/v/dht-sensor.svg)
1. [DRV8825] - DRV8825 Stepper Motor Driver (based on [Stepper]) - [Intro blog post][52] - ![crates.io](https://img.shields.io/crates/v/drv8825.svg)
1. [DS1307] - I2C - Real-time clock driver - [Intro blog post][13] - ![crates.io](https://img.shields.io/crates/v/ds1307.svg)
1. [EEPROM24x] - I2C - 24x series serial EEPROM driver - [Intro blog post][12] - ![crates.io](https://img.shields.io/crates/v/eeprom24x.svg)
1. [embedded-ccs811] - I2C - Gas and VOC sensor driver for monitoring indoor air quality - [Intro blog post][49] - ![crates.io](https://img.shields.io/crates/v/embedded-ccs811.svg)
1. [embedded-sdmmc] - SPI - SD/MMC Card Driver with MS-DOS Partition and FAT16/FAT32 support - [Intro post][20] ![crates.io](https://img.shields.io/crates/v/embedded-sdmmc.svg)
1. [ENC28J60] - SPI - Ethernet controller - [Intro blog post][4] - ![crates.io](https://img.shields.io/crates/v/enc28j60.svg)
1. [HTS221] - I2C - Humidity and temperature sensor - [Intro blog post][7] - ![crates.io](https://img.shields.io/crates/v/hts221.svg)
1. [IIS2MDC] - I2C - ST's High accuracy, ultra-low-power, 3-axis digital output magnetometer - ![crates.io](https://img.shields.io/crates/v/iis2mdc.svg)
1. [ISM330DHCX] - I2C - ST's IMU with 3D accelerometer, 3D gyroscope, ML core and more - ![crates.io](https://img.shields.io/crates/v/ism330dhcx.svg)
1. [keypad] - GPIO - Keypad matrix circuits - [Intro post][14] - ![crates.io](https://img.shields.io/crates/v/keypad.svg)
1. [KXCJ9] - I2C - KXCJ9/KXCJB 3-axis accelerometers - [Intro blog post][24] - ![crates.io](https://img.shields.io/crates/v/kxcj9.svg)
1. [L3GD20] - SPI - Gyroscope - [Intro blog post][1&2] - ![crates.io](https://img.shields.io/crates/v/l3gd20.svg)
1. [LSM303DLHC] - I2C - Accelerometer + compass (magnetometer) - [Intro blog post][1&2] - ![crates.io](https://img.shields.io/crates/v/lsm303dlhc.svg)
1. [MAX6955] - I2C - Driver for Alphanumeric LED display driver - [Intro blog post][46] - ![crates.io](https://img.shields.io/crates/v/max6955.svg)
1. [MCP25LCXX] - SPI - Driver for Microchip's 25LC series of EEPROMs - ![crates.io](https://img.shields.io/crates/v/microchip-eeprom-25lcxx.svg)
1. [MCP3008] - SPI - 8 channel 10-bit ADC - [Intro blog post][3] - ![crates.io](https://img.shields.io/crates/v/adc-mcp3008.svg)
1. [MCP3425] - I2C - 16-bit ADC - [Intro blog post][5] - ![crates.io](https://img.shields.io/crates/v/mcp3425.svg)
1. [MCP794xx] - I2C - Real-time clock / calendar driver - [Intro blog post][26] - ![crates.io](https://img.shields.io/crates/v/mcp794xx.svg)
1. [MMA7660FC] - I2C - 3-axis accelerometer - [Intro blog post][9]
1. [OPT300x] - I2C - Ambient light sensor family driver  - [Intro blog post][30] - ![crates.io](https://img.shields.io/crates/v/opt300x.svg)
1. [pwm-pca9685] - I2C - 16-channel, 12-bit PWM/Servo/LED controller - [Intro blog post][32] - ![crates.io](https://img.shields.io/crates/v/pwm-pca9685.svg)
1. [rotary-encoder-hal] - GPIO - A rotary encoder driver using `embedded-hal` - [Intro blog post][28] - ![crates.io](https://img.shields.io/crates/v/rotary-encoder-hal.svg)
1. [SGP30] - I2C - Gas sensor - [Intro blog post][6] - ![crates.io](https://img.shields.io/crates/v/sgp30.svg)
1. [SH1106] - I2C - Monochrome OLED display controller - [Intro post][19] ![crates.io](https://img.shields.io/crates/v/sh1106.svg)
1. [shared-bus] - I2C - utility driver for sharing a bus between multiple devices - [Intro post][16] ![crates.io](https://img.shields.io/crates/v/shared-bus.svg)
1. [shift-register-driver] - GPIO - Shift register - [Intro blog post][10] - ![crates.io](https://img.shields.io/crates/v/shift-register-driver.svg)
1. [Si4703] - I2C - FM radio turner (receiver) driver  - [Intro blog post][31] - ![crates.io](https://img.shields.io/crates/v/si4703.svg)
1. [SRAM23x] - SPI - Microchip 23x series serial SRAM/NVSRAM driver - [Intro blog post][51] - ![crates.io](https://img.shields.io/crates/v/sram23x.svg)
1. [SSD1306] - I2C/SPI - OLED display controller - [Intro blog post][8] - ![crates.io](https://img.shields.io/crates/v/ssd1306.svg)
1. [STSPIN220] - STSPIN220 Stepper Motor Driver (based on [Stepper]) - [Intro blog post][52] - ![crates.io](https://img.shields.io/crates/v/stspin220.svg)
1. [Sx127x] - SPI - Long Range Low Power Sub GHz (Gfsk, LoRa) RF Transceiver - [Intro blog post][34] - ![crates.io](https://img.shields.io/crates/v/radio-sx127x.svg)
1. [Sx128x] - SPI - Long range, low power 2.4 GHz (Gfsk, Flrc, LoRa) RF Transceiver - [Intro blog post][35] - ![crates.io](https://img.shields.io/crates/v/radio-sx128x.svg)
1. [TC72] - SPI - Microchip TC72 temperature sensor - ![crates.io](https://img.shields.io/crates/v/microchip-tc72r-rs.svg)
1. [TMP006] - I2C - Contact-less infrared (IR) thermopile temperature sensor driver - [Intro post][17] ![crates.io](https://img.shields.io/crates/v/tmp006.svg)
1. [TMP1x2] - I2C - TMP102 and TMP112x temperature sensor driver - [Intro blog post][22] ![crates.io](https://img.shields.io/crates/v/tmp1x2.svg)
1. [TSIC] - GPIO - TSIC 306 temperature sensor driver - [Intro blog post][50] ![crates.io](https://img.shields.io/crates/v/tsic.svg)
1. [TSL256X] - I2C - Light Intensity Sensor - [Intro blog post][11] - ![crates.io](https://img.shields.io/crates/v/tsl256x.svg)
1. [VEML6030/VEML7700] - I2C - Ambient light sensors - [Intro blog post][33] - ![crates.io](https://img.shields.io/crates/v/veml6030.svg)
1. [VEML6075] - I2C - UVA and UVB light sensor - [Intro blog post][27] - ![crates.io](https://img.shields.io/crates/v/veml6075.svg)
1. [usbd-serial] - USB CDC-ACM class (serial) implementation - [github][37] - ![crates.io](https://img.shields.io/crates/v/usbd-serial.svg)
1. [usbd-hid] - USB HID class implementation - [github][38] - ![crates.io](https://img.shields.io/crates/v/usbd-hid.svg)
1. [usbd-hid-device] - USB HID class implementation without `unsafe` - [github][40] - ![crates.io](https://img.shields.io/crates/v/usbd-hid-device.svg)
1. [usbd-midi] - USB MIDI class implementation - [github][41] - ![crates.io](https://img.shields.io/crates/v/usbd-midi.svg)
1. [usbd-webusb] - USB webUSB class implementation - [github][39] - ![crates.io](https://img.shields.io/crates/v/usbd-webusb.svg)
1. [SHTCx] - I2C - Temperature / humidity sensors - [github][42] - ![crates.io](https://img.shields.io/crates/v/shtcx.svg)
1. [ST7789] - SPI - An embedded-graphics compatible driver for the popular lcd family from Sitronix used in the PineTime watch [github][44] ![crates.io](https://img.shields.io/crates/v/st7789.svg)
1. [DW1000] - SPI - Radio transceiver (IEEE 802.15.4 and position tracking) - [Article][45] - ![crates.io](https://img.shields.io/crates/v/dw1000.svg)
1. [Adafruit-7segment] - I2C - Driver for Adafruit 7-segment LED Numeric Backpack based on the ht16k33 chip - [github][47] - ![crates.io](https://img.shields.io/crates/v/adafruit-7segment.svg)

[1&2]: http://blog.japaric.io/wd-1-2-l3gd20-lsm303dlhc-madgwick/
[3]: http://pramode.in/2018/02/24/an-introduction-to-writing-embedded-hal-based-drivers-in-rust/
[4]: http://blog.japaric.io/wd-4-enc28j60/
[5]: https://blog.dbrgn.ch/2018/3/13/rust-mcp3425-driver/
[6]: https://blog.dbrgn.ch/2018/4/1/rust-sgp30-driver/
[7]: https://medium.com/@pdanielgallagher/hts221-humidity-and-temperature-sensor-88056ea9e5fa
[8]: https://wapl.es/electronics/rust/2018/04/30/ssd1306-driver.html
[9]: https://rahul-thakoor.github.io/an-i2c-rust-driver-for-mma7660fc-based-3-axis-digital-accelerometer/
[10]: https://www.joshmcguigan.com/blog/shift-register-driver/
[11]: https://www.joshmcguigan.com/blog/tsl256x-light-intensity-sensor-driver/
[12]: https://blog.eldruin.com/24x-serial-eeprom-driver-in-rust/
[13]: https://blog.eldruin.com/ds1307-real-time-clock-rtc-driver-in-rust/
[14]: https://www.reddit.com/r/rust/comments/9j42o9/weekly_driver_keypad_matrix_circuits/
[15]: https://www.219design.com/bluetooth-low-energy-with-rust/
[16]: https://blog.rahix.de/001-shared-bus/
[17]: https://blog.eldruin.com/tmp006-contact-less-infrared-ir-thermopile-driver-in-rust/
[18]: https://www.reddit.com/r/rust/comments/ao4sqq/embeddedhal_bno055_9axis_imu_driver/
[19]: https://wapl.es/electronics/rust/2019/02/13/sh1106-driver.html
[20]: https://www.reddit.com/r/rust/comments/ascvls/introducing_embeddedsdmmc_a_purerust_no_std_sd/
[22]: https://blog.eldruin.com/tmp1x2-temperature-sensor-driver-in-rust/
[23]: https://blog.eldruin.com/ads1x1x-analog-to-digital-converter-driver-in-rust/
[24]: https://blog.eldruin.com/kxcj9-kxcjb-tri-axis-mems-accelerator-driver-in-rust/
[25]: https://blog.eldruin.com/ad983x-waveform-generator-dds-driver-in-rust/
[26]: https://blog.eldruin.com/mcp794xx-real-time-clock-rtc-driver-in-rust/
[27]: https://blog.eldruin.com/veml6075-uva-uvb-uv-index-light-sensor-driver-in-rust/
[28]: https://leshow.github.io/post/rotary_encoder_hal/
[29]: https://learn.adafruit.com/adafruit-led-backpack/0-54-alphanumeric
[30]: https://blog.eldruin.com/opt300x-ambient-light-sensor-driver-in-rust/
[31]: https://blog.eldruin.com/si4703-fm-radio-receiver-driver-in-rust/
[32]: https://blog.eldruin.com/pca9685-pwm-led-servo-controller-driver-in-rust/
[33]: https://blog.eldruin.com/veml6030-ambient-light-sensor-driver-in-rust/
[34]: https://ryan.kurte.nz/notes/2020-01-05-rust-radio
[35]: https://ryan.kurte.nz/notes/2020-01-05-rust-radio
[36]: https://ryan.kurte.nz/notes/2020-01-05-rust-radio
[37]: https://github.com/mvirkkunen/usbd-serial
[38]: https://github.com/twitchyliquid64/usbd-hid
[39]: https://github.com/redpfire/usbd-webusb
[40]: https://github.com/agalakhov/usbd-hid-device
[41]: https://github.com/btrepp/usbd-midi
[42]: https://github.com/dbrgn/shtcx-rs
[43]: https://jitter.company/blog/2020/02/14/adxl355-embedded-hal-driver-crate/
[44]: https://github.com/almindor/st7789
[45]: https://braun-embedded.com/dw1000/
[46]: https://lonesometraveler.github.io/2020/03/20/max6955.html
[47]: https://github.com/kallemooo/adafruit-7segment
[48]: https://github.com/michaelbeaumont/dht-sensor
[49]: https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
[50]: https://nitschinger.at/Rusty-PID-Porting-the-TSic-sensor-from-C-to-Rust/
[51]: https://blog.a1w.ca/p/rust-embedded-driver-microchip-23x-sram
[52]: https://flott-motion.org/news/announcing-step-dir/

[AD983x]: https://crates.io/crates/ad983x
[adafruit-alphanum4]: https://crates.io/crates/adafruit-alphanum4
[ADS1x1x]: https://crates.io/crates/ads1x1x
[ADXL313]: https://crates.io/crates/adxl313
[ADXL343]: https://crates.io/crates/adxl343
[ADXL355]: https://crates.io/crates/adxl355
[AHT20]: https://crates.io/crates/aht20
[AnyLeaf]: https://crates.io/crates/anyleaf
[at86rf212]: https://crates.io/crates/radio-at86rf212
[BlueNRG]: https://crates.io/crates/bluenrg
[BNO055]: https://crates.io/crates/bno055
[dht-sensor]: https://crates.io/crates/dht-sensor
[DRV8825]: https://crates.io/crates/drv8825
[DS1307]: https://crates.io/crates/ds1307
[EEPROM24x]: https://crates.io/crates/eeprom24x
[embedded-ccs811]: https://crates.io/crates/embedded-ccs811
[embedded-sdmmc]: https://crates.io/crates/embedded-sdmmc
[ENC28J60]: https://crates.io/crates/enc28j60
[HTS221]: https://crates.io/crates/hts221
[IIS2MDC]: https://crates.io/crates/iis2mdc
[ISM330DHCX]: https://crates.io/crates/ism330dhcx
[keypad]: https://crates.io/crates/keypad
[KXCJ9]: https://crates.io/crates/kxcj9
[L3GD20]: https://crates.io/crates/l3gd20
[LSM303DLHC]: https://crates.io/crates/lsm303dlhc
[MAX6955]: https://crates.io/crates/max6955
[MCP25LCXX]: https://crates.io/crates/microchip-eeprom-25lcxx
[MCP3008]: https://crates.io/crates/adc-mcp3008
[MCP3425]: https://crates.io/crates/mcp3425
[MCP794xx]: https://crates.io/crates/mcp794xx
[MMA7660FC]: https://crates.io/crates/mma7660fc
[OPT300x]: https://github.com/eldruin/opt300x-rs
[pwm-pca9685]: https://crates.io/crates/pwm-pca9685
[rotary-encoder-hal]: https://crates.io/crates/rotary-encoder-hal
[SGP30]: https://crates.io/crates/sgp30
[SH1106]: https://crates.io/crates/sh1106
[shared-bus]: https://github.com/Rahix/shared-bus
[shift-register-driver]: https://crates.io/crates/shift-register-driver
[Si4703]: https://crates.io/crates/si4703
[SRAM23x]: https://crates.io/crates/sram23x
[SSD1306]: https://crates.io/crates/ssd1306
[STSPIN220]: https://crates.io/crates/stspin220
[Sx127x]: https://crates.io/crates/radio-sx127x
[Sx128x]: https://crates.io/crates/radio-sx128x
[TC72]: https://crates.io/crates/microchip-tc72r-rs
[TMP006]: https://crates.io/crates/tmp006
[TMP1x2]: https://crates.io/crates/tmp1x2
[TSIC]: https://crates.io/crates/tsic
[TSL256X]: https://crates.io/crates/tsl256x
[VEML6030/VEML7700]: https://crates.io/crates/veml6030
[VEML6075]: https://crates.io/crates/veml6075
[usbd-serial]: http://crates.io/crates/usbd-serial
[usbd-hid]: http://crates.io/crates/usbd-hid
[usbd-hid-device]: http://crates.io/crates/usbd-hid-device
[usbd-midi]: http://crates.io/crates/usbd-midi
[usbd-webusb]: http://crates.io/crates/usbd-webusb
[SHTCx]: http://crates.io/crates/shtcx
[ST7789]: http://crates.io/crates/st7789
[DW1000]: https://crates.io/crates/dw1000
[Adafruit-7segment]: https://crates.io/crates/adafruit-7segment

*NOTE* You may be able to find even more driver crates by searching for the [`embedded-hal-driver`]
keyword on crates.io!

[`embedded-hal-driver`]: https://crates.io/keywords/embedded-hal-driver

### WIP

Work in progress drivers. Help the authors make these crates awesome!

1. [AFE4400] - SPI - Pulse oximeter
1. [APDS9960] - I2C - Proximity, ambient light, RGB and gesture sensor - ![crates.io](https://img.shields.io/crates/v/apds9960.svg)
1. [AS5048A] - SPI - AMS AS5048A Magnetic Rotary Encoder
1. [AXP209] - I2C - Power management unit
1. [BH1750] - I2C - ambient light sensor (lux meter)
1. [BME280] - A rust device driver for the Bosch BME280 temperature, humidity, and atmospheric pressure sensor and the Bosch BMP280 temperature and atmospheric pressure sensor. ![crates.io](https://img.shields.io/crates/v/bme280.svg)
1. [bme680] - I2C - Temperature / humidity / gas / pressure sensor - ![crates.io](https://img.shields.io/crates/v/bme680.svg)
1. [BMI160] - I2C / SPI - Inertial Measurement Unit - ![crates.io](https://img.shields.io/crates/v/bmi160.svg)
1. [BMP280] - A platform agnostic driver to interface with the BMP280 pressure sensor ![crates.io](https://img.shields.io/crates/v/bmp280-ehal.svg)
1. [CC1101] - SPI - Sub-1GHz RF Transceiver - ![crates.io](https://img.shields.io/crates/v/cc1101.svg)
1. [DS3231] - I2C - real time clock
1. [DS3234] - SPI - Real time clock
1. [DS323x] - I2C/SPI - Real-time clocks (RTC): DS3231, DS3232 and DS3234 - ![crates.io](https://img.shields.io/crates/v/ds323x.svg)
1. [epd-waveshare] - SPI - driver for E-Paper Modules from Waveshare ![crates.io](https://img.shields.io/crates/v/epd-waveshare.svg)
1. [embedded-morse] - Output morse messages - ![crates.io](https://img.shields.io/crates/v/embedded-morse.svg)
1. [embedded-nrf24l01] - SPI+GPIO - 2.4 GHz radio
1. [GridEYE] - I2C - Rust driver for Grid-EYE / Panasonic AMG88(33) - ![crates.io](https://img.shields.io/crates/v/GridEYE.svg)
1. [HC-SR04] - DIO - Ultrasound sensor
1. [HD44780-driver] - GPIO - LCD controller - ![crates.io](https://img.shields.io/crates/v/hd44780-driver.svg)
1. [HD44780] - Parallel port - LCD controller
1. [HM11] - USART - HM-11 bluetooth module AT configuration crate - ![crates.io](https://img.shields.io/crates/v/hm11.svg)
1. [HRS3300] - I2C - Heart rate sensor / monitor used in the PineTime smartwatch, for example. - ![crates.io](https://img.shields.io/crates/v/hrs3300.svg)
1. [HDC20xx] - I2C - Temperature and humidity sensor compatible with HDC2080, HDC2021 and HDC2010. - ![crates.io](https://img.shields.io/crates/v/hdc20xx.svg)
1. [hub75] - A driver for rgb led matrices with the hub75 interface  - ![crates.io](https://img.shields.io/crates/v/hub75.svg)
1. [hzgrow-r502] - UART capacitive fingerprint reader - ![crates.io](https://img.shields.io/crates/v/hzgrow-r502.svg)
1. [iAQ-Core] - I2C - iAQ-Core-C/iAQ-Core-P Gas and VOC sensor driver for monitoring indoor air quality.
1. [ILI9341] - SPI - TFT LCD display
1. [INA260] - I2C - power monitor - ![crates.io](https://img.shields.io/crates/v/ina260.svg)
1. [ISL29125] - I2C - RGB Color Light Sensor with IR Blocking Filter - ![crates.io](https://img.shields.io/crates/v/isl29125.svg)
1. [LM75] - I2C - Temperature sensor and thermal watchdog - ![crates.io](https://img.shields.io/crates/v/lm75.svg)
1. [LS010B7DH01] - SPI - Memory LCD
1. [LSM303C] - A platform agnostic driver to interface with the LSM303C (accelerometer + compass) ![crates.io](https://img.shields.io/crates/v/lsm303c.svg)
1. [LSM9DS1] - I2C/SPI - 9-axis motion sensor module ![crates.io](https://img.shields.io/crates/v/lsm9ds1.svg)
1. [ltr-559] - I2C - Ambient Light Sensor and Proximity sensor ![crates.io](https://img.shields.io/crates/v/ltr-559.svg)
1. [lvgl] - no_std [LittleVGL](https://github.com/littlevgl/lvgl) port - ![crates.io](https://img.shields.io/crates/v/lvgl.svg)
1. [MAG3110] - I2C - Magnetometer
1. [MAX17048/9] - I2C - LiPo Fuel gauge, battery monitoring IC - ![crates.io](https://img.shields.io/crates/v/max17048.svg)
1. [MAX170xx] - I2C - LiPo Fuel gauge, battery monitoring ICs compatible with MAX17043/4, MAX17048/9, MAX17058/9.
1. [MAX3010x] - I2C - Pulse oximeter and heart-rate sensor. ATM Compatible with MAX30102. -![crates.io](https://img.shields.io/crates/v/max3010x.svg)
1. [MAX31855] - SPI - Thermocouple digital converter -![crates.io](https://img.shields.io/crates/v/max31855.svg)
1. [MAX31865] - SPI - RTD to Digital converter - ![crates.io](https://img.shields.io/crates/v/max31865.svg)
1. [MAX44009] - I2C - Ambient light sensor - ![crates.io](https://img.shields.io/crates/v/max44009.svg)
1. [MAX7219] - SPI - LED display driver - ![crates.io](https://img.shields.io/crates/v/max7219.svg)
1. [MCP4725] - I2C - 12-bit DAC - ![crates.io](https://img.shields.io/crates/v/mcp4725)
1. [MCP49xx] - SPI - 8/10/12-bit DACs like MCP4921, MCP4922, MCP4801, etc. - ![crates.io](https://img.shields.io/crates/v/mcp49xx.svg)
1. [MCP9808] - I2C - Temperature sensor - ![crates.io](https://img.shields.io/crates/v/mcp9808.svg)
1. [MFRC522] - SPI - RFID tag reader/writer
1. [midi-port] - UART - MIDI input - ![crates.io](https://img.shields.io/crates/v/midi-port.svg)
1. [MLX9061x] - I2C - MLX90614/MLX90615 Contact-less infrared (IR) temperature sensor driver. - ![crates.io](https://img.shields.io/crates/v/mlx9061x.svg)
1. [motor-driver] - Motor drivers: L298N, TB6612FNG, etc.
1. [MPU6050] - I2C - no_std driver for the MPU6050 ![crates.io](https://img.shields.io/crates/v/mpu6050.svg)
1. [MPU9250] - no_std driver for the MPU9250 (and other MPU* devices) & onboard AK8963 (accelerometer + gyroscope +  magnetometer IMU) ![crates.io](https://img.shields.io/crates/v/mpu9250.svg)
1. [NRF24L01] - SPI - 2.4 GHz wireless communication
1. [OneWire] - 1wire - OneWire protocol implementation with drivers for devices such as [DS18B20](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf) - ![crates.io](https://img.shields.io/crates/v/onewire.svg)
1. [PCD8544] - SPI - 48x84 pixels matrix LCD controller
1. [PCD8544_rich] - SPI - Rich driver for 48x84 pixels matrix LCD controller  - ![crates.io](https://img.shields.io/crates/v/pcd8544.svg)
1. [PCF857x] - I2C - I/O expanders: PCF8574, PCF8574A, PCF8575 ![crates.io](https://img.shields.io/crates/v/pcf857x.svg)
1. [radio-at86rf212] - SPI - Sub GHz 802.15.4 radio transceiver ![crates.io](https://img.shields.io/crates/v/radio-at86rf212.svg)
1. [RFM69] - SPI - ISM radio transceiver
1. [RN2xx3] - Serial - A driver for the RN2483 / RN2903 LoRaWAN modems by Microchip
1. [SCD30] - I2C - CO₂ sensor - ![crates.io](https://img.shields.io/crates/v/scd30.svg)
1. [SHT2x] - I2C - temperature / humidity sensors
1. [SHT3x] - I2C - Temperature / humidity sensors
1. [SI5351] - I2C - clock generator
1. [SI7021] - I2C - Humidity and temperature sensor
1. [spi-memory] - SPI - A generic driver for various SPI Flash and EEPROM chips - ![crates.io](https://img.shields.io/crates/v/spi-memory.svg)
1. [SSD1322] - SPI - Graphical OLED display controller - ![crates.io](https://img.shields.io/crates/v/ssd1322.svg)
1. [SSD1351] - SPI - 16bit colour OLED display driver - ![crates.io](https://img.shields.io/crates/v/ssd1351.svg)
1. [SSD1675] - SPI - Tri-color ePaper display controller - ![crates.io](https://img.shields.io/crates/v/ssd1675.svg)
1. [st7032i] - I2C - Dot Matrix LCD Controller driver (Sitronix ST7032i or similar). - ![crates.io](https://img.shields.io/crates/v/st7032i.svg)
1. [ST7735-lcd] - SPI - An embedded-graphics compatible driver for the popular lcd family from Sitronix ![crates.io](https://img.shields.io/crates/v/st7735-lcd.svg)
1. [ST7920] - SPI - LCD displays using the ST7920 controller ![crates.io](https://img.shields.io/crates/v/st7920.svg)
1. [stm32-eth] - MCU - Ethernet
1. [SX1278] - SPI - Long range (LoRa) transceiver
1. [SX1509] - I2C - IO Expander / Keypad driver
1. [TCS3472] - I2C - RGB color light sensor - ![crates.io](https://img.shields.io/crates/v/tcs3472.svg)
1. [TPA2016D2] - I2C - A driver for interfacing with the Texas Instruments TPA2016D2 Class-D amplifier - ![crates.io](https://img.shields.io/crates/v/tpa2016d2.svg)
1. [VEML6040] - I2C - RGBW color light sensor - ![crates.io](https://img.shields.io/crates/v/veml6040.svg)
1. [VEML6070] - I2C - UVA light sensor - ![crates.io](https://img.shields.io/crates/v/veml6070.svg)
1. [vesc-comm] - A driver for communicating with [VESC-compatible electronic speed controllers](http://vedder.se/2015/01/vesc-open-source-esc/) ![crates.io](https://img.shields.io/crates/v/vesc-comm.svg)
1. [VL53L0X] - A platform agnostic driver to interface with the vl53l0x (time-of-flight sensor) ![crates.io](https://img.shields.io/crates/v/vl53l0x.svg)
1. [w5500] - SPI - Ethernet Module with hardwired protocols : TCP, UDP, ICMP, IPv4, ARP, IGMP, PPPoE - ![crates.io](https://img.shields.io/crates/v/w5500.svg)
1. [xCA9548A] - I2C - I2C switches/multiplexers: TCA9548A, PCA9548A - ![crates.io](https://img.shields.io/crates/v/xca9548a.svg)
1. [ublox-cellular-rs] - Serial - Cellular driver for the full Ublox cellular serial based family

[AFE4400]: https://github.com/ReeceStevens/afe4400
[APDS9960]: https://crates.io/crates/apds9960
[AS5048A]: https://github.com/uwearzt/as5048a
[AXP209]: https://github.com/RandomInsano/axp209-rs
[BH1750]: https://github.com/wose/bh1750
[BME280]: https://crates.io/crates/bme280
[bme680]: https://github.com/marcelbuesing/bme680
[BMI160]: https://crates.io/crates/bmi160
[BMP280]: https://crates.io/crates/bmp280-ehal
[CC1101]: https://crates.io/crates/cc1101
[DS3231]: https://github.com/wose/ds3231
[DS3234]: https://github.com/rust-embedded/wg/issues/39#issuecomment-375262785
[DS323x]: https://crates.io/crates/ds323x
[epd-waveshare]: https://crates.io/crates/epd-waveshare
[embedded-morse]: https://crates.io/crates/embedded-morse
[embedded-nrf24l01]: https://crates.io/crates/embedded-nrf24l01
[GridEYE]: https://crates.io/crates/grideye
[HC-SR04]: https://github.com/nordmoen/hc-sr04
[HD44780-driver]: https://crates.io/crates/hd44780-driver
[HD44780]: http://github.com/kunerd/clerk
[HDC20xx]: https://crates.io/crates/hdc20xx
[HM11]: https://crates.io/crates/hm11
[HRS3300]: https://crates.io/crates/hrs3300
[hub75]: https://github.com/david-sawatzke/hub75-rs
[hzgrow-r502]: https://crates.io/crates/hzgrow-r502
[iAQ-Core]: https://github.com/eldruin/iaq-core-rs
[ILI9341]: https://github.com/yuri91/ili9341-rs
[INA260]: https://crates.io/crates/ina260
[ISL29125]: https://crates.io/crates/isl29125
[LM75]: https://crates.io/crates/lm75
[LS010B7DH01]: https://github.com/byronwasti/ls010b7dh01
[LSM303C]: https://crates.io/crates/lsm303c
[LSM9DS1]: https://crates.io/crates/lsm9ds1
[ltr-559]: https://crates.io/crates/ltr-559
[lvgl]: https://github.com/rafaelcaricio/lvgl-rs
[MAG3110]: https://github.com/therealprof/mag3110
[MAX17048/9]: https://crates.io/crates/max17048
[MAX170xx]: https://github.com/eldruin/max170xx-rs
[MAX3010x]: https://crates.io/crates/max3010x
[MAX31855]: https://github.com/cs2dsb/max31855.rs
[MAX31865]: https://crates.io/crates/max31865
[MAX44009]: https://crates.io/crates/max44009
[MAX7219]: https://github.com/almindor/max7219
[MCP4725]: https://crates.io/crates/mcp4725
[MCP49xx]: https://crates.io/crates/mcp49xx
[MCP9808]: https://crates.io/crates/mcp9808
[MFRC522]: https://github.com/japaric/mfrc522
[midi-port]: https://crates.io/crates/midi-port
[MLX9061x]: https://crates.io/crates/mlx9061x
[motor-driver]: https://github.com/japaric/motor-driver
[MPU6050]: https://crates.io/crates/mpu6050
[MPU9250]: https://crates.io/crates/mpu9250
[NRF24L01]: https://github.com/maikelwever/nrf24l01
[OneWire]: https://crates.io/crates/onewire
[PCD8544]: https://github.com/pcein/pcd8544
[PCD8544_rich]: https://crates.io/crates/pcd8544
[PCF857x]: https://crates.io/crates/pcf857x
[radio-at86rf212]: https://crates.io/crates/radio-at86rf212
[RFM69]: https://github.com/lolzballs/rfm69
[RN2xx3]: https://github.com/dbrgn/rn2xx3-rs/
[SCD30]: https://crates.io/crates/scd30
[SHT2x]: https://github.com/dbrgn/sht2x-rs
[SHT3x]: https://github.com/miek/sht3x-rs
[SI5351]: https://github.com/ilya-epifanov/si5351
[SI7021]: https://github.com/wose/si7021
[spi-memory]: https://github.com/jonas-schievink/spi-memory/
[SSD1322]: https://crates.io/crates/ssd1322
[SSD1351]: https://crates.io/crates/ssd1351
[SSD1675]: https://crates.io/crates/ssd1675
[st7032i]: https://github.com/dotcypress/st7032i
[ST7735-lcd]: https://crates.io/crates/st7735-lcd
[ST7920]: https://crates.io/crates/st7920
[stm32-eth]: https://github.com/stm32-rs/stm32-eth
[SX1278]: https://github.com/susu/sx1278
[SX1509]: https://github.com/wez/sx1509
[TCS3472]: https://crates.io/crates/tcs3472
[TPA2016D2]: https://crates.io/crates/tpa2016d2
[VEML6040]: https://crates.io/crates/veml6040
[VEML6070]: https://crates.io/crates/veml6070
[vesc-comm]: https://github.com/chocol4te/vesc-comm
[VL53L0X]: https://crates.io/crates/vl53l0x
[w5500]: https://crates.io/crates/w5500
[xCA9548A]: https://crates.io/crates/xca9548a
[ublox-cellular-rs]: https://github.com/BlackbirdHQ/ublox-cellular-rs

## no-std crates

[`#![no_std]` crates][no-std-category] designed to run on resource constrained devices.

1. [atomic](https://crates.io/crates/atomic): Generic Atomic<T> wrapper type. ![crates.io](https://img.shields.io/crates/v/atomic.svg)
1. [bbqueue](https://crates.io/crates/bbqueue): A SPSC, statically allocatable queue based on BipBuffers suitable for DMA transfers - ![crates.io](https://img.shields.io/crates/v/bbqueue.svg)
1. [bitmatch]: A crate that allows you to match, bind, and pack the individual bits of integers. - ![crates.io](https://img.shields.io/crates/v/bitmatch.svg)
1. [biquad]: A library for creating second order IIR filters for signal processing based on Biquads, where both a Direct Form 1 (DF1) and Direct Form 2 Transposed (DF2T) implementation is available. ![crates.io](https://img.shields.io/crates/v/biquad.svg)
1. [bit_field](https://crates.io/crates/bit_field): manipulating bitfields and bitarrays - ![crates.io](https://img.shields.io/crates/v/bit_field.svg)
1. [bluetooth-hci](https://crates.io/crates/bluetooth-hci): device-independent Bluetooth Host-Controller Interface implementation. ![crates.io](https://img.shields.io/crates/v/bluetooth-hci.svg)
1. [bounded-registers](https://crates.io/crates/bounded-registers) A high-assurance memory-mapped register code generation and interaction library. `bounded-registers` provides a Tock-like API for MMIO registers with the addition of type-based bounds checking. - ![crates.io](https://img.shields.io/crates/v/bounded-registers.svg)
1. [combine](https://crates.io/crates/combine): parser combinator library - ![crates.io](https://img.shields.io/crates/v/combine.svg)
1. [console-traits](https://github.com/thejpster/console-traits): Describes a basic text console. Used by [menu] and implemented by [vga-framebuffer]. ![crates.io](https://img.shields.io/crates/v/console-tratis.svg)
1. [`cmim`], or Cortex-M Interrupt Move: A crate for Cortex-M devices to move data to interrupt context, without needing a critical section to access the data within an interrupt, and to remove the need for the "mutex dance" - ![crates.io](https://img.shields.io/crates/v/cmim.svg)
1. [dcmimu]: An algorithm for fusing low-cost triaxial MEMS gyroscope and accelerometer measurements ![crates.io](https://img.shields.io/crates/v/dcmimu.svg)
1. [debouncr]: A simple no-std input debouncer to detect rising/falling edges with minimal RAM requirements. ![crates.io](https://img.shields.io/crates/v/debouncr.svg)
1. [device-driver]: A toolkit to write better device drivers, faster. ![crates.io](https://img.shields.io/crates/v/device-driver.svg)
1. [embassy]: A set of embedded async tools to make async/await a first-class option for embedded development
1. [embedded-crc-macros](https://crates.io/crates/embedded-crc-macros): Macros implementing portable CRC algorithms and build-time lookup table generation. ![crates.io](https://img.shields.io/crates/v/embedded-crc-macros.svg)
1. [embedded-websocket](https://crates.io/crates/embedded-websocket): A lightweight server and client websocket library for embedded systems. ![crates.io](https://img.shields.io/crates/v/embedded-websocket.svg)
1. [endian_codec]: (En/De)code rust types as packed bytes with specific order (endian). Supports derive. - [![crates.io](https://img.shields.io/crates/v/endian_codec.svg)](https://crates.io/crates/endian_codec)
1. [gcode](https://github.com/Michael-F-Bryan/gcode-rs): A gcode parser for no-std applications - [![crates.io](https://img.shields.io/crates/v/gcode.svg)](https://crates.io/crates/gcode)
1. [gdbstub](https://crates.io/crates/gdbstub): zero-allocation, pure Rust implementation of the GDB Remote Serial Protocol - [![crates.io](https://img.shields.io/crates/v/gdbstub.svg)](https://crates.io/crates/gdbstub)
1. [heapless](https://crates.io/crates/heapless): provides `Vec`, `String`, `LinearMap`, `RingBuffer` backed by fixed-size buffers  - ![crates.io](https://img.shields.io/crates/v/heapless.svg)
1. [ieee802154](https://crates.io/crates/ieee802154): Partial implementation of the IEEE 802.15.4 standard - ![crates.io](https://img.shields.io/crates/v/ieee802154.svg)
1. [infrared](https://crates.io/crates/infrared): infrared remote control library for embedded rust - ![crates.io](https://img.shields.io/crates/v/infrared.svg)
1. [intrusive-collections](https://crates.io/crates/intrusive-collections): intrusive (non-allocating) singly/doubly linked lists and red-black trees - ![crates.io](https://img.shields.io/crates/v/intrusive-collections.svg)
1. [irq](https://crates.io/crates/irq): utilities for writing interrupt handlers (allows moving data into interrupts, and sharing data between them) - ![crates.io](https://img.shields.io/crates/v/irq.svg)
1. [managed](https://crates.io/crates/managed): provides `ManagedSlice`, `ManagedMap` backed by either their std counterparts or fixed-size buffers for `#![no_std]`. - ![crates.io](https://img.shields.io/crates/v/managed.svg)
1. [menu]: A basic command-line interface library. Has nested menus and basic help functionality. ![crates.io](https://img.shields.io/crates/v/menu.svg)
1. [mqtt-sn](https://crates.io/crates/mqtt-sn): Implementation of the MQTT-SN protocol - ![crates.io](https://img.shields.io/crates/v/mqtt-sn.svg)
1. [microfft](https://crates.io/crates/microfft): Embedded-friendly (`no_std`, no-`alloc`) fast fourier transforms - ![crates.io](https://img.shields.io/crates/v/microfft.svg)
1. [micromath](https://github.com/NeoBirth/micromath): Embedded Rust math library featuring fast, safe floating point approximations for common arithmetic operations, 2D and 3D vector types, and statistical analysis - ![crates.io](https://img.shields.io/crates/v/micromath.svg)
1. [nalgebra](https://crates.io/crates/nalgebra): general-purpose and low-dimensional linear algebra library - ![crates.io](https://img.shields.io/crates/v/nalgebra.svg)
1. [nom](https://crates.io/crates/nom): parser combinator framework - ![crates.io](https://img.shields.io/crates/v/nom.svg)
1. [null-terminated](https://crates.io/crates/null-terminated): generic null-terminated arrays - ![crates.io](https://img.shields.io/crates/v/null-terminated.svg)
1. [num-format](https://crates.io/crates/num-format): Crate for producing string representations of numbers, formatted according to international standards, e.g. "1,000,000" for US English - ![crates.io](https://img.shields.io/crates/v/num-format.svg)
1. [`panic-persist`]: A panic handler crate inspired by `panic-ramdump` that logs panic messages to a region of RAM defined by the user, allowing for discovery of panic messages post-mortem using normal program control flow. - ![crates.io](https://img.shields.io/crates/v/panic-persist.svg)
1. [pc-keyboard]: A PS/2 keyboard protocol driver. Transport (bit-banging or SPI) agnostic, but can convert Set 2 Scancodes into Unicode. ![crates.io](https://img.shields.io/crates/v/pc-keyboard.svg)
1. [qei](https://crates.io/crates/qei) : A qei wrapper that allows you to extend your qei timers from a 16 bit integer to a 64 bit integer. - ![crates.io](https://img.shields.io/crates/v/qei.svg)
1. [qemu-exit]: Quit a running QEMU session with user-defined exit code. Useful for unit or integration tests using QEMU. - ![crates.io](https://img.shields.io/crates/v/qemu-exit.svg)
1. [RampMaker](https://crates.io/crates/ramp-maker): Stepper Motor Acceleration Ramp Generator - ![crates.io](https://img.shields.io/crates/v/ramp-maker.svg)
1. [register-rs](https://github.com/rust-embedded/register-rs): Unified interface for MMIO and CPU registers. Provides type-safe bitfield manipulation. `register-rs` is Tock registers with added support for CPU register definitions using the same API as for the MMIO registers. This enables homogeneous interfaces to registers of all kinds. - ![crates.io](https://img.shields.io/crates/v/register.svg)
1. [scroll](https://crates.io/crates/scroll): extensible and endian-aware Read/Write traits for generic containers - ![crates.io](https://img.shields.io/crates/v/scroll.svg)
1. [smbus-pec](https://crates.io/crates/smbus-pec): Minimal portable System Management Bus Packet Error Code calculation algorithm. ![crates.io](https://img.shields.io/crates/v/smbus-pec.svg)
1. [smoltcp](https://github.com/m-labs/smoltcp): a small TCP/IP stack that runs without `alloc`. ![crates.io](https://img.shields.io/crates/v/smoltcp.svg)
1. [static-bytes](https://github.com/xoac/static-bytes): Help work with buffers without dynamic allocation. Implement traits from bytes crate. [![crate.io](https://img.shields.io/crates/v/static-bytes.svg)](https://crates.io/crates/static-bytes)
1. [Stepper]: Universal Stepper Motor Interface - ![crates.io](https://img.shields.io/crates/v/stepper.svg)
1. [tinybmp](https://crates.io/crates/tinybmp): No-std, no-alloc BMP parser for embedded systems. [Introductory blog post](https://wapl.es/rust/2019/03/04/embedded-graphics-0.4.7-bmp-support.html) - ![crates.io](https://img.shields.io/crates/v/tinybmp.svg)
1. [vga-framebuffer]: A VGA signal generator and font renderer for VGA-less microcontrollers. Used by [Monotron](https://github.com/thejpster/monotron) to generate 48 by 36 character display using 3 SPI peripherals and a timer. ![crates.io](https://img.shields.io/crates/v/vga-framebuffer.svg)
1. [wyhash]: A fast, simple and portable hashing algorithm and random number generator. - ![crates.io](https://img.shields.io/crates/v/wyhash.svg)

[`cmim`]: https://crates.io/crates/cmim
[`panic-persist`]: https://crates.io/crates/panic-persist
[bitmatch]: https://crates.io/crates/bitmatch
[biquad]: https://crates.io/crates/biquad
[embassy]: https://github.com/akiles/embassy
[dcmimu]: https://crates.io/crates/dcmimu
[debouncr]: https://crates.io/crates/debouncr
[device-driver]: https://crates.io/crates/device-driver
[endian_codec]: https://crates.io/crates/endian_codec
[menu]: https://github.com/thejpster/menu
[pc-keyboard]: https://github.com/thejpster/pc-keyboard
[qemu-exit]: https://crates.io/crates/qemu-exit
[Stepper]: https://crates.io/crates/stepper
[vga-framebuffer]: https://github.com/thejpster/vga-framebuffer-rs
[wyhash]: https://crates.io/crates/wyhash

### WIP

Work in progress crates. Help the authors make these crates awesome!

- [light-cli](https://github.com/rudihorn/light-cli): a lightweight heapless cli interface ![crates.io](https://img.shields.io/crates/v/light_cli.svg)
- [OxCC](https://github.com/jonlamb-gh/oxcc): A port of Open Source Car Control written in Rust
- [Rubble](https://github.com/jonas-schievink/rubble): A pure-Rust embedded BLE stack ![crates.io](https://img.shields.io/crates/v/rubble.svg)

[no-std-category]: https://crates.io/categories/no-std

## Firmware projects

- [anne-key](https://github.com/ah-/anne-key): Alternate keyboard firmware for the Obins ANNE Pro
- [e.ziclean cube vacuum cleaner](https://github.com/geomatsi/e.ziclean-cube): Experiments with open firmware for e.ziclean cube vacuum cleaner

## Old books, blogs and training materials

These materials may be outdated and reflect earlier practices, but might still be useful for reference.

-   [Exploring Rust on Teensy](https://branan.github.io/teensy/) by @branan — Beginner set of articles on getting into embedded dev in Rust.
-   [Pragmatic Bare Metal Rust](http://www.hashmismatch.net/pragmatic-bare-metal-rust/) A starter article about starting Rust development on STM32 microcontrollers (cubeMX + FFI).
-   [Using Rust in an Embedded Project: A Simple Example](https://spin.atomicobject.com/2016/07/08/rust-embedded-project-example/#.V3-os-6qlZw.hackernews) Article and some links on setting up Rust cross-compiling.
-   [Robigalia](https://robigalia.org) general purpose robust operating system in Rust running on secure seL4 microkernel.
-   [intermezzOS](http://intermezzos.github.io) A small teaching operating system in Rust. A book with some explanations is also included.
-   [Fearless concurrency](http://blog.japaric.io/fearless-concurrency/) by @japaric — How to easily develop Rust programs for pretty much any ARM Cortex-M microcontroller with memory-safe concurrency.
-   [Internet of Streams](https://www.youtube.com/playlist?list=PLX44HkctSkTewrL9frlUz0yeKLKecebT1) A video series  by [@jamesmunns] building a bare metal IoT Sensor Node Platform from (nearly) scratch in Rust
-   [Ferrous Systems' Embedded Training Courses: 2019 edition](https://github.com/ferrous-systems/embedded-trainings/) A hands-on training course for beginner and advanced learners of Embedded Rust, based on Nordic Semiconductor's nRF52 and Decawave's DWM1001-DEV harware. This training was given at Oxidize Conferences and by Ferrous Systems to corporate customers. 

[@jamesmunns]: https://github.com/jamesmunns

## License

This list is licensed under

- CC0 1.0 Universal License ([LICENSE-CC0](LICENSE-CC0) or
  https://creativecommons.org/publicdomain/zero/1.0/legalcode)

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], the maintainer of this crate, the [Resources team][team], promises
to intervene to uphold that code of conduct.

[CoC]: CODE_OF_CONDUCT.md
[team]: https://github.com/rust-embedded/wg#the-resources-team
