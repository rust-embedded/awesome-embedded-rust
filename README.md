# Embedded Rust

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

This is a curated list of resources related to embedded and low-level programming in the Rust programming language, including a selection of useful crates.

[<img src="https://rawgit.com/rust-embedded/awesome-embedded-rust/master/rust-embedded-logo-256x256.png" align="right" width="256">](http://www.rust-embedded.org)

This project is developed and maintained by the [Resources team][team].

**Don't see something you want or need here?** Add it to the [Not Yet Awesome Embedded Rust](https://github.com/rust-embedded/not-yet-awesome-embedded-rust) list!

## Table of contents

- [Embedded Rust](#embedded-rust)
  - [Table of contents](#table-of-contents)
  - [Community](#community)
    - [Community Chat Rooms](#community-chat-rooms)
  - [Books, blogs, and training materials](#books-blogs-and-training-materials)
  - [Tools](#tools)
  - [Real-time](#real-time)
    - [Real-time Operating System (RTOS)](#real-time-operating-system-rtos)
    - [Real-time tools](#real-time-tools)
  - [Peripheral Access Crates](#peripheral-access-crates)
    - [Microchip](#microchip)
    - [Nordic](#nordic)
    - [NXP](#nxp)
    - [Raspberry Pi Silicon](#raspberry-pi-silicon)
    - [SiFive](#sifive)
    - [Silicon Labs](#silicon-labs)
    - [StarFive](#starfive)
    - [STMicroelectronics](#stmicroelectronics)
    - [Texas Instruments](#texas-instruments)
    - [MSP430](#msp430)
    - [Espressif](#espressif)
    - [Ambiq Micro](#ambiq-micro)
    - [GigaDevice](#gigadevice)
    - [XMC](#xmc)
    - [Vorago](#vorago)
    - [Wiznet](#wiznet)
    - [Renesas](#renesas)
  - [HAL implementation crates](#hal-implementation-crates)
    - [OS](#os)
    - [Microchip](#microchip-1)
    - [Nordic](#nordic-1)
    - [NXP](#nxp-1)
    - [Raspberry Pi Silicon](#raspberry-pi-silicon-1)
    - [SiFive](#sifive-1)
    - [STMicroelectronics](#stmicroelectronics-1)
    - [Texas Instruments](#texas-instruments-1)
    - [MSP430](#msp430-1)
    - [Espressif](#espressif-1)
    - [Silicon Labs](#silicon-labs-1)
    - [XMC](#xmc-1)
    - [GigaDevice](#gigadevice-1)
    - [Vorago](#vorago-1)
    - [Renesas](#renesas-1)
    - [StarFive](#starfive-1)
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
    - [Pimoroni](#pimoroni)
    - [Raspberry Pi](#raspberry-pi)
    - [Sparkfun](#sparkfun)
    - [SeeedStudio](#seeedstudio)
    - [SiFive](#sifive-2)
    - [Sipeed](#sipeed)
    - [Sony](#sony)
    - [STMicroelectronics](#stmicroelectronics-2)
    - [Teensy](#teensy)
    - [Vorago](#vorago-2)
    - [Texas Instruments](#texas-instruments-2)
    - [Special Purpose](#special-purpose)
    - [Sodaq](#sodaq)
    - [Other](#other)
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

In 2018, the Rust community created an embedded working group to help drive adoption in the Rust ecosystem.

- [Embedded WG](https://github.com/rust-embedded/wg/), including newsletters with progress updates.

### Community Chat Rooms

- You can usually find community members (including embedded WG members) in the official [`#rust-embedded:matrix.org` Matrix room].
- [#rust-embedded-space:matrix.org] Most Embedded Rust related Matrix rooms are in the Rust Embedded Space
- [embedded.rs-wasm-iot] - English Telegram chat about Rust and WASM for microcontrollers and IoT
- [embedded.rs] - Telegram chat about Rust for microcontrollers in the Russian language.
- [#avr-rust:gitter.im] - For discussion of using Embedded Rust on AVR devices
- [#esp-rs:matrix.org] - For discussion of using Embedded Rust on Espressif devices
- [#nrf-rs:matrix.org] - For discussion of using Embedded Rust on Nordic Semiconductor devices
- [#probe-rs:matrix.org] - For discussion of the Probe-rs debugging toolkit
- [#rp-rs:matrix.org] - For discussion of using Embedded Rust on RP2040 based devices
- [#rtic-rs:matrix.org] - For discussion of the Real-Time Interrupt-driven Concurrency framework
- [#rust-embedded-graphics:matrix.org] - For discussion of the [`embedded-graphics`] crate and ecosystem
- [#stm32-rs:matrix.org] - For discussion of using Embedded Rust on STM32 based devices
- [#atsamd-rs:gitter.im] - For discussions of using Embedded Rust on ATSAMD devices
- [#ethercrab:matrix.org] - For discussion of general EtherCAT and the Rust implementation, EtherCrab

[#rust-embedded-graphics:matrix.org]: https://matrix.to/#/#rust-embedded-graphics:matrix.org
[#esp-rs:matrix.org]: https://matrix.to/#/#esp-rs:matrix.org
[`#rust-embedded:matrix.org` Matrix room]: https://matrix.to/#/#rust-embedded:matrix.org
[#rust-embedded-space:matrix.org]: https://matrix.to/#/#rust-embedded-space:matrix.org
[embedded.rs-wasm-iot]: https://t.me/embeddedrust
[embedded.rs]: https://t.me/embedded_rs
[#rtic-rs:matrix.org]: https://matrix.to/#/#rtic-rs:matrix.org
[#nrf-rs:matrix.org]: https://matrix.to/#/#nrf-rs:matrix.org
[#probe-rs:matrix.org]: https://matrix.to/#/#probe-rs:matrix.org
[`embedded-graphics`]: https://crates.io/crates/embedded-graphics
[#stm32-rs:matrix.org]: https://matrix.to/#/#stm32-rs:matrix.org
[#avr-rust:gitter.im]: https://matrix.to/#/#avr-rust_Lobby:gitter.im
[#rp-rs:matrix.org]: https://matrix.to/#/#rp-rs:matrix.org
[#atsamd-rs:gitter.im]: https://matrix.to/#/#atsamd-rs_community:gitter.im
[#ethercrab:matrix.org]: https://matrix.to/#/#ethercrab:matrix.org

## Books, blogs, and training materials

-   [The Embedded Rust Book](https://rust-embedded.github.io/book/) - An introductory book about using the Rust Programming Language on "Bare Metal" embedded systems, such as Microcontrollers.
-   [The Rust on ESP Book](https://esp-rs.github.io/book/) - This book aims to provide a comprehensive guide on using the Rust programming language with Espressif SoCs and modules.
-   [Discovery](https://rust-embedded.github.io/discovery) by @rust-embedded — this book is an introductory course on microcontroller-based embedded systems that uses Rust as the teaching language. Original author: @japaric
-   [Cortex-M Quickstart](https://docs.rs/cortex-m-quickstart/0.3.1/cortex_m_quickstart/) by @japaric – a template and introduction to embedded Rust, suitable for developers familiar with embedded development but new to embedded Rust.
- [Writing an OS in Rust](https://os.phil-opp.com/) A blog series creating a small operating system in Rust
-   [MicroRust](https://droogmic.github.io/microrust/) Introductory book for embedded development in Rust on the micro:bit.
-   [Physical Computing With Rust](https://rahul-thakoor.github.io/physical-computing-rust/) A (WIP) guide to physical computing with Rust on the Raspberry Pi.
-   [Writing an embedded OS in Rust on the Raspberry Pi](https://github.com/rust-embedded/rust-raspi3-OS-tutorials) A set of tutorials that give a guided, step-by-step tour of how to write a monolithic Operating System kernel for an embedded system from scratch. Runs on the Raspberry Pi 3 and the Raspberry Pi 4.
-   [Writing embedded drivers in Rust isn't that hard](https://hboeving.dev/blog/rust-2c-driver-p1/) A guide to building an embedded-hal driver. [Part 2](https://hboeving.dev/blog/rust-i2c-driver-p2/)
-   [Ferrous Systems' Embedded Training Courses: 2020-current edition](https://github.com/ferrous-systems/embedded-trainings-2020) A hands-on training course for beginner and advanced learners of Embedded Rust, based on Nordic Semiconductor's nRF52840 hardware. This training was given at Oxidize Conferences and by [Ferrous Systems] to corporate customers.
-   [Ferrous Systems' Knurling Sessions](https://knurling.ferrous-systems.com/sessions/) are hands-on embedded projects that explore specific concepts using generally available hardware, building full systems and components using microcontrollers, sensors, and actuators.
-   [Ferrous Systems' Embedded Rust on Espressif](https://esp-rs.github.io/std-training) - Training Material for learning to use Embedded Rust with the Espressif ESP32-C3.
-   [DSP on STM32F407G-DISC1](https://github.com/jacobrosenthal/dsp-discoveryf4-rust/) Unofficial oxidization of the [Digital Signal Processing using Arm Cortex-M based Microcontrollers: Theory and Practice](https://www.amazon.com/Digital-Signal-Processing-Cortex-M-Microcontrollers/dp/1911531166) book. The book isn't necessary to enjoy the examples and learn a functional DSP Rust coding style.
-   [Building a sailing starter board with Rust (RTIC)](https://gill.net.in/posts/stm32-pcb-sailing-and-rust/) A step-by-step story/guide to build STM32-based PCB and program it with Rust for fun and games.
-   [STM32F4xx with Embedded Rust at the HAL](https://apollolabsblog.hashnode.dev/series/stm32f4-embedded-rust-hal) A blog containing a series of tutorials demonstrating the use of several peripherals through simple examples leveraging the stm32f4xx-hal crate.
-   [Embedded Rust programming playlist](https://www.youtube.com/playlist?list=PLP_X41VhYn5X6Wwjnm0bRwI3n2pdaszxU) Various livestreams with Embedded Rust live coding
-   [ESP32-C3 Rust Tutorials](https://youtube.com/playlist?list=PLkch9g9DEE0Lkm1LqcD7pZNDmXEczOo-a) Short videos and [Github project](https://github.com/shanemmattner/ESP32-C3_Rust_Tutorials) implementing various peripherals of the ESP32-C3 with the end goal of creating a complete data logger application.
-   [Tweede golf's workshop](https://workshop.tweede.golf) - A full workshop about Rust and embedded Rust. The embedded parts use the nRF52840-DK and a LIS3DH breakout board. ([github source](https://github.com/tweedegolf/rust-workshop))

[Ferrous Systems]: https://ferrous-systems.com

## Tools

-   [xargo](https://github.com/japaric/xargo) Rust package manager with support for non-default std libraries — build Rust runtime for your embedded system.
    - xargo is great, but since it's in maintenance mode, [cargo-xbuild](https://github.com/rust-osdev/cargo-xbuild) is catching up as its intended replacement.
-   [svd2rust](https://github.com/japaric/svd2rust) Generate Rust structs with register mappings from SVD files.
-   [edc2svd](https://github.com/kiffie/edc2svd) Generate SVD files for PIC32 devices from EDC files. - ![crates.io](https://img.shields.io/crates/v/edc2svd.svg)
-   [embedded-hal-mock] Mock implementation of `embedded-hal` traits for testing without accessing real hardware. - ![crates.io](https://img.shields.io/crates/v/embedded-hal-mock.svg)
-   [bindgen](https://crates.io/crates/bindgen) Automatically generates Rust FFI bindings to C and C++ libraries. - ![crates.io](https://img.shields.io/crates/v/bindgen.svg)
-   [cortex-m semihosting](https://github.com/japaric/cortex-m-semihosting) Semihosting for ARM Cortex-M processors
-   [bobbin-cli](https://github.com/bobbin-rs/bobbin-cli) A Rust command line tool to simplify embedded development and deployment.
-   [ferros](https://github.com/auxoncorp/ferros) A Rust-based userland which also adds compile-time assurances to seL4 development.
-   [cargo-flash](https://probe.rs/docs/tools/cargo-flash/) A small cargo subcommand to download your binary to your target chip. - ![crates.io](https://img.shields.io/crates/v/cargo-flash.svg)
-   [cargo-embed](https://probe.rs/docs/tools/cargo-embed/) A superset of cargo-flash with additional useful features like configuration file support, an RTT terminal, or a GDB server. - ![crates.io](https://img.shields.io/crates/v/cargo-embed.svg)
-   [cargo-hf2](https://github.com/jacobrosenthal/hf2-rs)  A small cargo subcommand to download cargo builds to Microsoft UF2 bootloaders via HID USB . - ![crates.io](https://img.shields.io/crates/v/cargo-hf2.svg)
-   [cargo-bloat](https://github.com/RazrFalcon/cargo-bloat) Find out what takes most of the space in your executable.
-   [cargo-call-stack](https://crates.io/crates/cargo-call-stack) Static, whole program stack usage analyzer.
-   [cargo-dfu](https://crates.io/crates/cargo-dfu) Cargo extension for flashing embedded rust programs via DFU.
-   [espflash](https://github.com/esp-rs/espflash) Serial flasher utility for Espressif SoCs and modules. - ![crates.io](https://img.shields.io/crates/v/espflash.svg)
-   [espup](https://github.com/esp-rs/espup) Tool for installing and maintaining Espressif Rust ecosystem. - ![crates.io](https://img.shields.io/crates/v/espup.svg)
-   [uf2](https://github.com/sajattack/uf2conv-rs) Converts binary files to Microsoft's UF2 format for copying over to mass storage device uf2 bootloaders - ![crates.io](https://img.shields.io/crates/v/uf2.svg)
-   [Knurling Tools](https://knurling.ferrous-systems.com/tools/) are developed by [Ferrous Systems] to ease the development process for building, debugging, and testing embedded Rust systems. These tools include:
    - [Probe Run](https://github.com/knurling-rs/probe-run): a cargo runner to flash and run embedded applications just like you would native applications, including backtraces and panicking behavior
    - [defmt](https://github.com/knurling-rs/defmt): a highly efficient logging framework that targets resource-constrained devices, like microcontrollers.
    - [flip-link](https://github.com/knurling-rs/flip-link), a linker wrapper that provides stack overflow protection without an MMU by flipping the standard memory layout of ARM Cortex-M programs
    - [app-template](https://github.com/knurling-rs/app-template), a `cargo-generate` powered project template for quickly setting up new projects using the Knurling Tools.
    - [defmt-test](https://github.com/knurling-rs/defmt-test), an embedded test harness that lets you write and run unit tests as if you were using the built-in `#[test]` attribute, but will run on an embedded target
- [embedded-hal-compat](https://github.com/ryankurte/embedded-hal-compat), a compatibility layer to provide interoperability between `v0.2.x` and `v1.x.x` hal implementations and drivers
- [Embassy start](https://github.com/titanclass/embassy-start) is a GitHub repo template for setting up async embedded Rust projects that use [Embassy](https://github.com/embassy-rs/embassy). This particular template targets nRF hardware and networking using the Uarte for the purposes of illustration only.


[embedded-hal-mock]: https://crates.io/crates/embedded-hal-mock

## Real-time

### Real-time Operating System (RTOS)

-   [Drone OS](https://drone-os.github.io) An Embedded Operating System for writing real-time applications in Rust.
-   [FreeRTOS.rs](https://github.com/hashmismatch/freertos.rs) Rust interface for the FreeRTOS API
-   [FreeRTOS-rust](https://github.com/lobaro/FreeRTOS-rust) Rust interface for FreeRTOS with Rust entry point and build support crate.
-   [RIOT-OS](https://doc.riot-os.org/using-rust.html) directly supports applications written in Rust, both in terms of build system integration and by having safe and idiomatic wrappers.
-   [Tock](https://www.tockos.org) An embedded operating system designed for running multiple concurrent, mutually distrustful applications on low-memory and low-power microcontrollers
-   [Hubris](https://github.com/oxidecomputer/hubris) A real-time operating system built by Oxide Computer to run the Service Controller processor in the mainboards of their rack-mount servers.

### Real-time tools

-   [RTIC v1.0](https://rtic.rs/1/book/en/) Real-Time Interrupt-driven Concurrency — A concurrency framework for building real-time systems:
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
- [`atsame53`](https://github.com/atsamd-rs/atsamd) Peripheral access API for Microchip (formerly Atmel) SAME53 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`atsame54`](https://github.com/atsamd-rs/atsamd) Peripheral access API for Microchip (formerly Atmel) SAME54 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`atsamx7x-rust`](https://github.com/atsams-rs/atsamx7x-rust) Peripheral access API for Microchip (formerly Atmel) SAM S70/E70/V70/V71 microcontrollers.  This git repo hosts both the peripheral access crate and the hal.
- [`avr-device`](https://github.com/Rahix/avr-device) Peripheral access API for Microchip (formerly Atmel) AVR microcontroller family.
- [`sam3x8e`](https://crates.io/crates/sam3x8e) Peripheral access API for Atmel SAMD3X8E microcontrollers (generated using svd2rust)  - ![crates.io](https://img.shields.io/crates/v/sam3x8e.svg)
- [`pic32-pac`](https://crates.io/crates/pic32mx2xx) Peripheral access API for PIC32MX1/2xx - ![crates.io](https://img.shields.io/crates/v/pic32mx2xx)

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
- [`SKEAZN642`](https://crates.io/crates/SKEAZN642) Peripheral access API for KEA64 family microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/SKEAZN642.svg)

### Raspberry Pi Silicon

- [`rp2040-pac`](https://crates.io/crates/rp2040-pac) - Peripheral access API for the RP2040 dual-core system-on-chip (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/rp2040-pac.svg)

### SiFive

- [`e310x`](https://github.com/riscv-rust/e310x) - svd2rust generated interface to SiFive [Freedom E310](https://www.sifive.com/cores/e31) MCUs - ![crates.io](https://img.shields.io/crates/v/e310x.svg)

### Silicon Labs

- [`efm32pg12-pac`](https://crates.io/crates/efm32pg12-pac) - Peripheral access API for Silicon Labs EFM32PG12 microcontrollers - ![crates.io](https://img.shields.io/crates/v/efm32pg12-pac)

The [`efm32-rs`](https://github.com/efm32-rs) project has peripheral access APIs for most EFM32 microcontrollers (generated using svd2rust):

- [`efm32g-pac`](https://crates.io/crates/efm32g-pac) - ![crates.io](https://img.shields.io/crates/v/efm32g-pac)
- [`efm32gg-pac`](https://crates.io/crates/efm32gg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32gg-pac)
- [`efm32gg11b-pac`](https://crates.io/crates/efm32gg11b-pac) - ![crates.io](https://img.shields.io/crates/v/efm32gg11b-pac)
- [`efm32gg12b-pac`](https://crates.io/crates/efm32gg12b-pac) - ![crates.io](https://img.shields.io/crates/v/efm32gg12b-pac)
- [`efm32hg-pac`](https://crates.io/crates/efm32hg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32hg-pac)
- [`efm32jg1b-pac`](https://crates.io/crates/efm32jg1b-pac) - ![crates.io](https://img.shields.io/crates/v/efm32jg1b-pac)
- [`efm32jg12b-pac`](https://crates.io/crates/efm32jg12b-pac) - ![crates.io](https://img.shields.io/crates/v/efm32jg12b-pac)
- [`efm32lg-pac`](https://crates.io/crates/efm32lg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32lg-pac)
- [`efm32pg-pac`](https://crates.io/crates/efm32pg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32pg-pac)
- [`efm32pg22-pac`](https://crates.io/crates/efm32pg22-pac) - ![crates.io](https://img.shields.io/crates/v/efm32pg22-pac)
- [`efm32pg23-pac`](https://crates.io/crates/efm32pg23-pac) - ![crates.io](https://img.shields.io/crates/v/efm32pg23-pac)
- [`efm32tg-pac`](https://crates.io/crates/efm32tg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32tg-pac)
- [`efm32tg11b-pac`](https://crates.io/crates/efm32tg11b-pac) - ![crates.io](https://img.shields.io/crates/v/efm32tg11b-pac)
- [`efm32wg-pac`](https://crates.io/crates/efm32wg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32wg-pac)
- [`efm32zg-pac`](https://crates.io/crates/efm32zg-pac) - ![crates.io](https://img.shields.io/crates/v/efm32zg-pac)

### StarFive

- [`j71xx-pac`](https://crates.io/crates/jh71xx-pac) - svd2rust generated interface to StarFive [JH71xx](https://www.starfivetech.com/en/site/soc) MCUs - ![crates.io](https://img.shields.io/crates/v/jh71xx-pac.svg)

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
    - [msp430 quickstart](https://github.com/rust-embedded/msp430-quickstart) some examples for msp430
-   [`msp430fr2355`](https://crates.io/crates/msp430fr2355) Peripheral access API for MSP430FR2355 microcontrollers (generated using svd2rust)
- [`msp430fr6972`](https://crates.io/crates/msp430fr6972) - ![Crates.io](https://img.shields.io/crates/v/msp430fr6972)

### Espressif

- [`esp32`](https://github.com/esp-rs/esp-pacs/tree/main/esp32) - ![crates.io](https://img.shields.io/crates/v/esp32.svg)
- [`esp32c2`](https://github.com/esp-rs/esp-pacs/tree/main/esp32c2) - ![crates.io](https://img.shields.io/crates/v/esp32c2.svg)
- [`esp32c3`](https://github.com/esp-rs/esp-pacs/tree/main/esp32c3) - ![crates.io](https://img.shields.io/crates/v/esp32c3.svg)
- [`esp32c6`](https://github.com/esp-rs/esp-pacs/tree/main/esp32c6) - ![crates.io](https://img.shields.io/crates/v/esp32c6.svg)
- [`esp32s2`](https://github.com/esp-rs/esp-pacs/tree/main/esp32s2) - ![crates.io](https://img.shields.io/crates/v/esp32s2.svg)
- [`esp32s3`](https://github.com/esp-rs/esp-pacs/tree/main/esp32s3) - ![crates.io](https://img.shields.io/crates/v/esp32s3.svg)
- [`esp8266`](https://github.com/esp-rs/esp-pacs/tree/main/esp8266) - ![crates.io](https://img.shields.io/crates/v/esp8266.svg)

### Ambiq Micro

- [`ambiq-apollo1-pac`](https://crates.io/crates/ambiq-apollo1-pac) Peripheral access API for Ambiq Apollo 1 microcontrollers (generated using svd2rust)
- [`ambiq-apollo2-pac`](https://crates.io/crates/ambiq-apollo2-pac) Peripheral access API for Ambiq Apollo 2 microcontrollers (generated using svd2rust)
- [`ambiq-apollo3-pac`](https://crates.io/crates/ambiq-apollo3-pac) Peripheral access API for Ambiq Apollo 3 microcontrollers (generated using svd2rust)
- [`ambiq-apollo3p-pac`](https://crates.io/crates/ambiq-apollo3p-pac) Peripheral access API for Ambiq Apollo 3 Plus microcontrollers (generated using svd2rust)

### GigaDevice

- [`gd32vf103-pac`](https://github.com/riscv-rust/gd32vf103-pac) Peripheral access API for GD32VF103 RISC-V microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/gd32vf103-pac.svg)
- [`gd32e2`](https://crates.io/crates/gd32e2) Peripheral access API for GD32E23x Cortex-M23 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/gd32e2.svg)
- [`gd32f1`](https://crates.io/crates/gd32f1) Peripheral access API for GD32F1x0 Cortex-M3 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/gd32f1.svg)
- [`gd32f2`](https://crates.io/crates/gd32f2) Peripheral access API for GD32F20x Cortex-M3 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/gd32f2.svg)

### XMC

Peripheral access crates for the different XMC4xxx families of microcontrollers

- [`xmc4100`](https://github.com/xmc-rs/xmc4100) - ![crates.io](https://img.shields.io/crates/v/xmc4100.svg)
- [`xmc4200`](https://github.com/xmc-rs/xmc4200) - ![crates.io](https://img.shields.io/crates/v/xmc4200.svg)
- [`xmc4300`](https://github.com/xmc-rs/xmc4300) - ![crates.io](https://img.shields.io/crates/v/xmc4300.svg)
- [`xmc4400`](https://github.com/xmc-rs/xmc4400) - ![crates.io](https://img.shields.io/crates/v/xmc4400.svg)
- [`xmc4500`](https://github.com/xmc-rs/xmc4500) - ![crates.io](https://img.shields.io/crates/v/xmc4500.svg)
- [`xmc4700`](https://github.com/xmc-rs/xmc4700) - ![crates.io](https://img.shields.io/crates/v/xmc4700.svg)
- [`xmc4800`](https://github.com/xmc-rs/xmc4800) - ![crates.io](https://img.shields.io/crates/v/xmc4800.svg)

### Vorago

- [`va108xx`](https://egit.irs.uni-stuttgart.de/rust/va108xx) - ![crates.io](https://img.shields.io/crates/v/va108xx.svg)
- [`va416xx`](https://egit.irs.uni-stuttgart.de/rust/va416xx) - ![crates.io](https://img.shields.io/crates/v/va416xx.svg)

### Wiznet

- [`w7500x-pac`](https://crates.io/crates/w7500x-pac) Peripheral Access Crate for Wiznet's W7500x microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/w7500x-pac.svg)

### Renesas
- [`ra2a1`](https://github.com/ra-rs/ra/tree/main/pac/ra2a1) Peripheral Access Crate for ra2a1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra2a1.svg)
- [`ra2e1`](https://github.com/ra-rs/ra/tree/main/pac/ra2e1) Peripheral Access Crate for ra2e1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra2e1.svg)
- [`ra2e2`](https://github.com/ra-rs/ra/tree/main/pac/ra2e2) Peripheral Access Crate for ra2e2 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra2e2.svg)
- [`ra2l1`](https://github.com/ra-rs/ra/tree/main/pac/ra2l1) Peripheral Access Crate for ra2l1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra2l1.svg)
- [`ra4e1`](https://github.com/ra-rs/ra/tree/main/pac/ra4e1) Peripheral Access Crate for ra4e1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra4e1.svg)
- [`ra4m1`](https://github.com/ra-rs/ra/tree/main/pac/ra4m1) Peripheral Access Crate for ra4m1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra4m1.svg)
- [`ra4m2`](https://github.com/ra-rs/ra/tree/main/pac/ra4m2) Peripheral Access Crate for ra4m2 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra4m2.svg)
- [`ra4m3`](https://github.com/ra-rs/ra/tree/main/pac/ra4m3) Peripheral Access Crate for ra4m3 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra4m3.svg)
- [`ra4w1`](https://github.com/ra-rs/ra/tree/main/pac/ra4w1) Peripheral Access Crate for ra4w1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra4w1.svg)
- [`ra6e1`](https://github.com/ra-rs/ra/tree/main/pac/ra6e1) Peripheral Access Crate for ra6e1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6e1.svg)
- [`ra6m1`](https://github.com/ra-rs/ra/tree/main/pac/ra6m1) Peripheral Access Crate for ra6m1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6m1.svg)
- [`ra6m2`](https://github.com/ra-rs/ra/tree/main/pac/ra6m2) Peripheral Access Crate for ra6m2 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6m2.svg)
- [`ra6m3`](https://github.com/ra-rs/ra/tree/main/pac/ra6m3) Peripheral Access Crate for ra6m3 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6m3.svg)
- [`ra6m4`](https://github.com/ra-rs/ra/tree/main/pac/ra6m4) Peripheral Access Crate for ra6m4 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6m4.svg)
- [`ra6t1`](https://github.com/ra-rs/ra/tree/main/pac/ra6t1) Peripheral Access Crate for ra6t1 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6t1.svg)
- [`ra6t2`](https://github.com/ra-rs/ra/tree/main/pac/ra6t2) Peripheral Access Crate for ra6t2 microcontrollers (generated using svd2rust) - ![crates.io](https://img.shields.io/crates/v/ra6t2.svg)
- [`da14531`](https://crates.io/crates/da14531) Peripheral Access Crate for DA14531 Ultra-Low Power BT 5.1 System-on-Chip - ![crates.io](https://img.shields.io/crates/v/da14531.svg)


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
- [`freebsd-embedded-hal`] for embedded (or [not](https://www.freebsd.org/cgi/man.cgi?query=cp2112&sektion=4)) FreeBSD systems. - ![crates.io](https://img.shields.io/crates/v/freebsd-embedded-hal.svg)

[`bitbang-hal`]: https://crates.io/crates/bitbang-hal
[`ftdi-embedded-hal`]: https://crates.io/crates/ftdi-embedded-hal
[`linux-embedded-hal`]: https://crates.io/crates/linux-embedded-hal
[`freebsd-embedded-hal`]: https://crates.io/crates/freebsd-embedded-hal

### Microchip

- [`atsam4-hal`](https://crates.io/crates/atsam4-hal) - HAL for SAM4E, SAM4N and SAM4S - ![crates.io](https://img.shields.io/crates/v/atsam4-hal.svg)
- [`atsamd-hal`](https://crates.io/crates/atsamd-hal) - HAL for SAMD11, SAMD21, SAMD51 and SAME54 - ![crates.io](https://img.shields.io/crates/v/atsamd-hal.svg)
- [`atsamx7x-hal`](https://crates.io/crates/atsamx7x-hal) - HAL for SAM S70/E70/V70/V71-based devices - ![crates.io](https://img.shields.io/crates/v/atsamx7x-hal.svg)
- [`avr-hal`](https://github.com/Rahix/avr-hal) - HAL for AVR microcontroller family and AVR-based boards
- [`pic32-hal`](https://crates.io/crates/pic32-hal) - HAL for PIC32MX - ![crates.io](https://img.shields.io/crates/v/pic32-hal.svg)

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

### Raspberry Pi Silicon

- [`rp2040-hal`](https://crates.io/crates/rp2040-hal) - HAL for the RP2040 dual-core system-on-chip - ![crates.io](https://img.shields.io/crates/v/rp2040-hal.svg)

### SiFive

- [`e310x-hal`](https://github.com/riscv-rust/e310x-hal) - HAL for SiFive [Freedom E310](https://www.sifive.com/cores/e31) MCUs - ![crates.io](https://img.shields.io/crates/v/e310x-hal.svg)

### STMicroelectronics

Also check the list of [STMicroelectronics board support crates][stm-bsc]!

[stm-bsc]: #stmicroelectronics-2

- [`stm32f0xx-hal`](https://crates.io/crates/stm32f0xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f0xx-hal.svg)
  - Has examples that can run on boards like the [Nucleo-F042K6] and similar boards

[Nucleo-F042K6]: http://www.st.com/en/evaluation-tools/nucleo-f042k6.html

- [`stm32f1xx-hal`](https://github.com/stm32-rs/stm32f1xx-hal) - ![crates.io](https://img.shields.io/crates/v/stm32f1xx-hal.svg)
  - Can be run on boards like the [Blue pill], [Nucleo-F103RB], and similar boards

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
   - HAL implementation for STM32 devices across multiple families, with a focus on newer ones like L4, L5, and H7.

[Nucleo-L432KC]: https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-l432kc.html
[Solo]: https://solokeys.com/

### Texas Instruments

- [`tm4c123x-hal`](https://github.com/rust-embedded-community/tm4c-hal/)

### MSP430

- [`msp430fr2x5x-hal`](https://crates.io/crates/msp430fr2x5x-hal)
    - HAL implementation for the MSP430FR2x5x family of microcontrollers

### Espressif

- [`esp-idf-hal`](https://github.com/esp-rs/esp-idf-hal)
  - An embedded-hal implementation for Rust on ESP32 microcontrollers and ESP-IDF
- [`esp-hal`](https://github.com/esp-rs/esp-hal)
  - A `no_std` Hardware Abstraction Layers for ESP32 microcontrollers


### Silicon Labs

- [`tomu-hal`](https://github.com/fudanchii/imtomu-rs)
   - HAL implementation targeted for [Tomu] USB board with EFM32HG309F64 ARMv6-M core. Has support to configure [tomu bootloader] directly from an application via the `toboot_config` macro.

[Tomu]: https://tomu.im/
[tomu bootloader]: https://github.com/im-tomu/tomu-bootloader

### XMC

- [`xmc1100-hal`](https://github.com/david-sawatzke/xmc1100-hal) - ![crates.io](https://img.shields.io/crates/v/xmc1100-hal.svg)
- [`xmc4-hal`](https://github.com/xmc-rs/xmc4-hal) - ![crates.io](https://img.shields.io/crates/v/xmc4-hal.svg)

### GigaDevice

- [`gd32vf103xx-hal`](https://github.com/riscv-rust/gd32vf103xx-hal) - ![crates.io](https://img.shields.io/crates/v/gd32vf103xx-hal.svg)
  - HAL for GD32VF103xx microcontrollers
- [`gd32vf103-hal`](https://github.com/luojia65/gd32vf103-hal) - ![crates.io](https://img.shields.io/crates/v/gd32vf103-hal.svg)
  - (WIP) Hardware abstract layer (HAL) for the GD32VF103 RISC-V microcontroller
- [`gd32f1x0-hal`](https://crates.io/crates/gd32f1x0-hal) - ![crates.io](https://img.shields.io/crates/v/gd32f1x0-hal.svg)
  - HAL implementation for GD32F1x0 microcontrollers

### Vorago

- [`va108xx-hal`](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal) - ![crates.io](https://img.shields.io/crates/v/va108xx-hal.svg)
  - [Blogpost](https://robamu.github.io/post/rust-ecosystem/)

### Renesas
- [`da14531-hal`](https://crates.io/crates/da14531-hal) HAL crate for DA14531 Ultra-Low Power BT 5.1 System-on-Chip - ![crates.io](https://img.shields.io/crates/v/da14531-hal.svg)

### StarFive

- [`j71xx-hal`](https://crates.io/crates/jh71xx-hal) - HAL crate for StarFive [JH71xx](https://www.starfivetech.com/en/site/soc) MCUs - ![crates.io](https://img.shields.io/crates/v/jh71xx-hal.svg)

## Architecture support crates

Crates tailored for general CPU architectures.

### ARM

- [`cortex-a`](https://github.com/andre-richter/cortex-a) Low-level access to Cortex-A processors (early state) - ![crates.io](https://img.shields.io/crates/v/cortex-a.svg)
- [`cortex-m`](https://github.com/japaric/cortex-m) Low-level access to Cortex-M processors - ![crates.io](https://img.shields.io/crates/v/cortex-m.svg)

### RISC-V

- [`riscv`](https://github.com/rust-embedded/riscv) Low-level access to RISC-V processors - ![crates.io](https://img.shields.io/crates/v/riscv.svg)

### MIPS

- [`mips`](https://github.com/Harry-Chen/rust-mips) Low-level access to MIPS32 processors - ![crates.io](https://img.shields.io/crates/v/mips.svg)
- [`mips-mcu`](https://github.com/kiffie/pic32-rs/tree/master/mips-mcu) Low-level access to MIPS MCU cores - ![crates.io](https://img.shields.io/crates/v/mips-mcu.svg)

## Board support crates

Crates tailored for specific boards.

[STM32F3DISCOVERY]: http://www.st.com/en/evaluation-tools/stm32f3discovery.html
[STM32F4DISCOVERY]: https://www.st.com/en/evaluation-tools/stm32f4discovery.html
[STM32F429DISCOVERY]: https://www.st.com/en/evaluation-tools/32f429idiscovery.html
[atsamd-rs]: https://github.com/atsamd-rs/atsamd
[atsamd-rs tier 1 support]: https://github.com/atsamd-rs/atsamd#how-to-use-a-bsp-ie-getting-started-writing-your-own-code
[atsamd-rs tier 2 support]: https://github.com/atsamd-rs/atsamd#how-to-use-a-bsp-ie-getting-started-writing-your-own-code

### 1BitSquared

- [`onebitsy`](https://crates.io/crates/onebitsy) - Board support crate for the [1bitsy] STM32F4-based board - ![crates.io](https://img.shields.io/crates/v/onebitsy.svg)

[1bitsy]: https://1bitsy.org/

### Adafruit

- [`metro_m0`](https://crates.io/crates/metro_m0) - Board support for the [Metro M0 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 1 support] board. ![crates.io](https://img.shields.io/crates/v/metro_m0.svg)
- [`metro_m4`](https://crates.io/crates/metro_m4) - Board support for the [Metro M4 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 1 support] board. ![crates.io](https://img.shields.io/crates/v/metro_m4.svg)
- [`pyportal`](https://crates.io/crates/pyportal) - Board support for the [PyPortal board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/pyportal.svg)
- [`pygamer`](https://crates.io/crates/pygamer) - Board support for the [PyGamer board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 1 support] board. ![crates.io](https://img.shields.io/crates/v/pygamer.svg)
- [`trellis_m4`](https://crates.io/crates/trellis_m4) - Board support for the [NeoTrellis M4 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/trellis_m4.svg)
- [`feather-f405`](https://crates.io/crates/feather-f405) - Board support for the [Feather STM32F405 Express]. ![crates.io](https://img.shields.io/crates/v/feather-f405.svg)
- [`feather_m0`](https://crates.io/crates/feather_m0) - Board support for the [Feather M0 board], and some variants in the [atsamd-rs] repo. It is an [atsamd-rs tier 1 support] board. ![crates.io](https://img.shields.io/crates/v/feather_m0.svg)
- [`feather_m4`](https://crates.io/crates/feather_m4) - Board support for the [Feather M4 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 1 support] board. ![crates.io](https://img.shields.io/crates/v/feather_m4.svg)
- [`circuit_playground_express`](https://crates.io/crates/circuit_playground_express) - Board support for the [Circuit Playground Express board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/circuit_playground_express.svg)
- [`edgebadge`](https://crates.io/crates/edgebadge) - Board support for the [EdgeBadge board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/edgebadge.svg)
- [`gemma_m0`](https://crates.io/crates/gemma_m0) - Board support for the [Gemma M0 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/gemma_m0.svg)
- [`itsybitsy_m0`](https://crates.io/crates/itsybitsy_m0) - Board support for the [ItsyBitsy M0 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/itsybitsy_m0.svg)
- [`itsybitsy_m4`](https://crates.io/crates/itsybitsy_m4) - Board support for the [ItsyBitsy M4 Express board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/itsybitsy_m4.svg)
- [`trinket_m0`](https://crates.io/crates/trinket_m0) - Board support for the [Trinket M0 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/trinket_m0.svg)
- [`neo_trinkey`](https://crates.io/crates/neo_trinkey) - Board support for the [neo trinkey board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/neo_trinkey.svg)
- [`neokey_trinkey`](https://crates.io/crates/neokey_trinkey) - Board support for the [neokey trinkey board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/neokey_trinkey.svg)
- [`grand_central_m4`](https://crates.io/crates/grand_central_m4) - Board support for the [grand central m4 board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/grand_central_m4.svg)
- [`qt_py_m0`](https://crates.io/crates/qt_py_m0) - Board support for the [QT Py board] in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/qt_py_m0.svg)
- [`adafruit-feather-rp2040`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the [Adafruit Feather RP2040] ![crates.io](https://img.shields.io/crates/v/adafruit-feather-rp2040.svg)
- [`adafruit-itsy-bitsy-rp2040`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the [Adafruit ItsyBitsy RP2040] ![crates.io](https://img.shields.io/crates/v/adafruit-itsy-bitsy-rp2040.svg)
- [`adafruit-kb2040`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the [Adafruit KB2040] ![crates.io](https://img.shields.io/crates/v/adafruit-kb2040.svg)
- [`adafruit-macropad`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the [Adafruit Macropad] ![crates.io](https://img.shields.io/crates/v/adafruit-macropad.svg)
- [`adafruit-qt-py-rp2040`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the [Adafruit QT Py RP2040] ![crates.io](https://img.shields.io/crates/v/adafruit-qt-py-rp2040.svg)

[Metro M0 board]: https://www.adafruit.com/product/3505
[Metro M4 board]: https://www.adafruit.com/product/3382
[PyPortal board]: https://www.adafruit.com/product/4116
[PyGamer board]: https://www.adafruit.com/product/4242
[NeoTrellis M4 board]: https://www.adafruit.com/product/3938
[Feather STM32F405 Express]: https://www.adafruit.com/product/4382
[Feather M0 board]: https://www.adafruit.com/product/2772
[Feather M4 board]: https://www.adafruit.com/product/3857
[Circuit Playground Express board]: https://www.adafruit.com/product/3333
[EdgeBadge board]: https://www.adafruit.com/product/4400
[Gemma M0 board]: https://www.adafruit.com/product/3501
[ItsyBitsy M0 board]: https://www.adafruit.com/product/3727
[ItsyBitsy M4 Express board]: https://www.adafruit.com/product/3800
[Trinket M0 board]: https://www.adafruit.com/product/3500
[neo trinkey board]: https://www.adafruit.com/product/4870
[neokey trinkey board]: https://www.adafruit.com/product/5020
[grand central m4 board]: https://www.adafruit.com/product/4064
[QT Py board]: https://www.adafruit.com/product/4600
[Adafruit Feather RP2040]: https://www.adafruit.com/product/4884
[Adafruit ItsyBitsy RP2040]: https://www.adafruit.com/product/4888
[Adafruit KB2040]: https://www.adafruit.com/product/5302
[Adafruit Macropad]: https://www.adafruit.com/product/5128
[Adafruit QT Py RP2040]: https://www.adafruit.com/product/4900

### Arduino

- [`avr-hal`](https://github.com/Rahix/avr-hal) - Board support crate for several AVR-based boards including the Arduino Uno and the Arduino Leonardo
- [`arduino_mkr1000`](https://crates.io/crates/arduino_mkr1000) - Board support for the [MKR 1000 WiFi board](https://docs.arduino.cc/hardware/mkr-1000-wifi) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/arduino_mkr1000.svg)
- [`arduino_mkrvidor4000`](https://crates.io/crates/arduino_mkrvidor4000) - Board support for the [MKR Vidor board](https://store.arduino.cc/usa/mkr-vidor-4000) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/arduino_mkrvidor4000.svg)
- [`arduino_mkrzero`](https://crates.io/crates/arduino_mkrzero) - Board support for the [mkrzero board](https://store.arduino.cc/arduino-mkrzero) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/arduino_mkrzero.svg)
- [`arduino_nano33iot`](https://crates.io/crates/arduino_nano33iot) - Board support for the [Arduino Nano 33 IoT](https://store.arduino.cc/products/arduino-nano-33-iot) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/arduino_nano33iot.svg)

### Nordic

- [`dwm1001`](https://crates.io/crates/dwm1001) - [Decawave DWM1001-DEV] - ![crates.io](https://img.shields.io/crates/v/dwm1001.svg)
- [`microbit`](https://crates.io/crates/microbit) - [micro:bit] - ![crates.io](https://img.shields.io/crates/v/microbit.svg)
- [`nrf52840-dk-bsp`](https://crates.io/crates/nrf52840-dk-bsp) - [nrf52840-dk] - ![crates.io](https://img.shields.io/crates/v/nrf52840-dk-bsp.svg)
- [`Thingy:91-nrf9160`](https://crates.io/crates/thingy-91-nrf9160-bsp) - [thingy:91] - ![crates.io](https://img.shields.io/crates/v/thingy-91-nrf9160-bsp.svg)

[Decawave DWM1001-DEV]: https://www.decawave.com/product/dwm1001-development-board/
[micro:bit]: http://microbit.org/
[nrf52840-dk]: https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dk
[thingy:91]: https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-91

### NXP

- [`frdm-kw41z`](https://crates.io/crates/frdm-kw41z) - [FRDM-KW41Z] - ![crates.io](https://img.shields.io/crates/v/frdm-kw41z.svg)

[FRDM-KW41Z]: https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/kinetis-cortex-m-mcus/w-serieswireless-conn.m0-plus-m4/freedom-development-kit-for-kinetis-kw41z-31z-21z-mcus:FRDM-KW41Z

### Pimoroni

- [`pimoroni-pico-explorer`](https://github.com/rp-rs/rp-hal) - Board Support for the [Pimoroni Pico Explorer]
- [`pimoroni-pico-lipo-16mb`](https://github.com/rp-rs/rp-hal) - Board Support for the [Pimoroni Pico Lipo 16MB]

[pimoroni-pico-explorer]: https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni-pico-explorer
[Pimoroni Pico Explorer]: https://shop.pimoroni.com/products/pico-explorer-base
[pimoroni-pico-lipo-16mb]: https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni_pico_lipo_16mb
[Pimoroni Pico Lipo 16MB]: https://shop.pimoroni.com/products/pimoroni-pico-lipo?variant=39335427080275

### Raspberry Pi

- [`rp-pico`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the RP2040-based Raspberry Pi Pico.

### Sparkfun

- [`samd21_mini`](https://crates.io/crates/samd21_mini) - Board support for the [SAMD21 Mini Breakout](https://www.sparkfun.com/products/13664) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/samd21_mini.svg)
- [`sparkfun-pro-micro-rp2040`](https://github.com/rp-rs/rp-hal) - Board Support Crate for the RP2040 based Sparkfun Pro Micro.

### SeeedStudio

- [`seedstudio-gd32v`](https://github.com/riscv-rust/seedstudio-gd32v) - Board support crate for the [GD32 RISC-V Dev Board](https://www.seeedstudio.com/SeeedStudio-GD32-RISC-V-Dev-Board-p-4302.html)
  ![crates.io](https://img.shields.io/crates/v/seedstudio-gd32v.svg)
  - Contains runnable examples for this board
- [`xiao_m0`](https://crates.io/crates/xiao_m0) - Board support for the [Seeed Studio Seeeduino XIAO](http://wiki.seeedstudio.com/Seeeduino-XIAO/) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/xiao_m0.svg)
- [`wio_lite_mg126`](https://crates.io/crates/wio_lite_mg126) - Board support for the [Seeed Studio wio_lite_mg126](https://wiki.seeedstudio.com/Wio-Lite-MG126) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io.crates/v/wio_lite_mg126.svg)
- [`wio_lite_w600`](https://crates.io/crates/wio_lite_w600) - Board support for the [Seeed Studio wio_lite_w600](https://wiki.seeedstudio.com/Wio-Lite-W600/) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io.crates/v/wio_lite_w600.svg)
- [`wio_terminal`](https://crates.io/crates/wio_terminal) - Board support for the [Seeed Studio wio_terminal](https://wiki.seeedstudio.com/Wio-Terminal-Getting-Started/) in the [atsamd-rs] repo. It is an [atsamd-rs tier 1 support] board ![crates.io](https://img.shields.io.crates/v/wio_terminal.svg)

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
- [`nucleo-h743zi`](https://github.com/astraw/nucleo-h743zi) Beginner-oriented support crate for the Nucleo H743ZI and Nucleo H743ZI2
- [`nucleo-h723zg`](https://github.com/jlogan03/nucleo-h723zg) Board support crate for the Nucleo H723ZG
- [`solo-bsp`](https://crates.io/crates/solo-bsp) Board Support Crate for [Solo], an open source security token (WIP) - ![crates.io](https://img.shields.io/crates/v/solo-bsp.svg)
- [`stm32f407g-disc`](https://crates.io/crates/stm32f407g-disc) Board Support Crate for the [STM32F4DISCOVERY] (WIP) - ![crates.io](https://img.shields.io/crates/v/stm32f407g-disc.svg)
- [`stm32f429i-disc`](https://crates.io/crates/stm32f429i-disc) Board Support Crate for the [STM32F429DISCOVERY] (WIP) - ![crates.io](https://img.shields.io/crates/v/stm32f429i-disc.svg)
- [`stm32f3-discovery`](https://crates.io/crates/stm32f3-discovery) Board Support Crate for the [STM32F3DISCOVERY] used in [Rust Embedded Discovery book](https://rust-embedded.github.io/discovery/index.html) - ![crates.io](https://img.shields.io/crates/v/stm32f3-discovery)


[Nucleo-F401RE]: https://www.st.com/en/evaluation-tools/nucleo-f401re.html

### Teensy
- [`teensy4-rs`](https://github.com/mciantyre/teensy4-rs) Board Support crate for the [Teensy 4.0]

### Vorago

- [`vorago-reb1`](https://egit.irs.uni-stuttgart.de/rust/vorago-reb1) - ![crates.io](https://img.shields.io/crates/v/vorago-reb1.svg)
  - [Blogpost](https://robamu.github.io/post/rust-ecosystem/)

### Texas Instruments

- [`monotron`](https://github.com/thejpster/monotron) - A 1980s home-computer style application for the Texas Instruments Stellaris Launchpad. PS/2 keyboard input, text output on a bit-bashed 800x600 VGA signal. Uses [menu], [vga-framebuffer] and [pc-keyboard].
- [`stellaris-launchpad`](https://crates.io/crates/stellaris-launchpad) - For the Texas Instruments Stellaris Launchpad and Tiva-C Launchpad ![crates.io](https://img.shields.io/crates/v/stellaris-launchpad.svg)
- [`tm4c129-launchpad`](https://github.com/jlogan03/tm4c129-launchpad) - For the Texas Instruments TM4C129-XL Launchpad board

### Special Purpose

- [`betafpv-f3`](https://github.com/JoshMcguigan/betafpv-f3) - For the BetaFPV F3 drone flight controller

### Sodaq

- [`sodaq_one`](https://crates.io/crates/sodaq_one) - Board support for the [Sodaq one board](https://shop.sodaq.com/sodaq-one-eu-rn2483-v3.html) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/sodaq_one.svg)
- [`sodaq_sara_aff`](https://crates.io/crates/sodaq_sara_aff) - Board support for the [Sodaq sara aff board](https://shop.sodaq.com/sodaq-sara-sff-r410m.html) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/sodaq_sara_aff.svg)

### Other

- [`serpente`](https://crates.io/crates/serpente) - Board support for the [serpente board](https://www.solder.party/docs/serpente/) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/serpente.svg)
- [`p1am_100`](https://crates.io/crates/p1am_100) - Board support for the [p1am_100 board](https://facts-engineering.github.io) in the [atsamd-rs] repo. It is an [atsamd-rs tier 2 support] board. ![crates.io](https://img.shields.io/crates/v/p1am_100.svg)

## Component abstraction crates

The following crates provide HAL-like abstractions for subcomponents of embedded
devices that go beyond what is available in [`embedded-hal`]:

- [`accelerometer`](https://github.com/NeoBirth/accelerometer.rs) - Generic accelerometer support, including traits and types for taking readings from 2 or 3-axis accelerometers and tracking device orientations - ![crates.io](https://img.shields.io/crates/v/accelerometer.svg)
- [`embedded-graphics`]: 2D drawing library for any size display - ![crates.io](https://img.shields.io/crates/v/embedded-graphics.svg)
- [`radio`](https://github.com/ryankurte/rust-radio) - Generic radio transceiver traits, mocks, and helpers - ![crates.io](https://img.shields.io/crates/v/radio.svg)
- [`smart-leds`](https://github.com/smart-leds-rs): Support for addressable LEDs including WS2812 and APA102
- [`usb-device`](https://github.com/mvirkkunen/usb-device): Abstraction layer between USB peripheral crates & USB class crates - ![crates.io](https://img.shields.io/crates/v/usb-device.svg)
- [`atat`](https://github.com/BlackbirdHQ/atat): Abstraction crate to ease writing AT based driver crates - ![crates.io](https://img.shields.io/crates/v/atat.svg)
- [`embedded-nal`](https://github.com/rust-embedded-community/embedded-nal): An Embedded Network Abstraction Layer - ![crates.io](https://img.shields.io/crates/v/embedded-nal.svg)
- [`embedded-storage`](https://github.com/rust-embedded-community/embedded-storage): An Embedded Storage Abstraction Layer
- [`switch-hal`](https://github.com/rubberduck203/switch-hal): An "on"/"off" abstraction for input and output switches - ![crates.io](https://img.shields.io/crates/v/switch-hal.svg)


## Driver crates

Platform agnostic crates to interface external components. These crates use the [`embedded-hal`]
interface to support [all the devices and systems that implement the `embedded-hal`
traits][hal-impl].

[hal-impl]: #hal-implementation-crates

The list below contains drivers that have achieved the "released" status. To add a driver
to this list, please ensure that your driver has a short blog post, article, or sufficiently
explanatory README showing an example of its use. Ideally, this post would demonstrate using the
device in a small project so that a Rust and/or embedded newcomer can also understand it.
Otherwise, please add it to the [WIP section](#WIP) below.

1. [AD983x] - SPI - AD9833/AD9837 waveform generators / DDS - [Intro blog post][25] - ![crates.io](https://img.shields.io/crates/v/ad983x.svg)
1. [adafruit-alphanum4] - I2C - Driver for [Adafruit 14-segment LED Alphanumeric Backpack][29] based on the ht16k33 chip - ![crates.io](https://img.shields.io/crates/v/adafruit-alphanum4.svg)
1. [ADE791x] - SPI - ADE7912/ADE7913 3-Channel, Isolated, Sigma-Delta ADC - [github][66] - ![crates.io](https://img.shields.io/crates/v/ade791x.svg)
1. [ADS1x1x] - I2C - 12/16-bit ADCs like ADS1013, ADS1015, ADS1115, etc. - [Intro blog post][23] - ![crates.io](https://img.shields.io/crates/v/ads1x1x.svg)
1. [ADXL313] - SPI - 3-axis accelerometer - ![crates.io](https://img.shields.io/crates/v/adxl313.svg)
1. [ADXL343] - I2C - 3-axis accelerometer - ![crates.io](https://img.shields.io/crates/v/adxl343.svg)
1. [ADXL355] - SPI - 3-axis accelerometer - [Intro blog post][43] - ![crates.io](https://img.shields.io/crates/v/adxl355.svg)
1. [AFE4404] - I2C - Pulse oximeter - ![crates.io](https://img.shields.io/crates/v/afe4404.svg)
1. [AHT20] - I2C - Humidity and temperature sensor - [github](https://github.com/chocol4te/aht20) - ![crates.io](https://img.shields.io/crates/v/aht20.svg)
1. [AHT20-driver] - I2C - Humidity and temperature sensor - [Intro blog post][61] - [github][62] - ![crates.io](https://img.shields.io/crates/v/aht20-driver.svg)
1. [AnyLeaf] - I2C - pH sensor module - [github](https://github.com/AnyLeaf/ph-rust) - ![crates.io](https://img.shields.io/crates/v/anyleaf.svg)
1. [AT86RF212] - SPI - Low power IEEE 802.15.4-2011 ISM RF Transceiver - [Intro blog post][36] - ![crates.io](https://img.shields.io/crates/v/radio-at86rf212.svg)
1. [BlueNRG] - SPI - driver for BlueNRG-MS Bluetooth module - [Intro post][15] ![crates.io](https://img.shields.io/crates/v/bluenrg.svg)
1. [BMA400] - I2C/SPI - Bosch 12-bit 3-axis accelerometer - [github][70] ![crates.io](https://img.shields.io/crates/v/bma400.svg)
1. [BNO055] - I2C - Bosch Sensortec BNO055 9-axis IMU driver - [Intro post][18] ![crates.io](https://img.shields.io/crates/v/bno055.svg)
1. [CD74HC4067] - GPIO - 16-channel digital and analog multiplexer - [Intro blog post][55] - [github][54] - ![crates.io](https://img.shields.io/crates/v/cd74hc4067.svg)
1. [dht-sensor] - 1-Wire - DHT11/DHT22 temperature/humidity sensor driver - [github][48] - ![crates.io](https://img.shields.io/crates/v/dht-sensor.svg)
1. [DRV8825] - DRV8825 Stepper Motor Driver (based on [Stepper]) - [Intro blog post][52] - ![crates.io](https://img.shields.io/crates/v/drv8825.svg)
1. [DS1307] - I2C - Real-time clock driver - [Intro blog post][13] - ![crates.io](https://img.shields.io/crates/v/ds1307.svg)
1. [ebyte-e32] - SERIAL - Ebyte E32 LoRa module driver - [Intro blog post][67] - ![crates.io](https://img.shields.io/crates/v/ebyte-e32.svg)
1. [EEPROM24x] - I2C - 24x series serial EEPROM driver - [Intro blog post][12] - ![crates.io](https://img.shields.io/crates/v/eeprom24x.svg)
1. [embedded-ccs811] - I2C - Gas and VOC sensor driver for monitoring indoor air quality - [Intro blog post][49] - ![crates.io](https://img.shields.io/crates/v/embedded-ccs811.svg)
1. [embedded-sdmmc] - SPI - SD/MMC Card Driver with MS-DOS Partition and FAT16/FAT32 support - [Intro post][20] ![crates.io](https://img.shields.io/crates/v/embedded-sdmmc.svg)
1. [ENC28J60] - SPI - Ethernet controller - [Intro blog post][4] - ![crates.io](https://img.shields.io/crates/v/enc28j60.svg)
1. [FUSB302B] - I2C - Programmable USB Type‐C Controller with USB Power Delivery - [github][69]
1. [HC-12] - SERIAL - Wireless serial transceiver module - [Intro blog post][56] - [github][53] - ![crates.io](https://img.shields.io/crates/v/hc12-at.svg)
1. [HTS221] - I2C - Humidity and temperature sensor - [Intro blog post][7] - ![crates.io](https://img.shields.io/crates/v/hts221.svg)
1. [IIS2MDC] - I2C - ST's High accuracy, ultra-low-power, 3-axis digital output magnetometer - ![crates.io](https://img.shields.io/crates/v/iis2mdc.svg)
1. [ISM330DHCX] - I2C - ST's IMU with 3D accelerometer, 3D gyroscope, ML core and more - ![crates.io](https://img.shields.io/crates/v/ism330dhcx.svg)
1. [keypad] - GPIO - Keypad matrix circuits - [Intro post][14] - ![crates.io](https://img.shields.io/crates/v/keypad.svg)
1. [KXCJ9] - I2C - KXCJ9/KXCJB 3-axis accelerometers - [Intro blog post][24] - ![crates.io](https://img.shields.io/crates/v/kxcj9.svg)
1. [L3GD20] - SPI - Gyroscope - [Intro blog post][1&2] - ![crates.io](https://img.shields.io/crates/v/l3gd20.svg)
1. [LSM303DLHC] - I2C - Accelerometer + compass (magnetometer) - [Intro blog post][1&2] - ![crates.io](https://img.shields.io/crates/v/lsm303dlhc.svg)
1. [MAX6675] - SPI - A driver for the MAX6675 digital thermocouple converter - [Intro blog post][73] - ![crates.io](https://img.shields.io/crates/v/max6675-hal.svg)
1. [MAX6955] - I2C - Driver for Alphanumeric LED display driver - [Intro blog post][46] - ![crates.io](https://img.shields.io/crates/v/max6955.svg)
1. [MAX116xx-10bit] - SPI - Driver for the MAX11619-MAX11621, MAX11624 and MAX11625 10-bit ADCs - [Intro blog post][59] - ![crates.io](https://img.shields.io/crates/v/max116xx-10bit.svg)
1. [MCP25LCXX] - SPI - Driver for Microchip's 25LC series of EEPROMs - ![crates.io](https://img.shields.io/crates/v/microchip-eeprom-25lcxx.svg)
1. [MCP3008] - SPI - 8 channel 10-bit ADC - [Intro blog post][3] - ![crates.io](https://img.shields.io/crates/v/adc-mcp3008.svg)
1. [MCP3425] - I2C - 16-bit ADC - [Intro blog post][5] - ![crates.io](https://img.shields.io/crates/v/mcp3425.svg)
1. [MCP794xx] - I2C - Real-time clock / calendar driver - [Intro blog post][26] - ![crates.io](https://img.shields.io/crates/v/mcp794xx.svg)
1. [MMA7660FC] - I2C - 3-axis accelerometer - [Intro blog post][9]
1. [OPT300x] - I2C - Ambient light sensor family driver  - [Intro blog post][30] - ![crates.io](https://img.shields.io/crates/v/opt300x.svg)
1. [PAC194X] - I2C - Single/multi channel power monitor - [Intro blog post][63] - ![crates.io](https://img.shields.io/crates/v/pac194x)
1. [port-expander] - I2C - Driver for I2C port expanders (supports `PCA95xx`, `PCF85xx`) - [Intro blog post][58] - ![crates.io](https://img.shields.io/crates/v/port-expander.svg)
1. [pwm-pca9685] - I2C - 16-channel, 12-bit PWM/Servo/LED controller - [Intro blog post][32] - ![crates.io](https://img.shields.io/crates/v/pwm-pca9685.svg)
1. [rainbow-hat-rs] - I2C/SPI/GPIO - Pimoroni Rainbow HAT driver for Raspberry Pi - [github][57] - ![crates.io](https://img.shields.io/crates/v/rainbow-hat-rs.svg)
1. [rotary-encoder-hal] - GPIO - A rotary encoder driver using `embedded-hal` - [Intro blog post][28] - ![crates.io](https://img.shields.io/crates/v/rotary-encoder-hal.svg)
1. [sega-controller] - GPIO - Sega controller input - [github][68] - ![crates.io](https://img.shields.io/crates/v/sega-controller.svg)
1. [SGP30] - I2C - Gas sensor - [Intro blog post][6] - ![crates.io](https://img.shields.io/crates/v/sgp30.svg)
1. [SH1106] - I2C - Monochrome OLED display controller - [Intro post][19] ![crates.io](https://img.shields.io/crates/v/sh1106.svg)
1. [shared-bus] - I2C - utility driver for sharing a bus between multiple devices - [Intro post][16] ![crates.io](https://img.shields.io/crates/v/shared-bus.svg)
1. [shift-register-driver] - GPIO - Shift register - [Intro blog post][10] - ![crates.io](https://img.shields.io/crates/v/shift-register-driver.svg)
1. [Si4703] - I2C - FM radio turner (receiver) driver  - [Intro blog post][31] - ![crates.io](https://img.shields.io/crates/v/si4703.svg)
1. [SRAM23x] - SPI - Microchip 23x series serial SRAM/NVSRAM driver - [Intro blog post][51] - ![crates.io](https://img.shields.io/crates/v/sram23x.svg)
1. [SSD1306] - I2C/SPI - OLED display controller - [Intro blog post][8] - ![crates.io](https://img.shields.io/crates/v/ssd1306.svg)
1. [SSD1309] - I2C/SPI - OLED display controller - [Intro blog post][60] - ![crates.io](https://img.shields.io/crates/v/ssd1309.svg)
1. [STSPIN220] - STSPIN220 Stepper Motor Driver (based on [Stepper]) - [Intro blog post][52] - ![crates.io](https://img.shields.io/crates/v/stspin220.svg)
1. [Sx127x] - SPI - Long Range Low Power Sub GHz (Gfsk, LoRa) RF Transceiver - [Intro blog post][34] - ![crates.io](https://img.shields.io/crates/v/radio-sx127x.svg)
1. [Sx128x] - SPI - Long range, low power 2.4 GHz (Gfsk, Flrc, LoRa) RF Transceiver - [Intro blog post][35] - ![crates.io](https://img.shields.io/crates/v/radio-sx128x.svg)
1. [TC72] - SPI - Microchip TC72 temperature sensor - ![crates.io](https://img.shields.io/crates/v/microchip-tc72r-rs.svg)
1. [TCN75A] - I2C - Microchip TCN75A temperature sensor - ![crates.io](https://img.shields.io/crates/v/tcn75a.svg)
1. [TMP006] - I2C - Contact-less infrared (IR) thermopile temperature sensor driver - [Intro post][17] ![crates.io](https://img.shields.io/crates/v/tmp006.svg)
1. [TMP1x2] - I2C - TMP102 and TMP112x temperature sensor driver - [Intro blog post][22] ![crates.io](https://img.shields.io/crates/v/tmp1x2.svg)
1. [TSIC] - GPIO - TSIC 306 temperature sensor driver - [Intro blog post][50] ![crates.io](https://img.shields.io/crates/v/tsic.svg)
1. [TSL256X] - I2C - Light Intensity Sensor - [Intro blog post][11] - ![crates.io](https://img.shields.io/crates/v/tsl256x.svg)
1. [VEML6030/VEML7700] - I2C - Ambient light sensors - [Intro blog post][33] - ![crates.io](https://img.shields.io/crates/v/veml6030.svg)
1. [VEML6075] - I2C - UVA and UVB light sensor - [Intro blog post][27] - ![crates.io](https://img.shields.io/crates/v/veml6075.svg)
1. [usbd-serial] - USB CDC-ACM class (serial) implementation - [github][37] - ![crates.io](https://img.shields.io/crates/v/usbd-serial.svg)
1. [usbd-hid] - USB HID class implementation - [github][38] - ![crates.io](https://img.shields.io/crates/v/usbd-hid.svg)
1. [usbd-hid-device] - USB HID class implementation without `unsafe` - [github][40] - ![crates.io](https://img.shields.io/crates/v/usbd-hid-device.svg)
1. [usbd-human-interface-device] - Batteries included embedded USB HID library for `usb-device`. Includes concrete Keyboard (boot and NKRO), Mouse and Consumer Control implementations as well as support for building your own HID classes - [github][65] - ![crates.io](https://img.shields.io/crates/v/usbd-human-interface-device.svg)
1. [usbd-midi] - USB MIDI class implementation - [github][41] - ![crates.io](https://img.shields.io/crates/v/usbd-midi.svg)
1. [usbd-webusb] - USB webUSB class implementation - [github][39] - ![crates.io](https://img.shields.io/crates/v/usbd-webusb.svg)
1. [SHTCx] - I2C - Temperature / humidity sensors - [github][42] - ![crates.io](https://img.shields.io/crates/v/shtcx.svg)
1. [ST7789] - SPI - An embedded-graphics compatible driver for the popular lcd family from Sitronix used in the PineTime watch - [github][44] - ![crates.io](https://img.shields.io/crates/v/st7789.svg)
1. [DW1000] - SPI - Radio transceiver (IEEE 802.15.4 and position tracking) - [Article][45] - ![crates.io](https://img.shields.io/crates/v/dw1000.svg)
1. [Adafruit-7segment] - I2C - Driver for Adafruit 7-segment LED Numeric Backpack based on the ht16k33 chip - [github][47] - ![crates.io](https://img.shields.io/crates/v/adafruit-7segment.svg)
1. [ST7565] - SPI - An embedded-graphics compatible driver for LCD displays based on the ST7565 chip - [github][64] - ![crates.io](https://img.shields.io/crates/v/st7565.svg)
1. [tb6612fng] - A `no_std` driver for the TB6612FNG motor driver - ![Crates.io](https://img.shields.io/crates/v/tb6612fng.svg)
1. [vl53l1x-uld] - I2C - A pure-rust driver for the [ST VL53L1X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) - ![crates.io](https://img.shields.io/crates/v/vl53l1x-uld.svg)
1. [i2c-multiplexer] - I2C - An I2C Multiplexer library that supports the PCA9546 and TCA9546A chips - [github][71] - ![crates.io](https://img.shields.io/crates/v/i2c-multiplexer.svg)
1. [SHT31-rs] - I2C - Fully supported SHT temperature / humidity sensors - [github][72] - ![crates.io](https://img.shields.io/crates/v/sht31.svg)

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
[53]: https://github.com/barafael/hc12-at-rs
[54]: https://github.com/barafael/cd74hc4067-rs
[55]: https://barafael.github.io/A-Platform-Agnostic-Driver-for-the-CD74HC4067
[56]: https://barafael.github.io/A-Platform-Agnostic-Driver-for-the-HC12-serial-radio-module/
[57]: https://github.com/yannart/rainbow-hat-rs
[58]: https://blog.rahix.de/port-expander/
[59]: https://robamu.github.io/post/max11619-driver-rust/
[60]: https://antonok.com/projects/ssd1309
[61]: http://www.rawmeat.org/code/20220130-aht20_driver/
[62]: https://github.com/anglerud/aht20-driver
[63]: https://blog.kiranshila.com/post/pac_rust_driver
[64]: https://github.com/Finomnis/st7565
[65]: https://github.com/dlkj/usbd-human-interface-device
[66]: https://github.com/GrepitAB/ade791x-rs
[67]: https://barafael.github.io/A-Platform-Agnostic-Driver-for-EBYTE-E32-LoRa-Modules/
[68]: https://github.com/UnderLogic/sega-controller
[69]: https://github.com/fmckeogh/usb-pd-rs
[70]: https://github.com/cfrenette/bma400-rs
[71]: https://github.com/FloppyDisck/i2c-multiplexer
[72]: https://github.com/FloppyDisck/SHT31-rs
[73]: https://barretts.club/posts/max6675-hal/

[AD983x]: https://crates.io/crates/ad983x
[adafruit-alphanum4]: https://crates.io/crates/adafruit-alphanum4
[ADE791x]: https://crates.io/crates/ade791x
[ADS1x1x]: https://crates.io/crates/ads1x1x
[ADXL313]: https://crates.io/crates/adxl313
[ADXL343]: https://crates.io/crates/adxl343
[ADXL355]: https://crates.io/crates/adxl355
[AFE4404]: https://github.com/pulse-loop/afe4404
[AHT20]: https://crates.io/crates/aht20
[AHT20-driver]: https://crates.io/crates/aht20-driver
[AnyLeaf]: https://crates.io/crates/anyleaf
[at86rf212]: https://crates.io/crates/radio-at86rf212
[BlueNRG]: https://crates.io/crates/bluenrg
[BMA400]: https://crates.io/crates/bma400
[BNO055]: https://crates.io/crates/bno055
[dht-sensor]: https://crates.io/crates/dht-sensor
[DRV8825]: https://crates.io/crates/drv8825
[DS1307]: https://crates.io/crates/ds1307
[ebyte-e32]: https://crates.io/crates/ebyte-e32
[EEPROM24x]: https://crates.io/crates/eeprom24x
[embedded-ccs811]: https://crates.io/crates/embedded-ccs811
[embedded-sdmmc]: https://crates.io/crates/embedded-sdmmc
[ENC28J60]: https://crates.io/crates/enc28j60
[FUSB302B]: https://github.com/fmckeogh/usb-pd-rs
[HTS221]: https://crates.io/crates/hts221
[IIS2MDC]: https://crates.io/crates/iis2mdc
[ISM330DHCX]: https://crates.io/crates/ism330dhcx
[keypad]: https://crates.io/crates/keypad
[KXCJ9]: https://crates.io/crates/kxcj9
[L3GD20]: https://crates.io/crates/l3gd20
[LSM303DLHC]: https://crates.io/crates/lsm303dlhc
[MAX6675]: https://crates.io/crates/max6675-hal
[MAX6955]: https://crates.io/crates/max6955
[MAX116xx-10bit]: https://crates.io/crates/max116xx-10bit
[MCP25LCXX]: https://crates.io/crates/microchip-eeprom-25lcxx
[MCP3008]: https://crates.io/crates/adc-mcp3008
[MCP3425]: https://crates.io/crates/mcp3425
[MCP794xx]: https://crates.io/crates/mcp794xx
[MMA7660FC]: https://crates.io/crates/mma7660fc
[OPT300x]: https://github.com/eldruin/opt300x-rs
[PAC194X]: https://github.com/kiranshila/pac194x
[port-expander]: https://crates.io/crates/port-expander
[pwm-pca9685]: https://crates.io/crates/pwm-pca9685
[rainbow-hat-rs]: https://crates.io/crates/rainbow-hat-rs
[rotary-encoder-hal]: https://crates.io/crates/rotary-encoder-hal
[sega-controller]: https://crates.io/crates/sega-controller
[SGP30]: https://crates.io/crates/sgp30
[SH1106]: https://crates.io/crates/sh1106
[shared-bus]: https://github.com/Rahix/shared-bus
[shift-register-driver]: https://crates.io/crates/shift-register-driver
[Si4703]: https://crates.io/crates/si4703
[SRAM23x]: https://crates.io/crates/sram23x
[SSD1306]: https://crates.io/crates/ssd1306
[SSD1309]: https://crates.io/crates/ssd1309
[STSPIN220]: https://crates.io/crates/stspin220
[Sx127x]: https://crates.io/crates/radio-sx127x
[Sx128x]: https://crates.io/crates/radio-sx128x
[TC72]: https://crates.io/crates/microchip-tc72r-rs
[TCN75A]: https://crates.io/crates/tcn75a
[TMP006]: https://crates.io/crates/tmp006
[TMP1x2]: https://crates.io/crates/tmp1x2
[TSIC]: https://crates.io/crates/tsic
[TSL256X]: https://crates.io/crates/tsl256x
[VEML6030/VEML7700]: https://crates.io/crates/veml6030
[VEML6075]: https://crates.io/crates/veml6075
[usbd-serial]: http://crates.io/crates/usbd-serial
[usbd-hid]: http://crates.io/crates/usbd-hid
[usbd-hid-device]: http://crates.io/crates/usbd-hid-device
[usbd-human-interface-device]: https://github.com/dlkj/usbd-human-interface-device
[usbd-midi]: http://crates.io/crates/usbd-midi
[usbd-webusb]: http://crates.io/crates/usbd-webusb
[SHTCx]: http://crates.io/crates/shtcx
[ST7789]: http://crates.io/crates/st7789
[DW1000]: https://crates.io/crates/dw1000
[Adafruit-7segment]: https://crates.io/crates/adafruit-7segment
[ST7565]: http://crates.io/crates/st7565
[tb6612fng]: https://crates.io/crates/tb6612fng
[vl53l1x-uld]: https://crates.io/crates/vl53l1x-uld
[i2c-multiplexer]: https://crates.io/crates/i2c-multiplexer
[SHT31-rs]: https://crates.io/crates/sht31

*NOTE* You may be able to find even more driver crates by searching for the [`embedded-hal-driver`]
keyword on crates.io!

[`embedded-hal-driver`]: https://crates.io/keywords/embedded-hal-driver

### WIP

Work in progress drivers. Help the authors make these crates awesome!

1. [AD9850] - Embedded driver for the AD9850 DDS synthesizer chip - ![crates.io](https://img.shields.io/crates/v/ad9850.svg)
1. [AFE4400] - SPI - Pulse oximeter
1. [APDS9960] - I2C - Proximity, ambient light, RGB, and gesture sensor - ![crates.io](https://img.shields.io/crates/v/apds9960.svg)
1. [AS5048A] - SPI - AMS AS5048A Magnetic Rotary Encoder
1. [AXP209] - I2C - Power management unit
1. [BH1750] - I2C - ambient light sensor (lux meter)
1. [BME280] - A rust device driver for the Bosch BME280 temperature, humidity, and atmospheric pressure sensor and the Bosch BMP280 temperature and atmospheric pressure sensor. ![crates.io](https://img.shields.io/crates/v/bme280.svg)
1. [bme680] - I2C - Temperature / humidity / gas / pressure sensor - ![crates.io](https://img.shields.io/crates/v/bme680.svg)
1. [BMI160] - I2C / SPI - Inertial Measurement Unit - ![crates.io](https://img.shields.io/crates/v/bmi160.svg)
1. [BMP280] - A platform agnostic driver to interface with the BMP280 pressure sensor ![crates.io](https://img.shields.io/crates/v/bmp280-ehal.svg)
1. [CC1101] - SPI - Sub-1GHz RF Transceiver - ![crates.io](https://img.shields.io/crates/v/cc1101.svg)
1. [DS3231] - I2C - real-time clock
1. [DS3234] - SPI - Real-time clock
1. [DS323x] - I2C/SPI - Real-time clocks (RTC): DS3231, DS3232 and DS3234 - ![crates.io](https://img.shields.io/crates/v/ds323x.svg)
1. [epd-waveshare] - SPI - driver for E-Paper Modules from Waveshare ![crates.io](https://img.shields.io/crates/v/epd-waveshare.svg)
1. [embedded-morse] - Output morse messages - ![crates.io](https://img.shields.io/crates/v/embedded-morse.svg)
1. [embedded-nrf24l01] - SPI+GPIO - 2.4 GHz radio
1. [Ft6x36] - I2C - Rust driver for focal tech touch screen FT6236, FT6336 - ![crates.io](https://img.shields.io/crates/v/ft6x36.svg)
1. [grove-matrix-led-my9221-rs] - I2C - Rust driver for Grove RGB Matrix Led with my-9221 Driver - ![crates.io](https://img.shields.io/crates/v/grove-matrix-led-my9221-rs.svg)
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
1. [IST7920] - SPI monochrome LCD display - ![crates.io](https://img.shields.io/crates/v/ist7920.svg)
1. [LM75] - I2C - Temperature sensor and thermal watchdog - ![crates.io](https://img.shields.io/crates/v/lm75.svg)
1. [LS010B7DH01] - SPI - Memory LCD
1. [LSM303C] - A platform agnostic driver to interface with the LSM303C (accelerometer + compass) ![crates.io](https://img.shields.io/crates/v/lsm303c.svg)
1. [LSM9DS1] - I2C/SPI - 9-axis motion sensor module ![crates.io](https://img.shields.io/crates/v/lsm9ds1.svg)
1. [ltr-559] - I2C - Ambient Light Sensor and Proximity sensor ![crates.io](https://img.shields.io/crates/v/ltr-559.svg)
1. [lvgl] - no_std [LittleVGL](https://github.com/littlevgl/lvgl) port - ![crates.io](https://img.shields.io/crates/v/lvgl.svg)
1. [M95320] - SPI - STMicroelectronics Serial flash EEPROM - ![crates.io](https://img.shields.io/crates/v/m95320.svg)
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
1. [MS5637] - no_std I2C driver for the MS5637 temperature and pressure sensor ![crates.io](https://img.shields.io/crates/v/ms5637.svg)
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
1. [SPL06-007] - I2C - Pressure and temperature sensor - ![crates.io](https://img.shields.io/crates/v/spl06-007.svg)
1. [spi-memory] - SPI - A generic driver for various SPI Flash and EEPROM chips - ![crates.io](https://img.shields.io/crates/v/spi-memory.svg)
1. [SSD1320] - SPI - Graphical OLED display controller - ![crates.io](https://img.shields.io/crates/v/ssd1320.svg)
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
1. [VEML7700] - I2C - Ambient light sensors - ![crates.io](https://img.shields.io/crates/v/veml7700.svg)
1. [vesc-comm] - A driver for communicating with [VESC-compatible electronic speed controllers](http://vedder.se/2015/01/vesc-open-source-esc/) ![crates.io](https://img.shields.io/crates/v/vesc-comm.svg)
1. [VL53L0X] - A platform agnostic driver to interface with the vl53l0x (time-of-flight sensor) ![crates.io](https://img.shields.io/crates/v/vl53l0x.svg)
1. [w5500] - SPI - Ethernet Module with hardwired protocols : TCP, UDP, ICMP, IPv4, ARP, IGMP, PPPoE - ![crates.io](https://img.shields.io/crates/v/w5500.svg)
1. [xCA9548A] - I2C - I2C switches/multiplexers: TCA9548A, PCA9548A - ![crates.io](https://img.shields.io/crates/v/xca9548a.svg)
1. [ublox-cellular-rs] - Serial - Cellular driver for the full Ublox cellular serial based family
1. [atwinc1500-rs] - SPI - A host driver for the Atwinc1500 network controller
1. [HX711] - GPIO - An interrupt-based driver for the HX711 Load Cell Amplifier IC. no-std.

[AD9850]: https://crates.io/crates/ad9850
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
[CD74HC4067]: https://crates.io/crates/cd74hc4067
[DS3231]: https://github.com/wose/ds3231
[DS3234]: https://github.com/rust-embedded/wg/issues/39#issuecomment-375262785
[DS323x]: https://crates.io/crates/ds323x
[epd-waveshare]: https://crates.io/crates/epd-waveshare
[embedded-morse]: https://crates.io/crates/embedded-morse
[embedded-nrf24l01]: https://crates.io/crates/embedded-nrf24l01
[Ft6x36]: https://crates.io/crates/ft6x36
[grove-matrix-led-my9221-rs]: https://crates.io/crates/grove-matrix-led-my9221-rs
[GridEYE]: https://crates.io/crates/grideye
[HC-12]: https://crates.io/crates/hc12-at
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
[IST7920]: https://crates.io/crates/ist7920
[LM75]: https://crates.io/crates/lm75
[LS010B7DH01]: https://github.com/byronwasti/ls010b7dh01
[LSM303C]: https://crates.io/crates/lsm303c
[LSM9DS1]: https://crates.io/crates/lsm9ds1
[ltr-559]: https://crates.io/crates/ltr-559
[lvgl]: https://github.com/rafaelcaricio/lvgl-rs
[M95320]: https://crates.io/crates/m95320
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
[MS5637]: https://crates.io/crates/ms5637
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
[SPL06-007]: https://github.com/roxgib/SPL06-007
[spi-memory]: https://github.com/jonas-schievink/spi-memory/
[SSD1320]: https://crates.io/crates/ssd1320
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
[VEML7700]: https://crates.io/crates/veml7700
[vesc-comm]: https://github.com/chocol4te/vesc-comm
[VL53L0X]: https://crates.io/crates/vl53l0x
[w5500]: https://crates.io/crates/w5500
[xCA9548A]: https://crates.io/crates/xca9548a
[ublox-cellular-rs]: https://github.com/BlackbirdHQ/ublox-cellular-rs
[atwinc1500-rs]: https://crates.io/crates/atwinc1500
[HX711]: https://github.com/DaneSlattery/hx711

## no-std crates

[`#![no_std]` crates][no-std-category] designed to run on resource-constrained devices.

1. [adskalman](https://crates.io/crates/adskalman): Kalman filter and Rauch-Tung-Striebel smoothing implementation. ![crates.io](https://img.shields.io/crates/v/adskalman.svg)
1. [atomic](https://crates.io/crates/atomic): Generic Atomic<T> wrapper type. ![crates.io](https://img.shields.io/crates/v/atomic.svg)
1. [bbqueue](https://crates.io/crates/bbqueue): A SPSC, statically allocatable queue based on BipBuffers suitable for DMA transfers - ![crates.io](https://img.shields.io/crates/v/bbqueue.svg)
1. [bitmatch]: A crate that allows you to match, bind, and pack the individual bits of integers. - ![crates.io](https://img.shields.io/crates/v/bitmatch.svg)
1. [biquad]: A library for creating second-order IIR filters for signal processing based on Biquads, where both a Direct Form 1 (DF1) and Direct Form 2 Transposed (DF2T) implementation is available. ![crates.io](https://img.shields.io/crates/v/biquad.svg)
1. [bit_field](https://crates.io/crates/bit_field): manipulating bitfields and bitarrays - ![crates.io](https://img.shields.io/crates/v/bit_field.svg)
1. [bluetooth-hci](https://crates.io/crates/bluetooth-hci): device-independent Bluetooth Host-Controller Interface implementation. ![crates.io](https://img.shields.io/crates/v/bluetooth-hci.svg)
1. [bounded-registers](https://crates.io/crates/bounded-registers) A high-assurance memory-mapped register code generation and interaction library. `bounded-registers` provides a Tock-like API for MMIO registers with the addition of type-based bounds checking. - ![crates.io](https://img.shields.io/crates/v/bounded-registers.svg)
1. [cam-geom](https://crates.io/crates/cam-geom): Geometric models of cameras for photogrammetry. ![crates.io](https://img.shields.io/crates/v/cam-geom.svg)
1. [combine](https://crates.io/crates/combine): parser combinator library - ![crates.io](https://img.shields.io/crates/v/combine.svg)
1. [console-traits](https://github.com/thejpster/console-traits): Describes a basic text console. Used by [menu] and implemented by [vga-framebuffer]. ![crates.io](https://img.shields.io/crates/v/console-tratis.svg)
1. [`cmim`], or Cortex-M Interrupt Move: A crate for Cortex-M devices to move data to interrupt context, without needing a critical section to access the data within an interrupt, and to remove the need for the "mutex dance" - ![crates.io](https://img.shields.io/crates/v/cmim.svg)
1. [`cmsis-dsp-sys`](https://github.com/jacobrosenthal/cmsis-dsp-sys): Rust FFI bindings to the [Arm CMSIS_5](https://github.com/ARM-software/CMSIS_5) math library - ![crates.io](https://img.shields.io/crates/v/cmsis-dsp-sys.svg)
1. [dcmimu]: An algorithm for fusing low-cost triaxial MEMS gyroscope and accelerometer measurements ![crates.io](https://img.shields.io/crates/v/dcmimu.svg)
1. [debouncr]: A simple no-std input debouncer to detect rising/falling edges with minimal RAM requirements. ![crates.io](https://img.shields.io/crates/v/debouncr.svg)
1. [device-driver]: A toolkit to write better device drivers, faster. ![crates.io](https://img.shields.io/crates/v/device-driver.svg)
1. [drogue-device](https://github.com/drogue-iot/drogue-device): A distribution of tools and examples for building embedded applications in Rust.
1. [dummy-pin](https://crates.io/crates/dummy-pin): Dummy implementations of the input/output pin traits. ![crates.io](https://img.shields.io/crates/v/dummy-pin.svg)
1. [crossbus](https://github.com/hominee/crossbus): A Platform-Less, Runtime-Less Actor Computing Model.
1. [ector](https://github.com/drogue-iot/ector): An async actor framework for embedded, based on embassy.
1. [embassy]: A set of embedded async tools to make async/await a first-class option for embedded development
1. [embedded-cli](https://crates.io/crates/embedded-cli): CLI library with autocompletion, subcommands, options, help and history support. ![crates.io](https://img.shields.io/crates/v/embedded-cli.svg)
1. [embedded-crc-macros](https://crates.io/crates/embedded-crc-macros): Macros implementing portable CRC algorithms and build-time lookup table generation. ![crates.io](https://img.shields.io/crates/v/embedded-crc-macros.svg)
1. [embedded-update](https://github.com/drogue-iot/embedded-update): Pluggable firmware update protocol for embedded devices.
1. [embedded-tls](https://github.com/drogue-iot/embedded-tls): A TLS 1.3 implementation that runs in a no-std environment.
1. [embedded-websocket](https://crates.io/crates/embedded-websocket): A lightweight server and client websocket library for embedded systems. ![crates.io](https://img.shields.io/crates/v/embedded-websocket.svg)
1. [endian_codec]: (En/De)code rust types as packed bytes with specific order (endian). Supports derive. - [![crates.io](https://img.shields.io/crates/v/endian_codec.svg)](https://crates.io/crates/endian_codec)
1. [ethercrab](https://github.com/ethercrab-rs/ethercrab): A Rust implementation of the [EtherCAT](https://ethercat.org) industrial automation protocol - ![crates.io](https://img.shields.io/crates/v/ethercrab.svg)
1. [fixed-fft](https://crates.io/crates/fixed-fft): Fixed-point Fast Fourier Transform - [![Crates.io](https://img.shields.io/crates/v/fixed-fft.svg)](https://crates.io/crates/fixed-fft)
1. [gcode](https://github.com/Michael-F-Bryan/gcode-rs): A gcode parser for no-std applications - [![crates.io](https://img.shields.io/crates/v/gcode.svg)](https://crates.io/crates/gcode)
1. [gdbstub](https://crates.io/crates/gdbstub): zero-allocation, pure Rust implementation of the GDB Remote Serial Protocol - [![crates.io](https://img.shields.io/crates/v/gdbstub.svg)](https://crates.io/crates/gdbstub)
1. [heapless](https://crates.io/crates/heapless): provides `Vec`, `String`, `LinearMap`, `RingBuffer` backed by fixed-size buffers  - ![crates.io](https://img.shields.io/crates/v/heapless.svg)
1. [idsp](https://crates.io/crates/idsp): integer DSP algorithms (trigonometry, filtering, PLL...) tuned for precision and speed - ![crates.io](https://img.shields.io/crates/v/idsp.svg)
1. [ieee802154](https://crates.io/crates/ieee802154): Partial implementation of the IEEE 802.15.4 standard - ![crates.io](https://img.shields.io/crates/v/ieee802154.svg)
1. [infrared](https://crates.io/crates/infrared): infrared remote control library for embedded rust - ![crates.io](https://img.shields.io/crates/v/infrared.svg)
1. [intrusive-collections](https://crates.io/crates/intrusive-collections): intrusive (non-allocating) singly/doubly linked lists and red-black trees - ![crates.io](https://img.shields.io/crates/v/intrusive-collections.svg)
1. [inverted-pin](https://crates.io/crates/inverted-pin): Implementations of the input/output pin traits with inverted logic. ![crates.io](https://img.shields.io/crates/v/inverted-pin.svg)
1. [irq](https://crates.io/crates/irq): utilities for writing interrupt handlers (allows moving data into interrupts, and sharing data between them) - ![crates.io](https://img.shields.io/crates/v/irq.svg)
1. [lorawan-encoding](https://github.com/lora-rs/lora-rs/tree/main/lorawan-device): A LoRaWAN packet codec.
1. [lorawan-device](https://github.com/lora-rs/lora-rs/tree/main/lorawan-device): A LoRaWAN MAC implementation supporting both event-driven and async mode.
1. [managed](https://crates.io/crates/managed): provides `ManagedSlice`, `ManagedMap` backed by either their std counterparts or fixed-size buffers for `#![no_std]`. - ![crates.io](https://img.shields.io/crates/v/managed.svg)
1. [menu]: A basic command-line interface library. Has nested menus and basic help functionality. ![crates.io](https://img.shields.io/crates/v/menu.svg)
1. [mqtt-sn](https://crates.io/crates/mqtt-sn): Implementation of the MQTT-SN protocol - ![crates.io](https://img.shields.io/crates/v/mqtt-sn.svg)
1. [microfft](https://crates.io/crates/microfft): Embedded-friendly (`no_std`, no-`alloc`) fast fourier transforms - ![crates.io](https://img.shields.io/crates/v/microfft.svg)
1. [micromath](https://github.com/NeoBirth/micromath): Embedded Rust math library featuring fast, safe floating point approximations for common arithmetic operations, 2D and 3D vector types, and statistical analysis - ![crates.io](https://img.shields.io/crates/v/micromath.svg)
1. [minimq](https://crates.io/crates/minimq): A minimal MQTT5 client designed for no_std platforms - ![crates.io](https://img.shields.io/crates/v/minimq.svg)
1. [moonboot](https://github.com/jhbruhn/moonboot): OTA Bootloader Construction Framework for Rust no_std environments, especially embedded devices without an OS
1. [nalgebra](https://crates.io/crates/nalgebra): general-purpose and low-dimensional linear algebra library - ![crates.io](https://img.shields.io/crates/v/nalgebra.svg)
1. [nom](https://crates.io/crates/nom): parser combinator framework - ![crates.io](https://img.shields.io/crates/v/nom.svg)
1. [null-terminated](https://crates.io/crates/null-terminated): generic null-terminated arrays - ![crates.io](https://img.shields.io/crates/v/null-terminated.svg)
1. [num-format](https://crates.io/crates/num-format): Crate for producing string representations of numbers, formatted according to international standards, e.g. "1,000,000" for US English - ![crates.io](https://img.shields.io/crates/v/num-format.svg)
1. [`panic-persist`]: A panic handler crate inspired by `panic-ramdump` that logs panic messages to a region of RAM defined by the user, allowing for discovery of panic messages post-mortem using normal program control flow. - ![crates.io](https://img.shields.io/crates/v/panic-persist.svg)
1. [pc-keyboard]: A PS/2 keyboard protocol driver. Transport (bit-banging or SPI) agnostic, but can convert Set 2 Scancodes into Unicode. ![crates.io](https://img.shields.io/crates/v/pc-keyboard.svg)
1. [qei](https://crates.io/crates/qei) : A qei wrapper that allows you to extend your qei timers from a 16-bit integer to a 64-bit integer. - ![crates.io](https://img.shields.io/crates/v/qei.svg)
1. [qemu-exit]: Quit a running QEMU session with user-defined exit code. Useful for unit or integration tests using QEMU. - ![crates.io](https://img.shields.io/crates/v/qemu-exit.svg)
1. [RampMaker](https://crates.io/crates/ramp-maker): Stepper Motor Acceleration Ramp Generator - ![crates.io](https://img.shields.io/crates/v/ramp-maker.svg)
1. [register-rs](https://github.com/rust-embedded/register-rs): Unified interface for MMIO and CPU registers. Provides type-safe bitfield manipulation. `register-rs` is Tock registers with added support for CPU register definitions using the same API as for the MMIO registers. This enables homogeneous interfaces to registers of all kinds. - ![crates.io](https://img.shields.io/crates/v/register.svg)
1. [scapegoat](https://crates.io/crates/scapegoat): Safe, stack-only alternative to `BTreeSet` and `BTreeMap`. - [![crates.io](https://img.shields.io/crates/v/scapegoat.svg)](https://crates.io/crates/scapegoat)
1. [scroll](https://crates.io/crates/scroll): extensible and endian-aware Read/Write traits for generic containers - ![crates.io](https://img.shields.io/crates/v/scroll.svg)
1. [Slint](https://crates.io/crates/slint): Declarative GUI framework that works on microcontrollers. [![crates.io](https://img.shields.io/crates/v/slint.svg)](https://crates.io/crates/slint)
1. [smbus-pec](https://crates.io/crates/smbus-pec): Minimal portable System Management Bus Packet Error Code calculation algorithm. ![crates.io](https://img.shields.io/crates/v/smbus-pec.svg)
1. [smoltcp](https://github.com/m-labs/smoltcp): a small TCP/IP stack that runs without `alloc`. ![crates.io](https://img.shields.io/crates/v/smoltcp.svg)
1. [sntpc]: Rust SNTP client to get a timestamp from NTP servers. - ![crates.io](https://img.shields.io/crates/v/sntpc)
1. [static-bytes](https://github.com/xoac/static-bytes): Help work with buffers without dynamic allocation. Implement traits from bytes crate. [![crate.io](https://img.shields.io/crates/v/static-bytes.svg)](https://crates.io/crates/static-bytes)
1. [Stepper]: Universal Stepper Motor Interface - ![crates.io](https://img.shields.io/crates/v/stepper.svg)
1. [tinybmp](https://crates.io/crates/tinybmp): No-std, no-alloc BMP parser for embedded systems. [Introductory blog post](https://wapl.es/rust/2019/03/04/embedded-graphics-0.4.7-bmp-support.html) - ![crates.io](https://img.shields.io/crates/v/tinybmp.svg)
1. [vga-framebuffer]: A VGA signal generator and font renderer for VGA-less microcontrollers. Used by [Monotron](https://github.com/thejpster/monotron) to generate 48 by 36 character display using 3 SPI peripherals and a timer. ![crates.io](https://img.shields.io/crates/v/vga-framebuffer.svg)
1. [usb-pd](https://github.com/fmckeogh/usb-pd-rs): USB-PD library, supports using the FUSB302B as a Sink driver, with Source functionality planned.
1. [wyhash]: A fast, simple and portable hashing algorithm and random number generator. - ![crates.io](https://img.shields.io/crates/v/wyhash.svg)
1. [adafruit-bluefruit-protocol]: A `no_std` parser for the [Adafruit Bluefruit LE Connect controller protocol]. - ![crates.io](https://img.shields.io/crates/v/adafruit-bluefruit-protocol)

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
[sntpc]: https://crates.io/crates/sntpc
[Stepper]: https://crates.io/crates/stepper
[vga-framebuffer]: https://github.com/thejpster/vga-framebuffer-rs
[wyhash]: https://crates.io/crates/wyhash
[adafruit-bluefruit-protocol]: https://crates.io/crates/adafruit-bluefruit-protocol
[Adafruit Bluefruit LE Connect controller protocol]: https://learn.adafruit.com/bluefruit-le-connect/controller

### WIP

Work in progress crates. Help the authors make these crates awesome!

- [light-cli](https://github.com/rudihorn/light-cli): a lightweight heapless cli interface ![crates.io](https://img.shields.io/crates/v/light_cli.svg)
- [OxCC](https://github.com/jonlamb-gh/oxcc): A port of Open Source Car Control written in Rust
- [Rubble](https://github.com/jonas-schievink/rubble): A pure-Rust embedded BLE stack ![crates.io](https://img.shields.io/crates/v/rubble.svg)

[no-std-category]: https://crates.io/categories/no-std

## Firmware projects

- [rmk](https://github.com/HaoboGu/rmk): Mechanical keyboard firmware for stm32/rp2040, supports vial/dynamic keymap/eeprom, written in Rust
- [anne-key](https://github.com/ah-/anne-key): Alternate keyboard firmware for the Obins ANNE Pro
- [μLA](https://github.com/dotcypress/ula): Micro Logic Analyzer for RP2040
- [air-gradient-pro-rs](https://github.com/jonlamb-gh/air-gradient-pro-rs): Bootloader, firmware, and CLI tools for the AirGradient PRO
- [Stabilizer](https://github.com/quartiq/stabilizer): Firmware for a DSP tool used in quantum physics experimentation, includes telemetry via MQTT and run-time configuration
- [Booster](https://github.com/quartiq/booster): Firmware for an RF power amplifier, including telemetry via MQTT and run-time configuration
- [Thermostat EEM](https://github.com/quartiq/thermostat-eem): Firmware for a multi-channel temperature controller used in physics experiments
- [Card/IO ECG](https://github.com/card-io-ecg/card-io-fw): Firmware for a business-card-sized ECG device with Wifi connectivity
- [BillMock](https://github.com/pmnxis/billmock-app-rs): Firmware for credit card terminal add-on hardware to install on Korean arcade machines
- [LuLuu](https://github.com/fu5ha/luluu): Firmware for a custom RP2040-based display controller that streams animated images from a microSD card to a small LCD display. 

## Old books, blogs, and training materials

These materials may be outdated and reflect earlier practices, but they might still be helpful for reference.

-   [Exploring Rust on Teensy](https://branan.github.io/teensy/) by @branan — Beginner set of articles on getting into embedded dev in Rust.
-   [Pragmatic Bare Metal Rust](http://www.hashmismatch.net/pragmatic-bare-metal-rust/) A starter article about starting Rust development on STM32 microcontrollers (cubeMX + FFI).
-   [Using Rust in an Embedded Project: A Simple Example](https://spin.atomicobject.com/2016/07/08/rust-embedded-project-example/#.V3-os-6qlZw.hackernews) Article and some links on setting up Rust cross-compiling.
-   [Robigalia](https://robigalia.org) general purpose robust operating system in Rust running on secure seL4 microkernel.
-   [intermezzOS](http://intermezzos.github.io) A small teaching operating system in Rust. A book with some explanations is also included.
-   [Fearless concurrency](http://blog.japaric.io/fearless-concurrency/) by @japaric — How to easily develop Rust programs for pretty much any ARM Cortex-M microcontroller with memory-safe concurrency.
-   [Internet of Streams](https://www.youtube.com/playlist?list=PLX44HkctSkTewrL9frlUz0yeKLKecebT1) A video series  by [@jamesmunns] building a bare metal IoT Sensor Node Platform from (nearly) scratch in Rust
-   [Ferrous Systems' Embedded Training Courses: 2019 edition](https://github.com/ferrous-systems/embedded-trainings/) A hands-on training course for beginner and advanced learners of Embedded Rust, based on Nordic Semiconductor's nRF52 and Decawave's DWM1001-DEV hardware. This training was given at Oxidize Conferences and by Ferrous Systems to corporate customers.

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
