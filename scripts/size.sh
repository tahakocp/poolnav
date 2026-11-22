#!/usr/bin/env bash
# Flash and RAM the controller needs on a Cortex-M0+ (RP2040), soft-float included.
# Needs rustup target thumbv6m-none-eabi and arm-none-eabi-gcc on PATH.
set -euo pipefail
cd "$(dirname "$0")/../size-probe"
cargo build --release --target thumbv6m-none-eabi -q
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -nostdlib -Wl,--gc-sections \
  -Wl,-e,probe_init -Wl,--undefined=probe_step \
  target/thumbv6m-none-eabi/release/libsize_probe.a -lgcc -o target/probe.elf
arm-none-eabi-size target/probe.elf
