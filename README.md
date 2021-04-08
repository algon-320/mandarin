[![Build](https://github.com/algon-320/mandarin/actions/workflows/build.yml/badge.svg)](https://github.com/algon-320/mandarin/actions/workflows/build.yml)
[![Test on QEMU](https://github.com/algon-320/mandarin/actions/workflows/test.yml/badge.svg)](https://github.com/algon-320/mandarin/actions/workflows/test.yml)

# Mandarin OS

a hobby OS for x86_64 based on [MikanOS](https://github.com/uchan-nos/mikanos).

## Prerequisites

- qemu-system-x86_86
- lld
- cargo
- cargo-make

## Dependencies

Crates:

- [uefi-rs](https://github.com/rust-osdev/uefi-rs)

Nightly features:

- `asm`
- `custom_test_frameworks`
- `const_maybe_uninit_assume_init`

## Build image
```
$ cargo make build-disk
```

## Run with QEMU
```
$ cargo make run      # release mode
$ cargo make run-dev  # dev mode
```

## Debug with GDB
```
$ cargo make debug
```
and
```
$ cargo make gdb-attach
```
on another terminal.

## Test on QEMU
```
$ cargo make test  # without display
$ cargo make test-console  # with display
```
