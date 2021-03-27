[![Build](https://github.com/algon-320/mandarin/actions/workflows/build.yml/badge.svg)](https://github.com/algon-320/mandarin/actions/workflows/build.yml)
[![Test on QEMU](https://github.com/algon-320/mandarin/actions/workflows/test.yml/badge.svg)](https://github.com/algon-320/mandarin/actions/workflows/test.yml)

# Mandarin OS

a hobby OS for x86_64 based on [MikanOS](https://github.com/uchan-nos/mikanos).

## Prerequisites

- qemu-system-x86_86
- lld
- cargo
- cargo-make

## Build image
```
$ cargo make build-disk
```

## Run with QEMU
```
$ cargo make run
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
$ cargo make test
```