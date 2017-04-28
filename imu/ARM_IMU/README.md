# STM32 IMU

## Building

```
git submodule update --init --recursive
cd libopencm3
make -j4
cd ..
make -j4 hex
```

## Flashing

```
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
```

```
telnet localhost 4444
reset halt
flash write_image erase main.hex
reset
```

