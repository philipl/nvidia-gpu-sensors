# NVIDIA GPU sensor tools for Linux

This tools provides low level access to various temperature and voltage sensors
that are present on NVIDIA GPUs but not exposed by the official NVIDIA tools.

While there are various Windows programs that expose this information, it has
traditionally been hard to come by on Linux.

## Information Reported

* GPU Temperature (nothing new, but I included it for completeness)
* Memory Temperature
* GPU Voltage(s) (NVVDD and MSVDD - the two main GPU voltage rails)
* Blackwell-specific Hotspot temperature (Max of 12 raw sensor readings)

## Requirements

* Hotspot temperature requires running as root - it does raw PCIE reads
* Hotspot temperature is Blackwell specific and won't be read on other hardware

## Driver Compatibility

This program uses the partially documented ioctl interface provided by the nvidia
open-gpu-kernel-modules. It's not clear how unstable this interface really is -
it has definitely changed over time, but doesn't appear to change _all the time_.

For example, I developed this using the 595.71.05 drivers, but it should work
fine on 610.xx based on the source code diffs.

The ioctl usage was reverse engineered from `nvidia-smi` and `libnvidia-ml`.

## Hardware Compatibility

The ioctls we use aren't clearly hardware-specific but I don't know whether you
will actually get a Memory temp or any voltages on other hardware - I've only
run it on Blackwell.

The Hotspot data is definitely Blackwell specific, and the program won't even
try and read it if it doesn't detect a Blackwell GPU.

## Build and Run

```sh
meson setup build
ninja -C build
sudo ./build/nvidia-gpu-sensors
```

## Future work

There are a bunch of other memory locations that appear to hold temperatures,
containing values in a similar range to the known hotspot sensors, and changing
in response to GPU load. I'm stil exploring what these are, but it could be
things like VRM temperature. There's no indication that the raw Memory
temperature is in there, as the values don't match the value returned by the
ioctl interface.
