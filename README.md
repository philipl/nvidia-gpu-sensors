# NVIDIA GPU sensor tools for Linux

This tools provides low level access to various temperature and voltage sensors
that are present on NVIDIA GPUs but not exposed by the official NVIDIA tools.

While there are various Windows programs that expose this information, it has
traditionally been hard to come by on Linux.

## Information Reported

* GPU Temperature (nothing new, but I included it for completeness)
* Memory Temperature
* GPU Voltage(s) (NVVDD and MSVDD - the two main GPU voltage rails)
* Blackwell-specific Hotspot temperature (Max of the raw on-die sensor readings)

Both the memory temperature and the second voltage reading were unexpected. As
we're using the same low-level ioctl interface that the official nvidia tools
use, I expected not to see a working memory reading (nvidia-smi does not
show memory temps for consumer GPUs) and the old versions of nvidia-smi that
included Voltage only ever saw one value. So it was definitely a bonus to get
these.

## Requirements

* Hotspot temperature requires running as root. We avoid doing a raw PCI BAR
  read by using an nvidia driver ioctl, but the driver still requires we do
  this as root. However, it does avoid any conflicts with `iomem=strict` and
  lockdown modes.
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
seen data returned on Blackwell.

The Hotspot data is definitely Blackwell specific, and the program won't even
try and read it if it doesn't detect a Blackwell GPU.

I still don't know if there is variation in terms of the Blackwell sensor array
between models. I've tested on a 5090, and feedback from other people has been
limited to other 5090s and RTX Pro 6000s. At least on the 5090, there are 12
sensors; other chips may report more or fewer. If the Hotspot column shows `n/a`
on a Blackwell GPU, run with `--sensors` to dump the raw array and which slots
the scan accepted, and please include that output in a bug report.

## Build and Run

```sh
meson setup build
ninja -C build
sudo ./build/nvidia-gpu-sensors
```

## Future work

Other folks have reverse-engineered where the low level memory chip temperature
readings are for Blackwell, and I will add support for reading those.

There are a bunch of other memory locations that appear to hold temperatures,
containing values in a similar range to the known hotspot sensors, and changing
in response to GPU load. I'm stil exploring what these are, but it could be
things like VRM temperature.
