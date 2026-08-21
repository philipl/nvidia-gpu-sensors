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
* Blackwell-specific per-module GDDR7 temperatures (the raw per-chip
  readings, shown with `--sensors`)
* Blackwell-specific secondary thermal zones (the other temperature readings
  in the GPU's thermal window, shown with `--sensors`)

Both the memory temperature and the second voltage reading were unexpected. As
we're using the same low-level ioctl interface that the official nvidia tools
use, I expected not to see a working memory reading (nvidia-smi does not
show memory temps for consumer GPUs) and the old versions of nvidia-smi that
included Voltage only ever saw one value. So it was definitely a bonus to get
these.

### Per-module GDDR7 temperatures

On Blackwell, each GDDR7 chip has its own die temperature sensor; the
per-partition DQR status registers carry the raw per-chip readings (2 °C
granularity, the step of the sensor's mode-register code) that the memory
controller firmware itself polls.

The driver's single reported memory temperature is already the hottest reading
across all the individual memory modules, so we don't actually use the
per-module data except when run with `--sensors` to show all the individual
values.

For example:

```
GPU 0: FBPA DQR scan, 32 memory module(s)
  Memory modules: available via EXEC_REG_OPS (chip 0x1B2, 32 memory module(s))
  DQ base  IC0_S0   IC0_S1   IC1_S0   IC1_S1   VLD      modules
  9024C0   30303030 2F2F2F2F 2F2F2F2F 30303030 FF000000   -> m0  56 C  -> m1  54 C  -> m2  54 C  -> m3  56 C
  ...
  91E4C0   30303030 2E2E2E2E 2E2E2E2E 2F2F2F2F FF000000   -> m28 56 C  -> m29 52 C  -> m30 52 C  -> m31 54 C
  (no live DQR slots in partitions 8-15)
```

The memory layout suggests that there could be up to 16 DQR registers, but even
on an RTX Pro 6000, only 8 are used. This is because each register can report
on four modules, so we get the 8x4==32 readings we'd expect for that model.

On the other hand, an RTX 5090 has only 16 memory modules, but we still see
eight registers with four sensor readings - and that's because the readings
are duplicates - there are two modules associated with each register, with
one module's temperature reading showing up in two slots.

Credit to the [gddr6](https://github.com/olealgoritme/gddr6) project for the
register map; I don't know if they reverse-engineered in themselves or got it
from one of the, now many, other projects that can report the memory readings,
but big thanks either way.

## Requirements

* Hotspot and per-module memory temperatures require running as root. We avoid
  doing a raw PCI BAR read by using an nvidia driver ioctl, but the driver still
  requires we do this as root. However, it does avoid any conflicts with
  `iomem=strict` and lockdown modes.

## Driver Compatibility

This program uses the partially documented ioctl interface provided by the nvidia
open-gpu-kernel-modules. It's not clear how unstable this interface really is -
it has definitely changed over time, but doesn't appear to change _all the time_.

For example, I developed this using the 595.71.05 drivers, but it should work
fine on 610.xx based on the source code diffs.

The ioctl usage was reverse engineered from `nvidia-smi` and `libnvidia-ml`.

### Voltage Readings

As a concrete example of this behaviour, the 615.xx drivers changed the size
of the voltage-rails status struct, as well as changing the offset of the
voltage value. I've attempted to maintain compatibility with both layouts.

## Hardware Compatibility

The ioctls we use aren't clearly hardware-specific but I don't know whether you
will actually get a Memory temp or any voltages on other hardware - I've only
seen data returned on Blackwell.

The Hotspot data is definitely Blackwell specific, and the program won't even
try and read it if it doesn't detect a Blackwell GPU.

The hotspot array holds 12 live sensors on both the RTX 5090 and the RTX Pro
6000 (both GB202); other chips may report more or fewer. The secondary
thermal zones are also present and valid on both, but their behaviour differs
somewhat between the two boards (see Secondary thermal zones above). If the
Hotspot column shows `n/a` on a Blackwell GPU, run with `--sensors` to dump
the raw arrays and which slots the scans accepted, and please include that
output in a bug report.

The GDDR7 per-module memory temperature reading should work on any Blackwell
GPU, but also hasn't been tested on anything except a 5090 or RTX Pro 6000.
Based on what we've seen on these GPUs, every other model should have either
fewer modules per status register, or fewer status registers, but I don't know
how clean the output will be. I would expect unused registers to be cleanly
marked as unusued, but given that the RTX 5090 readings show duplicate values -
each register reports four modules even if only two are connected - I would not
be surprised if cards with fewer modules still see four reports that are all
repeats of a single module.

## Build and Run

```sh
meson setup build
ninja -C build
sudo ./build/nvidia-gpu-sensors
```

## Secondary thermal zones

Besides the on-die hotspot array, the GPU's thermal window holds a number of
other temperature readings that respond to load. `--sensors` reads and prints
all of them, grouped by what the load-correlation analysis suggests they are:

* **die** — zones that track the core temperature within a couple of degrees
* **power** — a strongly load-sensitive mirror pair, by far the hottest
  readings on the board (up to ~108 C at sustained ~400 W on an RTX Pro
  6000; shorter bursts peak lower), consistent with a power-stage/VRM sensor
* **aux** — the five-zone group identified on the 5090 by a fixed ~4.2 s
  clear-then-write update cycle (off-die I2C polling was the working
  hypothesis there, never proven); on the RTX Pro 6000 the cycle is absent,
  four of them update continuously and track the die within ~0.5 C, and one
  reads ambient
* **ambient** — cool, slowly varying zones (~40 C, ~1.5 C swing)

The group labels are hypotheses, not proven roles: the analysis establishes
how the zones track each other under load, not where the sensors physically
are. The layout may also differ between GPU models: the zones are probed at
fixed offsets, anything that does not validate as a plausible temperature
shows `--`, and attaching `--sensors` output to a bug report is the way to
map new boards.

Three of the zones (the power pair and two ambient zones) carry no `0x4000`
valid tag, which is the marker every confirmed sensor carries; `--sensors`
flags such readings with a `~`. Their temperature status rests on load
correlation and physical plausibility, and the /256=°C decode is inferred
rather than calibrated. The full-page census in docs/12 shows why the
distinction matters: a zone there decoded as a plausible temperature in
every state tested and still was not a temperature.
