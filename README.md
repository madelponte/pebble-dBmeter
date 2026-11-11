# dB Meter

A Pebble watchapp that measures the ambient sound level with the watch microphone.

This is still under development and has not yet been tested on real hardware yet as I am still waiting on my watch to ship. 

## Using it

**Main screen**

| Button        | Action                                         |
| ------------- | ---------------------------------------------- |
| Select        | Pause / resume measuring                       |
| Select (hold) | Reset the session                              |
| Up            | Settings                                       |
| Down          | Switch between the Live and Session views      |

* **Live view:** the current level with "Fast" (125 ms) time weighting; the bar runs from 30 to
  120 dB (green < 70, yellow < 85, red ≥ 85); the white tick and MAX show the session maximum.
* **Session view:** the session **average** (Leq, the energy average, which is what noise
  exposure limits are based on), **MIN** and **MAX** of the Fast level, the **time** measured,
  and the current level (NOW).

A session runs from launch until you reset it. Time paused (or with the app in the background)
doesn't count. Changing the weighting starts a new session.

**Settings** (Up)

| Item          | What it does                                                          |
| ------------- | --------------------------------------------------------------------- |
| Weighting     | **A** (dBA, default): matches human hearing; what most meters and hearing-safety guidance use. **Z**: flat, unweighted. |
| Calibrate     | Opens the Calibrate screen (below).                                   |
| Reset session | Clears average, min, max and time.                                    |

**Calibrate**

The microphone's absolute sensitivity isn't published and varies between devices, so readings are
estimates until calibrated (the main screen says **UNCALIBRATED** until you have been through
this screen once). Set a real sound level meter (or a calibrated phone SPL app) to the **same
weighting** as the watch, hold them side by side in a steady sound, and adjust until both read the
same.

| Button        | Action                              |
| ------------- | ----------------------------------- |
| Up / Down     | Offset +1 / −1 dB (hold to repeat)  |
| Select (hold) | Reset the offset to 0               |
| Select / Back | Done                                |

The offset (−40 to +40 dB) is saved on the watch and applies to every reading. Uncalibrated
readings are `dBFS + 120` (a typical MEMS mic gives −26 dBFS at 94 dB SPL).

## How it measures

* 16 kHz mono audio arrives in 20 ms frames.
* **A-weighting** (`src/c/aweight.c`) is three fixed-point biquads (the watch CPU has no FPU),
  within 0.1 dB of the IEC 61672 curve from 10 Hz to 7 kHz. The low-frequency poles come from a
  bilinear transform; the 12.2 kHz poles lie above Nyquist, so the high-frequency roll-off is a
  fitted biquad. **Z** just removes DC.
* Each frame's mean square feeds the Fast average; the session sums frame energy for Leq.

## Requirements

* **Watch:** the Microphone API exists only on **emery** (Pebble Time 2), **flint** (Pebble 2 Duo)
  and **gabbro** (Pebble Round 2), on firmware 4.44.3+. The app installs on the older platforms but
  just says the watch has no microphone access.
* **SDK:** 4.44 or newer (`pebble sdk install latest`).
* **Permission:** the app declares `WatchMicrophone`. In the Pebble phone app, turn on
  *Settings → Plugins → Plugins / permissions preview* and allow the microphone for dB Meter.
* The system shows its *Listening* indicator whenever the meter is running. Capture stops when the
  app leaves the foreground and resumes automatically when you come back. Only a foreground app
  may use the microphone, so there is no watchface complication or background measurement.

## Building & running

```sh
pebble build
pebble install --emulator emery --logs
pebble emu-mic --emulator emery --tone 1000 --volume 10         # 100 dBA uncalibrated (−20 dBFS)
pebble emu-mic --emulator emery --tone 100 --volume 10          # 81 dBA (A-weighting: −19.1 dB)
pebble emu-permission --emulator emery WatchMicrophone --deny   # try the refusal path
```

## Project layout

```
src/c/dbmeter.c   Mic capture, levels and session, the Live/Session views, Settings, Calibrate
src/c/aweight.c   Fixed-point A-weighting filter (16 kHz)
tools/aweight_design.py   Designs the filter coefficients and checks them (needs numpy)
package.json      Metadata and the WatchMicrophone permission
```
