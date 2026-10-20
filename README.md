# Sea Platform

A three-leg stepper motion platform that reproduces the heave, pitch and roll of a vessel in sea states 1 to 4.
One firmware program with a menu of six modes, a visualiser that runs the same waves in the browser, and a browser control panel that talks to the board over USB.

- **Project site:** `docs/index.html` (published with GitHub Pages, see [Publish the site](#publish-the-site))
- **Visualiser:** `docs/visualiser.html` — sea states 1 to 4 on a 3D model, MAX mode, live view of the real platform
- **Control panel:** `docs/control.html` — Chrome or Edge, Web Serial, same keys as the terminal
- **Run guide:** `docs/guide.html` — step by step from wiring check to shutdown, with the exact messages the program prints (printable: `docs/pdf/sea-platform-run-guide.pdf`)
- **Measurements:** `docs/measure.html` — drawings of the seven numbers to measure, redrawn live from what you type (printable: `docs/pdf/sea-platform-measure-sheet.pdf`)

## Contents

| Folder | What is in it |
|---|---|
| `firmware/Core/Inc`, `firmware/Core/Src` | The STM32CubeIDE source. Copy these into the project's `Core` folders. |
| `firmware/legacy/main_standalone_tests.c` | The earlier single-file test program (homing, jogging, heave cycle, tilt circles). A fallback that was tested on the rig. |
| `docs/` | The website: project page, run guide, measurements page, visualiser, control panel (plain HTML and JavaScript, no build step). `docs/pdf/` holds printable copies of the run guide and the measurement sheet. |
| `sim/` | PC tests: a model of the three legs and a scripted run of the whole menu. |
| `tools/verify-js.js` | Checks that the JavaScript wave and kinematics code gives the same numbers as the C code. |

## Hardware

- STM32 Nucleo-G474RE, project name `Test2` in STM32CubeIDE.
- Three CL86T closed-loop drivers set to 2000 pulses/rev, 10:1 gearboxes: 20 000 steps = one crank turn = 0.018 degrees per step.
- Normally-closed limit switches, common to ground, signal to the pin with the internal pull-up. A pressed or broken switch reads high.

| Leg | STEP | DIR | Limit switch |
|---|---|---|---|
| X (front) | PA0 | PA1 | PB9 |
| Y | PB4 | PB5 | PB0 |
| Z (back left) | PA8 | PA9 | PA4 |

PUL− and DIR− of the drivers go to ground. Pitch is positive when the front (X) leg rises, roll is positive when the back-left (Z) side rises.

## Install the firmware

1. Copy everything in `firmware/Core/Inc` and `firmware/Core/Src` into the CubeIDE project's `Core/Inc` and `Core/Src`, replacing `main.c` and `main.h`.
2. Build and flash. If the linker reports an undefined `sinf` or `sqrtf`, add `m` under *Properties → C/C++ Build → Settings → MCU GCC Linker → Libraries*.
3. If you regenerate code from the `.ioc`, check `main.c` afterwards. The three limit inputs must be `GPIO_PULLUP` (the firmware sets this in `MX_GPIO_Init`, a regenerated file may not).
4. Open the serial port at 115200, 8N1: PuTTY on the ST-Link COM port, or the control panel.

## Using the program

The full step-by-step is in the [run guide](docs/guide.html). In short: start with `1` (Setup), which homes the legs, then set the minimums, the maximum, the tilt limits and the leg difference limit, and press `s` to save. Everything else uses the saved limits.

| Key | Mode | Keys inside the mode |
|---|---|---|
| `1` | **Setup**: home, set minimums one leg at a time, set maximum with all legs, speed limits, tilt limits, leg difference limit, save | `1` minimums, `2` maximum, `3` speed limits, `4` tilt limits, `5` leg difference limit, `s` save to flash, `d` defaults, `h` re-home. In minimums: `x y z` choose leg, `d`/`a` leg up/down, `u`/`j` all up/down, `+`/`-` step size, `m` mark, `n` mark all, `g` go to minimums. In maximum: `u`/`j`, `+`/`-`, `m`, `g`. In tilt limits: `w`/`s` pitch, `d`/`a` roll, `u`/`j` height, `+`/`-` step, `z` level, `p` mark pitch limit, `r` mark roll limit, `l` list, `c` clear. In leg difference: `x y z` choose leg, `d`/`a` that leg up/down, `u`/`j` all, `m` mark the limit, `c` clear, `g` minimums. |
| `2` | **Heave cycle**: all legs minimum ↔ maximum, pause at each end | `p` or space pause, `+`/`-` speed, `[`/`]` end pause |
| `3` | **Pitch and roll circles**, optional heave | `+`/`-` tilt, `[`/`]` period, `h` heave on/off, `r` reverse, `p` |
| `4` | **Waves**, sea states 1 to 4 | `1`–`4` sea state, `r` new sea, `+`/`-` size, `[`/`]` time, `p` |
| `5` | **MAX platform**: exaggerated waves using the full travel | as mode 4, plus `d` full-range sweep of heave, pitch, roll |
| `6` | **Manual** heave, pitch, roll | `u`/`j` heave, `w`/`s` pitch, `d`/`a` roll, `+`/`-` step, `z` zero, or type `h12.5` / `p-3` / `r2` and Enter |

`q` goes back one level (to the menu from a mode), `x` stops everything (in the Setup screens `x` picks leg X instead). At the menu: `g` sends every leg back to rest (the minimums) without homing again, `h` homes again (a raised platform first comes down with all three legs together), `v` turns telemetry for the web pages on or off.

### Safety limits

- **Tilt limits** (Setup `4`): you tilt the platform by hand and mark the largest pitch up and down and the largest roll each way. Every mode, and the visualiser model when you enter the same numbers, keeps inside them: waves, circles and the MAX sweep are scaled down to fit, manual values are clipped.
- **Leg difference limit** (Setup `5`): the largest allowed difference between two legs, measured as the height of each leg above its own minimum. The mode planning respects it, and the step engine enforces it live: a leg that would get further ahead is held until the others catch up (*Leg difference limit reached...*).
- **Homing** lowers all three legs together in small alternating bursts (`HOME_CHUNK_STEPS`, 40), so no leg is ever far ahead of the others, then each leg backs off `BACKOFF_STEPS` (500) and latches its switch. Re-homing a raised platform first lowers it to the minimums.
- **Jog keys** cannot put a target more than `JOG_LEAD_STEPS` (600) ahead of where the leg really is, so a burst of keys cannot start a long run-away move.
- **Serial input** uses the UART receive FIFO. If a character is still lost while you type a Manual value, the half-typed number is thrown away (*Input lost - that value was NOT used*).
- **Presentation lock** (`l` at the menu, unlock by typing `UNLOCK`): refuses Setup, Manual and re-home. Not saved, so a power cycle unlocks it.
- **TV display** (button in the control panel): full-screen mode, sea state, live pose in large numbers, 3D model, graph and warning banner.
- The control panel sends keys one at a time, 18 ms apart, with STOP jumping the queue (see the run guide).

The limits are stored in flash with the other Setup values. Old saved records stay valid (the limits start as *not set*).

### How the waves become motion

The wave generator (Bretschneider spectrum, 24 components, fixed seed) gives heave, pitch and roll. They are scaled so the sea state 4 peaks use 65 % of the platform's range (95 % in MAX mode) and passed through inverse kinematics to leg positions. If the legs could not follow at 85 % of the speed and acceleration limits, the wave clock slows down first (floors `WAVE_TIME_FLOOR_NORMAL` 0.5 and `WAVE_TIME_FLOOR_MAX` 0.2 in `plat_config.h`) and only then is the amplitude reduced. The step engine keeps every leg inside its limits and brakes in front of them.

Heights are shown as `Hs4 × (Hs/Hs4)^0.75` so sea states 1 and 2 stay visible; set `WAVE_HS_EXPONENT` to `1.0f` for true proportions.

## Enter the real geometry

Until this is done the millimetre and degree numbers are estimates (the menu says *geometry NOT SET*). The leg limits protect the hardware whatever the geometry says.

Measure, in millimetres and degrees:

| Name in `plat_config.h` | What to measure |
|---|---|
| `GEOM_CRANK_R` | crank arm: motor shaft centre to the pin joint centre |
| `GEOM_LINK_L` | link: joint centre to joint centre |
| `GEOM_BASE_R` | middle of the platform to each motor shaft, horizontal |
| `GEOM_PLATFORM_R` | middle of the platform to each link top joint, horizontal |
| `GEOM_THETA_MIN_DEG` | crank angle above horizontal when the platform is at its level minimum (negative if below) |
| `GEOM_CRANK_AZ_DEG` | which way the crank points at 0 degrees: 0 outward, 180 inward, ±90 sideways |
| `GEOM_SENSE` | `1` if more steps raise the crank pin, `-1` if they lower it |

Open `docs/measure.html`. It draws each measurement (side view, top view, a letter for each number) and redraws as you type, so you can check the model looks like the real platform. Press *Show plat_config.h lines* and paste them over the matching lines in `plat_config.h`. *Use in the visualiser* passes the same numbers to the visualiser and control panel in this browser. The model assumes the platform centre is held on the vertical axis (no sideways slip, no yaw). If your mechanism constrains it differently, tell whoever maintains `plat_kin.c`.

## Test on the PC

```
sh sim/build_tests.sh        # needs gcc; node is optional for the JavaScript check
```

It syntax-checks the STM32 files against stub headers, tests the kinematics, waves, tilt and leg difference limits and the step engine, tests the calibration storage (including records written before the limits existed), runs scripted sessions of the whole menu on a model of the legs (setup, every mode, finding the limits, a flood of jog keys, a lost character, re-homing a raised platform, a power cycle), and compares the JavaScript with the C code. The control panel's key sending is checked separately in a browser against a simulated serial port: `npm i playwright && node tools/test-control.js`. None of this replaces a first run on the rig with a hand near the driver power switches.

## Troubleshooting

- **A leg homes the wrong way or never stops:** check the switch wiring (NC to the pin) and `HOME_DIR_LEGn` in `plat_config.h`.
- **Homing asks for `y` before it starts:** a switch reads pressed while its crank is away from it. Check the wiring before continuing.
- **"Leg difference limit reached":** the legs differ by more than the limit set in Setup `5`. It clears when the others catch up; if it never does, look for a stalled leg or clear the limit with `c` in Setup `5`.
- **"HOME SWITCH X FIRED":** a leg reached its switch while the program was moving it. Motion stops and the platform must be homed again.
- **Red LED 7 on a driver blinks:** following error. Lower the speed or acceleration limits in Setup, option `3`.
- **Web page shows no motion:** press *Telemetry* in the panel (or `v` at the menu). Telemetry only flows while a mode is running.
- **The new program misbehaves on the rig:** the single-file test program in `firmware/legacy/` still works; swap `main.c` back.

## Licence

All rights reserved