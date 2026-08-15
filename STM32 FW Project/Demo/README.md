# Demo — Tethered Auto-Hover

A stripped build of `Official release - 170120` that flies a fixed, open-loop throttle
profile on power-up while the original cascade PID holds attitude level. No radio, no BLE,
no USB. Intended to be run **on a tether**.

The source release is untouched — this is a standalone copy.

```
power on ──► calibrate 2 s ──► settle 2 s ──► spin-up 1 s ──► hover 10 s ──► descend 2 s ──► off (latched)
             LED1 on                          LED2 on
```

## Read this before powering it up

**The board must be flat and completely still for the first 4 seconds.** The gyro/accel
offsets and the level reference are captured in that window. Nothing verifies it — if the
board moves during calibration, the level reference is wrong and the drone will fly off at
an angle.

**This holds attitude, not altitude.** There is no altitude controller anywhere in this
codebase — no rangefinder, no optical flow, and the barometer is not used for control.
`PWM_HOVER` is an open-loop guess. Whether the drone hovers, climbs, or sinks depends on
battery voltage, payload, and prop condition. It will also drift horizontally, because
nothing measures position.

**Check motor numbering and prop direction before the first spin-up.** The mixer assumes
motors 1/3 spin one way and 2/4 the other, in the X layout. Wrong wiring flips the airframe
instantly and no amount of PID will save it.

## The one number you have to calibrate

`PWM_HOVER` in `Src/main.c`. It is currently **1100** and that is a guess.

1. Set it to `800`, flash, and watch it spin up and do nothing. This confirms the profile,
   the LED sequence, and the timing.
2. Raise it 50 counts at a time until the frame goes light on the string.
3. Mind the headroom: `set_motor_pwm()` clamps at 1900 and the PID can add ±800 per axis,
   so above roughly 1100 the mixer starts saturating during corrections and the attitude
   loop loses authority.

## Flight profile knobs

All at the top of `Src/main.c`:

| Macro | Default | Meaning |
|---|---|---|
| `T_SETTLE_S` | 2.0 | motors off, AHRS pulls level at `AHRS_KP_BIG` |
| `T_SPINUP_S` | 1.0 | ramp `PWM_SPINUP_FLOOR` → `PWM_HOVER` |
| `T_HOVER_S` | 10.0 | hold `PWM_HOVER` |
| `T_DESCEND_S` | 2.0 | ramp `PWM_HOVER` → 0 |
| `PWM_SPINUP_FLOOR` | 700 | where props just begin to turn — verify on your motors |
| `PWM_HOVER` | 1100 | **calibrate this** |
| `TILT_ABORT_RAD` | 0.87 | ~50°, cuts motors and latches off |

`HARD_CEILING_TICKS` is a belt-and-braces limit: nothing runs longer than the profile plus
3 s, whatever else happens.

## What changed from the release

| File | Change |
|---|---|
| `Src/main.c` | Rewritten, 1377 → 585 lines. BLE, USB, ADC, TIM2/receiver and the software timer removed. Adds the flight-profile state machine. Sensor read, offset calibration, gyro IIR, FIFO and yaw integration are unchanged. |
| `Src/flight_control.c` | Removed the `0.05f*gTHR + 633.333f` throttle mapping — `motor_thr` is now written directly by `main.c` in raw PWM counts. Deleted the two dead functions (`FlightControlPID()`, `PIDOuterLoopFrameTrans()`). **Both live loops are otherwise byte-for-byte unchanged.** |
| `Inc/flight_control.h` | Dropped the two deleted prototypes. All gains untouched. |
| `Inc/config_drone.h` | `USE_MAG_SENSOR` and `USE_PRESSURE_SENSOR` → 0. Neither was ever used for control; this saves two SPI transactions per 807.7 Hz tick. |
| `Src/stm32f4xx_it.c` | Removed the TIM2 and USB handlers. `EXTI4_IRQHandler` is deliberately kept — `HAL_SPI_MspInit(SPI1)` enables `EXTI4_IRQn`, and an unhandled EXTI4 would hard-fault. |

`euler_rc` is fixed at `{0, 0, 0}` — that is the entire "remote control" now.

`gTHR` survives as a **synthetic arm flag, not a throttle**. `ahrs.c` and `flight_control.c`
both key their `< MIN_THR` behaviour off it, so it still switches `AHRS_KP` between 10.0 and
0.4 and still gates the integrator resets.

## Removed from the build

- **Sources:** `rc.c`, `sensor_service.c`, `console.c`, `timer.c`, `usb_device.c`, `usbd_*.c`
  and their headers. Also `PID.c`, `filter.c`, `stm32f4xx_nucleo.c`, `main_backup.c`,
  `main_davide_debug.c` — these were never in the CubeIDE project to begin with.
- **Middlewares:** the whole folder. BlueNRG and the USB Device Library are both gone.
- **Drivers:** `BSP/Components/spbtlerf` (BlueNRG driver) and `BSP/Board`
  (`board.c` includes `lsm6ds33.h`, `lis3mdl.h` and `lps25hb.h`, none of which exist in this
  tree — it is a leftover from a different board variant and was never compiled).
- **Projects:** only the CubeIDE project is kept. EWARM, MDK-ARM, SW4STM32 and TrueSTUDIO
  were dropped along with the stale `Debug/` build artifacts and the `.ioc` — regenerating
  from CubeMX would put BLE and USB straight back.

The CubeIDE project links each source file individually. Its links and include paths have
been pruned to match: 51 file links, all verified to resolve. The project is renamed
**"ToyDrone Demo - Tethered Auto-Hover"** so it does not collide with the original in the
same workspace.

## Not verified

This has **not been compiled** — there was no `arm-none-eabi-gcc` available. What was
checked: brace/paren balance, that every symbol removed from `main.c` has no remaining
referrer, that all 51 project links resolve, that every `#include` in `Src/*.c` is findable
on the include path, and that TIM9's NVIC setup in `HAL_TIM_Base_MspInit` is untouched.
