# `system-show` — Boardcore integration demo report

**Target executable:** `system-show`
**Board:** `stm32f767zi_skyward_compute_unit` (STM32F767ZI, 2 MB flash / 512 KB
internal SRAM, external SDRAM)
**Entrypoint source:** `src/entrypoints/system-show.cpp`
**Build system:** SBS (Miosix 3.0 one-board-per-configure model)
**Firmware size:** 234 240 bytes of text; `.bin` ≈ 230 KB

---

## 1. Purpose

`system-show` is a single entrypoint that demonstrates that the current
Boardcore + Miosix 3.0 configuration *builds, runs and behaves correctly* on
the compute unit. It is designed to be shown live to a professor:

- on **boot** it runs a deterministic self-test battery across shared Boardcore
  modules and reports `[PASS]/[FAIL]` on the kernel console (USART1);
- it then runs **hardware probes** for the PWM/timer and I2C drivers that
  degrade gracefully when no device is attached;
- it enters an **interactive USART3 CLI** (`help`, `info`, `tests`, `leds`,
  `sched`, `events`, `math`, `probes`, `version`, `time`, `echo`);
- it continuously blinks all user LEDs at 1 Hz and pushes a **live telemetry
  line once per second over USART using DMA**.

---

## 2. Board, toolchain and how to run it

```
./sbs build system-show
./sbs flash -r system-show        # -r = st-flash --connect-under-reset
./sbs run system-show             # opens the USART3 console (or use minicom)
```

Serial channels:

| Channel | Peripheral | Who drives it | What appears |
|---|---|---|---|
| Kernel console | USART1, PA9/PA10, 115200 | kernel `STM32DmaSerial` (`defaultSerialDma=true`) | `PrintLogger` lines (`[PASS] ...`), probe details, all logs |
| Demo CLI / live stream | USART3, PB10/PB11, 115200 8N1 (AF7) | Boardcore `USART` (RX) + Boardcore `DMA` driver (TX) | menu, replies, `LIVE dma#...` telemetry |
| LEDs | PB7, PE3, PC13+PG9, PC2, PC14, PC15 | BSP `bsp_impl.h` + raw GPIO | 1 Hz heartbeat |

`/dev/ttyACM0` is the ST-Link virtual COM port wired to USART1. The interactive
CLI and the DMA telemetry are on USART3, so a USB-UART adapter must be
connected to PB10 (TX) / PB11 (RX) to interact with the CLI.

---

## 3. Boot sequence (exact order)

1. Enable the `PrintLogger` stdout sink (it is disabled by default in Release)
   and set its level to `LOGL_INFO`.
2. Configure PC14/PC15 as GPIO outputs (extra board LEDs).
3. Print the startup banner with the git version string (`version.h`).
4. Configure USART3 pins (PB10/PB11, AF7) and instantiate the shared
   `Boardcore::USART(USART3, 115200)` driver (used for RX).
5. Send the welcome banner over USART3 — already via DMA.
6. Run the **self-test battery** (`runSelfTests()`), then the **hardware
   probes** (`probePwm()`, `probeI2c()`).
7. Start a `TaskScheduler` with two 1 Hz tasks:
   - LED heartbeat (`allLedsOn/allLedsOff`);
   - live DMA telemetry (`dmaLiveTask()`).
8. Enter the CLI main loop: read bytes from USART3, assemble lines, dispatch
   commands, answer over DMA.

---

## 4. Self-test battery

All tests are deterministic and need no external hardware. Each result is
printed with `reportTest()` (console + USART3) and recorded in the global
bitmask `g_selfTestBits` (RAM address can be inspected with a debugger).

| Bit | Value | Module | What is verified | Pass criterion |
|---|---|---|---|---|
| 0 | `0x01` | `utils/Stats/Stats.h` | add samples 1..5, `StatsResult` | `n==5`, mean≈3, min≈1, max≈5 |
| 1 | `0x02` | `utils/collections/CircularBuffer.h` | `CircularBuffer<int,4>` put 1..4, pop FIFO | 4 items, full, pops 1,2,3,4 in order |
| 2 | `0x04` | `events/EventBroker.h` + `EventCounter` | publish/subscribe on topic 1, event 7 | 10/10 events received |
| 3 | `0x08` | `utils/SkyQuaternion` (Eigen) | euler(30,10,-20) → quat → euler round trip (degrees, roll/pitch/yaw order) + 2×45° yaw product | round trip ≈ (-20,10,30) ±0.2°, product → 90° yaw |
| 4 | `0x10` | `scheduler/TaskScheduler.h` | 20 Hz + 10 Hz tasks run for 1 s | ≥12 and ≥6 executions (slack allowed) |
| 5 | `0x20` | `drivers/dma/DMA.h` | memory→`USART3->TDR` DMA marker transfer | transfer-complete within timeout |

Expected result at boot: `self-test bits: 0x3f` (all six tests PASS).

The DMA self-test is meaningful: by then every printed `[PASS]` line has
already been transmitted over USART3 through the DMA driver, so the DMA path
is exercised continuously, not only by the explicit marker.

---

## 5. DMA implementation notes

All USART3 output goes through `dmaSendUart()`:

1. serialized with a `FastMutex` (shared by the CLI thread and the telemetry
   task);
2. `DMADriver::instance().acquireStreamForPeripheral(PE_USART3_TX)` →
   DMA1 Stream3 / Channel 4 (mapping from
   `src/shared/drivers/dma/board_mappings/stm32f767xx_mappings.cpp`);
3. `DMATransaction`: `MEM_TO_PER`, 8-bit source/destination, source = RAM
   buffer, destination = `USART3->TDR`, source increment, no circular mode;
4. `USART3->CR3 |= DMAT` (route TXE to DMA);
5. `setup()`, `enable()`, `timedWaitForTransferComplete(250 ms)`, `disable()`.

**Important driver caveat discovered during bring-up:** the shared DMA driver
registers the stream IRQ in `setup()` only when the transaction requests the
transfer-complete interrupt, and *unregisters* it otherwise. Calling
`setup()` with interrupts disabled on a fresh stream therefore tried to
unregister an IRQ that was never registered → kernel
`INTERRUPT_REGISTRATION_ERROR` → reboot. The fix used here: every transfer
registers the IRQ first (interrupt-enabled transaction) and explicitly
unregisters right after (`tx.enableTransferCompleteInterrupt = false; setup()`)
so the next transfer can register again.

USART1 console output is also DMA, but performed by the kernel driver
(`STM32DmaSerial`, because `defaultSerialDma=true` in the board settings); the
Boardcore DMA driver cannot take USART1 while the kernel console owns its IRQ.

---

## 6. Hardware probes (boot + `probes` command)

### `probePwm()` — `drivers/timer/PWM`

- Creates `Boardcore::PWM(TIM4, 1000)`; TIM4 is unused by the compute unit BSP.
- **No output channel is enabled**, so no GPIO is reconfigured (TIM4_CH1 would
  land on PB6, which the BSP uses for SDRAM — deliberately avoided).
- Verifies the driver programmed the timer: `TIM4->CR1.CEN == 1` and
  `TIM4->ARR > 0`.
- Deterministic; expected `[PASS] drivers/timer/PWM (TIM4 init)`.

### `probeI2c()` — `drivers/i2c/I2CDriver`

- Creates `I2CDriver(I2C1, PB8=SCL, PB9=SDA)` and scans the 7-bit address
  range `0x08..0x77` at `STANDARD` speed (single-byte read per address).
- Every ACK is collected and printed (`I2C1 scan: N device(s): 0x.. 0x..`).
- **No device attached is a valid result**: the bus is initialized and
  exercised, so the probe reports the scan detail and marks the driver
  `[PASS]`; the detail line says `no devices found (nothing attached -> SKIP)`.
- PB8/PB9 are the conventional I2C1 pins; if the compute unit routes its
  sensor bus elsewhere, the probe should be pointed at the real bus/pins to
  find onboard devices.

---

## 7. CLI reference (USART3)

| Command | Action |
|---|---|
| `help` | prints the command list |
| `info` | board name, git version, uptime (ms), DMA TX ok/fail counters |
| `tests` | re-runs the six self-tests |
| `leds <n>` | blinks all six LEDs `n` times (clamped 1..20) |
| `sched` | re-runs the TaskScheduler demo |
| `events` | re-runs the EventBroker demo |
| `math` | re-runs Stats + CircularBuffer + SkyQuaternion |
| `probes` | re-runs the PWM and I2C probes |
| `version` | prints the git version string |
| `time` | prints uptime in ms |
| `echo <text>` | echoes the text |

Every reply is transmitted over USART3 via DMA.

---

## 8. Live telemetry (`dmaLiveTask`, 1 Hz)

Once per second the task builds:

```
LIVE dma#<seq> t=<uptime ms> txOk=<n> txFail=<n>
```

and pushes it twice:

1. over **USART3** with the shared Boardcore DMA driver (explicit transfer);
2. over the **USART1 console** with `printf` (kernel `STM32DmaSerial` → DMA).

`txOk`/`txFail` are the cumulative DMA transfer counters, so the professor can
see the DMA driver working continuously (and can also trigger transfers by
typing any CLI command).

---

## 9. On-target verification evidence (2026-09-03)

All checks were performed live over SWD (`st-util` + `arm-miosix-eabi-gdb`)
and through serial capture:

- **Chip identity:** DBGMCU IDCODE `0x10016451` (device `0x451` = STM32F767),
  2 MB flash / 512 KB SRAM — matches the BSP MCU.
- **BSP pin map:** GPIO MODER values after boot match the compute unit BSP
  (PB5/PB6 SDRAM AF, PB7 LED output, PB10/PB11 USART3 AF, PC0 SDNWE AF,
  PC2/PC13 LEDs, PE3 LED, PE7-15 SDRAM AF, ...).
- **SDRAM:** `FMC_SDSR` active; stack pointer lives at `0xD0004290` in SDRAM —
  only possible on a board with the compute unit SDRAM.
- **Self-tests:** `g_selfTestBits == 0x3f` read from RAM after boot (all six
  PASS, including DMA).
- **Heartbeat:** GPIOB/GPIOE/GPIOC/GPIOG ODR sampled across ticks — LEDs
  toggle as designed; PC14/PC15 added on request.
- **DMA TX:** DMA1 Stream3 NDTR = 0 at telemetry ticks (previous transfer
  fully drained), transfer-complete waits succeed, USART3 `CR1=0x3D`
  (UE bit 0 on F7 + RE/TE/IDLEIE/RXNEIE), TEACK/REACK + TXE/TC set in ISR.
- **Console:** boot log shows six `[PASS]` lines, probe details, then the
  1 Hz `LIVE dma#...` stream.

---

## 10. What is NOT exercised by this show

The full Boardcore library (103 shared sources) is linked, so I2C, SPI, CAN,
PWM outputs, sensors, radio, actuators, the data Logger, mxgui etc. compile
and link, but only the modules listed in §4–§6 are actively executed by
`system-show`. Drivers that require external wiring (sensors, radio, CAN bus,
SPI slaves, servo/stepper outputs) are intentionally left out rather than
shown failing; `probeI2c()` is the only bus driver touched, and it reports
gracefully when the bus is empty.

---

## 11. Known caveats

- **PC14/PC15** are the OSC32 pins on the MCU; driving them as LEDs is fine
  as long as the LSE/RTC is not used.
- **USART1** is owned by the kernel console; its DMA is the kernel's
  `STM32DmaSerial`, not the Boardcore DMA driver.
- **I2C probe pins** (PB8/PB9) are assumed; scan should be pointed at the
  real sensor bus of the board to enumerate actual devices.
- **`sbs flash`** needs `-r` (`--connect-under-reset`) on this board.
