# FT8 receive mode (ESP32-P4 SDR)

FT8 decoding ported from the DeepSDR 101 (GD32F450) firmware. The DSP and
decoder are the same code that was validated on real hardware there, with the
same parameters. Only the platform glue is new.

## What it does

- **FT8 button** in the bottom button row toggles the mode. On entry the VFO
  switches to USB and snaps to the FT8 dial frequency of the current band
  (160 m to 2 m). If the VFO is outside those bands it goes to 14.074 MHz.
- **FT8 panel** covers the spectrum and waterfall area while active:
  - A status line with the clock and its source, the band-sync state and DT
    error, symbol progress (x/93), and the last slot's decodes, candidates
    and decode time.
  - A **TIME** button (rough manual time entry, see below).
  - A 0-1600 Hz scale and a scrolling cascade of the FT8 search window,
    covering about 30 s (two slots).
  - The decoded messages, in the DeepSDR format plus a DT column:
    `HH:MM freqHz snrdB DT ~ message [distance km]`.
- **Tapping the cascade** hides it and gives the message list the full
  height. Tapping the list brings the cascade back.
- **Changing the mode away from USB** anywhere (menu, MODE button) leaves FT8
  automatically.
- **Search window**: 0-1600 Hz of audio above the dial frequency. This is the
  same limitation as on the DeepSDR, see `ft8_waterfall_adapter.h`.

## Time synchronization: the band is the clock

FT8 needs the capture window aligned to the 15 s slots within a fraction of
a symbol. This receiver does **not** need NTP, an RTC or the PC tool for that:
it locks its slot clock to the transmissions on the air
(`ft8/ft8_band_sync.c`). Every station keys to the same UTC boundary, so the
band itself is the time reference.

The idea comes from the PortaPack Mayhem FT8 RX (PR #3320). The measurement
here is different: every CRC-valid decode is timed with sub-step resolution,
by parabolic interpolation of the Costas sync score around the decoded
candidate. The loop then steers the system clock with the median DT of the
slot. Only CRC-valid decodes are trusted. The Costas array repeats every 36
symbols, so a window that is one period off still scores well on sync alone,
but it never decodes.

The status line shows the loop state:

- `SEARCHING`: nothing decodes and the band sounds quiet.
- `HEARD, NO DECODE`: signals are present but nothing decodes. The clock is
  too far off, or the receive chain has a problem. This state is reported
  separately on purpose.
- `SYNCING dt±x.xx s`: decodes are arriving and the grid is being steered
  onto them.
- `LOCKED dt±x.xx s`: the median DT has been within ±60 ms for two decoded
  slots. From then on the loop only applies small, damped trims.

Every decoded line also shows its own DT, using the WSJT-X convention
(0 = on time).

### How to start

- **Rough time.** Press **TIME** on the FT8 panel, type the UTC time as
  HHMMSS, and press OK when a reference clock reaches it. Being a second or
  two off is fine. The decoder searches -1.6 to +3.2 s around the boundary,
  so the first slot usually decodes and the first correction lands the grid.
  The **Sync slot** key in the same popup snaps to the nearest 15 s
  boundary: press it when a reference clock shows :00, :15, :30 or :45.
  That alone is enough for the band loop, which only needs the slot phase.
- **Badly set clock.** If signals are HEARD but nothing decodes for 4 slots,
  the loop probes 0, +4, -4, +8 and -8 s around the time you set.
- **No clock at all.** The loop sweeps the grid in 4 s steps every two slots
  and covers the whole 15 s circle. On an active band this takes about three
  minutes at most. The HH:MM on decoded lines is meaningless until the time
  of day is set.
- **Quiet band.** A clock that was set is never swept while the band is
  quiet.

The serial tool (`time_sync_sdr101.py` over UART0, same frames as the
DeepSDR) and a future NTP/RTC source still work. They only set the starting
point, and the band loop refines from there. The console command `utc`
prints the current clock.

### Host verification (test/host)

- **`calibrate_dt.py`.** Matched 508 decodes against WSJT-X's DT column for
  the ft8_lib corpus. The pipeline bias is 0.163 s, now set as
  `FT8_DT_BIAS_S`; the per-decode spread is 0.032 s.
- **`band_sync_sim`.** Replays a band made of concatenated corpus slots with
  a wrong local clock, reproducing `ft8_app.c`'s scheduling exactly.

| Initial clock error | Clock set? | First slot LOCKED |
|---|---|---|
| +1.7 s | yes | 3 |
| -1.2 s | yes | 2 |
| -7.0 s | yes | 8 |
| +5.5 s | yes | 20 (decodes from slot 8) |
| +6.3 s | no | 7 |
| -4.9 s | no | 8 |
| +11 s | no | 4 |

On noise only, with the clock set, the loop issues no corrections.

## Architecture

```
sdrTask (core 1)     USB demod @12 kHz --ft8_app_feed_audio()--> stream buffer (PSRAM, 3 s)
FT8 task (core 0)    AGC -> 12k->3200 Hz polyphase resampler -> 1024-pt FFT every 80 ms -> mag[]
                     slot boundary (clock ms / 15000 changes): snapshot mag[], re-arm, decode,
                     time each decode -> band-sync loop -> trim the clock
LVGL task            ft8_ui.c: drains decoded lines, draws cascade, status
```

The tap is taken after USB demodulation but before the passband filter, NR
and the receiver AGC. This is the same point as `s_ssb_dec` on the DeepSDR.

The snapshot-then-rearm-then-decode order is kept from the DeepSDR fix for the
structural "capture + decode > 15 s" drift.

### Changes from the GD32 version

- **Oversampling.** `time_osr` is back to 2, ft8_lib's default. The GD32
  dropped it for RAM. On the host corpus this gives 376 decodes against 316
  with (1,2), plus the 80 ms timing resolution the band loop uses.
- **Memory.** All FT8 buffers (~214 KB) live in one PSRAM block
  (`ft8_ram.h`) instead of a union with the waterfall. The "FFT tables
  overwritten by the waterfall" class of bug cannot happen.
- **Threading.** The resampler runs in the FT8 task, not an ISR, so no
  critical sections are needed. The decoded-message queue is lock-protected
  (FT8 task to LVGL task).
- **Resampler input.** Float input, and a doubled ring buffer instead of a
  120-tap shift register. The coefficient table is copied verbatim.
- **FFT tables.** Built with libm `sinf`/`cosf`. The same fast-log2 dB scale
  is kept, so `FT8_ADAPTER_DB_OFFSET` (-112) still applies.
- **Clock.** The system clock replaces the RTC, and slot boundaries are
  detected by change of `now_ms / 15000` with sub-second precision. The
  band-sync loop steers it.
- **Cascade.** A row is pushed every 320 ms and shown on 96 rows.

## Build notes

- New component: `components/ft8_lib`. This is kgoba/ft8_lib, MIT license,
  decode sources only, commit 9fec6ca.
- Optional: enable **LVGL -> Font usage -> UNSCII 16** in menuconfig for a
  monospaced message list with aligned columns. Otherwise Montserrat 16 (or
  the default font) is used.
- The FT8 task stack (16 KB) is allocated in PSRAM when possible, with an
  internal-RAM fallback. Internal RAM use is otherwise negligible.

## Host regression test

`test/host` builds the exact pipeline with plain gcc and decodes a 12 kHz WAV:

```
cd test/host && make
./ft8_host_test path/to/ft8_lib/test/wav/20m_busy/test_01.wav [gain]
```

Results on the ft8_lib corpus (see also the band-sync results above):

- Decodes most of what the reference ft8_lib decoder finds inside 0-1600 Hz.
  It finds fewer overall because of the reduced (1,2) oversampling chosen on
  the DeepSDR.
- The result is level-independent: input scaled x0.0003 to x3 gives 162-174
  decodes over 20 files.

## Tuning after first on-air tests

- Each slot logs `dB[min..max]` (tag `FT8`). On the DeepSDR, good operation
  was around -104 / -7. If the max is far lower, check RF gain.
- If `AUDIO OVERRUN` appears in the status line, decodes are taking longer
  than the 3 s stream buffer. Increase `FT8_STREAM_SECONDS` in `ft8_app.c`.
