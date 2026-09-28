# AIS receive mode (ESP32-P4 SDR)

This mode receives ship AIS on both channels at once:

- 87B, 161.975 MHz
- 88B, 162.025 MHz

## Use

Open the menu and press **AIS**. The radio saves the current frequency and
mode, tunes 162.000 MHz on the WFM wide path, and shows the AIS panel over
the spectrum. Pressing AIS again restores the previous frequency and mode.
FT8, DMR and AIS exclude each other.

The panel has three parts:

- **Radar view**, centred on your QTH. It uses the same grid as FT8, set
  with the PC tool's MSG_SET_GRID. The rings scale automatically
  (2/5/10/20/50/100 NM). Symbols:
  - class A: green triangle along the course over ground;
  - class B: cyan triangle;
  - base station: yellow diamond;
  - aid to navigation: magenta diamond.
- **Vessel list**: the most recently heard vessels, with name or MMSI,
  class, SOG, COG, distance, bearing and age. Vessels not heard for 10
  minutes drop out.
- **Status line**: frames per channel and the last base-station UTC time.

**Clock.** Base-station reports (message 4) set the shared clock, shown as
"AIS". This happens only when no better source (serial tool or NTP) is
present, and at most every 10 minutes. The same clock gives FT8 its initial
time and timestamps the DMR log.

## Architecture (`main/ais/`)

```
sdrTask (WFM branch)  192 kSps IQ --ais_app_feed_iq()--> stream buffer (PSRAM, 0.5 s)
AIS task              per channel: NCO to 0 Hz -> 64-tap low-pass, /4 -> 48 kSps (5 samples/symbol)
                      -> one-symbol differential phase detector -> burst-gated DC (carrier offset)
                      -> 5 parallel sampling phases: slicer -> NRZI -> HDLC -> CRC-16/X.25 -> dedupe
                      -> ais_msg: types 1/2/3, 4, 5, 18, 19, 21, 24 -> vessel table (64)
LVGL task             ais_ui.c
```

Design notes, all measured on the host (`test/host_ais`):

- **Parallel sampling phases instead of a symbol-clock loop.** The loop
  often had not locked by the end of the 24-bit training sequence, and it
  lost about 25 % of frames even at 30 dB.
- **HDLC flag handling.** The closing 0 of a flag must not become the first
  data bit of the next frame. It did in the first version, which shifted
  every real frame after a noise-opened false frame.
- **Slow carrier-offset tracking** (0.004 per sample), with fast acquisition
  over the training sequence after a burst onset. Frozen while idle.
- **One-symbol differential detection** rather than a per-sample FM
  discriminator, so single-sample ±2π noise jumps cannot corrupt the
  decisions.

## Verification

- **Synthesizer.** `ais_synth` encodes known messages (types 1, 5, 4, 18,
  24A and 21), frames them and modulates them as GMSK on both channels.
  **AIS-catcher decodes all of them with exactly the intended content**,
  which validates the test signals.
- **Our receiver, board orientation**, 180 messages per point, +500 Hz
  offset:

  | SNR | Decoded |
  |---|---|
  | 20 dB | 100 % |
  | 15 dB | 99 % |
  | 12 dB | 76 % |
  | 10 dB | 33 % |

  Carrier offsets up to about 2 kHz are tolerated. The RTL-SDR V4 alone is
  within ±160 Hz at 162 MHz.
- **Headroom.** AIS-catcher, which uses coherent demodulation, still
  decodes everything at 10 dB, so there are about 3-4 dB left for a future
  coherent demodulator. Nearby traffic is unaffected.

Run the tests with `test/host_ais/run_ais_tests.sh`. Set
`AISCATCHER=...` to add the cross-check.

## On the radio

- **Antenna.** Your 2 m antenna works reasonably at 162 MHz. Range is
  line-of-sight to the sea.
- **Frames with CRC errors.** Most are noise that happened to contain a
  flag pattern. Only CRC-valid frames are used.
