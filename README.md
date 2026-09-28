# ESP32-P4 SDR (JC4880P443CIW + RTL-SDR)

A standalone software-defined receiver built on the Guition JC4880P443CIW
board. It combines an ESP32-P4, a 4.3" 800x480 touch display, 32 MB of PSRAM
and 16 MB of flash.
An RTL-SDR Blog V4, connected over USB Host, is the RF
front end. The radio decodes FT8, DMR and AIS on its own, with no PC.

## Hardware

- **Board:** JC4880P443CIW. ESP32-P4 at 360 MHz, MIPI-DSI display, GT911
  touch, rotary encoder.
- **RF:** RTL-SDR Blog V4 on the USB Host port, driven by `esp_rtl_sdr`. The
  dongle runs at 960 kS/s and is decimated on the board (CIC, drift
  resampler, FIR) to:
  - 48 kS/s for the narrow modes;
  - 192 kS/s for WFM and AIS.
- **Audio:** the on-board ES8311 codec at 48 kHz. It is a **mono** codec, so
  FM stereo is intentionally not implemented.

## Features

### Receiver

- **Modes:** AM, SAM (synchronous AM, also on USB/LSB sidebands), USB, LSB,
  NFM and WFM.
  - WFM uses the 192 kS/s wide path and is tuned on-frequency.
  - The narrow modes use a 12 kHz LO offset.
- **Display:** spectrum and waterfall with 15 colour palettes (the SDR++ set
  plus "Fire"), selectable from the menu and saved in NVS.
- **Tuning:** drag the spectrum or turn the encoder.
  - Changes within ±24 kHz are made by a digital NCO, so the I/Q stream
    never stops and the display does not freeze.
  - The dongle is retuned physically only beyond that window, and re-centred
    once after tuning goes idle.
- **Other:** S-meter, noise reduction (NR), selectable filters and steps.

### FT8 (FT8 button)

- **Decoder:** ft8_lib with a 0-1600 Hz search window, ported from the
  DeepSDR 101 firmware.
- **Band-referenced slot clock:** each decode is timed to about 30 ms and a
  loop steers the local clock onto the transmissions on the air. A rough
  time setting is enough. Press TIME, then either enter HHMMSS or tap
  "Sync slot" at :00, :15, :30 or :45.
- **Display:** cascade view, decode list with SNR, DT and distance to your
  grid.

### DMR (DMR button)

- **Metadata:** colour code, talkgroup or destination, source ID, talker
  alias and the **GPS position** carried in the embedded LC. The position is
  shown as a locator and a distance.
- **Streams:** repeaters (both timeslots), hotspots and simplex.
- **Voice (optional):** AMBE+2 decoding with mbelib. mbelib is **not
  included**, because of patents in some countries:
  1. run `tools/fetch_mbelib.sh`;
  2. enable *menuconfig → SDR: DMR → DMR voice*.
- **Slot selection:** tap a slot card to listen to that slot only; tap again
  to return to auto.
- **Voice robustness:** corrupted-frame concealment and a soft limiter.

### AIS (AIS button)

- **Reception:** both channels at once, 161.975 and 162.025 MHz. The radio
  tunes 162.000 MHz on the WFM wide path.
- **Radar view:** centred on your QTH, with class A, class B, base stations
  and aids to navigation. Tap it to change the range (AUTO, 2 to 100 NM).
- **Vessel list:** name or MMSI, class, SOG, COG, distance, bearing and age.
- **Clock:** base-station reports set the shared UTC clock when no better
  source is available.

### Shared clock

- **Sources:** the serial PC tool (`time_sync_sdr101.py` on UART0), manual
  entry, or an AIS base station.
- **Used by:** the UI clock, FT8 and the DMR call log.

## Screen layout

Select the profile in *menuconfig → SDR: Display → Screen layout*:

- **1024 x 600**: the original design, the default.
- **800 x 480**

All UI geometry is derived from `main/include/ui_layout.h`: spectrum and
waterfall, button row, header, S-meter, menu, and the FT8, DMR and AIS
panels.

- **The FFT is unchanged.** It stays at 1024 bins. On an 800 px screen each
  display column shows the maximum of the bins it covers, so narrow or weak
  signals remain visible. The tuned-frequency marker and the passband box
  map onto the same columns as their bins.
- **What the 800 x 480 profile changes:**
  - spectrum 140 px and waterfall 100 px;
  - 124 px buttons;
  - an FT8 cascade of 3 px per bin;
  - two 388 px DMR cards;
  - a 240 px AIS radar;
  - a 7-column AIS table 510 px wide.

## Bottom-row buttons

| Button | Action |
|---|---|
| MENU | modes, filters, NR, gains, palette |
| MODE | cycle the demodulation mode |
| AIS | AIS receiver on/off (restores the previous frequency and mode) |
| STEP | tuning step |
| FT8 | FT8 mode on/off |
| DMR | DMR mode on/off |

FT8, DMR and AIS exclude each other; each one takes over the spectrum area.

## Building

Requires ESP-IDF v5.5.

1. Create a custom partition table for the 16 MB flash, in `partitions.csv`
   at the project root:
   ```
   nvs,      data, nvs,     0x9000,  0x6000,
   phy_init, data, phy,     0xf000,  0x1000,
   factory,  app,  factory, 0x10000, 0x600000,
   ```
2. In menuconfig:
   - set the flash size to 16 MB;
   - select the custom partition table `partitions.csv`.
3. On an 800x480 screen, select *SDR: Display → Screen layout → 800 x 480*.
4. Optional settings:
   - *SDR: DMR → DMR voice (mbelib)* for DMR voice;
   - *LVGL → Fonts → UNSCII 16* for a monospaced FT8 list.
5. Build and flash with `idf.py fullclean build flash monitor`.

## Serial console (UART0, 115200)

| Command | Purpose |
|---|---|
| `utc` | show the current clock, its source and sync statistics |
| `usbguard` | clear the `esp_rtl_sdr` USB fault guard after repeated crashes during USB enumeration, then reboot |

Binary time and grid frames from `time_sync_sdr101.py` are accepted on the
same port.

## Documentation

- `README_FT8.md`: FT8 port, band-sync loop, calibration.
- `README_DMR.md`: DMR demodulator, protocol, voice, GPS, verification.
- `README_AIS.md`: AIS receiver, design notes, results.
- `THIRD_PARTY_NOTICES.md`: licences for ft8_lib (MIT), the SDR++
  colormaps (GPL-3.0), DSD AMBE tables and mbelib (ISC), and the test
  references.

## Licence

This project includes GPL-3.0 material (the SDR++ colormaps), so the
combined work is distributed under GPL-3.0-compatible terms. See
`THIRD_PARTY_NOTICES.md`.
