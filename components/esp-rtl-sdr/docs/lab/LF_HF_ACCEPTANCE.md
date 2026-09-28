# LF/HF acceptance

Record V4 and V3c separately. A build or host test is not RF proof.

## Current capability matrix

`SUPPORTED` below means the profile selects a valid software path; it is not a
claim that this branch received RF at every listed frequency.

| Frequency | Blog V4 | Blog V3c | RF mode / notes |
|---|---|---|---|
| 24 kHz | EXPERIMENTAL | EXPERIMENTAL | V4 upconverter; V3 Q-direct. P4 RF UNVERIFIED. |
| 60 kHz | EXPERIMENTAL | EXPERIMENTAL | V4 upconverter; V3 Q-direct. V3 PC control/IQ captured. |
| 135.6 kHz | EXPERIMENTAL | EXPERIMENTAL | V4 upconverter; V3 Q-direct. V3 PC control/IQ captured. |
| 147.3 kHz | EXPERIMENTAL | EXPERIMENTAL | V4 tuner target 28.947300 MHz; V3 Q-direct. V3 PC control/IQ captured; DDH47 RF UNVERIFIED. |
| 474 kHz | EXPERIMENTAL | EXPERIMENTAL | V4 upconverter; V3 Q-direct. V3 control captured at 472.5 kHz; exact 474 kHz host-tested. |
| 500 kHz | SUPPORTED | EXPERIMENTAL | V4 advertised-range upconverter; V3 Q-direct. |
| 1 MHz | SUPPORTED | EXPERIMENTAL | V4 upconverter; V3 Q-direct. V3 capture used 1.000123 MHz. |
| 10 MHz | SUPPORTED | EXPERIMENTAL | V4 upconverter; V3 Q-direct. V3 PC control/IQ captured. |
| 20 MHz | SUPPORTED | EXPERIMENTAL | V4 upconverter; V3 Q-direct. V3 PC control/IQ captured. |
| 24 MHz | SUPPORTED | SUPPORTED / UNVERIFIED | V4 upconverter; V3 normal tuner boundary. |
| 28.8 MHz | SUPPORTED | SUPPORTED / UNVERIFIED | Both normal tuner; V4 still selects its HF input route at the edge. |
| 96.1 MHz | LAB VERIFIED | LAB VERIFIED | Normal tuner; V3 matched 3.570 MHz IF. |
| 162.4 MHz | SUPPORTED | SUPPORTED / UNVERIFIED | Normal tuner. |
| 433 MHz | SUPPORTED | SUPPORTED / UNVERIFIED | Normal tuner. |
| 1090 MHz | SUPPORTED | SUPPORTED / UNVERIFIED | Normal tuner. |

Nooelec SMArt V5 remains `UNSUPPORTED` below 24 MHz; it does not inherit the
V3c direct-sampling capability.

## ESP32-P4 RF acceptance record

| Field | Blog V4 | Blog V3c |
|---|---|---|
| Exact driver commit | Pending release commit; base `00a0bf7` | Pending release commit; base `00a0bf7` |
| Antenna and suitable band | MLA-30+; LF/HF receive antenna | Dipole and MLA-30+ for LF control tests; 915 MHz antenna for 906.875 MHz RF test |
| Cold start at 147300 Hz | PASS | PASS |
| Stable IQ counters / overruns / drops | PASS: zero / zero / zero in scoped soak | PASS: zero / zero / zero in scoped soak and 906.875 MHz observation |
| 147.300 kHz signal observed | UNVERIFIED | UNVERIFIED |
| MF AM audio received | PASS: 1450 and 1600 kHz; 1280 kHz intermittent | UNVERIFIED |
| LF → HF → VHF → LF retunes | PASS | PASS |
| Requested RF still reported exactly | PASS | PASS |
| Notes / propagation limits | Control/IQ/upconverter path proven; LF station reception unverified | 906.875 MHz RF proven; LF station reception unverified |

### V3c Tab5 run — 2026-09-14, 27-inch-per-leg dipole

- Hardware: ESP32-P4 rev 1.3 Tab5 on COM17; Realtek `RTL2838UHIDIR`, serial
  `00000001`, detected as `blog_v3_r820t2`.
- Cold start: exact RF 147300 Hz, Q-branch direct sampling, NCO `0x3fac34`.
- Quiet soak: 960 kS/s requested, 959488 scoped SPS (99.9%), 15351808 bytes
  over 8 seconds, zero overruns, zero consumer drops, zero short transfers.
- Mode transitions passed: 147.3 kHz -> 96.1 MHz -> 10 MHz -> 147.3 kHz ->
  96.1 MHz -> 147.3 kHz -> 96.1 MHz. Direct-to-normal restored the captured
  3.570 MHz demod IF and normal tuner gain behavior.
- Direct-mode tuner-gain rejection passed. User RF metrics remained exact.
- Antenna: center-fed dipole, 27 inches per leg. This is unsuitable for useful
  147.3 kHz sensitivity, so this run proves P4 control/IQ/transition behavior,
  not DDH47 reception.
- The final smoke `uninstall` row returned `ERR_USB` because ESP-IDF 5.5.4
  reported `usb_host_uninstall()` as `ESP_ERR_INVALID_STATE`. This is the
  example's already-documented one-host-install-per-boot limitation; all LF/HF
  rows and normal-tuner restoration passed before it.

### V3c Tab5 repeat — 2026-09-14, MLA-30+

- Hardware and flashed image were unchanged from the dipole run; only the
  antenna was replaced with the user-confirmed MLA-30+.
- Cold start again selected exact RF 147300 Hz, Q-branch direct sampling, and
  NCO `0x3fac34`.
- Quiet soak repeated at 959488 scoped SPS (99.9%) and 15351808 bytes over 8
  seconds, with zero overruns, zero consumer drops, and zero short transfers.
- The same transition sequence passed: 147.3 kHz -> 96.1 MHz -> 10 MHz ->
  147.3 kHz -> 96.1 MHz -> 147.3 kHz -> 96.1 MHz. Direct-mode gain rejection,
  normal-tuner restoration, and exact reported RF also passed.
- This smoke test validates control and stable IQ transport but does not
  measure or identify RF energy. DDH47 reception therefore remains unverified.
- The only failed row was the same post-test `usb_host_uninstall()` limitation;
  all radio-path checks completed first.

### V3c Tab5 RF run — 2026-09-14, 915 MHz antenna

- Setup: user-confirmed 915 MHz antenna on the V3c/Tab5 receiver at COM17.
  A Heltec V4 at COM24 transmitted US LongFast on 906.875 MHz.
- The driver restored normal tuner mode after the LF matrix, tuned and reported
  exactly 906875000 Hz, then read IQ continuously for 600 seconds.
- Transport result: zero read errors, overruns, consumer drops, and short
  transfers during the observation. `SMOKE rf915_observation` passed.
- Quiet DC-removed IQ power was 0.449. The first controlled message produced a
  7507.675 peak. Test 002 produced a 7142.924 peak; Test 003 and subsequent
  traffic produced additional distinct burst groups.
- Three public messages were sent: `Orc Test 001 - Please reply`, followed by
  numbered 002 and 003 repeats. The mesh returned an implicit acknowledgement.
- Two text replies were decoded independently by the Heltec: `Test heard`
  (RSSI -28 dBm, SNR 5.75 dB) and `Replied` from node `!435baa2c`
  (RSSI -29 dBm, SNR 5.75 dB).
- The retained serial portion covers 3755 measurement bins from 224.505 through
  600.007 seconds. It contains 16 burst groups above power 2 and peaks at
  7142.924; its SHA-256 is
  `5F36686C2D7D0C4DF1068B1D70EF9AE19161768EBE5180F90C1F9E97F9F5E37F`.
- Boundary: the V3c measurement proves real 906.875 MHz RF energy and stable IQ
  reception with this setup. Message contents and sender identity came from the
  Heltec decoder; this raw-IQ probe did not decode LoRa itself.
- The final overall row was 45 passed / 1 failed. The sole failure was the same
  post-test `usb_host_uninstall()` limitation after the RF window completed.

### Blog V4 Tab5 run — 2026-09-14, MLA-30+

- Hardware: ESP32-P4 rev 1.3 Tab5 on COM17; `RTLSDRBlog Blog V4`, serial
  `00000001`, detected as `blog_v4_r828d`.
- Release-test image SHA-256:
  `A205360FD6EC92646048429145072A6B4DCBB4DAA6E77666E34884182879EAB1`.
- Cold start: exact RF 147300 Hz, 28.947300 MHz tuner target, HF front-end
  route, and the V4 upconverter enabled.
- Quiet soak: 960 kS/s requested, 959488 scoped SPS (99.9%), 15351808 bytes
  over 8 seconds, with zero overruns, zero consumer drops, and zero short
  transfers.
- Mode transitions passed: 147.3 kHz -> 96.1 MHz -> 10 MHz -> 147.3 kHz ->
  96.1 MHz -> 147.3 kHz -> 96.1 MHz. Every step returned IQ and reported the
  requested RF exactly while selecting the expected HF or VHF front-end route.
- Manual gain, measured V4 tuner AGC, RTL AGC independence, and final manual
  gain restoration all passed.
- Antenna: user-confirmed MLA-30+. No LF station or other signal was identified,
  so this run proves the physical upconverter/control/IQ/transition path but
  does not claim 147.3 kHz RF reception.
- User-observed OrcSDR AM-dashboard reception with audible program audio at
  1450 and 1600 kHz; 1280 kHz was received intermittently. This verifies the
  V4 upconverter through AM demodulation and audio for that setup, while true
  LF reception remains unverified.
- The final overall row was 43 passed / 1 failed. The sole failure was the same
  post-test `usb_host_uninstall()` limitation after all radio checks passed.

Required boundary checks: 24 kHz, 60 kHz, 135.6 kHz, 147.3 kHz, 474 kHz,
500 kHz, 1 MHz, 10 MHz, 28.799999 MHz, 28.8 MHz, 28.800001 MHz, 96.1 MHz,
162.4 MHz, 433 MHz, and 1090 MHz. For V3c also record the captured direct-
sampling mode and both normal/direct transition directions. Do not mark V3c
LF/HF accepted from public documentation or unlabeled USB traffic.
