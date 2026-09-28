# DMR receive mode (ESP32-P4 SDR): metadata + optional voice

Decodes amateur DMR traffic in the clear: colour code, talkgroup or
destination, source ID, talker alias, and call start/end. It works with
repeaters (BS sourced, both timeslots), hotspots and simplex (MS sourced /
direct mode). Voice (AMBE+2) is optional and needs mbelib; see "Voice" below. Without it,
the NFM audio is muted while DMR is on.

## Use

- **DMR button** in the bottom row. Tune to the repeater output or simplex
  frequency first. DMR switches to NFM and keeps the frequency. FT8 and DMR
  exclude each other.
- **The panel** covers the spectrum area and shows:
  - A status line: lock, repeater/simplex, polarity, the measured frequency
    offset (useful to trim the dongle's ppm) and the deviation (nominal
    ~1944 Hz).
  - One card per timeslot: activity (VOICE / IDLE / DATA / CSBK), CC, TG or
    ID, SRC, talker alias and call time.
  - The last calls, timestamped with the shared UTC clock (see
    README_FT8.md).
- **Serial log.** Every call event is also logged on the console with the tag
  `DMR`.

## Position (GPS Info LC)

Some radios send their position inside the embedded LC of voice. This is
ETSI TS 102 361-2 FLCO 0x08, with the standard FID 0 or Hytera's 0x68. The
decoder shows it on the slot card and in the call log:

- the Maidenhead locator;
- the distance to your own QTH, which is the same grid as FT8
  (`ft8_decoder_get_own_latlon()`, set with the PC tool's MSG_SET_GRID);
- the full position with its error bound, in the serial log.

Field layout, checked against dsd-fme's decoder:

| Bits | Field |
|---|---|
| 16-19 | reserved |
| 20-22 | position error: 2·10^n m, 7 = unknown |
| 23-47 | longitude, 25-bit two's complement, 360/2^25 °/LSB |
| 48-71 | latitude, 24-bit two's complement, 180/2^24 °/LSB |

The resolution is about 1-2 m. The sign is undone as exact two's complement.
dsd-fme uses "0x800001 - value", which is one LSB off, so it shows 16.64251°
where the transmitted value is 16.64250°.

Test: `dmr_synth` embeds the Teide summit (28.27239 N 16.64250 W, less than
20 m). Both dsd-fme and this decoder recover it; ours reads IL18qg.

LRRP, Motorola's packet-data location, is not decoded yet. It needs data
packet reassembly, including rate 3/4 trellis.

## Architecture (`main/dmr/`)

```
sdrTask (core 1)   raw 48 kS/s IQ of the NFM branch --dmr_app_feed_iq()--> stream buffer (PSRAM, 0.5 s)
DMR task (core 0)  fs/4 rotation -> 41-tap 7 kHz channel filter -> FM discriminator (Hz)
                   -> RRC matched filter (a=0.2) -> sync search / burst framer -> 4-level slicer
                   -> dmr_proto: CACH/TACT, slot type (Golay), BPTC(196,96)+RS(12,9) LC headers
                      and terminators, CSBK (CRC-CCITT), EMB (QR) + embedded LC (BPTC 8x16 +
                      CRC5) with IDs and talker alias
LVGL task          dmr_ui.c
```

The files:

- `dmr_fec.c`: every code built from its generator polynomial, then checked
  codeword by codeword against dsd-fme's matrices. RS(12,9) includes
  single-byte correction.
- `dmr_demod.c`: 10 samples/symbol. Sync is found by normalized correlation
  against the 8 sync words. Level and offset come from a least-squares fit on
  the sync symbols, so the slicer needs no AGC and ignores tuning error. Once
  locked, bursts are cut every 30 ms (60 ms for MS), and each sync field is
  re-searched ±4 samples to re-centre timing.
- **Polarity** is found by elimination. It cannot be read off the sync,
  because the voice and data sync words are exact complements. The CACH TACT
  cannot help either: its Hamming(7,4) code contains the all-ones word. Only
  the slot-type Golay and the EMB QR codes, which are not closed under
  complement, count as proof of a correctly sliced stream.
- **Cost.** On the host it runs ~180x real time on IQ. On the P4 it is
  estimated at 10-20 % of one core.

## Verification (`test/host_dmr/run_dmr_tests.sh`)

- **FEC.** The six codes match dsd-fme's generator matrices for every
  codeword, with minimum distances 3/3/3/4/8/6. BPTC, embedded LC and RS were
  tested with injected errors.
- **Real repeater capture** (DSDcc's `samples/dmr_it_8.dis`, Italy, 20 s),
  against dsd-fme on the same file:

  | | dsd-fme | this decoder |
  |---|---|---|
  | Colour code, call | CC 4, TS2, TG 19535, SRC 2222223 | same |
  | Bursts / idle | 663 / 332 | 664 / 333 |
  | Embedded LC clean | 44 / 53 | 47 / 53 |
  | Voice LC header | 1 | 1 |

- **Synthetic signals** (`dmr_synth`), repeater and simplex, with headers,
  voice superframes, talker alias, terminators and CSBK. They are first
  validated by dsd-fme, which decodes them with no errors. Through the full
  IQ chain the results are:
  - Complete metadata (IDs, alias, headers, terminators) down to 7 dB SNR
    in 12.5 kHz.
  - IDs alone down to 5-6 dB.
  - Tuning error of ±2 kHz is tolerated.
  - The measured frequency offset is accurate to about ±5 Hz.
  - Inverted polarity is detected automatically.

The BPTC(196,96) interleave direction was first implemented backwards. That
mistake is self-consistent, so our own encoder and decoder still agreed.
Only feeding our encoding to an independent decoder exposed it. Keep that
cross-check (the `DSDFME=` step) when touching the FEC.

## Front-end orientation (confirmed on the radio)

This board's `rtl_source` delivers conjugated IQ. The DMR channel sits at
-12 kHz, which is why the fs/4 rotation taken from sdr.c's SSB path is right,
and the FM deviation comes out negated. A wrong rotation would put the channel
at 24 kHz, outside the channel filter, and not a single sync would be found.

The demodulator therefore starts with inverted polarity
(`DMR_DEFAULT_POLARITY`), and the panel shows "INVERTED". That is expected.
The displayed frequency offset is corrected so that it reads as the real RF
tuning error. Automatic polarity detection still flips it if another front
end ever delivers the other orientation.

The channel filter is 7 kHz. On crowded bands it can be narrowed a little
(`CH_CUTOFF_HZ`).

## Voice (optional, mbelib)

mbelib (ISC licence) decodes the AMBE+2 speech. The AMBE+2 algorithm is
covered by patents in some countries, so the library is **not included** in
this repository. Fetching and enabling it is the builder's decision.

To enable it:

1. `tools/fetch_mbelib.sh` clones mbelib, at the tested commit, into
   `components/mbelib/mbelib`. That folder is git-ignored.
2. `idf.py menuconfig` -> **SDR: DMR** -> **DMR voice (mbelib, AMBE+2)**.

With the option off (the default), everything builds as before and DMR is
metadata-only. The status line then shows "voice: off (no mbelib)".

How it works (`main/dmr/dmr_voice.c`, `dmr_app.c`):

- **Extraction.** Each voice burst A-F carries three 72-bit AMBE+2 frames:
  dibits 0-35, 36-53 + 78-95 (split around the centre field) and 96-131.
  They are de-interleaved into mbelib's `ambe_fr[4][24]` with DSD's DMR
  schedule (ISC licence), then passed to
  `mbe_processAmbe3600x2450Framef()`, with one vocoder state per slot.
- **One slot at a time.**
  - AUTO follows the first slot that starts talking until its call ends
    (terminator, or 500 ms without voice).
  - Tapping a slot card fixes playback to that slot; tapping it again
    returns to auto.
  - The card being heard shows a speaker symbol.
  - Only that slot is decoded, which saves CPU.
- **Encrypted calls** (privacy bit in the LC service options) are shown as
  "ENC" and not played.
- **No garbage audio.** Nothing reaches the vocoder until a burst has passed
  a polarity-sensitive FEC check. While the demodulator is still on the wrong
  polarity, an inverted idle burst reads as a voice sync.
- **Audio path.** Audio goes from 8 kHz through a x6 polyphase interpolator
  (unity gain, images more than 71 dB down) to a 48 kHz stream buffer in
  PSRAM, and then to sdrTask's NFM branch in place of the discriminator
  audio. Playout starts after a 200 ms cushion (speech arrives in 60 ms
  bursts) and re-buffers after an underrun.

### CPU and choppy audio

With uvquality 3, mbelib calls `cosf()` about 21 000 times per 20 ms frame.
On the ESP32-P4, newlib's `cosf` is a software routine. That can take half a
core, starve the DMR task, drop IQ and make the voice break up.

`components/mbelib/mbe_fastmath.h` is force-included into mbelib only, so
mbelib itself is not modified. It replaces `cosf` with a Cody-Waite reduction
and a Taylor polynomial of degree 12:

- about 15 FPU operations per call;
- maximum error 6e-7 rad over ±30 000 rad;
- decoded speech within 65 dB of libm's.

Other measures against choppy voice:

- The DMR task runs at priority 5 with no core affinity.
- The IQ buffer holds 1 s.
- After a mid-call underrun, playout resumes after 120 ms (two bursts),
  instead of the full 200 ms.

The status line shows `voc x.x ms/f` (vocoder time per 20 ms frame),
`drop` (IQ blocks lost) and `und` (audio underruns). The same figures are
logged every 10 s with the tag `DMR`. If `drop` keeps rising, the CPU is
short: set the uvquality option in menuconfig to 1.

### Lost superframes (fixed)

The first on-air tests showed choppy voice with `drop` at 0 and `und`
rising. The cause was the BS slot assignment, not the CPU.

- **What went wrong.** A CACH TACT with two bit errors "corrects" to the
  wrong word, so a voice burst was sent to the other slot. Meanwhile that
  slot's idle burst, a valid data burst, was sent to the voice slot and cut
  the superframe short, losing up to 360 ms at a time.
- **The fix.** Slots are now predicted by their strict 30 ms alternation. A
  TACT that disagrees only re-phases the prediction when two consecutive
  TACTs agree.
- **Result.** On a synthetic 18 s call, no voice is lost down to 6 dB SNR.
  Before the fix, 0.3 s was lost at 9 dB and 0.66 s at 7 dB.
- **Note on `und`.** It also counts the normal drain at the end of each
  call.

### Short loud squawks (concealment)

On weak or fading signals, mbelib occasionally synthesises a corrupted frame
as a loud 20 ms burst, sometimes at full scale. This was reproduced on the
host by re-transmitting the real AMBE frames of the capture through the
synthetic channel at 6 dB SNR. Every such burst had a few corrected errors
(errs2 1-3) and several times the energy of the speech around it.

`dmr_voice.c` now handles it in three ways:

- **Concealment.** A frame with corrected errors whose rms exceeds 2x the
  recent error-free speech level is scaled down to that level. Error-free
  frames, and errored frames of normal energy, are left untouched.
- **Soft limiter.** It replaces the hard clip at ±1. It touches 0.07 % of
  clean speech samples.
- **Fades.** 2 ms fade-in and fade-out at playout start and at underruns
  avoid clicks.

At 6 dB, this removed the full-scale bursts and left no clipped frames. The
status line shows `conc`, the count of concealed frames.

Verification: the speech of the real capture was decoded and compared with
dsd-fme at the level of the 49-bit AMBE parameters, not the audio. mbelib
synthesises unvoiced sounds from random noise, and dsd-fme adds its own gain
and filtering, so comparing the audio would say little.

- **96.9 %** of frames are bit-identical.
- 28 of the 29 differing frames differ by 1-3 bits, with no FEC error
  reported by either decoder. Those are unprotected C2/C3 bits that the two
  demodulators sliced differently.
- On the identical frames, ours reports fewer FEC errors than dsd-fme (7
  against 12).

Run it with `MBELIB=... DSDFME=... DSDCC_SAMPLE=... run_dmr_tests.sh`.
