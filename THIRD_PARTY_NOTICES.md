# Third-Party Notices

This project incorporates code and data from the following open-source
projects.

## ft8_lib (FT8/FT4 decoder)

- **Author:** Karlis Goba, YL3JG
- **Source:** https://github.com/kgoba/ft8_lib (commit 9fec6ca)
- **License:** MIT (see `components/ft8_lib/LICENSE`)
- **Used for:** FT8 decoding. The decode-side sources are vendored unchanged
  in `components/ft8_lib/`. Integration was ported from the DeepSDR 101
  firmware (`main/ft8/`).

## SDR++ color maps

- **Project:** SDR++, by Alexandre Rouma (Ryzerth)
- **Source:** https://github.com/AlexandreRouma/SDRPlusPlus, `root/res/colormaps/`
- **License:** GPL-3.0
- **Used for:** the waterfall color palettes in `main/palettes.c`: Classic,
  Viridis, Grayscale, Turbo, Inferno, Magma, Plasma, GQRX, Electric, Classic
  Green, Smoke, Temper Colors, Vivid and WebSDR. The palette data was ported
  via the DeepSDR 101 firmware.
- **Authors of the individual maps**, as credited in the SDR++ files:
  - Classic: Youssef Touil
  - GQRX: csete
  - Turbo: Google AI
  - Viridis, Inferno, Magma, Plasma: B.I.D.S.
  - Electric, WebSDR: Ryzerth
  - Classic Green: Paul (PD0SWL)
  - Smoke, Temper Colors, Vivid: Yaroslav Andrianov
- **Not from SDR++:** "Fire" is the DeepSDR 101's own palette.
- **License note:** GPL-3.0 material requires the combined work to be
  distributed under GPL-3.0-compatible terms, with source available. Make
  sure this repository's own license is compatible.

## DMR AMBE+2 interleave schedule (from DSD)

- **Source:** the DMR AMBE interleave tables (rW, rX, rY, rZ) in
  `main/dmr/dmr_voice.c`.
- **Copyright:** (C) 2010 DSD Author.
- **Licence:** ISC. Permission to use, copy, modify and/or distribute this
  software for any purpose, with or without fee, is granted provided that
  the copyright notice and this permission notice appear in all copies. The
  software is provided "as is".

## mbelib (optional, not included)

- **Source:** https://github.com/lwvmobile/mbelib (commit 34adf9f).
- **Copyright:** (C) 2010 mbelib Author. **Licence:** ISC.
- **Used for:** DMR voice, when fetched with `tools/fetch_mbelib.sh` and
  enabled in menuconfig.
- **Not distributed with this repository.** The AMBE+2 algorithm is covered
  by patents in some jurisdictions; check that using it is lawful where you
  are before enabling it.

## Test references (not part of the firmware)

The files in `test/host_dmr/` use these projects only as independent
references. No code from them is included in the firmware.

- **dsd-fme** (lwvmobile, GPL-2.0), https://github.com/lwvmobile/dsd-fme
  - Its generator matrices are dumped into `test/host_dmr/fme_ref_matrices.h`
    for cross-checking only.
  - The optional test step runs its binary on our synthetic signals.
- **AIS-catcher** (jvde-github, GPL-3.0), https://github.com/jvde-github/AIS-catcher
  - Used to validate `test/host_ais/ais_synth.c` by decoding our synthetic
    AIS signals, and as a sensitivity baseline.
  - No code from it is included in the firmware.
- **DSDcc** (Edouard Griffiths F4EXB, GPL-3.0), https://github.com/f4exb/dsdcc
  - The real DMR capture `samples/dmr_it_8.dis` is used as a test input.
  - It is not redistributed here; fetch it from the DSDcc repository.
