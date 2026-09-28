#pragma once

#include <stdint.h>

/*
 * Spectrum / waterfall color palettes - the full set ported from the DeepSDR
 * 101 (GD32F450) firmware, which in turn took them from SDR++'s own colormap
 * files (root/res/colormaps/ (one JSON per map), AlexandreRouma/SDRPlusPlus, GPL-3.0; each
 * map's author is credited in palettes.c). "Fire" is the DeepSDR's own
 * addition. The RGB565 format is identical on both boards, so the tables are
 * copied verbatim (extracted by script, not retyped).
 *
 * Every palette is rendered into a 256-entry RGB565 LUT (index 0 = weakest),
 * which ui.c's fft_color_map() reads for the waterfall and the FT8 cascade.
 */
typedef enum
{
    PALETTE_CLASSIC = 0,
    PALETTE_FIRE,
    PALETTE_VIRIDIS,
    PALETTE_GRAYSCALE,
    PALETTE_TURBO,
    PALETTE_INFERNO,
    PALETTE_MAGMA,
    PALETTE_PLASMA,
    PALETTE_GQRX,
    PALETTE_ELECTRIC,
    PALETTE_CLASSIC_GREEN,
    PALETTE_SMOKE,
    PALETTE_TEMPER_COLORS,
    PALETTE_VIVID,
    PALETTE_WEBSDR,
    PALETTE_COUNT
} palette_id_t;

/* Short display name ("Classic", "Viridis", ...). */
const char *palette_name(int id);

/* Renders palette `id` into lut[256]. Out-of-range ids render Classic. */
void palette_build_lut(int id, uint16_t lut[256]);
