#include "ft8_fft1024.h"
#include "ft8_ram.h"
#include <math.h>

/* Typed views into the PSRAM block (see ft8_ram.h) - keeps the arithmetic below
 * identical to the GD32 original, which used the same names. */
#define s_re          (g_ft8_ram->fft_re)
#define s_im          (g_ft8_ram->fft_im)
#define s_twiddle_cos (g_ft8_ram->fft_twiddle_cos)
#define s_twiddle_sin (g_ft8_ram->fft_twiddle_sin)
#define s_hann        (g_ft8_ram->fft_hann)
#define s_bitrev      (g_ft8_ram->fft_bitrev)

#define FT8_PI_F 3.14159265358979f

/* Fast log2 approximation (float bit pattern trick) - kept from the GD32
 * version on purpose: FT8_ADAPTER_DB_OFFSET was calibrated against exactly
 * this formula's output scale. */
static float log2_approx(float x)
{
    union
    {
        float f;
        uint32_t i;
    } vx;
    float y;
    if (x <= 0.0f)
    {
        x = 1.0e-9f;
    }
    vx.f = x;
    y = (float)vx.i;
    y *= 1.1920929e-7f;
    return y - 126.94269504f;
}

void ft8_fft1024_init(void)
{
    uint32_t n;
    uint32_t bits = 0U;
    uint32_t size = FT8_FFT_SIZE;

    while (size > 1U)
    {
        size >>= 1U;
        bits++;
    }

    for (n = 0; n < FT8_FFT_SIZE / 2U; n++)
    {
        float angle = -2.0f * FT8_PI_F * (float)n / (float)FT8_FFT_SIZE;
        s_twiddle_cos[n] = cosf(angle);
        s_twiddle_sin[n] = sinf(angle);
    }

    for (n = 0; n < FT8_FFT_SIZE; n++)
    {
        s_hann[n] = 0.5f - 0.5f * cosf(2.0f * FT8_PI_F * (float)n / (float)(FT8_FFT_SIZE - 1U));
    }

    for (n = 0; n < FT8_FFT_SIZE; n++)
    {
        uint32_t v = n;
        uint32_t r = 0U;
        uint32_t b;
        for (b = 0; b < bits; b++)
        {
            r = (r << 1U) | (v & 1U);
            v >>= 1U;
        }
        s_bitrev[n] = (uint16_t)r;
    }
}

static void fft_run(void)
{
    uint32_t i, j;
    uint32_t half_size, step;

    for (half_size = 1U, step = FT8_FFT_SIZE / 2U; half_size < FT8_FFT_SIZE;
         half_size <<= 1U, step >>= 1U)
    {
        for (i = 0; i < FT8_FFT_SIZE; i += (half_size << 1U))
        {
            for (j = 0; j < half_size; j++)
            {
                uint32_t tw_idx = j * step;
                float tre = s_twiddle_cos[tw_idx];
                float tim = s_twiddle_sin[tw_idx];
                uint32_t a = i + j;
                uint32_t b = a + half_size;
                float br = s_re[b] * tre - s_im[b] * tim;
                float bi = s_re[b] * tim + s_im[b] * tre;
                s_re[b] = s_re[a] - br;
                s_im[b] = s_im[a] - bi;
                s_re[a] = s_re[a] + br;
                s_im[a] = s_im[a] + bi;
            }
        }
    }
}

void ft8_fft1024_compute_db(const int16_t *samples, float *db_out)
{
    uint32_t n;

    for (n = 0; n < FT8_FFT_SIZE; n++)
    {
        uint16_t src = s_bitrev[n];
        s_re[n] = (float)samples[src] * s_hann[src];
        s_im[n] = 0.0f;
    }

    fft_run();

    for (n = 0; n < FT8_FFT_BINS_USEFUL; n++)
    {
        float power = s_re[n] * s_re[n] + s_im[n] * s_im[n];
        db_out[n] = 3.0103f * log2_approx(power);
    }
}
