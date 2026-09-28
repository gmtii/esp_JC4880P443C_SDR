#include "ft8_decimator.h"
#include "ft8_waterfall_adapter.h"

/*
 * Polyphase rational resampler coefficients: 480-tap Blackman-windowed-sinc
 * lowpass, cutoff 1600 Hz referenced to the 48 kHz virtual (post-
 * interpolation) rate, gain = L = 4. Pre-split into L=4 phases of 120 taps:
 * phase r holds h[r], h[r+4], h[r+8], ... Copied verbatim from the DeepSDR 101
 * firmware (generated offline there - not hand-typed).
 * Simulated response: flat (+12 dB = L) to ~1400 Hz, -63 dB by 2000 Hz,
 * better than -110 dB from 4400 Hz on.
 */
#define FT8_RESAMPLER_TAPS_PER_PHASE 120U
static const float s_resampler_phase[FT8_RESAMPLER_L][FT8_RESAMPLER_TAPS_PER_PHASE] = {
{
    /* phase 0 */
    7.711798034e-21f, -1.084814782e-06f, -5.352920494e-06f, -6.295617728e-06f,
    7.088871390e-06f, 3.362889998e-05f, 4.983366585e-05f, 2.365648799e-05f,
    -5.163992772e-05f, -1.322862573e-04f, -1.399712912e-04f, -2.270712728e-05f,
    1.797295117e-04f, 3.278819889e-04f, 2.649130918e-04f, -4.950188372e-05f,
    -4.545458974e-04f, -6.473114721e-04f, -3.872259492e-04f, 2.784459909e-04f,
    9.527252062e-04f, 1.097391389e-03f, 4.256912039e-04f, -7.866833146e-04f,
    -1.751138692e-03f, -1.642276898e-03f, -2.398219103e-04f, 1.729901949e-03f,
    2.905016359e-03f, 2.178536883e-03f, -3.804902075e-04f, -3.285113929e-03f,
    -4.421689274e-03f, -2.511481456e-03f, 1.721798711e-03f, 5.637867228e-03f,
    6.236314034e-03f, 2.330826887e-03f, -4.163305937e-03f, -8.985162006e-03f,
    -8.195137404e-03f, -1.167501993e-03f, 8.242228285e-03f, 1.359216824e-02f,
    1.004573047e-02f, -1.735912350e-03f, -1.489252757e-02f, -2.001404094e-02f,
    -1.141299733e-02f, 7.906139454e-03f, 2.635872143e-02f, 2.996356232e-02f,
    1.164265242e-02f, -2.194324403e-02f, -5.096643955e-02f, -5.142342106e-02f,
    -8.441918558e-03f, 7.339480894e-02f, 1.690913211e-01f, 2.432057709e-01f,
    2.661725457e-01f, 2.285762065e-01f, 1.457731651e-01f, 5.037048049e-02f,
    -2.339129215e-02f, -5.507882147e-02f, -4.550238496e-02f, -1.303389039e-02f,
    1.817561525e-02f, 3.104368171e-02f, 2.263652821e-02f, 2.597799630e-03f,
    -1.485572071e-02f, -1.992133995e-02f, -1.200169618e-02f, 1.692384217e-03f,
    1.184627188e-02f, 1.297095558e-02f, 6.009795901e-03f, -3.368192834e-03f,
    -9.030659752e-03f, -8.188595697e-03f, -2.510421177e-03f, 3.678794056e-03f,
    6.511750747e-03f, 4.867467725e-03f, 5.675958453e-04f, -3.274152790e-03f,
    -4.401650078e-03f, -2.644353206e-03f, 3.701040762e-04f, 2.560571731e-03f,
    2.760596488e-03f, 1.254972446e-03f, -6.878150899e-04f, -1.797676994e-03f,
    -1.584057895e-03f, -4.704795200e-04f, 6.658599682e-04f, 1.134678810e-03f,
    8.138196519e-04f, 9.073501585e-05f, -4.985175818e-04f, -6.356407082e-04f,
    -3.604809845e-04f, 4.737238901e-05f, 3.058347823e-04f, 3.054517631e-04f,
    1.275244062e-04f, -6.351451720e-05f, -1.489009898e-04f, -1.157735450e-04f,
    -2.970538246e-05f, 3.531453756e-05f, 4.864500419e-05f, 2.669385367e-05f,
    2.094219199e-06f, -7.038615552e-06f, -4.168009843e-06f, -5.023455431e-07f,
},
{
    /* phase 1 */
    -2.554809599e-08f, -1.923261847e-06f, -6.361449877e-06f, -4.593557782e-06f,
    1.303256191e-05f, 3.997574821e-05f, 4.824496707e-05f, 8.652212259e-06f,
    -7.411158285e-05f, -1.441161630e-04f, -1.227441761e-04f, 2.397642027e-05f,
    2.286674379e-04f, 3.365164325e-04f, 2.071912714e-04f, -1.528423985e-04f,
    -5.350603327e-04f, -6.291603712e-04f, -2.486819775e-04f, 4.675243178e-04f,
    1.057252392e-03f, 1.006079763e-03f, 1.489142191e-04f, -1.087703462e-03f,
    -1.847978290e-03f, -1.400938203e-03f, 2.471564906e-04f, 2.153964417e-03f,
    2.924369566e-03f, 1.674299314e-03f, -1.156245927e-03f, -3.811109046e-03f,
    -4.240639613e-03f, -1.593192467e-03f, 2.858409288e-03f, 6.191452975e-03f,
    5.662779658e-03f, 8.082208384e-04f, -5.710383803e-03f, -9.413534167e-03f,
    -6.945690158e-03f, 1.196386233e-03f, 1.021293493e-02f, 1.362827311e-02f,
    7.697098706e-03f, -5.264535961e-03f, -1.726214485e-02f, -1.920307008e-02f,
    -7.254151722e-03f, 1.317432738e-02f, 2.911758124e-02f, 2.744080765e-02f,
    4.085353855e-03f, -3.059095690e-02f, -5.459851135e-02f, -4.495156692e-02f,
    9.043274377e-03f, 9.734089707e-02f, 1.910305042e-01f, 2.545334483e-01f,
    2.622578678e-01f, 2.110313228e-01f, 1.216597224e-01f, 2.877179758e-02f,
    -3.559947218e-02f, -5.606382902e-02f, -3.857870672e-02f, -4.241892100e-03f,
    2.348702480e-02f, 3.072820061e-02f, 1.816444258e-02f, -2.524350181e-03f,
    -1.747484194e-02f, -1.897592755e-02f, -8.739616072e-03f, 4.878309904e-03f,
    1.304799999e-02f, 1.181952704e-02f, 3.624437512e-03f, -5.318377187e-03f,
    -9.435807326e-03f, -7.075901711e-03f, -8.284666761e-04f, 4.802068104e-03f,
    6.491682840e-03f, 3.924460952e-03f, -5.530973588e-04f, -3.855906663e-03f,
    -4.191789279e-03f, -1.922814251e-03f, 1.064118853e-03f, 2.810398050e-03f,
    2.504416608e-03f, 7.528745903e-04f, -1.079471316e-03f, -1.865473774e-03f,
    -1.358397764e-03f, -1.539633985e-04f, 8.612159134e-04f, 1.119911556e-03f,
    6.490547677e-04f, -8.737970318e-05f, -5.796167215e-04f, -5.969514983e-04f,
    -2.581610400e-04f, 1.339513587e-04f, 3.295584537e-04f, 2.715077913e-04f,
    7.477530689e-05f, -9.713788811e-05f, -1.500118065e-04f, -9.592656802e-05f,
    -9.333971878e-06f, 4.348780400e-05f, 4.517357264e-05f, 1.968257613e-05f,
    -1.813025753e-06f, -6.993483109e-06f, -2.980071999e-06f, -1.660779086e-07f,
},
{
    /* phase 2 */
    -1.660779086e-07f, -2.980071999e-06f, -6.993483109e-06f, -1.813025753e-06f,
    1.968257613e-05f, 4.517357264e-05f, 4.348780400e-05f, -9.333971878e-06f,
    -9.592656802e-05f, -1.500118065e-04f, -9.713788811e-05f, 7.477530689e-05f,
    2.715077913e-04f, 3.295584537e-04f, 1.339513587e-04f, -2.581610400e-04f,
    -5.969514983e-04f, -5.796167215e-04f, -8.737970318e-05f, 6.490547677e-04f,
    1.119911556e-03f, 8.612159134e-04f, -1.539633985e-04f, -1.358397764e-03f,
    -1.865473774e-03f, -1.079471316e-03f, 7.528745903e-04f, 2.504416608e-03f,
    2.810398050e-03f, 1.064118853e-03f, -1.922814251e-03f, -4.191789279e-03f,
    -3.855906663e-03f, -5.530973588e-04f, 3.924460952e-03f, 6.491682840e-03f,
    4.802068104e-03f, -8.284666761e-04f, -7.075901711e-03f, -9.435807326e-03f,
    -5.318377187e-03f, 3.624437512e-03f, 1.181952704e-02f, 1.304799999e-02f,
    4.878309904e-03f, -8.739616072e-03f, -1.897592755e-02f, -1.747484194e-02f,
    -2.524350181e-03f, 1.816444258e-02f, 3.072820061e-02f, 2.348702480e-02f,
    -4.241892100e-03f, -3.857870672e-02f, -5.606382902e-02f, -3.559947218e-02f,
    2.877179758e-02f, 1.216597224e-01f, 2.110313228e-01f, 2.622578678e-01f,
    2.545334483e-01f, 1.910305042e-01f, 9.734089707e-02f, 9.043274377e-03f,
    -4.495156692e-02f, -5.459851135e-02f, -3.059095690e-02f, 4.085353855e-03f,
    2.744080765e-02f, 2.911758124e-02f, 1.317432738e-02f, -7.254151722e-03f,
    -1.920307008e-02f, -1.726214485e-02f, -5.264535961e-03f, 7.697098706e-03f,
    1.362827311e-02f, 1.021293493e-02f, 1.196386233e-03f, -6.945690158e-03f,
    -9.413534167e-03f, -5.710383803e-03f, 8.082208384e-04f, 5.662779658e-03f,
    6.191452975e-03f, 2.858409288e-03f, -1.593192467e-03f, -4.240639613e-03f,
    -3.811109046e-03f, -1.156245927e-03f, 1.674299314e-03f, 2.924369566e-03f,
    2.153964417e-03f, 2.471564906e-04f, -1.400938203e-03f, -1.847978290e-03f,
    -1.087703462e-03f, 1.489142191e-04f, 1.006079763e-03f, 1.057252392e-03f,
    4.675243178e-04f, -2.486819775e-04f, -6.291603712e-04f, -5.350603327e-04f,
    -1.528423985e-04f, 2.071912714e-04f, 3.365164325e-04f, 2.286674379e-04f,
    2.397642027e-05f, -1.227441761e-04f, -1.441161630e-04f, -7.411158285e-05f,
    8.652212259e-06f, 4.824496707e-05f, 3.997574821e-05f, 1.303256191e-05f,
    -4.593557782e-06f, -6.361449877e-06f, -1.923261847e-06f, -2.554809599e-08f,
},
{
    /* phase 3 */
    -5.023455431e-07f, -4.168009843e-06f, -7.038615552e-06f, 2.094219199e-06f,
    2.669385367e-05f, 4.864500419e-05f, 3.531453756e-05f, -2.970538246e-05f,
    -1.157735450e-04f, -1.489009898e-04f, -6.351451720e-05f, 1.275244062e-04f,
    3.054517631e-04f, 3.058347823e-04f, 4.737238901e-05f, -3.604809845e-04f,
    -6.356407082e-04f, -4.985175818e-04f, 9.073501585e-05f, 8.138196519e-04f,
    1.134678810e-03f, 6.658599682e-04f, -4.704795200e-04f, -1.584057895e-03f,
    -1.797676994e-03f, -6.878150899e-04f, 1.254972446e-03f, 2.760596488e-03f,
    2.560571731e-03f, 3.701040762e-04f, -2.644353206e-03f, -4.401650078e-03f,
    -3.274152790e-03f, 5.675958453e-04f, 4.867467725e-03f, 6.511750747e-03f,
    3.678794056e-03f, -2.510421177e-03f, -8.188595697e-03f, -9.030659752e-03f,
    -3.368192834e-03f, 6.009795901e-03f, 1.297095558e-02f, 1.184627188e-02f,
    1.692384217e-03f, -1.200169618e-02f, -1.992133995e-02f, -1.485572071e-02f,
    2.597799630e-03f, 2.263652821e-02f, 3.104368171e-02f, 1.817561525e-02f,
    -1.303389039e-02f, -4.550238496e-02f, -5.507882147e-02f, -2.339129215e-02f,
    5.037048049e-02f, 1.457731651e-01f, 2.285762065e-01f, 2.661725457e-01f,
    2.432057709e-01f, 1.690913211e-01f, 7.339480894e-02f, -8.441918558e-03f,
    -5.142342106e-02f, -5.096643955e-02f, -2.194324403e-02f, 1.164265242e-02f,
    2.996356232e-02f, 2.635872143e-02f, 7.906139454e-03f, -1.141299733e-02f,
    -2.001404094e-02f, -1.489252757e-02f, -1.735912350e-03f, 1.004573047e-02f,
    1.359216824e-02f, 8.242228285e-03f, -1.167501993e-03f, -8.195137404e-03f,
    -8.985162006e-03f, -4.163305937e-03f, 2.330826887e-03f, 6.236314034e-03f,
    5.637867228e-03f, 1.721798711e-03f, -2.511481456e-03f, -4.421689274e-03f,
    -3.285113929e-03f, -3.804902075e-04f, 2.178536883e-03f, 2.905016359e-03f,
    1.729901949e-03f, -2.398219103e-04f, -1.642276898e-03f, -1.751138692e-03f,
    -7.866833146e-04f, 4.256912039e-04f, 1.097391389e-03f, 9.527252062e-04f,
    2.784459909e-04f, -3.872259492e-04f, -6.473114721e-04f, -4.545458974e-04f,
    -4.950188372e-05f, 2.649130918e-04f, 3.278819889e-04f, 1.797295117e-04f,
    -2.270712728e-05f, -1.399712912e-04f, -1.322862573e-04f, -5.163992772e-05f,
    2.365648799e-05f, 4.983366585e-05f, 3.362889998e-05f, 7.088871390e-06f,
    -6.295617728e-06f, -5.352920494e-06f, -1.084814782e-06f, 7.711798034e-21f,
}
};

/* Doubled ring buffer: every sample is written at [idx] and [idx + TAPS], so
 * the most recent TAPS samples are always contiguous, oldest first, starting
 * at s_hist[s_hist_idx] (s_hist_idx = next write position = oldest sample). */
static float s_hist[2U * FT8_RESAMPLER_TAPS_PER_PHASE];
static uint32_t s_hist_idx;
static uint32_t s_next_output_vpos; /* virtual position (units of 1/48000 s) of the next output sample */
static uint32_t s_input_count;      /* real 12 kHz samples fed since reset */
static uint32_t s_raw_sample_count; /* see ft8_decimator_get_raw_sample_count() */

/*
 * Own, independent AGC for the FT8 pipeline - same design and constants as
 * the GD32 version (instant attack, slow exponential release, peak-based gain,
 * fixed target). The tap point in sdr.c sits BEFORE the receiver's own AGC
 * and filters, so the level reaching here depends on RF gain and band noise;
 * this normalizes it so the waterfall's fixed dB offset stays meaningful.
 *
 * All three constants are unchanged from the GD32. Host test (test/host,
 * 20 files of the ft8_lib 20m_busy corpus) with the input scaled over four
 * decades (x0.0003 .. x3) gives 162-174 decodes at every level, so the float
 * input of the P4 needs no retuning of the AGC floor.
 */
#define FT8_AGC_TARGET   8000.0f   /* peak target, comfortably under int16 full scale */
#define FT8_AGC_PEAK_MIN 50.0f     /* floor for the peak tracker - avoids a huge gain spike in near-silence */
#define FT8_AGC_RELEASE  0.99995f  /* slow release at 12 kHz - doesn't pump within one ~12.6 s FT8 burst */
static float s_agc_peak;

static int16_t s_out_block[FT8_ADAPTER_SUBBLOCK_SIZE];
static uint16_t s_out_block_fill;

void ft8_decimator_reset(void)
{
    uint32_t i;

    for (i = 0; i < 2U * FT8_RESAMPLER_TAPS_PER_PHASE; i++)
    {
        s_hist[i] = 0.0f;
    }
    s_hist_idx = 0U;
    s_next_output_vpos = 0U;
    s_input_count = 0U;
    s_raw_sample_count = 0U;
    s_agc_peak = FT8_AGC_TARGET;
    s_out_block_fill = 0U;
}

static void produce_output_sample(uint32_t phase)
{
    const float *h = s_resampler_phase[phase];
    /* newest sample = s_hist[s_hist_idx + TAPS - 1], pairs with tap j = 0 */
    const float *x = &s_hist[s_hist_idx + FT8_RESAMPLER_TAPS_PER_PHASE - 1U];
    float acc = 0.0f;
    uint32_t j;

    for (j = 0; j < FT8_RESAMPLER_TAPS_PER_PHASE; j++)
    {
        acc += h[j] * x[-(int32_t)j];
    }

    if (acc > 32767.0f)
    {
        acc = 32767.0f;
    }
    if (acc < -32768.0f)
    {
        acc = -32768.0f;
    }

    s_out_block[s_out_block_fill++] = (int16_t)acc;

    if (s_out_block_fill >= FT8_ADAPTER_SUBBLOCK_SIZE)
    {
        s_out_block_fill = 0U;
        ft8_waterfall_feed_subblock(s_out_block);
    }
}

static void feed_one(float sample_f)
{
    float mag;
    uint32_t base_n;

    s_raw_sample_count++;

    /* Own AGC - instant attack, exponential release */
    mag = (sample_f < 0.0f) ? -sample_f : sample_f;
    s_agc_peak *= FT8_AGC_RELEASE;
    if (mag > s_agc_peak)
    {
        s_agc_peak = mag;
    }
    {
        float pk = (s_agc_peak < FT8_AGC_PEAK_MIN) ? FT8_AGC_PEAK_MIN : s_agc_peak;
        sample_f *= FT8_AGC_TARGET / pk;
    }
    if (sample_f > 32767.0f)
    {
        sample_f = 32767.0f;
    }
    if (sample_f < -32768.0f)
    {
        sample_f = -32768.0f;
    }

    /* Push into the doubled ring */
    s_hist[s_hist_idx] = sample_f;
    s_hist[s_hist_idx + FT8_RESAMPLER_TAPS_PER_PHASE] = sample_f;
    s_hist_idx++;
    if (s_hist_idx >= FT8_RESAMPLER_TAPS_PER_PHASE)
    {
        s_hist_idx = 0U;
    }

    s_input_count++;
    base_n = s_input_count - 1U; /* index of the sample just added */

    /*
     * Polyphase bookkeeping (unchanged from the GD32 version): output k sits at
     * virtual position P = 15k (units of 1/48000 s). The newest real input
     * sample occupies virtual position 4*base_n and becomes usable as output
     * k's j=0 tap once base_n == floor(P/4).
     */
    while ((s_next_output_vpos / (uint32_t)FT8_RESAMPLER_L) == base_n)
    {
        produce_output_sample(s_next_output_vpos % (uint32_t)FT8_RESAMPLER_L);
        s_next_output_vpos += (uint32_t)FT8_RESAMPLER_M;
    }
}

void ft8_decimator_feed(const float *samples, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++)
    {
        feed_one(samples[i]);
    }
}

uint32_t ft8_decimator_get_raw_sample_count(void)
{
    return s_raw_sample_count;
}
