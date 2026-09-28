/*
  Noise reduction by Spectral Subtraction

    File:   nr_ss.h
    Author: JF3HZB / T. Uebo

    Created on June 10, 2025
*/

#include "esp_dsp.h"
#include "math.h"
#include "sdr.h"

#include "nr_ss.h"


#define Nframe 4
#define Nstfft (SAMPLE_BUFFER_SIZE / DR * Nframe) //(Nframe*BLOCK_SAMPLES/DOWN_SAMPLE)
#define ATT_Ratio sqrt(2.667f)                    // for Hann window

// Threshold setting range of NR
#define Noise_min 0.0f
#define Noise_max 11.0f

extern int V_Rsv;
extern bool nr_ss_debug;

float dat_stfft[2 * Nstfft];
float sigf[Nframe][Nstfft];
float sigt[Nstfft];
float wf_stfft[Nstfft];
int pt_w = 0;
int pt_frm = 0;
float th_nr;
float inv_th;
float th_nr_val;

void NR_SS_init()
{
  dsps_fft2r_init_fc32(NULL, Nstfft);

  // generate window function
  for (int i = 0; i < Nstfft; i++)
  {
    float t = (float)i / (float)Nstfft;
    // wf_stfft[i] = 0.54f - 0.46f*cos(2.0f*M_PI*t); //Hamming
    wf_stfft[i] = 0.5f - 0.5f * cosf(2.0f * M_PI * t); // Hann
    // wf_stfft[i] = 0.35875f - 0.48828f*cos(2.0f*M_PI*t) + 0.14128f*cos(4.0f*M_PI*t) - 0.01168f*cos(6.0f*M_PI*t); //Balckman-Harris
    // wf_stfft[i] = 1 - 1.93f*cos(2.0f*M_PI*t) + 1.29f*cos(4.0f*M_PI*t) - 0.388f*cos(6.0f*M_PI*t) + 0.032f*cos(8.0*M_PI*t); //Flat-top
  }

  th_nr_val = (Noise_max - Noise_min) * V_Rsv / 4096.0f + Noise_min;
  th_nr = pow(10.0f, th_nr_val);
  inv_th = 1.0f / th_nr;
}

void NR_SS(float *demod_out_d, int BUFFER_SIZE)
{

  if (nr_ss_debug)
  {
    //printf("th_nr=%f inv_th=%f th_nr_val=%f\n", th_nr, inv_th, th_nr_val);
    nr_ss_debug = false;
  }

  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    sigt[pt_w] = demod_out_d[i];
    pt_w++;
    if (pt_w == Nstfft)
      pt_w = 0;
  }

  for (int i = 0; i < Nstfft; i++)
  {
    dat_stfft[2 * i] = sigt[pt_w] * wf_stfft[i];
    dat_stfft[2 * i + 1] = 0;
    pt_w++;
    if (pt_w == Nstfft)
      pt_w = 0;
  }

  // fft
  dsps_fft2r_fc32(dat_stfft, Nstfft);
  dsps_bit_rev2r_fc32(dat_stfft, Nstfft);

  //--- Spectral Subtraction -----------------------------------------
  for (int i = 0; i < Nstfft; i++)
  {
    float xabsr = fabs(dat_stfft[2 * i]);
    float xabsi = fabs(dat_stfft[2 * i + 1]);
    float amp;
    if (xabsr > xabsi)
      amp = (0.96043457f) * xabsr + (0.39782422f) * xabsi;
    else
      amp = (0.96043457f) * xabsi + (0.39782422f) * xabsr;
    amp *= (1.0f / (float)Nstfft);

    float g_reduce = amp * inv_th;
    if (g_reduce > 1.0f)
      g_reduce = 1.0f;
    dat_stfft[2 * i] *= g_reduce;
    dat_stfft[2 * i + 1] *= g_reduce;
  }

  // ifft
  for (int i = 0; i < Nstfft; i++)
    dat_stfft[2 * i + 1] = -dat_stfft[2 * i + 1];

  dsps_fft2r_fc32(dat_stfft, Nstfft);
  dsps_bit_rev2r_fc32(dat_stfft, Nstfft);

  for (int i = 0; i < Nstfft; i++)
    dat_stfft[2 * i + 1] = -dat_stfft[2 * i + 1];

  for (int i = 0; i < Nstfft; i++)
    sigf[pt_frm][i] = dat_stfft[2 * i] * (1.0f / (float)Nstfft); //*wf_stfft[i];
  pt_frm++;
  if (pt_frm == Nframe)
    pt_frm = 0;

  // overlap-add
  int i_frm[Nframe];
  for (int i = 0; i < Nframe; i++)
  {
    int k = pt_frm + i;
    if (k >= Nframe)
      k -= Nframe;
    i_frm[i] = k;
  }
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
#if Nframe == 4
    demod_out_d[i] = sigf[i_frm[0]][i + 3 * (Nstfft / Nframe)] +
                     sigf[i_frm[1]][i + 2 * (Nstfft / Nframe)] +
                     sigf[i_frm[2]][i + 1 * (Nstfft / Nframe)] +
                     sigf[i_frm[3]][i + 0 * (Nstfft / Nframe)];
    // demod_out_d[i] *= ATT_Ratio * (1.0f / (float)Nframe);

    demod_out_d[i] *= 500;

#elif Nframe == 2
    demod_out_d[i] = sigf[i_frm[0]][i + 1 * (Nstfft / Nframe)] +
                     sigf[i_frm[1]][i + 0 * (Nstfft / Nframe)];
    demod_out_d[i] *= ATT_Ratio * (1.0f / (float)Nframe);
#endif
  }
}