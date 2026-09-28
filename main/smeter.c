
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stdbool.h"

#include "math.h"
#include "sdr.h"
#include "sdr_math.h"

#include "smeter.h"

extern int demod_modo;
extern VFO currentVFO;

// Variables calculatedbm()

typedef struct
{
    float dbm;
    float dbm_old;
    uint8_t sch;

    float dbmhz;
    float m_AttackAvedbm;
    float m_DecayAvedbm;
    float m_AverageMagdbm;
    float m_AttackAvedbmhz;
    float m_DecayAvedbmhz;
    float m_AverageMagdbmhz;

    float dbm_calibration;
    int8_t RF_attenuation;

    float m_AttackAlpha;
    float m_DecayAlpha;
} ui_calculatedbm_params_t;

ui_calculatedbm_params_t ui_calculatedbm_variables;

void init_smeter(void)
{

    ui_calculatedbm_variables.dbm = -145.0;
    ui_calculatedbm_variables.dbm_old = -145.0;
    ui_calculatedbm_variables.sch = 0;

    ui_calculatedbm_variables.dbmhz = -145.0;
    ui_calculatedbm_variables.m_AttackAvedbm = -73.0;
    ui_calculatedbm_variables.m_DecayAvedbm = -73.0;
    ui_calculatedbm_variables.m_AverageMagdbm = -73.0;
    ui_calculatedbm_variables.m_AttackAvedbmhz = -103.0;
    ui_calculatedbm_variables.m_DecayAvedbmhz = -103.0;
    ui_calculatedbm_variables.m_AverageMagdbmhz = -103.0;

    ui_calculatedbm_variables.dbm_calibration = 3.0; //
    ui_calculatedbm_variables.RF_attenuation = 0;

    ui_calculatedbm_variables.m_AttackAlpha = 0.4; // 0.1; //0.08; //0.2;
    ui_calculatedbm_variables.m_DecayAlpha = 0.05; // 0.02; //0.05;
}

float calculadBm(void)
{
    // calculation of the signal level inside the filter bandwidth
    // taken from the spectrum display FFT
    // taking into account the analog gain before the ADC
    // analog gain is adjusted in steps of 1.5dB
    // bands[current_band].RFgain = 0 --> 0dB gain
    // bands[current_band].RFgain = 15 --> 22.5dB gain

    // spectrum display is generated from 256 samples based on 1024 samples of the FIR FFT . . .
    // could this cause errors in the calculation of the signal strength ?

    //  float slope = 19.8; //
    float slope = 10.0; //
    float cons = -92;   //
    float Lbin, Ubin;
    float bw_LSB = 0.0;
    float bw_USB = 0.0;
    float sum_db = 0.0;                                       // FIXME: mabye this slows down the FPU, because the FPU does only process 32bit floats ???
    int posbin = SAMPLE_BUFFER_SIZE / 4; // Para la traslación de frecuencia
    // bin_bandwidth = samplerate / 256bins
    float bin_bandwidth = (float)(SAMPLE_RATE / (SAMPLE_BUFFER_SIZE)); // width of a 256 tap FFT bin @ 96ksps = 375Hz

    switch (demod_modo)
    {
    case DEMOD_USB:
    case DEMOD_SAMU:
        bw_LSB = (float)-currentVFO.f_alta;
        bw_USB = (float)-currentVFO.f_baja;
        break;
    case DEMOD_LSB:
    case DEMOD_SAML:
        bw_LSB = (float)currentVFO.f_baja;
        bw_USB = (float)currentVFO.f_alta;
        break;
    default:
        bw_LSB = (float)-currentVFO.f_alta;
        bw_USB = (float)currentVFO.f_alta;
        break;
    }

    // calculate upper and lower limit for determination of signal strength
    // = filter passband is between the lower bin Lbin and the upper bin Ubin
    Lbin = (float)posbin + roundf(bw_LSB / bin_bandwidth); // bin on the lower/left side
    Ubin = (float)posbin + roundf(bw_USB / bin_bandwidth); // bin on the upper/right side

    // take care of filter bandwidths that are larger than the displayed FFT bins
    if (Lbin < 0)
    {
        Lbin = 0;
    }
    if (Ubin > SAMPLE_BUFFER_SIZE - 1)
    {
        Ubin = SAMPLE_BUFFER_SIZE - 1;
    }
    // Serial.print("Lbin = "); Serial.println(Lbin);
    // Serial.print("Ubin = "); Serial.println(Ubin);
    if ((int)Lbin == (int)Ubin)
    {
        Ubin = 1.0 + Lbin;
    }
    // determine the sum of all the bin values in the passband
    for (int c = (int)Lbin; c <= (int)Ubin; c++) // sum up all the values of all the bins in the passband
    {
        sum_db = sum_db + fft_mag[c];
    }

    if (sum_db > 0)
    {
        ui_calculatedbm_variables.dbm = ui_calculatedbm_variables.dbm_calibration + (float)ui_calculatedbm_variables.RF_attenuation + slope * log10f_fast(sum_db) + cons;
        ui_calculatedbm_variables.dbmhz = 0;
    }
    else
    {
        ui_calculatedbm_variables.dbm = -140.0;
        ui_calculatedbm_variables.dbmhz = -165.0;
    }

    // lowpass IIR filter
    // Wheatley 2011: two averagers with two time constants
    // IIR filter with one element analog to 1st order RC filter
    // but uses two different time constants (ALPHA = 1 - e^(-T/Tau)) depending on
    // whether the signal is increasing (attack) or decreasing (decay)
    // m_AttackAlpha = 0.8647; //  ALPHA = 1 - e^(-T/Tau), T = 0.02s (because dbm routine is called every 20ms!)
    // Tau = 10ms = 0.01s attack time
    // m_DecayAlpha = 0.0392; // 500ms decay time
    //
    ui_calculatedbm_variables.m_AttackAvedbm = (1.0f - ui_calculatedbm_variables.m_AttackAlpha) * ui_calculatedbm_variables.m_AttackAvedbm + ui_calculatedbm_variables.m_AttackAlpha * ui_calculatedbm_variables.dbm;
    ui_calculatedbm_variables.m_DecayAvedbm = (1.0f - ui_calculatedbm_variables.m_DecayAlpha) * ui_calculatedbm_variables.m_DecayAvedbm + ui_calculatedbm_variables.m_DecayAlpha * ui_calculatedbm_variables.dbm;
    ui_calculatedbm_variables.m_AttackAvedbmhz = (1.0f - ui_calculatedbm_variables.m_AttackAlpha) * ui_calculatedbm_variables.m_AttackAvedbmhz + ui_calculatedbm_variables.m_AttackAlpha * ui_calculatedbm_variables.dbmhz;
    ui_calculatedbm_variables.m_DecayAvedbmhz = (1.0f - ui_calculatedbm_variables.m_DecayAlpha) * ui_calculatedbm_variables.m_DecayAvedbmhz + ui_calculatedbm_variables.m_DecayAlpha * ui_calculatedbm_variables.dbmhz;

    if (ui_calculatedbm_variables.m_AttackAvedbm > ui_calculatedbm_variables.m_DecayAvedbm)
    {                                                                                         // if attack average is larger then it must be an increasing signal
        ui_calculatedbm_variables.m_AverageMagdbm = ui_calculatedbm_variables.m_AttackAvedbm; // use attack average value for output
        ui_calculatedbm_variables.m_DecayAvedbm = ui_calculatedbm_variables.m_AttackAvedbm;   // set decay average to attack average value for next time
    }
    else
    { // signal is decreasing, so use decay average value
        ui_calculatedbm_variables.m_AverageMagdbm = ui_calculatedbm_variables.m_DecayAvedbm;
    }

    if (ui_calculatedbm_variables.m_AttackAvedbmhz > ui_calculatedbm_variables.m_DecayAvedbmhz)
    {                                                                                             // if attack average is larger then it must be an increasing signal
        ui_calculatedbm_variables.m_AverageMagdbmhz = ui_calculatedbm_variables.m_AttackAvedbmhz; // use attack average value for output
        ui_calculatedbm_variables.m_DecayAvedbmhz = ui_calculatedbm_variables.m_AttackAvedbmhz;   // set decay average to attack average value for next time
    }
    else
    { // signal is decreasing, so use decay average value
        ui_calculatedbm_variables.m_AverageMagdbmhz = ui_calculatedbm_variables.m_DecayAvedbmhz;
    }

    ui_calculatedbm_variables.dbm = ui_calculatedbm_variables.m_AverageMagdbm;     // write average into variable for S-meter display
    ui_calculatedbm_variables.dbmhz = ui_calculatedbm_variables.m_AverageMagdbmhz; // write average into variable for S-meter display

    return ui_calculatedbm_variables.dbm;
}
