#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_system.h"

#include "pins_config.h"

#include "msi001.h"
#include "sdr.h"

#define MIRISDR_DEBUG 0
#define DEFAULT_RATE 2000000
#define DEFAULT_FREQ 7238000
#define DEFAULT_GAIN 43

uint32_t freq = DEFAULT_FREQ;
int XtalFreq_ppm = -50;

#define clamp(val, lo, hi) min((typeof(val))max(val, lo), hi)

typedef enum
{
    MIRISDR_HW_DEFAULT,
    MIRISDR_HW_SDRPLAY,
} mirisdr_hw_flavour_t;

typedef enum
{
    MIRISDR_BAND_AM1,
    MIRISDR_BAND_AM2,
    MIRISDR_BAND_VHF,
    MIRISDR_BAND_3,
    MIRISDR_BAND_45,
    MIRISDR_BAND_L,
} mirisdr_band_t;

typedef enum
{
    MIRISDR_FORMAT_AUTO_ON = 0,
    MIRISDR_FORMAT_AUTO_OFF
} format_auto_enum;

typedef enum
{
    MIRISDR_FORMAT_252_S16 = 0,
    MIRISDR_FORMAT_336_S16,
    MIRISDR_FORMAT_384_S16,
    MIRISDR_FORMAT_504_S16,
    MIRISDR_FORMAT_504_S8
} format_enum;

typedef enum
{
    MIRISDR_BW_200KHZ = 0,
    MIRISDR_BW_300KHZ,
    MIRISDR_BW_600KHZ,
    MIRISDR_BW_1536KHZ,
    MIRISDR_BW_5MHZ,
    MIRISDR_BW_6MHZ,
    MIRISDR_BW_7MHZ,
    MIRISDR_BW_8MHZ
} bandwidth_enum;

typedef enum
{
    MIRISDR_IF_ZERO = 0,
    MIRISDR_IF_450KHZ,
    MIRISDR_IF_1620KHZ,
    MIRISDR_IF_2048KHZ
} if_freq_enum;

typedef enum
{
    MIRISDR_XTAL_19_2M = 0,
    MIRISDR_XTAL_22M,
    MIRISDR_XTAL_24M,
    MIRISDR_XTAL_24_576M,
    MIRISDR_XTAL_26M,
    MIRISDR_XTAL_38_4M
} xtal_enum;

typedef struct
{
    /* parametry */
    uint32_t index;
    uint32_t freq;
    uint32_t rate;
    int gain;
    int gain_reduction_lna;
    int gain_reduction_mixbuffer;
    int gain_reduction_mixer;
    int gain_reduction_baseband;
    mirisdr_hw_flavour_t hw_flavour;
    mirisdr_band_t band;
    format_auto_enum format_auto;
    format_enum format;
    bandwidth_enum bandwidth;
    if_freq_enum if_freq;
    xtal_enum xtal;
} mirisdr_dev_t;

typedef struct
{
    uint32_t low_cut;
    int mode;
    int upconvert_mixer_on;
    int am_port;
    int lo_div;
    uint32_t band_select_word;
} hw_switch_freq_plan_t;

hw_switch_freq_plan_t hw_switch_freq_plan_default[] = {
    {0, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf780},
    {12, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xff80},
    {30, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf280},
    {50, MIRISDR_MODE_VHF, 0, 0, 32, 0xf380},
    {108, MIRISDR_MODE_B3, 0, 0, 16, 0xfa80},
    {250, MIRISDR_MODE_B3, 0, 0, 16, 0xf680},
    {259, 6, 0, 0, 8, 0xf680},
    {330, MIRISDR_MODE_B45, 0, 0, 4, 0xf380},
    {960, MIRISDR_MODE_BL, 0, 0, 2, 0xfa80},
    {2400, -1, 0, 0, 0, 0x0000},
};

hw_switch_freq_plan_t hw_switch_freq_plan_sdrplay[] = {
    {0, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf780},
    {12, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xff80},
    {30, MIRISDR_MODE_AM, MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf280},
    {50, MIRISDR_MODE_VHF, 0, 0, 32, 0xf380},
    {120, MIRISDR_MODE_B3, 0, 0, 16, 0xfa80},
    {250, MIRISDR_MODE_B3, 0, 0, 16, 0xf680},
    {259, 6, 0, 0, 8, 0xf680},
    {404, MIRISDR_MODE_B45, 0, 0, 4, 0xf380},
    {1000, MIRISDR_MODE_BL, 0, 0, 2, 0xfa80},
    {2400, -1, 0, 0, 0, 0x0000},
};

hw_switch_freq_plan_t *hw_switch_freq_plan[2] = {
    hw_switch_freq_plan_default,
    hw_switch_freq_plan_sdrplay};

mirisdr_dev_t p;

void spi_transfer(uint8_t data)
{
    // function to actually bit shift the data byte out

    for (int i = 1; i <= 8; i++)
    { // setup a loop of 8 iterations, one for each bit
        if (data > 127)
        {                                         // test the most significant bit
            gpio_set_level(msi001_dat_pin, HIGH); // if it is a 1 (ie. B1XXXXXXX), set the master out pin high
        }
        else
        {
            gpio_set_level(msi001_dat_pin, LOW); // if it is not 1 (ie. B0XXXXXXX), set the master out pin low
        }

        gpio_set_level(msi001_clk_pin, HIGH); // set clock high, the pot IC will read the bit into its register
        data = data << 1;
        gpio_set_level(msi001_clk_pin, LOW); // set clock low, the pot IC will stop reading and prepare for the next iteration (next significant bit
    }
}

void mirisdr_write_reg(uint32_t val)
{

    uint8_t a = (val & 0xFF);         // extract first byte
    uint8_t b = ((val >> 8) & 0xFF);  // extract second byte
    uint8_t c = ((val >> 16) & 0xFF); // extract third byte

    // use it as you would the regular arduino SPI API

    gpio_set_level(msi001_cs_pin, LOW); // pull SS slow to prep other end for transfer

    spi_transfer(c);
    spi_transfer(b);
    spi_transfer(a);

    gpio_set_level(msi001_cs_pin, HIGH); // pull ss high to signify end of data transfer;
}

int mirisdr_set_soft(int XtalFreq_ppm)
{
    uint32_t reg0 = 0, reg2 = 2, reg3 = 3, reg5 = 5;
    uint64_t n, thresh, frac, lo_div = 0, fvco = 0, offset = 0, a, b, c;
    uint64_t rfvco = 0, afc = 0;

    int i;

    /*** registr0 - parameters zone ***/

    /* zone */

    i = 0;

    while (p.freq >= 1000000 * hw_switch_freq_plan[(int)p.hw_flavour][i].low_cut)
    {
        if (hw_switch_freq_plan[(int)p.hw_flavour][i].mode < 0)
        {
            break;
        }

        i++;
    }

    hw_switch_freq_plan_t switch_plan = hw_switch_freq_plan[(int)p.hw_flavour][i - 1];

    //  uint32_t low_cut;
    //  int mode;
    //  int upconvert_mixer_on;
    //  int am_port;
    //  int lo_div;
    //  uint32_t band_select_word;

#if MIRISDR_DEBUG >= 1
    Serial.printf("mirisdr_set_soft: i:%d flavour:%d flow:%u mode:%d up:%d port:%d lo:%d band_select_word:%d\n",
                  i - 1,
                  (int)p.hw_flavour,
                  switch_plan.low_cut,
                  switch_plan.mode,
                  switch_plan.upconvert_mixer_on,
                  switch_plan.am_port,
                  switch_plan.lo_div,
                  switch_plan.band_select_word);
#endif

    if (switch_plan.mode == MIRISDR_MODE_AM)
    {
        reg0 |= MIRISDR_MODE_AM << 4;
        reg0 |= switch_plan.upconvert_mixer_on << 9;
        reg0 |= switch_plan.am_port << 11;

        if (switch_plan.upconvert_mixer_on)
        {
            offset += (110000000UL + XtalFreq_ppm * 5);
        }

        lo_div = 16;

        if (switch_plan.am_port == 0)
        {
            p.band = MIRISDR_BAND_AM1;
        }
        else
        {
            p.band = MIRISDR_BAND_AM2;
        }
    }
    else
    {
        reg0 |= switch_plan.mode << 4;
        lo_div = switch_plan.lo_div;

        if (switch_plan.mode == MIRISDR_MODE_VHF)
        {
            p.band = MIRISDR_BAND_VHF;
        }
        else if (switch_plan.mode == MIRISDR_MODE_B3)
        {
            p.band = MIRISDR_BAND_3;
        }
        else if (switch_plan.mode == MIRISDR_MODE_B45)
        {
            p.band = MIRISDR_BAND_45;
        }
        else if (switch_plan.mode == MIRISDR_MODE_BL)
        {
            p.band = MIRISDR_BAND_L;
        }
    }

    /* RF synthesizer is always active */
    reg0 |= MIRISDR_RF_SYNTHESIZER_ON << 10;

    /* IF filter mode - has not worked? */
    switch (p.if_freq)
    {
    case MIRISDR_IF_ZERO:
        reg0 |= MIRISDR_IF_MODE_ZERO << 12;
        break;
    case MIRISDR_IF_450KHZ:
        reg0 |= MIRISDR_IF_MODE_450KHZ << 12;
        break;
    case MIRISDR_IF_1620KHZ:
        reg0 |= MIRISDR_IF_MODE_1620KHZ << 12;
        break;
    case MIRISDR_IF_2048KHZ:
        reg0 |= MIRISDR_IF_MODE_2048KHZ << 12;
        break;
    }

    /* Bandwidth - 8 MHz, the highest possible */
    switch (p.bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        reg0 |= 0x00 << 14;
        break;
    case MIRISDR_BW_300KHZ:
        reg0 |= 0x01 << 14;
        break;
    case MIRISDR_BW_600KHZ:
        reg0 |= 0x02 << 14;
        break;
    case MIRISDR_BW_1536KHZ:
        reg0 |= 0x03 << 14;
        break;
    case MIRISDR_BW_5MHZ:
        reg0 |= 0x04 << 14;
        break;
    case MIRISDR_BW_6MHZ:
        reg0 |= 0x05 << 14;
        break;
    case MIRISDR_BW_7MHZ:
        reg0 |= 0x06 << 14;
        break;
    case MIRISDR_BW_8MHZ:
        reg0 |= 0x07 << 14;
        break;
    }

    //  /* xtal frequency - we do not support change */
    //  switch (p.xtal)
    //  {
    //    case MIRISDR_XTAL_19_2M:
    //      reg0 |= 0x00 << 17;
    //      break;
    //    case MIRISDR_XTAL_22M:
    //      reg0 |= 0x01 << 17;
    //      break;
    //    case MIRISDR_XTAL_24M:
    //    case MIRISDR_XTAL_24_576M:
    //      reg0 |= 0x02 << 17;
    //      break;
    //    case MIRISDR_XTAL_26M:
    //      reg0 |= 0x03 << 17;
    //      break;
    //    case MIRISDR_XTAL_38_4M:
    //      reg0 |= 0x04 << 17;
    //      break;
    //  }

    uint64_t XtalFreq = 22000000;

    reg0 |= 0x02 << 17; // problema con el XTAL no sé por qué

    /* 4 bits for power saving modes */
    reg0 |= MIRISDR_IF_LPMODE_NORMAL << 20;
    reg0 |= MIRISDR_VCO_LPMODE_NORMAL << 23;

    /* VCO frequency is better to use a 64-bit range */
    fvco = (p.freq + offset) * lo_div;

    uint64_t XtalFreq4 = 4 * (22000000UL + XtalFreq_ppm); // Ajuste en ppm del cristal del msi001

    /* shift the main frequency */
    n = fvco / XtalFreq4;

    /* major registry, coarse tuning */
    thresh = XtalFreq4 / lo_div;

    /* side register, fine tuning */
    frac = (fvco % XtalFreq4) / lo_div;

    /* We find the greatest common divisor for thresh and frac */
    for (a = thresh, b = frac; a != 0;)
    {
        c = a;
        a = b % a;
        b = c;
    }

    /* divided */
    thresh /= b;
    frac /= b;

    /* In this section we reduce the resolution to the maximum extent registry */
    a = (thresh + 4094) / 4095;
    thresh = (thresh + (a / 2)) / a;
    frac = (frac + (a / 2)) / a;

    rfvco = (XtalFreq4 * (n * thresh * 4096UL + (frac * 4096UL))) / (thresh * 4096UL * lo_div);
    if (p.freq + offset < rfvco)
        frac--;
    rfvco = (XtalFreq4 * (n * thresh * 4096UL + (frac * 4096UL + afc))) / (thresh * 4096UL * lo_div);
    afc = ((p.freq + offset - rfvco) * thresh * 4096UL * lo_div) / XtalFreq4;

    reg3 |= (afc & 4095) << 4;
    reg5 |= (0xFFF & thresh) << 4;

    /* Reserved, must be 0x28 */
    reg5 |= MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING << 16;

    reg2 |= (0xFFF & frac) << 4;
    reg2 |= (0x3F & n) << 16;
    reg2 |= MIRISDR_LBAND_LNA_CALIBRATION_OFF << 22;

    // mirisdr_write_reg(switch_plan.band_select_word);

    mirisdr_write_reg(0x0e);
    mirisdr_write_reg(reg3);

    mirisdr_write_reg(reg0);
    mirisdr_write_reg(reg5);
    mirisdr_write_reg(reg2);

    return 0;
}

int mirisdr_set_center_freq(uint32_t freq)
{
    p.freq = freq;
    mirisdr_set_soft(XtalFreq_ppm);
    mirisdr_set_gain(); // restore gain
    return 0;
}

int mirisdr_set_if_freq(uint32_t freq)
{
    switch (freq)
    {
    case 0:
        p.if_freq = MIRISDR_IF_ZERO;
        break;
    case 450000:
        p.if_freq = MIRISDR_IF_450KHZ;
        break;
    case 1620000:
        p.if_freq = MIRISDR_IF_1620KHZ;
        break;
    case 2048000:
        p.if_freq = MIRISDR_IF_2048KHZ;
        break;
    default:
        break;
    }

    mirisdr_set_soft(XtalFreq_ppm);
    mirisdr_set_gain(); // restore gain
    return 0;
}

/* not supported yet */
int mirisdr_set_xtal_freq(uint32_t freq)
{
    (void)p;
    (void)freq;
    return -1;
}

uint32_t mirisdr_set_bandwidth(uint32_t bw)
{
    switch (bw)
    {
    case 200000:
        p.bandwidth = MIRISDR_BW_200KHZ;
        break;
    case 300000:
        p.bandwidth = MIRISDR_BW_300KHZ;
        break;
    case 600000:
        p.bandwidth = MIRISDR_BW_600KHZ;
        break;
    case 1536000:
        p.bandwidth = MIRISDR_BW_1536KHZ;
        break;
    case 5000000:
        p.bandwidth = MIRISDR_BW_5MHZ;
        break;
    case 6000000:
        p.bandwidth = MIRISDR_BW_6MHZ;
        break;
    case 7000000:
        p.bandwidth = MIRISDR_BW_7MHZ;
        break;
    case 8000000:
        p.bandwidth = MIRISDR_BW_8MHZ;
        break;
    default:
        break;
    }

    mirisdr_set_soft(XtalFreq_ppm);
    mirisdr_set_gain(); // restore gain
    return 0;
}

int mirisdr_set_offset_tuning(int on)
{

    if (on)
    {
        p.if_freq = MIRISDR_IF_450KHZ;
    }
    else
    {
        p.if_freq = MIRISDR_IF_ZERO;
    }

    return mirisdr_set_soft(XtalFreq_ppm);
}

int mirisdr_set_gain()
{
    uint32_t reg1 = 1, reg6 = 6;

    // Serial.println(buffer);

    //  unsigned int reg;
    //
    //  reg = 1 << 0;
    //  reg |= (59 - p.gain_reduction_ba	gpio_set_level(NAU8822_CS_PIN, HIGH); // desactiva CS NAU8822seband) << 4;
    //  reg |= 0 << 10;
    //  reg |= (1 - p.gain_reduction_mixer) << 12;
    //  reg |= (1 - p.gain_reduction_lna) << 13;
    //  reg |= 4 << 14;
    //  reg |= 0 << 17;
    //  mirisdr_write_reg( reg);
    //  reg = 6 << 0;
    //  reg |= 63 << 4;
    //  reg |= 4095 << 10;
    //  mirisdr_write_reg( reg);
    //  return 0;

    /* Receiver Gain Control */
    /* 0-3 => registr */
    /* 4-9 => baseband, 0 - 59, 60-63 je stejné jako 59 */
    /* 10-11 => mixer gain reduction pouze pro AM režim */
    /* 12 => mixer gain reduction -19dB */
    /* 13 => lna gain reduction -24dB */
    /* 14-16 => DC kalibrace */
    /* 17 => zrychlená DC kalibrace */
    reg1 |= p.gain_reduction_baseband << 4;

    // Mixbuffer is on AM1 and AM2 inputs only
    if (p.band == MIRISDR_BAND_AM1)
    {
        reg1 |= (p.gain_reduction_mixbuffer & 0x03) << 10;
    }
    else if (p.band == MIRISDR_BAND_AM2)
    {
        reg1 |= (p.gain_reduction_mixbuffer == 0 ? 0x0 : 0x03) << 10;
    }
    else
    {
        reg1 |= 0x0 << 10;
    }

    reg1 |= p.gain_reduction_mixer << 12;

    // LNA is not on AM1 nor AM2 inputs
    if ((p.band == MIRISDR_BAND_AM1) || (p.band == MIRISDR_BAND_AM2))
    {
        reg1 |= 0x0 << 13;
    }
    else
    {
        reg1 |= p.gain_reduction_lna << 13;
    }

    reg1 |= MIRISDR_DC_OFFSET_CALIBRATION_PERIODIC2 << 14;
    reg1 |= MIRISDR_DC_OFFSET_CALIBRATION_SPEEDUP_OFF << 17;

    mirisdr_write_reg(reg1);

    /* DC Offset Calibration setup */
    reg6 |= 0x1F << 4;
    reg6 |= 0x800 << 10;
    mirisdr_write_reg(reg6);

    return 0;
}

int mirisdr_set_tuner_gain(int gain)
{
    p.gain = gain;
    /*
     For VHF mode LNA is turned on to + 24 db, mixer to + 19 dB and baseband
     can be adjusted continuously from 0 to 59 db, of which the maximum gain of 102 db
  */
    if (p.gain > 102)
    {
        p.gain = 102;
    }
    else if (p.gain < 0)
    {
        p.gain = 0;
        return 0;	gpio_set_level(NAU8822_CS_PIN, HIGH); // desactiva CS NAU8822
    }

    /* Always the highest sensitivity without reducing the mixer and LNA */
    if (p.gain >= 43)
    {
        p.gain_reduction_lna = 0;
        p.gain_reduction_mixbuffer = 0; // LNA equivalent for AM inputs
        p.gain_reduction_mixer = 0;
        p.gain_reduction_baseband = 59 - (p.gain - 43);
    }
    else if (p.gain >= 19)
    {
        p.gain_reduction_lna = 1;
        p.gain_reduction_mixbuffer = 3; // LNA equivalent for AM inputs (AM1: 18dB / AM2: 24 dB)
        p.gain_reduction_mixer = 0;
        p.gain_reduction_baseband = 59 - (p.gain - 19);
    }
    else
    {
        p.gain_reduction_lna = 1;
        p.gain_reduction_mixbuffer = 3; // LNA equivalent for AM inputs (AM1: 18dB / AM2: 24 dB)
        p.gain_reduction_mixer = 1;
        p.gain_reduction_baseband = 59 - p.gain;
    }

    return mirisdr_set_gain();
}

int mirisdr_set_tuner_gain_mode(int mode)
{
    return 0;
}

/*
   Gain reduction is an index that depends on the AM mode (only applies to AM inputs)
            AM1     AM2
   0x00    0 dB    0 dB
   0x01    6 dB   24 dB
   0x10   12 dB   24 dB
   0x11   18 dB   24 dB
*/
int mirisdr_set_mixer_gain(int gain)
{
    p.gain_reduction_mixer = gain;

    return mirisdr_set_gain();
}

int mirisdr_set_mixbuffer_gain(int gain)
{
    p.gain_reduction_mixbuffer = gain;

    return mirisdr_set_gain();
}

int mirisdr_set_lna_gain(int gain)
{
    p.gain_reduction_lna = gain;

    return mirisdr_set_gain();
}

int mirisdr_set_baseband_gain(int gain)
{
    p.gain_reduction_baseband = gain;

    return mirisdr_set_gain();
}

///////////////
int mirisdr_get_lna_gain()
{
    return p.gain_reduction_lna;
}
int mirisdr_get_mixer_gain()
{
    return p.gain_reduction_mixer;
}
int mirisdr_get_mixbuffer_gain()
{
    return p.gain_reduction_mixbuffer;
}
int mirisdr_get_baseband_gain()
{
    return p.gain_reduction_baseband;
}
int mirisdr_get_tuner_gain()
{
    return p.gain;
}
void init_msi001()
{
    p.freq = DEFAULT_FREQ;
    p.rate = DEFAULT_RATE;
    p.gain = DEFAULT_GAIN;
    p.band = MIRISDR_BAND_AM2;

    p.gain_reduction_lna = 0;
    p.gain_reduction_mixer = 0;
    p.gain_reduction_baseband = 43;
    p.if_freq = MIRISDR_IF_ZERO;
    p.format_auto = MIRISDR_FORMAT_AUTO_ON;
    p.bandwidth = MIRISDR_BW_200KHZ;
    p.xtal = MIRISDR_XTAL_22M;

    mirisdr_set_center_freq(DEFAULT_FREQ);

    mirisdr_hw_flavour_t hw_flavour = MIRISDR_HW_SDRPLAY;

    p.hw_flavour = hw_flavour;
}
