#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "sdr.h"
#include "ui.h"
#include "ui_priv.h"
#include "agc.h"
#include "audio_out.h"
#include "smeter.h"
#include "smeter_draw.h"
#include "esp_heap_caps.h"
#include "math.h"

#include "rtl_source.h"
#include "esp_timer.h"

/* Uncomment to log how long the FFT, spectrum and waterfall steps take (every 2 s) */
// #define UI_PERF_LOG

#include "menu.h"

#include "ft8_app.h"
#include "ft8_ui.h"
#include "ft8_time.h"
#include "palettes.h"
#include "dmr_app.h"
#include "dmr_ui.h"
#include "ais_app.h"
#include "ais_ui.h"
#include "nvs_flash.h"
#include "nvs.h"

extern VFO currentVFO;

static char *TAG = "UI";

/*
 * Screen layout - reworked per Jorge, DeepSDR-inspired: S-meter/frequency/mode block
 * top-left, a single badge row (was 3 stacked rows), a clock top-right corner
 * (placeholder - see label_clock's comment), and the demod/filter/step controls
 * moved below the waterfall instead of overlapping the top info block. Panel is
 * 1024x600 (WAVEFORM_WIDTH x the literal 600 already used throughout this file for
 * the waveform/waterfall's own vertical placement).
 *
 * UI_SPECTRUM_TOP_Y replaces the previous bare "600 - WATERFALL_HEIGHT -
 * WAVEFORM_HEIGHT" (which put the waterfall flush against the bottom edge, leaving
 * no room below it) - same total spectrum+waterfall height (320 px, WAVEFORM_HEIGHT
 * + WATERFALL_HEIGHT), just shifted up by UI_BUTTON_ROW_H to free a strip at the
 * bottom for the control buttons. Neither WAVEFORM_HEIGHT nor WATERFALL_HEIGHT
 * themselves change, so none of the buffer/DSP sizing code that depends on them
 * elsewhere is affected - this is a pure repositioning.
 */
#define UI_BUTTON_ROW_H 80 /* was 60: taller (70 px) buttons now, see UI_CTRL_BTN_H */
#define UI_CTRL_BTN_W 150
#define UI_CTRL_BTN_H 70
#define UI_CTRL_BTN_GAP 20
#define UI_SPECTRUM_TOP_Y (600 - UI_BUTTON_ROW_H - WATERFALL_HEIGHT - WAVEFORM_HEIGHT)

/* Top-left info block (S-meter is 335x125, at its usual 0,0): frequency and mode
 * text sit to its right, roughly matching the S-meter's own vertical span. */
#define UI_FREQ_X 360
#define UI_FREQ_Y 15
#define UI_MODE_X 360
#define UI_MODE_Y 90
#define UI_BADGE_ROW_Y 140 /* single row, right below the info block */
#define UI_CLOCK_X_MARGIN 10
#define UI_CLOCK_Y 10

#define SMETER_MIN_ANGLE 135 // grados
#define SMETER_MAX_ANGLE 405 // grados
#define SMETER_RANGE 270     // arco total

static float smeter_value = 0.0f; // valor S (0..9)

extern unsigned int time_sdrtask;
int loops = 0;

extern int demod_modo;
extern int f_nrss;
extern agc_wdsp_params_t agc_wdsp_conf;
extern bool screen_update;

extern int pasos[6];
extern int pasos_indice;

extern bool f_actualiza;
extern int filtro_indice;
extern char *filtros_texto[5];
extern char *agc_texto[6];

extern char *demod_modos_texto[8];
extern char *pasos_texto[6];

static bool bloqueo_pulsacion = false;

static bool refresca_smeter = false;
int contador_printf = 0;

extern int32_t s_enc_diff;

static const char *TAG_MEM = "MEM";

/* Imprime estado de memoria LVGL + ESP32 */
void debug_print_mem(const char *tag)
{
  lv_mem_monitor_t m;
  lv_mem_monitor(&m);

  size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_8bit = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  ESP_LOGI(TAG_MEM, "---- MEM DEBUG: %s ----", tag);

  /* LVGL */
  ESP_LOGI(TAG_MEM,
           "LVGL: total=%lu free=%lu frag=%lu%%",
           (unsigned long)m.total_size,
           (unsigned long)m.free_size,
           (unsigned long)m.frag_pct);

  /* ESP32 heap */
  ESP_LOGI(TAG_MEM,
           "HEAP 8BIT: free=%lu largest_block=%lu",
           (unsigned long)free_8bit,
           (unsigned long)largest_8bit);

  ESP_LOGI(TAG_MEM, "-----------------------------");
}

static void desbloquear_cb(lv_timer_t *timer)
{
  bloqueo_pulsacion = false;
  lv_timer_pause(timer_debounce); // 🔥 Esto elimina el timer después de ejecutarse

  ESP_LOGI(TAG, "Debounce...");
}

/* Actual label update, with NO locking of its own: safe to call only from a context
 * that already holds the LVGL lock (an LVGL event callback, like spectrum_drag_cb -
 * that's the whole point) or doesn't need it (see refresca_VFO() below for everyone
 * else). Calling lvgl_port_lock() a second time from a context that already holds it
 * (as an event callback always does, since LVGL's own dispatcher holds the lock while
 * running callbacks) would self-deadlock the LVGL task - freezing the whole screen,
 * spectrum included, since its own redraw timer can then never run again. */
static void update_vfo_label(void)
{
  lv_label_set_text_fmt(freq_label, "%d%d%d.%d%d%d.%d%d%d",
                        (currentVFO.Frec % 1000000000) / 100000000,
                        (currentVFO.Frec % 100000000) / 10000000,
                        (currentVFO.Frec % 10000000) / 1000000,
                        (currentVFO.Frec % 1000000) / 100000,
                        (currentVFO.Frec % 100000) / 10000,
                        (currentVFO.Frec % 10000) / 1000,
                        (currentVFO.Frec % 1000) / 100,
                        (currentVFO.Frec % 100) / 10,
                        (currentVFO.Frec % 10) / 1);
}

/* For callers OUTSIDE the LVGL context (e.g. tarea_encoder, a plain FreeRTOS task):
 * takes the lock itself, since nothing else holds it there. */
void refresca_VFO(void)
{
  if (lvgl_port_lock(0))
  {
    update_vfo_label();
    lvgl_port_unlock();
  }
}

/* Full wipe of the spectrum and waterfall canvases. Needed after the FT8
 * panel has covered them: spectrum() draws incrementally (erases the previous
 * trace from pixelold[], then draws pixelnew[]), and calcula_fft() in sdr.c
 * keeps updating pixelold[] every frame even while nothing is drawn - so on
 * return the erase pass would hit a trace that is not the one on the canvas,
 * leaving stale fragments. The waterfall would also still show rows from
 * before FT8 was entered. After this wipe the first erase pass only paints
 * black over black, and spectrum() redraws the grid lines itself. */
static void spectrum_waterfall_full_clear(void)
{
  memset(waveformbuffer, 0x00, WAVEFORM_WIDTH * WAVEFORM_HEIGHT * sizeof(uint16_t));
  memset(waterfallbuffer, 0x00, 2u * WAVEFORM_WIDTH * WATERFALL_HEIGHT * sizeof(uint16_t));
  waterfall_head = 0;
  lv_canvas_set_buffer(waterfall_canvas, waterfallbuffer, WAVEFORM_WIDTH, WATERFALL_HEIGHT, LV_COLOR_FORMAT_RGB565);
  lv_obj_invalidate(waveform_canvas);
  lv_obj_invalidate(waterfall_canvas);
}

void timer_dibuja_pantalla(lv_timer_t *timer)
{
  static bool covered_by_ft8 = false;

  if (screen_update)
  {
#ifdef UI_PERF_LOG
    static int64_t acc_fft, acc_spec, acc_wf;
    static uint32_t n_frames;
    const int64_t t0 = esp_timer_get_time();
#endif
    calcula_fft();
#ifdef UI_PERF_LOG
    const int64_t t1 = esp_timer_get_time();
#endif
    refresca_smeter = true;

    /* Task lock */
    if (lvgl_port_lock(0))
    {
      /* While the FT8 panel covers the spectrum/waterfall, calcula_fft()
       * above keeps running (the S-meter depends on it) but the hidden
       * redraws are skipped. */
      if (ft8_ui_is_visible() || dmr_ui_is_visible() || ais_ui_is_visible())
      {
        covered_by_ft8 = true; /* FT8, DMR or AIS panel */
        lvgl_port_unlock();
        return;
      }
      if (covered_by_ft8)
      {
        /* First frame after leaving FT8 (any path: FT8 button, mode change
         * from the menu or the MODE button): start from clean canvases. */
        covered_by_ft8 = false;
        spectrum_waterfall_full_clear();
      }
      spectrum();
#ifdef UI_PERF_LOG
      const int64_t t2 = esp_timer_get_time();
#endif
      waterfall_update();
#ifdef UI_PERF_LOG
      const int64_t t3 = esp_timer_get_time();
      acc_fft += t1 - t0;
      acc_spec += t2 - t1;
      acc_wf += t3 - t2;
      if (++n_frames >= 60)
      {
        ESP_LOGI("PERF", "per frame: fft %d us, spectrum %d us, waterfall %d us",
                 (int)(acc_fft / n_frames), (int)(acc_spec / n_frames), (int)(acc_wf / n_frames));
        acc_fft = acc_spec = acc_wf = 0;
        n_frames = 0;
      }
#endif
      lvgl_port_unlock();
    }
  }
}

/* Clock: UTC from the system clock once it has been synced (serial time-sync
 * tool or the FT8 panel's SYNC button - see ft8_time.h); uptime since boot
 * until then. An NTP (ESP32-C6) or RTC source only needs to set the system
 * clock through ft8_time_set_epoch_ms() for this to show it. */
void timer_clock_update(lv_timer_t *timer)
{
  (void)timer;
  if (ft8_time_is_synced())
  {
    struct tm t;
    ft8_time_get_utc(&t);
    lv_label_set_text_fmt(label_clock, "%02d:%02d:%02d UTC", t.tm_hour, t.tm_min, t.tm_sec);
  }
  else
  {
    int64_t s = esp_timer_get_time() / 1000000;
    lv_label_set_text_fmt(label_clock, "%02d:%02d:%02d",
                          (int)((s / 3600) % 100), (int)((s / 60) % 60), (int)(s % 60));
  }
}

void timer_smeter_update(lv_timer_t *timer)
{
  if (refresca_smeter && lvgl_port_lock(0))
  {
    smeter_set_dbm(calculadBm());
    refresca_smeter = false;
    lvgl_port_unlock();
  }
}

void timer_uso_cpu(lv_timer_t *timer)
{

  return;
  /* Task lock */
  if (lvgl_port_lock(0))
  {

    lv_label_set_text_fmt(label5, "%u", time_sdrtask);
    lv_label_set_text_fmt(label6, "%u", esp_get_free_heap_size() / 1000);

    lvgl_port_unlock();
  }

  // loops++;
  // ESP_LOGI(TAG, "loops= %d --- sdrtask= %u", loops, time_sdrtask);
}

static inline void lv_draw_pixel(int x, int y, uint16_t color)
{
  waveformbuffer[y * CANVAS_W + x] = color;
}

static inline void lv_draw_vline(int x, int y, int h, uint16_t color)
{
  // if ((unsigned)x >= CANVAS_W)
  //   return;
  // if (h <= 0)
  //   return;

  // int max_h = CANVAS_H - y;
  // if (max_h <= 0)
  //   return;
  // if (h > max_h)
  //   h = max_h;

  uint32_t index = y * CANVAS_W + x;
  for (int i = 0; i < h; i++)
  {
    waveformbuffer[index] = color;
    index += CANVAS_W;
  }
}

/*
 * 5-point smoothing with weights 0.50 / 0.18 / 0.07, in integer math (64/23/9 out of 128).
 * The P4 FPU is single precision only: the previous expression used double constants,
 * so every column ran ~18 soft-float double operations, twice per column, every frame.
 */
static inline int smooth5(const int16_t *p, int x)
{
  return (64 * p[x] + 23 * (p[x - 1] + p[x + 1]) + 9 * (p[x - 2] + p[x + 2])) >> 7;
}

void spectrum(void)
{

  lv_obj_invalidate(waveform_canvas);

  int16_t y_old, y_new, y1_new, y1_old;
  int16_t y1_old_minus = 0;
  int16_t y1_new_minus = 0;

#define MARGEN_DERECHO 2
#define MARGEN_IZQUIERDO 2

  for (int16_t x = MARGEN_IZQUIERDO; x < SAMPLE_BUFFER_SIZE - MARGEN_DERECHO; x++)
  {

    // moving window - weighted average of 5 points of the spectrum to smooth spectrum in the frequency domain
    // weights:  x: 50% , x-1/x+1: 36%, x+2/x-2: 14%

    y_new = smooth5(pixelnew, x);
    y_old = smooth5(pixelold, x);

    if (y_old > (spectrum_height - 1))
    {
      y_old = (spectrum_height - 1);
    }

    if (y_new > (spectrum_height - 1))
    {
      y_new = (spectrum_height - 1);
    }

    if (y_old < 0)
      y_old = 0;
    if (y_new < 0)
      y_new = 0;

    y1_old = (spectrum_y + spectrum_height - 1) - y_old;
    y1_new = (spectrum_y + spectrum_height - 1) - y_new;

    if (x == MARGEN_IZQUIERDO)
    {
      y1_old_minus = y1_old;
      y1_new_minus = y1_new;
    }
    if (x == SAMPLE_BUFFER_SIZE - MARGEN_DERECHO)
    {
      y1_old_minus = y1_old;
      y1_new_minus = y1_new;
    }

    // DELETE OLD LINE/POINT
    if (y1_old - y1_old_minus > 1)
    { // plot line upwards
      lv_draw_vline(x + spectrum_x, y1_old_minus + 1, y1_old - y1_old_minus, 0);
    }
    else if (y1_old - y1_old_minus < -1)
    { // plot line downwards
      lv_draw_vline(x + spectrum_x, y1_old, y1_old_minus - y1_old, 0);
    }
    else
    {
      lv_draw_pixel(x + spectrum_x, y1_old, 0); // delete old pixel
    }

    // DRAW NEW LINE/POINT
    if (y1_new - y1_new_minus > 1)
    { // plot line upwards
      lv_draw_vline(x + spectrum_x, y1_new_minus + 1, y1_new - y1_new_minus, 0x07e0);
    }
    else if (y1_new - y1_new_minus < -1)
    { // plot line downwards
      lv_draw_vline(x + spectrum_x, y1_new, y1_new_minus - y1_new, 0x07e0);
    }
    else
    {
      lv_draw_pixel(x + spectrum_x, y1_new, 0x07e0); // write new pixel
    }

    y1_new_minus = y1_new;
    y1_old_minus = y1_old;
  }

  for (int16_t x = 0; x < SAMPLE_BUFFER_SIZE; x += 128)
  {
    lv_draw_vline(x, 0, H, 0x31A6);
  }

  // Dibuja centro de espectro de demodulación (F-F/4 OFFSET)
  lv_draw_vline(W - W / 4, 0, H, 0x075f);
}

static void slider_changed_cb(lv_event_t *e)
{
  lv_obj_t *s = lv_event_get_target_obj(e);
  int32_t v = lv_slider_get_value(s);

  audio_out_set_volume((int)v);

  lv_label_set_text_fmt(value_label, "%d", v);
}

/* ---------------------------------------------------------------------------
 * FT8 mode (bottom-row FT8 button). DSP side: ft8_app.c; panel: ft8_ui.c.
 * ------------------------------------------------------------------------- */
static lv_obj_t *btn_ft8;
static lv_obj_t *label_ft8;
static lv_obj_t *btn_dmr;   /* DMR button (see dmr_mode_toggle() below) - declared */
static lv_obj_t *label_dmr; /* here because entering FT8 also has to switch DMR off */

/* The bottom-row FILTER button now toggles AIS (filters stay in the menu). */
static void ais_button_refresh(void)
{
  if (label_filtros != NULL)
  {
    lv_label_set_text(label_filtros, ais_app_is_active() ? "ON" : "OFF");
  }
}

/* Standard FT8 dial frequencies (USB) and the band each one belongs to. On
 * entering FT8 the VFO snaps to the FT8 frequency of the band it is in; if it
 * is outside every amateur band listed here, it goes to 20 m. */
typedef struct
{
  uint32_t lo, hi, dial;
} ft8_band_t;

static const ft8_band_t k_ft8_bands[] = {
    {1800000, 2000000, 1840000},     /* 160 m */
    {3500000, 4000000, 3573000},     /* 80 m */
    {5250000, 5450000, 5357000},     /* 60 m */
    {7000000, 7300000, 7074000},     /* 40 m */
    {10100000, 10150000, 10136000},  /* 30 m */
    {14000000, 14350000, 14074000},  /* 20 m */
    {18068000, 18168000, 18100000},  /* 17 m */
    {21000000, 21450000, 21074000},  /* 15 m */
    {24890000, 24990000, 24915000},  /* 12 m */
    {28000000, 29700000, 28074000},  /* 10 m */
    {50000000, 54000000, 50313000},  /* 6 m */
    {144000000, 148000000, 144174000}, /* 2 m */
};

static uint32_t ft8_dial_for(uint32_t f)
{
  for (size_t i = 0; i < sizeof(k_ft8_bands) / sizeof(k_ft8_bands[0]); i++)
  {
    if (f >= k_ft8_bands[i].lo && f <= k_ft8_bands[i].hi)
    {
      return k_ft8_bands[i].dial;
    }
  }
  return 14074000;
}

static void ft8_button_refresh(void)
{
  if (label_ft8 != NULL)
  {
    lv_label_set_text(label_ft8, ft8_app_is_active() ? "ON" : "OFF");
  }
}

/* ft8_ui.c calls this when it leaves FT8 on its own (mode changed elsewhere). */
static void ft8_exit_cb(void)
{
  ft8_button_refresh();
}

static void ft8_mode_toggle(void)
{
  if (!ft8_app_available())
  {
    ESP_LOGW(TAG, "FT8 not available (init failed, see log)");
    return;
  }

  if (ft8_app_is_active())
  {
    ft8_app_set_active(false);
    ft8_ui_show(false);
  }
  else
  {
    const bool was_wfm = (demod_modo == DEMOD_WFM);

    if (dmr_app_is_active())
    {
      /* leave DMR first (both modes take over the spectrum area) */
      dmr_app_set_active(false);
      dmr_ui_show(false);
      if (label_dmr != NULL)
      {
        lv_label_set_text(label_dmr, "OFF");
      }
    }
    if (ais_app_is_active())
    {
      ais_app_set_active(false);
      ais_ui_show(false);
      ais_button_refresh();
    }
    demod_modo = DEMOD_USB;
    currentVFO.demod_modo = DEMOD_USB;
    currentVFO.Frec = ft8_dial_for(currentVFO.Frec);
    update_vfo_label(); /* already inside the LVGL context here */
    if (was_wfm)
    {
      /* WFM had switched the tuner to AGC - restore the manual gain */
      rtl_source_set_gain_auto(false);
      rtl_source_set_gain_db(menu_get_rtl_gain_db());
    }
    rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));

    lv_label_set_text_fmt(label_modos, "FT8 %s", demod_modos_texto[demod_modo]);
    dibuja_pasabanda();
    refresca_indicadores();

    ft8_app_set_active(true);
    ft8_ui_show(true);
  }
  ft8_button_refresh();
}

/* ---------------------------------------------------------------------------
 * DMR mode (bottom-row DMR button). DSP side: dmr/dmr_app.c; panel: dmr_ui.c.
 * Uses the NFM path, stays on the current frequency; FT8 and DMR exclude each
 * other (both take over the spectrum area).
 * ------------------------------------------------------------------------- */
static void dmr_button_refresh(void)
{
  if (label_dmr != NULL)
  {
    lv_label_set_text(label_dmr, dmr_app_is_active() ? "ON" : "OFF");
  }
}

static void dmr_exit_cb(void)
{
  dmr_button_refresh();
}

static void dmr_mode_toggle(void)
{
  if (!dmr_app_available())
  {
    ESP_LOGW(TAG, "DMR not available (init failed, see log)");
    return;
  }
  if (dmr_app_is_active())
  {
    dmr_app_set_active(false);
    dmr_ui_show(false);
  }
  else
  {
    const bool was_wfm = (demod_modo == DEMOD_WFM);

    if (ft8_app_is_active())
    {
      ft8_mode_toggle(); /* leave FT8 first */
    }
    if (ais_app_is_active())
    {
      ais_app_set_active(false); /* leave AIS (its panel hides itself) */
      ais_ui_show(false);
      ais_button_refresh();
    }
    demod_modo = DEMOD_FM;
    currentVFO.demod_modo = DEMOD_FM;
    if (was_wfm)
    {
      rtl_source_set_gain_auto(false);
      rtl_source_set_gain_db(menu_get_rtl_gain_db());
    }
    rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
    lv_label_set_text_fmt(label_modos, "DMR %s", demod_modos_texto[demod_modo]);
    dibuja_pasabanda();
    refresca_indicadores();

    dmr_app_set_active(true);
    dmr_ui_show(true);
  }
  dmr_button_refresh();
}

/* ---------------------------------------------------------------------------
 * AIS mode (menu -> AIS). Tunes 162.000 MHz on the WFM wide path (both AIS
 * channels at +-25 kHz); leaving restores the previous frequency and mode.
 * FT8, DMR and AIS exclude each other (all take over the spectrum area).
 * ------------------------------------------------------------------------- */
#define AIS_CENTER_HZ 162000000u
static uint32_t s_ais_prev_freq;
static int s_ais_prev_mode = -1;
static void (*s_ais_state_cb)(void);

void ui_ais_set_state_callback(void (*cb)(void))
{
  s_ais_state_cb = cb;
}

static void ais_apply_tuning(uint32_t f, int mode)
{
  const bool was_wfm = (demod_modo == DEMOD_WFM);
  demod_modo = mode;
  currentVFO.demod_modo = mode;
  currentVFO.Frec = f;
  update_vfo_label();
  if (was_wfm && mode != DEMOD_WFM)
  {
    rtl_source_set_gain_auto(false);
    rtl_source_set_gain_db(menu_get_rtl_gain_db());
  }
  rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
  lv_label_set_text_fmt(label_modos, "AIS %s", demod_modos_texto[demod_modo]);
  dibuja_pasabanda();
  refresca_indicadores();
}

static void ais_exit_cb(void)
{
  /* the panel left AIS on its own (mode changed elsewhere): keep that mode */
  s_ais_prev_mode = -1;
  ais_button_refresh();
  if (s_ais_state_cb != NULL)
  {
    s_ais_state_cb();
  }
}

void ui_ais_toggle(void)
{
  if (!ais_app_available())
  {
    ESP_LOGW(TAG, "AIS not available (init failed, see log)");
    return;
  }
  if (ais_app_is_active())
  {
    ais_app_set_active(false);
    ais_ui_show(false);
    if (s_ais_prev_mode >= 0)
    {
      ais_apply_tuning(s_ais_prev_freq, s_ais_prev_mode);
      s_ais_prev_mode = -1;
    }
  }
  else
  {
    if (ft8_app_is_active())
    {
      ft8_mode_toggle();
    }
    if (dmr_app_is_active())
    {
      dmr_mode_toggle();
    }
    s_ais_prev_freq = currentVFO.Frec;
    s_ais_prev_mode = demod_modo;
    ais_apply_tuning(AIS_CENTER_HZ, DEMOD_WFM);
    ais_app_set_active(true);
    ais_ui_show(true);
  }
  ais_button_refresh();
  if (s_ais_state_cb != NULL)
  {
    s_ais_state_cb();
  }
}

bool ui_ais_is_active(void)
{
  return ais_app_is_active();
}

void btn_event_cb(lv_event_t *e)
{
  if (bloqueo_pulsacion)
    return; // Ignora si está en cooldown

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target_obj(e);

  if (code == LV_EVENT_CLICKED)
  {
    bloqueo_pulsacion = true;

    if (obj == btn_modos)
    {
      demod_modo--;
      if (demod_modo < 0)
        demod_modo = 5;
      lv_label_set_text_fmt(label_modos, "%s", demod_modos_texto[demod_modo]);
      dibuja_pasabanda();
      refresca_indicadores();
    }
    else if (obj == btn_menu || obj == waterfall_canvas)
    {
      if (timer_pantalla)
        lv_timer_pause(timer_pantalla);
      if (timer_smeter)
        lv_timer_pause(timer_smeter);

      debug_print_mem("ANTES de crear menu");
      ui_create_control_panel();
      debug_print_mem("DESPUES de crear menu");
    }
    else if (obj == btn_step)
    {
      pasos_indice++;
      if (pasos_indice > 5)
        pasos_indice = 0;

      lv_label_set_text_fmt(label_step, "%s", pasos_texto[pasos_indice]);
      refresca_indicadores();
    }
    else if (obj == btn_filtros)
    {
      ui_ais_toggle(); /* FILTER button repurposed as AIS; filters: menu -> FILTRO */
    }
    else if (obj == btn2)
    {
      if (!f_nrss)
        f_nrss = true;
      else
        f_nrss = false;

      lv_label_set_text_fmt(label2, "NR=%d", f_nrss);
    }
    else if (obj == btn3)
    {
      agc_wdsp_conf.AGC_mode++;
      if (agc_wdsp_conf.AGC_mode > 5)
        agc_wdsp_conf.AGC_mode = 0;

      agc_wdsp_conf.agc_switch_mode = 1;
      AGC_prep();

      lv_label_set_text_fmt(label3, agc_texto[agc_wdsp_conf.AGC_mode], 0);
    }
    else if (obj == btn4)
    {
      /* Toggle between NFM and WFM (broadcast FM); any other mode's press just enters
       * NFM, matching this button's previous, simpler behaviour. */
      demod_modo = (demod_modo == DEMOD_FM) ? DEMOD_WFM : DEMOD_FM;
      currentVFO.demod_modo = demod_modo;
      lv_label_set_text_fmt(label4, "%s", (demod_modo == DEMOD_WFM) ? "WFM" : "NFM");

      /* WFM tunes on-frequency (no 12 kHz offset - see lo_offset_for_mode()) and its
       * signals are always strong broadcast carriers, so default to the tuner's AGC;
       * leaving WFM restores whatever manual gain the slider was last set to. */
      rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
      rtl_source_set_gain_auto(demod_modo == DEMOD_WFM);
      if (demod_modo != DEMOD_WFM)
      {
        rtl_source_set_gain_db(menu_get_rtl_gain_db());
      }

      dibuja_pasabanda();
      refresca_indicadores();
    }
    else if (obj == btn_ft8)
    {
      ft8_mode_toggle();
    }
    else if (obj == btn_dmr)
    {
      dmr_mode_toggle();
    }
    else if (obj == btn5)
    {
      if (!screen_update)
        screen_update = true;
      else
        screen_update = false;
    }
    else if (obj == btn_vol)
    {
      // Crear el slider_vol la primera vez
      if (slider_vol == NULL)
      {
        // slider_vol
        slider_vol = lv_slider_create(screen);
        lv_slider_set_range(slider_vol, 0, 100); /* percent of the ES8311 output range */
        lv_slider_set_value(slider_vol, audio_out_get_volume(), LV_ANIM_OFF);
        lv_obj_set_style_bg_color(slider_vol, lv_color_hex(0x202020), LV_PART_MAIN);
        lv_obj_set_style_border_color(slider_vol, lv_color_hex(0x404040), LV_PART_MAIN);

        // Colócalo cerca del botón (debajo, centrado respecto al botón)
        lv_obj_align_to(slider_vol, btn_vol, LV_ALIGN_OUT_TOP_MID, 0, -32);

        // Etiqueta con el valor
        value_label = lv_label_create(screen);
        lv_label_set_text_fmt(value_label, "", lv_slider_get_value(slider_vol) / 0x3F * 100);
        lv_obj_align_to(value_label, slider_vol, LV_ALIGN_OUT_TOP_MID, 60, 0);
        lv_obj_set_style_text_color(value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

        // Callback para actualizar la etiqueta
        lv_obj_add_event_cb(slider_vol, slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
      }
      else
      {
        // Alternar visibilidad
        if (lv_obj_has_flag(slider_vol, LV_OBJ_FLAG_HIDDEN))
        {
          lv_obj_clear_flag(slider_vol, LV_OBJ_FLAG_HIDDEN);
          lv_obj_clear_flag(value_label, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
          lv_obj_add_flag(slider_vol, LV_OBJ_FLAG_HIDDEN);
          lv_obj_add_flag(value_label, LV_OBJ_FLAG_HIDDEN);
        }
      }
    }

    lv_timer_resume(timer_debounce);
  }
}

/*
 * Shared look for the bottom control buttons: bigger, rounded, blue vertical
 * gradient (per Jorge's ESPHome-style reference photo), replacing the previous
 * small flat dark-gray 70x50 buttons.
 */
static void style_ctrl_button(lv_obj_t *btn)
{
  lv_obj_set_size(btn, UI_CTRL_BTN_W, UI_CTRL_BTN_H);
  lv_obj_set_style_radius(btn, 14, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x3d8fe0), LV_PART_MAIN);
  lv_obj_set_style_bg_grad_color(btn, lv_color_hex(0x0d4a8f), LV_PART_MAIN);
  lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN);
  lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(btn, lv_color_hex(0x1a5aa0), LV_PART_MAIN);
  lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
}

/* Small dimmed name label (top half) + a bigger value label (bottom half), both
 * inside the same button - "MODE" / "USB", "FILTER" / "2k3", "STEP" / "1000",
 * matching the reference layout. *out_value is the label callers update later
 * (label_modos/label_filtros/label_step already did that; only their creation
 * and position changes here). MENU has no second line - it is an action, not a
 * value display - so its own block below skips this helper's name/value split. */
static lv_obj_t *add_name_value_labels(lv_obj_t *btn, const char *name, lv_obj_t **out_value)
{
  lv_obj_t *name_lbl = lv_label_create(btn);
  lv_label_set_text(name_lbl, name);
  lv_obj_set_style_text_color(name_lbl, lv_color_hex(0xBFD9F5), LV_PART_MAIN);
  lv_obj_align(name_lbl, LV_ALIGN_TOP_MID, 0, 6);

  lv_obj_t *value_lbl = lv_label_create(btn);
  lv_obj_set_style_text_color(value_lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_align(value_lbl, LV_ALIGN_BOTTOM_MID, 0, -6);

  *out_value = value_lbl;
  return name_lbl;
}

void dibuja_botones(void)
{
  /* --- Botón Menu (action button: single centered label, no value line) --- */
  btn_menu = lv_btn_create(screen);
  style_ctrl_button(btn_menu);
  lv_obj_align(btn_menu, LV_ALIGN_BOTTOM_LEFT, 10, -10);

  label_menu = lv_label_create(btn_menu);
  lv_label_set_text(label_menu, "MENU");
  lv_obj_center(label_menu);

  /* --- Botón Modos: "MODE" / current demod mode --- */
  btn_modos = lv_btn_create(screen);
  style_ctrl_button(btn_modos);
  lv_obj_align(btn_modos, LV_ALIGN_BOTTOM_LEFT, 10 + (UI_CTRL_BTN_W + UI_CTRL_BTN_GAP), -10);

  add_name_value_labels(btn_modos, "MODE", &label_modos);
  lv_label_set_text_fmt(label_modos, "%s", demod_modos_texto[demod_modo]);

  /* --- Botón Step: "STEP" / current step value --- */
  btn_step = lv_btn_create(screen);
  style_ctrl_button(btn_step);
  lv_obj_align(btn_step, LV_ALIGN_BOTTOM_LEFT, 10 + 2 * (UI_CTRL_BTN_W + UI_CTRL_BTN_GAP), -10);

  add_name_value_labels(btn_step, "STEP", &label_step);
  lv_label_set_text_fmt(label_step, "%d", pasos[pasos_indice]);

  /* --- Botón AIS (antes FILTER): "AIS" / ON|OFF - filters stay in the menu --- */
  btn_filtros = lv_btn_create(screen);
  style_ctrl_button(btn_filtros);
  lv_obj_align(btn_filtros, LV_ALIGN_BOTTOM_LEFT, 10 + 3 * (UI_CTRL_BTN_W + UI_CTRL_BTN_GAP), -10);

  add_name_value_labels(btn_filtros, "AIS", &label_filtros);
  lv_label_set_text(label_filtros, "OFF");

  /* --- Botón FT8: "FT8" / ON|OFF --- */
  btn_ft8 = lv_btn_create(screen);
  style_ctrl_button(btn_ft8);
  lv_obj_align(btn_ft8, LV_ALIGN_BOTTOM_LEFT, 10 + 4 * (UI_CTRL_BTN_W + UI_CTRL_BTN_GAP), -10);

  add_name_value_labels(btn_ft8, "FT8", &label_ft8);
  lv_label_set_text(label_ft8, "OFF");

  /* --- Botón DMR: "DMR" / ON|OFF --- */
  btn_dmr = lv_btn_create(screen);
  style_ctrl_button(btn_dmr);
  lv_obj_align(btn_dmr, LV_ALIGN_BOTTOM_LEFT, 10 + 5 * (UI_CTRL_BTN_W + UI_CTRL_BTN_GAP), -10);

  add_name_value_labels(btn_dmr, "DMR", &label_dmr);
  lv_label_set_text(label_dmr, "OFF");

  return;

  /* --- Botón 2 --- */
  btn2 = lv_btn_create(screen);
  lv_obj_set_size(btn2, 100, 50);
  lv_obj_align(btn2, LV_ALIGN_BOTTOM_LEFT, 120, -10);
  lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn2, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn2, lv_color_hex(0x404040), LV_PART_MAIN);

  label2 = lv_label_create(btn2);
  lv_label_set_text_fmt(label2, "NR=%d", f_nrss);
  lv_obj_align(btn_modos, LV_ALIGN_BOTTOM_LEFT, 10, -10); // margen de 10 px desde el borde
  lv_obj_center(label2);

  /* --- Botón 4 --- */
  btn4 = lv_btn_create(screen);
  lv_obj_set_size(btn4, 100, 50);
  lv_obj_align(btn4, LV_ALIGN_BOTTOM_LEFT, 340, -10);
  lv_obj_add_event_cb(btn4, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn4, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn3, lv_color_hex(0x404040), LV_PART_MAIN);

  label4 = lv_label_create(btn4);
  lv_label_set_text_fmt(label4, "FM", 0);
  lv_obj_center(label4);

  /* --- Botón 5 --- */
  btn5 = lv_btn_create(screen);
  lv_obj_set_size(btn5, 100, 50);
  lv_obj_align(btn5, LV_ALIGN_BOTTOM_LEFT, 450, -10);
  lv_obj_add_event_cb(btn5, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn5, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn5, lv_color_hex(0x404040), LV_PART_MAIN);

  label5 = lv_label_create(btn5);
  lv_label_set_text_fmt(label5, "", 0);
  lv_obj_center(label5);

  /* --- Botón 6 --- */
  btn6 = lv_btn_create(screen);
  lv_obj_set_size(btn6, 100, 50);
  lv_obj_align(btn6, LV_ALIGN_BOTTOM_LEFT, 560, -10);
  lv_obj_add_event_cb(btn6, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn6, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn6, lv_color_hex(0x404040), LV_PART_MAIN);

  label6 = lv_label_create(btn6);
  lv_label_set_text_fmt(label6, "", 0);
  lv_obj_center(label6);

  /* --- Botón 6 --- */
  btn_vol = lv_btn_create(screen);
  lv_obj_set_size(btn_vol, 100, 50);
  lv_obj_align(btn_vol, LV_ALIGN_BOTTOM_LEFT, 670, -10);
  lv_obj_add_event_cb(btn_vol, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn_vol, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn_vol, lv_color_hex(0x404040), LV_PART_MAIN);

  label_vol = lv_label_create(btn_vol);
  lv_label_set_text_fmt(label_vol, "VOLUME", 0);
  lv_obj_center(label_vol);
}

/*
 * Drag left/right on the spectrum to change the VFO by whole multiples of the
 * current step (same pasos[]/pasos_indice the encoder and the step button use),
 * one step per SPECTRUM_DRAG_PX_PER_STEP pixels of horizontal movement - a
 * touchscreen equivalent of turning the encoder. Only horizontal movement is
 * tracked (no proportional/absolute frequency-from-x-position mapping), and a
 * short tap that doesn't reach one step's worth of movement changes nothing,
 * so this doesn't interfere with tapping elsewhere on the screen.
 *
 * Deliberately on waveform_canvas (the spectrum), not waterfall_canvas: the
 * latter already opens the menu on tap (see btn_event_cb), and keeping the two
 * canvases' touch behaviour separate avoids any conflict between them.
 */
#define SPECTRUM_DRAG_PX_PER_STEP 15 /* pixels per step; a starting value, adjust to taste on real hardware */

/*
 * Minimum time between actual RF retunes triggered by dragging. NOT about LVGL or
 * rendering (see the long comment below) - the ONLY thing this throttles is how
 * often rtl_source_set_freq() reaches the hardware; the on-screen frequency and
 * currentVFO.Frec update on every step regardless, so the drag still feels live.
 *
 * *** WHY THIS EXISTS - diagnosed 2026-09-22, per Jorge's own observation that the
 * WATERFALL KEPT SCROLLING but with the SAME (stale) data repeating while dragging,
 * resuming the instant the drag stopped. That ruled out an LVGL/rendering freeze
 * (three earlier attempts at that theory - a locking deadlock, an implicit LVGL
 * scroll gesture, and default press-state styling - were each real, worth having
 * fixed, but NONE of them were this bug). The actual cause, confirmed against
 * esp_rtl_sdr's own source: retune_hz(), called from an app task (this project's
 * rtl_source.c control task) as opposed to the driver's own event-callback context,
 * PAUSES AND DRAINS the USB bulk pipeline, applies the new LO over EP0, then
 * resubmits - a real, intentional stop-and-resume of the I/Q data stream every
 * single time, not a cheap register poke. rtl_source_set_freq() coalesces bursts
 * (only the latest requested frequency survives if several arrive before the
 * control task gets to them), but that only bounds the number of PHYSICAL retunes,
 * not how much of a fast, continuous drag's duration they eat: a quick swipe can
 * cross many step boundaries a second, keeping the control task busy pausing and
 * resuming the pipeline back-to-back for the whole gesture, so sdrTask's own
 * rtl_source_read_float() keeps timing out and - by design (see its ESP_ERR_TIMEOUT
 * path in sdr.c) - skips refreshing i_fft/q_fft on every failed read, so
 * calcula_fft() keeps re-computing the SAME stale frame. Nothing here is unique to
 * touch: the encoder drives the exact same rtl_source_set_freq() call per detent
 * and was never reported doing this, simply because a human turning a knob can't
 * generate anywhere close to the retune rate a fast finger swipe can - this
 * throttle just brings touch dragging down to a similarly gentle retune rate.
 */
/*
 * UPDATE: rtl_source_set_freq() now retunes digitally (NCO in rtl_dsp.c, no USB
 * pause) whenever the new frequency is within RTL_SOURCE_DIGITAL_WINDOW_HZ of the
 * dongle's physical LO, and re-centres the dongle once, only after tuning has
 * been idle for a moment. Dragging therefore no longer stops the I/Q stream, so
 * this throttle is disabled (0): every drag step is applied at once. It is kept
 * only as a knob in case a physical-retune-heavy use ever needs it again.
 */
#define SPECTRUM_DRAG_RETUNE_MIN_US 0

static void spectrum_drag_apply_freq(void)
{
  rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
}

static void spectrum_drag_cb(lv_event_t *e)
{
  static lv_point_t last_point;
  static int32_t drag_accum_px = 0;
  static int64_t last_retune_us = 0;
  static bool retune_pending = false;

  lv_indev_t *indev = lv_indev_get_act();
  if (indev == NULL)
    return;

  lv_point_t p;
  lv_indev_get_point(indev, &p);

  const lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_PRESSED)
  {
    last_point = p;
    drag_accum_px = 0;
    retune_pending = false;
    return;
  }

  if (code == LV_EVENT_RELEASED)
  {
    /* Always flush on release: a throttled-away retune from the last few steps of
     * the drag must still land, or the hardware could be left tuned a few steps
     * short of what the screen shows. */
    if (retune_pending)
    {
      spectrum_drag_apply_freq();
      retune_pending = false;
    }
    return;
  }

  drag_accum_px += (p.x - last_point.x);
  last_point = p;

  int32_t steps = drag_accum_px / SPECTRUM_DRAG_PX_PER_STEP;
  if (steps == 0)
    return;
  drag_accum_px -= steps * SPECTRUM_DRAG_PX_PER_STEP;

  currentVFO.Frec -= steps * pasos[pasos_indice]; /* inverted from the encoder's "+=": drag right now lowers frequency */
  update_vfo_label(); /* NOT refresca_VFO(): already inside an LVGL callback, see its comment */

  const int64_t now_us = esp_timer_get_time();
  if (now_us - last_retune_us >= SPECTRUM_DRAG_RETUNE_MIN_US)
  {
    spectrum_drag_apply_freq();
    last_retune_us = now_us;
    retune_pending = false;
  }
  else
  {
    /* Coalesced by rtl_source_set_freq() itself if a throttled window opens before
     * the next step - flushed for certain on LV_EVENT_RELEASED either way. */
    retune_pending = true;
  }
}

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  data->enc_diff = s_enc_diff; // entregar Δ acumulado a LVGL
  s_enc_diff = 0;              // limpiar después de reportarlo

  // Si tienes botón físico, cambiarías data->state aquí
  data->state = LV_INDEV_STATE_RELEASED;
}

lv_indev_t *lvgl_encoder_init(void)
{
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
  lv_indev_set_read_cb(indev, encoder_read_cb);
  return indev;
}

void tarea_encoder(void *arg)
{
  while (1)
  {
    // Leer delta del encoder
    int32_t diff = s_enc_diff;
    s_enc_diff = 0;

    if (diff != 0)
    {
      currentVFO.Frec += diff * pasos_indice[pasos];
      refresca_VFO();

      rtl_source_set_freq(currentVFO.Frec - lo_offset_for_mode(demod_modo));
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
  }
}

/*
 * Waterfall / FT8-cascade palette. The full SDR++ set (plus the DeepSDR's
 * "Fire") lives in palettes.c; the default, Classic, is bit-identical to the
 * single palette this file used to hard-code.
 *
 * Built into a 256-entry LUT (palette_lut_init()) rather than evaluated per
 * call: fft_color_map() runs once per pixel in waterfall_update(), the
 * hottest path in the UI. The selection is stored in NVS (namespace "ui",
 * key "palette") and restored at boot. The LUT is only rebuilt from the LVGL
 * context (init_ui(), menu button), which is also the only context that
 * reads it, so no locking is needed.
 */
static uint16_t s_palette_lut[256];
static bool s_palette_lut_ready = false;
static int s_palette_id = PALETTE_CLASSIC;

static void palette_lut_init(void)
{
  palette_build_lut(s_palette_id, s_palette_lut);
  s_palette_lut_ready = true;
}

static void palette_load_from_nvs(void)
{
  nvs_handle_t h;
  uint8_t v;

  (void)nvs_flash_init(); /* ESP_OK if already initialized (ft8_app_init() does it too) */
  if (nvs_open("ui", NVS_READONLY, &h) != ESP_OK)
  {
    return;
  }
  if (nvs_get_u8(h, "palette", &v) == ESP_OK && v < PALETTE_COUNT)
  {
    s_palette_id = v;
  }
  nvs_close(h);
}

static void palette_save_to_nvs(void)
{
  nvs_handle_t h;

  if (nvs_open("ui", NVS_READWRITE, &h) != ESP_OK)
  {
    return;
  }
  if (nvs_set_u8(h, "palette", (uint8_t)s_palette_id) == ESP_OK)
  {
    nvs_commit(h);
  }
  nvs_close(h);
}

int ui_get_palette(void)
{
  return s_palette_id;
}

void ui_set_palette(int id)
{
  if (id < 0 || id >= PALETTE_COUNT)
  {
    id = PALETTE_CLASSIC;
  }
  s_palette_id = id;
  palette_lut_init(); /* the next waterfall row already uses it */
  palette_save_to_nvs();
}

uint16_t fft_color_map(uint8_t v)
{
  if (!s_palette_lut_ready)
  {
    palette_lut_init(); /* safety net: normally already built by init_ui() */
  }
  return s_palette_lut[v];
}

void waterfall_update(void)
{
  /*
   * Step the ring back one slot. That slot currently holds the row that is about to
   * fall off the bottom (the oldest one, WATERFALL_HEIGHT steps behind); overwriting
   * it makes it the new top row, and every other row's memory is untouched - no
   * memmove, replacing what used to be a ~260 KB PSRAM copy every frame with two
   * WAVEFORM_WIDTH-pixel writes.
   */
  waterfall_head = (waterfall_head == 0) ? (WATERFALL_HEIGHT - 1) : (waterfall_head - 1);

  uint16_t *row_a = &waterfallbuffer[waterfall_head * WAVEFORM_WIDTH];
  uint16_t *row_b = &waterfallbuffer[(waterfall_head + WATERFALL_HEIGHT) * WAVEFORM_WIDTH];
  for (int x = 0; x < WAVEFORM_WIDTH; x++)
  {
    uint16_t c = fft_color_map((uint8_t)abs(pixelnew[x])); // 0-255 -> RGB565
    row_a[x] = c;
    row_b[x] = c;
  }

  /* Re-point the canvas at the new front of the ring. lv_canvas_set_buffer() only
   * updates the canvas's internal buffer descriptor (no pixel copy), so this is cheap. */
  lv_canvas_set_buffer(waterfall_canvas, row_a, WAVEFORM_WIDTH, WATERFALL_HEIGHT, LV_COLOR_FORMAT_RGB565);
  lv_obj_invalidate(waterfall_canvas);
}

void init_ui()
{
  palette_load_from_nvs();
  palette_lut_init();

  /* Obtén la pantalla activa */
  screen = lv_scr_act();

  // Padre "limpio"
  lv_obj_remove_style_all(screen);
  lv_obj_set_style_border_width(screen, 0, 0);
  lv_obj_set_style_pad_all(screen, 0, 0);
  lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  dibuja_botones();

  waveform_canvas = lv_canvas_create(screen);
  /* Same treatment `screen` already gets: strips the active theme's default styling,
   * including whatever it applies for LV_STATE_PRESSED (commonly a bg/opa or outline
   * change on any clickable object). Without this, touching the canvas visually
   * "freezes" it immediately - not a real stall, just the theme's press-feedback
   * overlay covering the live spectrum until release reverts the state. */
  lv_obj_remove_style_all(waveform_canvas);
  waveformbuffer = heap_caps_malloc(
      WAVEFORM_WIDTH * WAVEFORM_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  memset(waveformbuffer, 0x00, WAVEFORM_WIDTH * WAVEFORM_HEIGHT * sizeof(uint16_t)); // índice 0 (negro)

  // lv_canvas_set_buffer(waveform_canvas, waveformbuffer, WAVEFORM_WIDTH, WAVEFORM_HEIGHT, LV_COLOR_FORMAT_I4);

  lv_canvas_set_buffer(waveform_canvas, waveformbuffer, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);

  waterfall_canvas = lv_canvas_create(screen);
  /* Double-height ring buffer (see the field comment in ui_priv.h): was
   * WAVEFORM_WIDTH * WAVEFORM_HEIGHT (192 rows' worth, more than needed by mistake -
   * WATERFALL_HEIGHT is 128 - but harmless since it only over-allocated). Now sized
   * and named for what it actually holds: 2 * WATERFALL_HEIGHT rows. */
  waterfallbuffer = heap_caps_malloc(
      2u * WAVEFORM_WIDTH * WATERFALL_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  memset(waterfallbuffer, 0x00, 2u * WAVEFORM_WIDTH * WATERFALL_HEIGHT * sizeof(uint16_t)); // índice 0 (negro)
  waterfall_head = 0;
  lv_canvas_set_buffer(waterfall_canvas, waterfallbuffer, WAVEFORM_WIDTH, WATERFALL_HEIGHT, LV_COLOR_FORMAT_RGB565);

  // POSICION DE LOS CANVAS DE WATERFALL Y ESPECTRO!!!

  lv_obj_set_pos(waveform_canvas, 0, UI_SPECTRUM_TOP_Y);
  lv_obj_align_to(waterfall_canvas, waveform_canvas, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

  // Borde y fondo
  // lv_obj_set_style_bg_color(waveform_canvas, lv_color_hex(0x000000), 0);
  // lv_obj_set_style_bg_opa(waveform_canvas, LV_OPA_COVER, 0);
  // lv_obj_set_style_border_width(waveform_canvas, 2, 0);
  // lv_obj_set_style_border_color(waveform_canvas, lv_color_hex(0x999999), 0);
  // lv_obj_set_style_radius(waveform_canvas, 6, 0);
  // lv_obj_set_style_pad_all(waveform_canvas, 0, 0);

  // // Para una “aura” visible opcional:
  // lv_obj_set_style_outline_width(waveform_canvas, 2, 0);
  // lv_obj_set_style_outline_color(waveform_canvas, lv_color_hex(0x00FFA0), 0);

  lv_obj_add_flag(waterfall_canvas, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(waterfall_canvas, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_add_flag(waveform_canvas, LV_OBJ_FLAG_CLICKABLE);
  /* LVGL objects are scrollable by default; without removing that, a drag on a
   * clickable object is captured as an object-scroll gesture, and LVGL's own
   * scroll handling takes over the pointer until release - which looks exactly
   * like "the spectrum stops updating while dragging, resumes on release" (it's
   * not frozen: LVGL is just busy running its own scroll interaction instead of
   * dispatching PRESSING to spectrum_drag_cb in the meantime). */
  lv_obj_remove_flag(waveform_canvas, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(waveform_canvas, spectrum_drag_cb, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(waveform_canvas, spectrum_drag_cb, LV_EVENT_PRESSING, NULL);
  lv_obj_add_event_cb(waveform_canvas, spectrum_drag_cb, LV_EVENT_RELEASED, NULL);

  // ✅ Forzar actualización
  lv_obj_invalidate(waveform_canvas);

  // Crear un label
  freq_label = lv_label_create(screen);

  // Establecer estilo de fuente
  static lv_style_t style_freq;
  lv_style_init(&style_freq);
  lv_style_set_text_font(&style_freq, &lv_font_montserrat_46);
  lv_obj_add_style(freq_label, &style_freq, LV_STATE_DEFAULT);

  // Color verde
  lv_style_set_text_color(&style_freq, lv_color_hex(0x00FF00));

  // Posición: bloque superior izquierdo, junto al S-meter (was BOTTOM_RIGHT,-10,-550)
  lv_obj_align(freq_label, LV_ALIGN_TOP_LEFT, UI_FREQ_X, UI_FREQ_Y);

  lv_obj_add_flag(freq_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(freq_label, freq_label_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_style_text_decor(freq_label, LV_TEXT_DECOR_UNDERLINE, LV_STATE_PRESSED);

  refresca_VFO();

  /* Mode text, top-left info block (plain label, not a button - the actual
   * mode-cycle control is btn_modos, now down in the bottom button row).
   * Kept in sync from refresca_indicadores(), which every mode-changing
   * callback already calls. */
  label_modo_info = lv_label_create(screen);
  lv_obj_add_style(label_modo_info, &style_freq, LV_STATE_DEFAULT); /* same font/color as freq_label */
  lv_obj_align(label_modo_info, LV_ALIGN_TOP_LEFT, UI_MODE_X, UI_MODE_Y);
  lv_label_set_text(label_modo_info, "");

  /* Clock, top-right corner. PLACEHOLDER: shows uptime since boot (HH:MM:SS), not
   * wall-clock time - Jorge has both an external RTC and a WiFi (ESP32-C6
   * companion) module available, but wiring either up is deliberately deferred
   * until after this layout pass. Swap timer_clock_cb's body for a real
   * time-of-day read once that's in place; nothing else here needs to change. */
  label_clock = lv_label_create(screen);
  lv_obj_align(label_clock, LV_ALIGN_TOP_RIGHT, -UI_CLOCK_X_MARGIN, UI_CLOCK_Y);
  lv_label_set_text(label_clock, "00:00:00");

  /* TIMERS
   */

  timer_pantalla = lv_timer_create(timer_dibuja_pantalla, 33, NULL);
  // timer_cpu = lv_timer_create(timer_uso_cpu, 1000, NULL);
  timer_smeter = lv_timer_create(timer_smeter_update, 33, NULL);
  timer_clock = lv_timer_create(timer_clock_update, 1000, NULL);
  // timer debounce 
  timer_debounce = lv_timer_create(desbloquear_cb, 300, NULL);

  /* TIMERS
   */

  box_pasabanda = lv_obj_create(lv_screen_active());

  lv_obj_set_style_bg_opa(box_pasabanda, 80, LV_PART_MAIN); // 80/255 ≈ 30% opaco
  lv_obj_set_style_bg_color(box_pasabanda, lv_color_hex(0x000088), LV_PART_MAIN);

  // Quitar borde
  lv_obj_set_style_border_width(box_pasabanda, 0, LV_PART_MAIN);

  // Opcional: quitar sombra si no la quieres
  lv_obj_set_style_shadow_width(box_pasabanda, 0, LV_PART_MAIN);

  init_smeter();
  inicia_smeter_ui();
  dibuja_pasabanda();
  indicadores_create(screen);
  refresca_indicadores();

  /* FT8 panel last, so it sits above the canvases and the passband box. */
  ft8_ui_create(screen, UI_SPECTRUM_TOP_Y);
  ft8_ui_set_exit_callback(ft8_exit_cb);
  dmr_ui_create(screen, UI_SPECTRUM_TOP_Y);
  dmr_ui_set_exit_callback(dmr_exit_cb);
  ais_ui_create(screen, UI_SPECTRUM_TOP_Y);
  ais_ui_set_exit_callback(ais_exit_cb);
}

void smeter_set_dbm(float dbm)
{
  /* Meter ballistics, like a real rig: fast attack, slow release, and a peak
   * marker that holds ~1.5 s before falling. Called at the UI frame rate. */
  static float level = SMETER_DBM_S0 - 3.0f;
  static float peak = SMETER_DBM_S0 - 3.0f;
  static int64_t peak_t_us = 0;
  static int last_lit_x = -1, last_peak_x = -1;
  static char last_s[12] = "", last_dbm[16] = "", last_pk[20] = "";

  if (smeter_canvas_buf == NULL)
    return;

  if (dbm > level)
    level += (dbm - level) * 0.6f;
  else
    level += (dbm - level) * 0.15f;

  const int64_t now = esp_timer_get_time();
  if (level >= peak)
  {
    peak = level;
    peak_t_us = now;
  }
  else if (now - peak_t_us > 1500000)
  {
    peak -= 0.8f; /* ~24 dB/s at 30 fps */
    if (peak < level)
      peak = level;
  }

  /* Bar: redraw only when a segment boundary or the peak marker moved */
  const int lit_x = smeter_dbm_to_x(level);
  const int peak_x = smeter_dbm_to_x(peak);
  if (lit_x != last_lit_x || peak_x != last_peak_x)
  {
    smeter_render(smeter_canvas_buf, level, peak);
    lv_obj_invalidate(smeter_canvas);
    last_lit_x = lit_x;
    last_peak_x = peak_x;
  }

  /* Readouts: update a label only when its text actually changes */
  char txt[20];
  smeter_format_s(level, txt, sizeof(txt));
  if (strcmp(txt, last_s) != 0)
  {
    strcpy(last_s, txt);
    lv_label_set_text(smeter_lbl_s, txt);
    lv_obj_set_style_text_color(smeter_lbl_s, lv_color_hex(smeter_zone_rgb(level)), 0);
  }
  snprintf(txt, sizeof(txt), "%d dBm", (int)(level + (level < 0 ? -0.5f : 0.5f)));
  if (strcmp(txt, last_dbm) != 0)
  {
    strcpy(last_dbm, txt);
    lv_label_set_text(smeter_lbl_dbm, txt);
  }
  if (peak > SMETER_DBM_S0)
  {
    char ps[12];
    smeter_format_s(peak, ps, sizeof(ps));
    snprintf(txt, sizeof(txt), "PEAK %s", ps);
  }
  else
  {
    snprintf(txt, sizeof(txt), "PEAK --");
  }
  if (strcmp(txt, last_pk) != 0)
  {
    strcpy(last_pk, txt);
    lv_label_set_text(smeter_lbl_peak, txt);
  }
}

void inicia_smeter_ui(void)
{
  /* Panel: dark vertical gradient, thin border, rounded. remove_style_all()
   * BEFORE set_size (LVGL v9 keeps w/h in the style - see git history). */
  meter_cont = lv_obj_create(screen);
  lv_obj_remove_style_all(meter_cont);
  lv_obj_set_size(meter_cont, SMETER_PANEL_W, SMETER_PANEL_H);
  lv_obj_set_pos(meter_cont, SMETER_PANEL_X, SMETER_PANEL_Y);
  lv_obj_set_style_bg_opa(meter_cont, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(meter_cont, lv_color_hex(0x161b24), 0);
  lv_obj_set_style_bg_grad_color(meter_cont, lv_color_hex(0x0a0d12), 0);
  lv_obj_set_style_bg_grad_dir(meter_cont, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_border_width(meter_cont, 1, 0);
  lv_obj_set_style_border_color(meter_cont, lv_color_hex(0x3a4a5a), 0);
  lv_obj_set_style_radius(meter_cont, 8, 0);
  lv_obj_remove_flag(meter_cont, LV_OBJ_FLAG_SCROLLABLE);

  /* Bar canvas, rendered by smeter_draw.c (PSRAM buffer, ~19 KiB) */
  smeter_canvas_buf = heap_caps_malloc(SMETER_CANVAS_W * SMETER_CANVAS_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  smeter_canvas = lv_canvas_create(meter_cont);
  lv_obj_remove_style_all(smeter_canvas);
  if (smeter_canvas_buf != NULL)
  {
    smeter_render(smeter_canvas_buf, SMETER_DBM_S0 - 3.0f, SMETER_DBM_S0 - 3.0f);
    lv_canvas_set_buffer(smeter_canvas, smeter_canvas_buf, SMETER_CANVAS_W, SMETER_CANVAS_H, LV_COLOR_FORMAT_RGB565);
  }
  else
  {
    ESP_LOGE(TAG, "S-meter canvas PSRAM allocation failed");
  }
  lv_obj_set_pos(smeter_canvas, SMETER_CANVAS_X, SMETER_CANVAS_Y);

  /* Scale numbers, each centred on its tick (last one right-aligned to the bar end) */
  lv_obj_t *l = lv_label_create(meter_cont);
  lv_label_set_text(l, "S");
  lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(0xc8d2dc), 0);
  lv_obj_set_pos(l, SMETER_CANVAS_X - 2, SMETER_LABEL_Y);
  for (int i = 0; i < smeter_label_count; i++)
  {
    const int box_w = 40;
    int x = SMETER_CANVAS_X + smeter_dbm_to_x(smeter_labels[i].dbm) - box_w / 2;
    lv_text_align_t al = LV_TEXT_ALIGN_CENTER;
    if (x + box_w > SMETER_CANVAS_X + SMETER_CANVAS_W)
    {
      x = SMETER_CANVAS_X + SMETER_CANVAS_W - box_w;
      al = LV_TEXT_ALIGN_RIGHT;
    }
    l = lv_label_create(meter_cont);
    lv_label_set_text(l, smeter_labels[i].text);
    lv_obj_set_width(l, box_w);
    lv_obj_set_style_text_align(l, al, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(smeter_labels[i].red ? 0xf05040 : 0xc8d2dc), 0);
    lv_obj_set_pos(l, x, SMETER_LABEL_Y);
  }

  /* Readouts */
  smeter_lbl_s = lv_label_create(meter_cont);
  lv_obj_set_style_text_font(smeter_lbl_s, &lv_font_montserrat_26, 0);
  lv_obj_set_style_text_color(smeter_lbl_s, lv_color_hex(smeter_zone_rgb(SMETER_DBM_S0)), 0);
  lv_label_set_text(smeter_lbl_s, "S0");
  lv_obj_set_pos(smeter_lbl_s, 12, 74);

  smeter_lbl_dbm = lv_label_create(meter_cont);
  lv_obj_set_style_text_font(smeter_lbl_dbm, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(smeter_lbl_dbm, lv_color_hex(0xc8d2dc), 0);
  lv_label_set_text(smeter_lbl_dbm, "--- dBm");
  lv_obj_align(smeter_lbl_dbm, LV_ALIGN_TOP_RIGHT, -12, 80);

  smeter_lbl_peak = lv_label_create(meter_cont);
  lv_obj_set_style_text_font(smeter_lbl_peak, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(smeter_lbl_peak, lv_color_hex(0x8090a0), 0);
  lv_label_set_text(smeter_lbl_peak, "PEAK --");
  lv_obj_align(smeter_lbl_peak, LV_ALIGN_TOP_RIGHT, -12, 106);
}

void dibuja_pasabanda(void)
{

  int margen_alto = currentVFO.f_alta * SAMPLE_BUFFER_SIZE / SAMPLE_RATE;

  if (lvgl_port_lock(0))
  {

    if (demod_modo == DEMOD_AM || demod_modo == DEMOD_SAM)
    {
      lv_obj_set_size(box_pasabanda, 2 * margen_alto, H);
      lv_obj_set_pos(box_pasabanda, W - W / 4 - margen_alto, UI_SPECTRUM_TOP_Y);
    }
    else if (demod_modo == DEMOD_USB || demod_modo == DEMOD_SAMU)
    {
      lv_obj_set_size(box_pasabanda, margen_alto, H);
      lv_obj_set_pos(box_pasabanda, W - W / 4, UI_SPECTRUM_TOP_Y);
    }
    else if (demod_modo == DEMOD_LSB || demod_modo == DEMOD_SAML)
    {
      lv_obj_set_size(box_pasabanda, margen_alto, H);
      lv_obj_set_pos(box_pasabanda, W - W / 4 - margen_alto, UI_SPECTRUM_TOP_Y);
    }
    else if (demod_modo == DEMOD_FM || demod_modo == DEMOD_WFM)
    {
      lv_obj_set_size(box_pasabanda, 0, 0);
      lv_obj_set_pos(box_pasabanda, W, UI_SPECTRUM_TOP_Y);
    }

    lvgl_port_unlock();
  }
}

// ---------- Objetos (static fuera de funciones) ----------
static lv_obj_t *indicadores[12] = {0};

// ---------- Estilos (static fuera de funciones) ----------
static lv_style_t st_base;
static lv_style_t st_red;
static lv_style_t st_green;
static bool styles_inited = false;

// Usaremos LV_STATE_USER_1 = "verde"
#define IND_VERDE_STATE LV_STATE_USER_1

static void init_styles_once(void)
{
  if (styles_inited)
    return;
  styles_inited = true;

  lv_style_init(&st_base);
  lv_style_set_bg_opa(&st_base, LV_OPA_COVER);
  lv_style_set_bg_color(&st_base, lv_color_black());
  lv_style_set_border_width(&st_base, 2);
  lv_style_set_radius(&st_base, 4);
  lv_style_set_pad_all(&st_base, 8);
  lv_style_set_text_align(&st_base, LV_TEXT_ALIGN_CENTER);

  // Estilo por defecto: ROJO (estado 0)
  lv_style_init(&st_red);
  lv_style_set_text_color(&st_red, lv_color_hex(0xFF0000));
  lv_style_set_border_color(&st_red, lv_color_hex(0xFF0000));

  // Estilo cuando esté en estado VERDE (USER_1)
  lv_style_init(&st_green);
  lv_style_set_text_color(&st_green, lv_color_hex(0x00FF00));
  lv_style_set_border_color(&st_green, lv_color_hex(0x00FF00));
}

// Crea los 9 indicadores UNA SOLA VEZ
void indicadores_create(lv_obj_t *parent)
{
  init_styles_once();

  for (int i = 0; i < 12; i++)
  {

    indicadores[i] = lv_label_create(parent);

    lv_obj_add_style(indicadores[i], &st_base, LV_PART_MAIN);
    lv_obj_add_style(indicadores[i], &st_red, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_style(indicadores[i], &st_green, LV_PART_MAIN | IND_VERDE_STATE);

    /* Single row (was 3 stacked rows of 5/5/2): badges 0-9 at 60 px, 10-11 (the
     * filter width and AGC mode texts, longer strings) at 100 px, 10 px gaps
     * throughout. Total width 920 px, comfortably inside the 1024 px panel.
     * Sits right below the top-left info block (see UI_BADGE_ROW_Y). */
    if (i < 10)
    {
      lv_obj_set_pos(indicadores[i], 10 + i * 70, UI_BADGE_ROW_Y);
      lv_obj_set_width(indicadores[i], 60);
    }
    else
    {
      lv_obj_set_pos(indicadores[i], 710 + (i - 10) * 110, UI_BADGE_ROW_Y);
      lv_obj_set_width(indicadores[i], 100);
    }

    lv_label_set_text(indicadores[i], "X");
    lv_obj_clear_state(indicadores[i], IND_VERDE_STATE); // arranca en rojo
  }
}

// Actualiza texto y color SIN recrear y SIN tocar estilos
void indicador_update(int idx, const char *texto, bool verde)
{
  if (idx < 0 || idx >= 12)
    return;
  lv_obj_t *lbl = indicadores[idx];
  if (!lbl)
    return;

  // Cambia texto sólo si cambia (evita trabajo)
  const char *cur = lv_label_get_text(lbl);
  if (!cur || strcmp(cur, texto) != 0)
  {
    lv_label_set_text(lbl, texto);
  }

  // Cambia estado (rojo por defecto, verde con USER_1)
  if (verde)
    lv_obj_add_state(lbl, IND_VERDE_STATE);
  else
    lv_obj_clear_state(lbl, IND_VERDE_STATE);
}

void refresca_indicadores(void)
{
  indicador_update(0, "A", true);

  indicador_update(1, "RX", true);

  indicador_update(2, "AM", (demod_modo == DEMOD_AM) ? 1 : 0);
  indicador_update(3, "SAM", (demod_modo == DEMOD_SAM || demod_modo == DEMOD_SAMU || demod_modo == DEMOD_SAML) ? 1 : 0);
  indicador_update(4, "USB", (demod_modo == DEMOD_USB) ? 1 : 0);

  indicador_update(5, "B", true);

  indicador_update(6, "LSB", (demod_modo == DEMOD_LSB) ? 1 : 0);

  indicador_update(7, "DNR", (f_nrss) ? 1 : 0);
  indicador_update(8, "NFM", (demod_modo == DEMOD_FM) ? 1 : 0);
  indicador_update(9, "WFM", (demod_modo == DEMOD_WFM) ? 1 : 0);

  indicador_update(10, filtros_texto[filtro_indice], true);
  indicador_update(11, agc_texto[agc_wdsp_conf.AGC_mode], true);

  /* Top-left info block's plain mode text - see label_modo_info's creation comment. */
  if (label_modo_info != NULL)
  {
    lv_label_set_text(label_modo_info, demod_modos_texto[demod_modo]);
  }
}