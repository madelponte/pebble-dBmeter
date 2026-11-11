// dB Meter: measures the ambient sound level with the watch microphone.
//
// Audio arrives as 16 kHz mono 16-bit PCM in 20 ms frames (320 samples). Each frame is
// A-weighted (or, if the user picks Z, just has its DC removed), and its mean-square power feeds
// a "Fast" (125 ms) exponential average, the same time weighting a sound level meter uses. Levels
// are shown in dB: dBFS plus a base offset plus the user's calibration offset, since the
// microphone's absolute sensitivity varies and is not published.
//
// A session (since launch or the last reset) keeps the energy average (Leq), MIN and MAX of the
// Fast level, and the time measured.

#include <pebble.h>
#include "aweight.h"

// ---------------------------------------------------------------------------------------- tuning

#define FRAMES_PER_SEC 50
#define FAST_ALPHA 0.148f           // 1 - exp(-20 ms / 125 ms)
#define REDRAW_EVERY_FRAMES 10      // 200 ms
#define WARMUP_FRAMES 5             // filter transient after (re)start: Fast level just follows
#define SETTLE_FRAMES 15            // session stats start after this
#define DB_FLOOR_DBFS -100.0f

#define BASE_OFFSET_DB 120          // dB SPL at 0 dBFS (typical MEMS mic: -26 dBFS @ 94 dB SPL)
#define CAL_LIMIT_DB 40             // user offset range: -40 .. +40 dB

#define METER_MIN_DB 30
#define METER_MAX_DB 120
#define METER_WARN_DB 70
#define METER_DANGER_DB 85

// Persistent storage keys
#define PERSIST_CALIBRATED 4        // bool: the user has been through the Calibrate screen
#define PERSIST_CAL_OFFSET 5        // int: user calibration offset in dB
#define PERSIST_WEIGHTING 6         // int: Weighting

// ----------------------------------------------------------------------------------------- state

typedef enum {
  MeterStateStarting,
  MeterStateLive,
  MeterStatePaused,
  MeterStateInterrupted,
  MeterStateError,
} MeterState;

typedef enum {
  WeightingA = 0,
  WeightingZ = 1,
} Weighting;

typedef enum {
  ViewLive,
  ViewSession,
} View;

static Window *s_main_window;
static Layer *s_main_canvas;
static Window *s_cal_window;
static Layer *s_cal_canvas;
static Window *s_menu_window;
static MenuLayer *s_menu_layer;

static MeterState s_state = MeterStateStarting;
static const char *s_error_text;
static View s_view = ViewLive;

static Weighting s_weighting = WeightingA;
static int s_cal_offset;            // user calibration, dB
static bool s_calibrated;

static AWeightFilter s_filter;
static float s_fast_power;          // mean-square, normalised to full-scale sine = 1.0
static bool s_have_power;
static float s_fast_dbfs;
static int s_frames;                // since the mic (re)started

// Session, kept in dBFS so a calibration change applies to it too.
static double s_sess_energy;
static uint32_t s_sess_frames;
static float s_min_dbfs;
static float s_max_dbfs;

// --------------------------------------------------------------------------------------- maths

// log2 without libm: exponent from the float's bits, mantissa by a 2nd order polynomial on [1,2).
// Error < 0.005, i.e. < 0.02 dB.
static float prv_log2f(float x) {
  union { float f; uint32_t i; } u = { .f = x };
  const float exponent = (float)((int)((u.i >> 23) & 0xFF) - 127);
  u.i = (u.i & 0x007FFFFF) | 0x3F800000;
  const float m = u.f;
  return exponent + (-0.34484843f * m + 2.02466578f) * m - 1.67487759f;
}

static float prv_power_to_dbfs(float power) {
  if (power <= 1e-10f) {
    return DB_FLOOR_DBFS;
  }
  const float db = 3.01029996f * prv_log2f(power);  // 10 * log10(power)
  return db < DB_FLOOR_DBFS ? DB_FLOOR_DBFS : db;
}

static float prv_calibrated(float dbfs) {
  const float db = dbfs + (float)(BASE_OFFSET_DB + s_cal_offset);
  return db < 0 ? 0 : db;
}

static int prv_round(float v) {
  return (int)(v < 0 ? v - 0.5f : v + 0.5f);
}

static const char *prv_unit(void) {
  return s_weighting == WeightingA ? "dBA" : "dB";
}

static void prv_redraw(void) {
  if (s_main_canvas) {
    layer_mark_dirty(s_main_canvas);
  }
  if (s_cal_canvas) {
    layer_mark_dirty(s_cal_canvas);
  }
}

static void prv_reset_session(void) {
  s_sess_energy = 0;
  s_sess_frames = 0;
  prv_redraw();
}

// ----------------------------------------------------------------------------------- microphone

__attribute__((unused))
static void prv_mic_data(const int16_t *samples, size_t count, void *context) {
  if (count == 0) {
    return;
  }

  // Normalise so a full-scale sine reads 0 dBFS: mean square / (32768^2 / 2).
  float power;
  if (s_weighting == WeightingA) {
    const uint64_t sum_sq = aweight_sum_squares(&s_filter, samples, count);
    power = (float)sum_sq / (float)count /
            (536870912.0f * (float)(AWEIGHT_OUT_SCALE * AWEIGHT_OUT_SCALE));
  } else {
    int32_t sum = 0;
    for (size_t i = 0; i < count; i++) {
      sum += samples[i];
    }
    const int32_t mean = sum / (int32_t)count;
    uint64_t sum_sq = 0;
    for (size_t i = 0; i < count; i++) {
      const int32_t d = samples[i] - mean;
      sum_sq += (uint64_t)((int64_t)d * d);
    }
    power = (float)sum_sq / (float)count / 536870912.0f;
  }

  s_frames++;
  if (!s_have_power || s_frames <= WARMUP_FRAMES) {
    s_fast_power = power;
    s_have_power = true;
  } else {
    s_fast_power += FAST_ALPHA * (power - s_fast_power);
  }
  s_fast_dbfs = prv_power_to_dbfs(s_fast_power);

  if (s_frames > SETTLE_FRAMES) {
    if (s_sess_frames == 0) {
      s_min_dbfs = s_max_dbfs = s_fast_dbfs;
    } else {
      if (s_fast_dbfs < s_min_dbfs) s_min_dbfs = s_fast_dbfs;
      if (s_fast_dbfs > s_max_dbfs) s_max_dbfs = s_fast_dbfs;
    }
    s_sess_energy += power;
    s_sess_frames++;
  }

  if (s_state != MeterStateLive) {
    s_state = MeterStateLive;
    prv_redraw();
  }
  if (s_frames % REDRAW_EVERY_FRAMES == 0) {
    prv_redraw();
  }
}

__attribute__((unused))
static void prv_mic_stopped(MicCaptureStopReason reason, void *context) {
  APP_LOG(APP_LOG_LEVEL_INFO, "Mic stopped: %d", (int)reason);
  if (reason == MicCaptureStopReasonPermissionRevoked) {
    s_state = MeterStateError;
    s_error_text = "Microphone access was turned off in the Pebble app. "
                   "Turn it back on, then press Select.";
  } else {
    // Focus lost (a notification, say) or interrupted: resume when the app has focus again.
    s_state = MeterStateInterrupted;
  }
  prv_redraw();
}

static void prv_mic_start(void) {
  if (mic_capture_is_active()) {
    return;
  }
  s_have_power = false;
  s_frames = 0;
  aweight_reset(&s_filter);

  const MicCaptureStartResult result = mic_capture_start((MicCaptureHandlers) {
    .data = prv_mic_data,
    .stopped = prv_mic_stopped,
  }, NULL);
  APP_LOG(APP_LOG_LEVEL_INFO, "mic_capture_start: %d", (int)result);

  switch (result) {
    case MicCaptureStartOk:
      s_state = MeterStateStarting;
      s_error_text = NULL;
      break;
    case MicCaptureStartErrPermissionDenied:
      s_state = MeterStateError;
      s_error_text = "Allow microphone access for dB Meter in the Pebble app, then press Select.";
      break;
    case MicCaptureStartErrBusy:
      s_state = MeterStateError;
      s_error_text = "The microphone is in use. Press Select to try again.";
      break;
    case MicCaptureStartErrUnavailable:
      s_state = MeterStateError;
      s_error_text = "This watch has no microphone access for apps.";
      break;
    case MicCaptureStartErrNotForeground:
      s_state = MeterStateInterrupted;
      break;
    default:
      s_state = MeterStateError;
      s_error_text = "Could not start the microphone. Press Select to retry.";
      break;
  }
  prv_redraw();
}

static void prv_focus_changed(bool in_focus) {
  if (in_focus && s_state == MeterStateInterrupted) {
    prv_mic_start();
  }
}

static void prv_set_weighting(Weighting weighting) {
  if (weighting == s_weighting) {
    return;
  }
  s_weighting = weighting;
  persist_write_int(PERSIST_WEIGHTING, s_weighting);
  // A session mixing weightings would mean nothing; start the level and the session afresh.
  aweight_reset(&s_filter);
  s_have_power = false;
  s_frames = 0;
  prv_reset_session();
}

// ------------------------------------------------------------------------------ shared drawing

typedef struct {
  bool big;
  GFont number_font;
  int number_h;
  GFont unit_font;
  GFont status_font;
  int status_h;
} Fonts;

static Fonts prv_fonts(GRect bounds) {
  Fonts f;
#if defined(FONT_KEY_LECO_60_NUMBERS_AM_PM)
  f.big = bounds.size.w >= 180;
  f.number_font = fonts_get_system_font(f.big ? FONT_KEY_LECO_60_NUMBERS_AM_PM
                                              : FONT_KEY_LECO_42_NUMBERS);
#else
  f.big = false;
  f.number_font = fonts_get_system_font(FONT_KEY_LECO_42_NUMBERS);
#endif
  f.number_h = f.big ? 60 : 42;
  f.unit_font = fonts_get_system_font(f.big ? FONT_KEY_GOTHIC_28_BOLD : FONT_KEY_GOTHIC_24_BOLD);
  f.status_font = fonts_get_system_font(f.big ? FONT_KEY_GOTHIC_24_BOLD : FONT_KEY_GOTHIC_18_BOLD);
  f.status_h = f.big ? 28 : 22;
  return f;
}

static GColor prv_level_color(float db) {
#if defined(PBL_COLOR)
  if (db >= METER_DANGER_DB) return GColorRed;
  if (db >= METER_WARN_DB) return GColorYellow;
  return GColorGreen;
#else
  return GColorWhite;
#endif
}

static void prv_draw_text(GContext *ctx, const char *text, GFont font, GRect box,
                          GTextAlignment align, GColor color) {
  graphics_context_set_text_color(ctx, color);
  graphics_draw_text(ctx, text, font, box, GTextOverflowModeTrailingEllipsis, align, NULL);
}

static void prv_draw_message(GContext *ctx, GRect area, const char *title, const char *body) {
  const int inset = PBL_IF_ROUND_ELSE(area.size.w / 5, 8);
  GRect box = grect_inset(area, GEdgeInsets(0, inset));
  GFont title_font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  GFont body_font = fonts_get_system_font(FONT_KEY_GOTHIC_18);
  GSize body_size = graphics_text_layout_get_content_size(
      body, body_font, GRect(0, 0, box.size.w, 1000), GTextOverflowModeWordWrap, GTextAlignmentCenter);
  const int total = 28 + body_size.h;
  int y = box.origin.y + (box.size.h - total) / 2;
  prv_draw_text(ctx, title, title_font, GRect(box.origin.x, y - 4, box.size.w, 30),
                GTextAlignmentCenter, GColorWhite);
  graphics_context_set_text_color(ctx, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  graphics_draw_text(ctx, body, body_font, GRect(box.origin.x, y + 28, box.size.w, body_size.h + 4),
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
}

// A big "NN dBA" reading, centred on center_x, top at y.
static void prv_draw_reading(GContext *ctx, const Fonts *f, int center_x, int y, GColor color,
                             bool have, float db) {
  char number[8];
  if (have) {
    snprintf(number, sizeof(number), "%d", prv_round(db));
  } else {
    strcpy(number, "--");
  }
  const char *unit = prv_unit();
  const GSize num_size = graphics_text_layout_get_content_size(
      number, f->number_font, GRect(0, 0, 300, f->number_h + 20),
      GTextOverflowModeFill, GTextAlignmentLeft);
  const GSize unit_size = graphics_text_layout_get_content_size(
      unit, f->unit_font, GRect(0, 0, 80, 40), GTextOverflowModeFill, GTextAlignmentLeft);
  const int unit_gap = 4;
  const int row_x = center_x - (num_size.w + unit_gap + unit_size.w) / 2;
  // LECO glyphs sit low in their box; pull them up so the box top matches the digit tops.
  prv_draw_text(ctx, number, f->number_font,
                GRect(row_x, y - (f->big ? 8 : 6), num_size.w + 4, f->number_h + 20),
                GTextAlignmentLeft, color);
  prv_draw_text(ctx, unit, f->unit_font,
                GRect(row_x + num_size.w + unit_gap, y + f->number_h - unit_size.h - (f->big ? 2 : 0),
                      unit_size.w + 4, unit_size.h + 6),
                GTextAlignmentLeft, PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
}

static void prv_format_duration(char *buf, size_t size, uint32_t secs) {
  if (secs >= 3600) {
    snprintf(buf, size, "%d:%02d:%02d", (int)(secs / 3600), (int)(secs / 60 % 60), (int)(secs % 60));
  } else {
    snprintf(buf, size, "%d:%02d", (int)(secs / 60), (int)(secs % 60));
  }
}

// ---------------------------------------------------------------------------------- main window

// The status pill shared by both views; `normal` is shown when nothing more pressing applies.
static void prv_status(const char *normal, const char **text, GColor *color) {
  if (s_state == MeterStatePaused) {
    *text = "PAUSED";
    *color = PBL_IF_COLOR_ELSE(GColorChromeYellow, GColorWhite);
  } else if (s_state == MeterStateStarting || s_state == MeterStateInterrupted) {
    *text = "STARTING";
    *color = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
  } else if (!s_calibrated) {
    *text = "UNCALIBRATED";
    *color = PBL_IF_COLOR_ELSE(GColorChromeYellow, GColorWhite);
  } else {
    *text = normal;
    *color = GColorWhite;
  }
}

#define HEADER_TOP PBL_IF_ROUND_ELSE(10, 3)

static int prv_header_pill_h(const Fonts *f) {
  return f->big ? 24 : 20;
}

// Height the header takes in the vertical stack, including the space below the pill.
static int prv_header_h(const Fonts *f) {
  return HEADER_TOP + prv_header_pill_h(f) + (f->big ? 5 : 1);
}

static void prv_draw_header(GContext *ctx, GRect bounds, int y, const Fonts *f,
                            const char *normal) {
  const char *status;
  GColor status_color;
  prv_status(normal, &status, &status_color);

  GFont font = fonts_get_system_font(f->big ? FONT_KEY_GOTHIC_18_BOLD
                                            : FONT_KEY_GOTHIC_14_BOLD);
  const int h = prv_header_pill_h(f);
  const int max_w = bounds.size.w - 2 * PBL_IF_ROUND_ELSE(bounds.size.w / 5, 8);
  GSize size = graphics_text_layout_get_content_size(
      status, font, GRect(0, 0, max_w, h), GTextOverflowModeTrailingEllipsis, GTextAlignmentCenter);
  int w = size.w + (f->big ? 20 : 14);
  if (w > max_w) w = max_w;
  const GRect pill = GRect((bounds.size.w - w) / 2, y + HEADER_TOP, w, h);

  graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(GColorDarkGray, GColorBlack));
  graphics_fill_rect(ctx, pill, h / 2, GCornersAll);
#if !defined(PBL_COLOR)
  graphics_context_set_stroke_color(ctx, GColorWhite);
  graphics_draw_round_rect(ctx, pill, h / 2);
#endif
  prv_draw_text(ctx, status, font, GRect(pill.origin.x, pill.origin.y - 2, pill.size.w, h + 3),
                GTextAlignmentCenter, status_color);
}

static const char *prv_level_description(float db) {
  if (db < 55) return "QUIET";
  if (db < METER_WARN_DB) return "MODERATE";
  if (db < METER_DANGER_DB) return "LOUD";
  if (db < 100) return "VERY LOUD";
  return "EXTREME";
}

static void prv_draw_segment_meter(GContext *ctx, GRect bar, bool active, bool have_level,
                                   float level_db, bool have_max, float max_db) {
  const int segments = 18;
  const int gap = bar.size.w >= 150 ? 2 : 1;
  const int segment_space = bar.size.w - gap * (segments - 1);
  float frac = (level_db - METER_MIN_DB) / (float)(METER_MAX_DB - METER_MIN_DB);
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  const int lit = have_level ? (int)(frac * segments + 0.999f) : 0;

  for (int i = 0; i < segments; i++) {
    const int segment_x = i * segment_space / segments;
    const int next_x = (i + 1) * segment_space / segments;
    const int x = bar.origin.x + segment_x + i * gap;
    const int w = next_x - segment_x;
    if (i < lit) {
      const float segment_db = METER_MIN_DB +
          (i + 0.5f) * (METER_MAX_DB - METER_MIN_DB) / segments;
      graphics_context_set_fill_color(ctx, active ? prv_level_color(segment_db)
                                                  : PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
      graphics_fill_rect(ctx, GRect(x, bar.origin.y, w, bar.size.h), 2, GCornersAll);
    } else {
#if defined(PBL_COLOR)
      graphics_context_set_fill_color(ctx, GColorDarkGray);
      graphics_fill_rect(ctx, GRect(x, bar.origin.y, w, bar.size.h), 2, GCornersAll);
#else
      // Unlit on black and white: a thin baseline rather than outlined boxes.
      graphics_context_set_fill_color(ctx, GColorWhite);
      graphics_fill_rect(ctx, GRect(x, bar.origin.y + bar.size.h - 2, w, 2), 0, GCornerNone);
#endif
    }
  }

  if (have_max) {
    float max_frac = (max_db - METER_MIN_DB) / (float)(METER_MAX_DB - METER_MIN_DB);
    if (max_frac >= 0 && max_frac <= 1) {
      const int x = bar.origin.x + (int)(max_frac * (bar.size.w - 1));
      graphics_context_set_stroke_color(ctx, GColorWhite);
      graphics_draw_line(ctx, GPoint(x, bar.origin.y - 3), GPoint(x, bar.origin.y + bar.size.h + 2));
    }
  }
}

static void prv_draw_metric_card(GContext *ctx, GRect card, const char *label, const char *value,
                                 bool big) {
  graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(GColorDarkGray, GColorBlack));
  graphics_fill_rect(ctx, card, 4, GCornersAll);
#if !defined(PBL_COLOR)
  graphics_context_set_stroke_color(ctx, GColorWhite);
  graphics_draw_round_rect(ctx, card, 4);
#endif
  const GColor grey = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);
  prv_draw_text(ctx, label, fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD),
                GRect(card.origin.x, card.origin.y - (big ? 1 : 2), card.size.w, 18),
                GTextAlignmentCenter, grey);
  prv_draw_text(ctx, value,
                fonts_get_system_font(big ? FONT_KEY_GOTHIC_24_BOLD : FONT_KEY_GOTHIC_18_BOLD),
                GRect(card.origin.x, card.origin.y + (big ? 11 : 9), card.size.w, big ? 30 : 22),
                GTextAlignmentCenter, GColorWhite);
}

static void prv_draw_live(Layer *layer, GContext *ctx, GRect bounds, GRect area, const Fonts *f,
                          bool has_accessory, GRect accessory) {
  (void)layer;
  const bool live = (s_state == MeterStateLive);
  const float level_db = prv_calibrated(s_fast_dbfs);
  const bool have_max = s_sess_frames > 0;
  const int header_h = prv_header_h(f);
  const int desc_h = f->big ? 23 : 19;
  const int bar_h = f->big ? 13 : 10;
  const int scale_h = 17;
  const int footer_h = has_accessory ? 0 : (f->big ? 24 : 20);
  const int gap = f->big ? 5 : 3;
  // Must match the y advances below.
  const int total = header_h + f->number_h + gap + desc_h + gap + bar_h + 1 + scale_h +
                    (footer_h ? gap + footer_h : 0);
  int y = area.origin.y + (area.size.h - total) / 2;
  if (y < area.origin.y) y = area.origin.y;

  prv_draw_header(ctx, bounds, y, f, "LIVE");
  y += header_h;

  prv_draw_reading(ctx, f, bounds.size.w / 2, y,
                   live ? GColorWhite : PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite),
                   s_have_power, level_db);
  y += f->number_h + gap;

  // The loudness words describe hearing, so they only apply to A-weighted levels.
  const bool describe = s_have_power && s_weighting == WeightingA;
  const char *description = !s_have_power ? "MEASURING"
                          : describe      ? prv_level_description(level_db)
                                          : "UNWEIGHTED";
  prv_draw_text(ctx, description, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                GRect(0, y - 3, bounds.size.w, desc_h + 4), GTextAlignmentCenter,
                describe && live ? prv_level_color(level_db)
                                 : PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  y += desc_h + gap;

  const int side = PBL_IF_ROUND_ELSE(bounds.size.w / 6, f->big ? 14 : 10);
  const GRect bar = GRect(side, y, bounds.size.w - 2 * side, bar_h);
  prv_draw_segment_meter(ctx, bar, live, s_have_power, level_db, have_max,
                         prv_calibrated(s_max_dbfs));
  y += bar_h + 1;

  GFont scale_font = fonts_get_system_font(FONT_KEY_GOTHIC_14);
  static const int s_ticks[] = { METER_MIN_DB, METER_WARN_DB, METER_DANGER_DB, METER_MAX_DB };
  for (unsigned i = 0; i < ARRAY_LENGTH(s_ticks); i++) {
    const float tick_frac = (s_ticks[i] - METER_MIN_DB) / (float)(METER_MAX_DB - METER_MIN_DB);
    const int x = bar.origin.x + (int)(tick_frac * bar.size.w);
    char label[4];
    snprintf(label, sizeof(label), "%d", s_ticks[i]);
    int label_x = x - 14;
    if (i == 0) label_x = bar.origin.x - 2;
    if (i == ARRAY_LENGTH(s_ticks) - 1) label_x = bar.origin.x + bar.size.w - 27;
    prv_draw_text(ctx, label, scale_font, GRect(label_x, y - 3, 28, scale_h + 3),
                  i == 0 ? GTextAlignmentLeft :
                  (i == ARRAY_LENGTH(s_ticks) - 1 ? GTextAlignmentRight : GTextAlignmentCenter),
                  PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  }
  y += scale_h;

  char max_text[16];
  if (have_max) {
    snprintf(max_text, sizeof(max_text), "MAX %d %s", prv_round(prv_calibrated(s_max_dbfs)),
             prv_unit());
  } else {
    strcpy(max_text, "MAX --");
  }
  if (has_accessory) {
    prv_draw_text(ctx, max_text, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                  GRect(accessory.origin.x, accessory.origin.y + (accessory.size.h - 22) / 2 - 2,
                        accessory.size.w - 6, 24), GTextAlignmentRight, GColorWhite);
  } else {
    y += gap;
    prv_draw_text(ctx, max_text, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                  GRect(0, y - 3, bounds.size.w, footer_h + 3), GTextAlignmentCenter, GColorWhite);
  }
}

static void prv_draw_session(Layer *layer, GContext *ctx, GRect bounds, GRect area, const Fonts *f,
                             bool has_accessory, GRect accessory) {
  (void)layer;
  const bool have = s_sess_frames > 0;
  const bool show_now = (s_state == MeterStateLive);
  // Sized to fit the area left by the Listening indicator: 138 px on flint, 190 px on emery.
  const int header_h = prv_header_h(f);
  const int label_h = f->big ? 18 : 14;
  const int card_h = f->big ? 40 : 30;
  const int time_h = f->big ? 22 : 18;
  const int gap = f->big ? 5 : 3;
  // Must match the y advances below.
  const int total = header_h + f->number_h + gap + label_h + gap + card_h + gap + time_h;
  int y = area.origin.y + (area.size.h - total) / 2;
  if (y < area.origin.y) y = area.origin.y;

  prv_draw_header(ctx, bounds, y, f, "SESSION");
  y += header_h;

  const float leq_dbfs = have ? prv_power_to_dbfs((float)(s_sess_energy / (double)s_sess_frames))
                              : DB_FLOOR_DBFS;
  prv_draw_reading(ctx, f, bounds.size.w / 2, y, GColorWhite, have, prv_calibrated(leq_dbfs));
  y += f->number_h + gap;
  prv_draw_text(ctx, "LEQ AVERAGE", fonts_get_system_font(FONT_KEY_GOTHIC_14_BOLD),
                GRect(0, y - 3, bounds.size.w, label_h + 3), GTextAlignmentCenter,
                PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));
  y += label_h + gap;

  const int side = PBL_IF_ROUND_ELSE(bounds.size.w / 6, f->big ? 14 : 9);
  const int card_gap = f->big ? 8 : 5;
  const int card_w = (bounds.size.w - 2 * side - card_gap) / 2;
  char min_value[12];
  char max_value[12];
  if (have) {
    snprintf(min_value, sizeof(min_value), "%d %s", prv_round(prv_calibrated(s_min_dbfs)), prv_unit());
    snprintf(max_value, sizeof(max_value), "%d %s", prv_round(prv_calibrated(s_max_dbfs)), prv_unit());
  } else {
    strcpy(min_value, "--");
    strcpy(max_value, "--");
  }
  prv_draw_metric_card(ctx, GRect(side, y, card_w, card_h), "MIN", min_value, f->big);
  prv_draw_metric_card(ctx, GRect(side + card_w + card_gap, y, card_w, card_h),
                       "MAX", max_value, f->big);
  y += card_h + gap;

  // Time measured; the current level goes beside the Listening indicator, or on this row.
  char duration[12];
  char now[16];
  char time_text[32];
  prv_format_duration(duration, sizeof(duration), s_sess_frames / FRAMES_PER_SEC);
  snprintf(now, sizeof(now), "NOW %d", prv_round(prv_calibrated(s_fast_dbfs)));
  if (show_now && !has_accessory) {
    snprintf(time_text, sizeof(time_text), "TIME %s   %s", duration, now);
  } else {
    snprintf(time_text, sizeof(time_text), "TIME  %s", duration);
  }
  prv_draw_text(ctx, time_text, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                GRect(0, y - 3, bounds.size.w, time_h + 3), GTextAlignmentCenter,
                PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite));

  if (show_now && has_accessory) {
    prv_draw_text(ctx, now, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD),
                  GRect(accessory.origin.x, accessory.origin.y + (accessory.size.h - 22) / 2 - 2,
                        accessory.size.w - 6, 24), GTextAlignmentRight, GColorWhite);
  }
}

static void prv_main_update(Layer *layer, GContext *ctx) {
  const GRect bounds = layer_get_bounds(layer);
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  const GRect area = layer_get_unobstructed_bounds(layer);
  if (s_state == MeterStateError) {
    prv_draw_message(ctx, area, "dB Meter", s_error_text ? s_error_text : "");
    return;
  }

  const Fonts f = prv_fonts(bounds);
  GRect accessory;
  const bool has_accessory = layer_get_mic_accessory_bounds(layer, &accessory) &&
                             accessory.size.w > 40;
  if (s_view == ViewSession) {
    prv_draw_session(layer, ctx, bounds, area, &f, has_accessory, accessory);
  } else {
    prv_draw_live(layer, ctx, bounds, area, &f, has_accessory, accessory);
  }
}

// --------------------------------------------------------------------------- calibrate window
//
// Shows the live reading with the offset applied, so the user can hold the watch beside a
// reference meter and press Up/Down until both read the same.

static void prv_cal_update(Layer *layer, GContext *ctx) {
  const GRect bounds = layer_get_bounds(layer);
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  const GRect area = layer_get_unobstructed_bounds(layer);
  if (s_state == MeterStateError) {
    prv_draw_message(ctx, area, "Calibrate", s_error_text ? s_error_text : "");
    return;
  }

  const Fonts f = prv_fonts(bounds);
  GFont hint_font = fonts_get_system_font(f.big ? FONT_KEY_GOTHIC_18 : FONT_KEY_GOTHIC_14);
  GFont offset_font = fonts_get_system_font(f.big ? FONT_KEY_GOTHIC_28_BOLD : FONT_KEY_GOTHIC_24_BOLD);
  const GColor grey = PBL_IF_COLOR_ELSE(GColorLightGray, GColorWhite);

  // + and - on the right edge, level with the Up and Down buttons.
  const int hint_w = 20;
  const int hint_x = bounds.size.w - hint_w - PBL_IF_ROUND_ELSE(22, 2);
  const int up_y = PBL_IF_ROUND_ELSE(bounds.size.h * 30 / 100, bounds.size.h * 22 / 100);
  const int down_y = PBL_IF_ROUND_ELSE(bounds.size.h * 70 / 100, bounds.size.h * 72 / 100);
  const int arm = f.big ? 7 : 6;      // half-length of the + and - strokes
  const int hcx = hint_x + hint_w / 2;
  graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(GColorCyan, GColorWhite));
  graphics_fill_rect(ctx, GRect(hcx - arm, up_y - 1, 2 * arm + 1, 3), 0, GCornerNone);
  graphics_fill_rect(ctx, GRect(hcx - 1, up_y - arm, 3, 2 * arm + 1), 0, GCornerNone);
  graphics_fill_rect(ctx, GRect(hcx - arm, down_y - 1, 2 * arm + 1, 3), 0, GCornerNone);

  // Content column, left of the hints.
  const int left = PBL_IF_ROUND_ELSE(bounds.size.w / 8, 4);
  const int right = hint_x - 2;
  const int col_w = right - left;
  const int cx = left + col_w / 2;

  const int title_h = f.status_h;
  const int offset_h = f.big ? 32 : 28;
  const int hint_h = f.big ? 40 : 32;
  const int gap = f.big ? 8 : 4;
  const int total = title_h + gap + f.number_h + gap + offset_h + gap + hint_h;
  int y = area.origin.y + (area.size.h - total) / 2;
  if (y < area.origin.y) {
    y = area.origin.y;
  }

  prv_draw_text(ctx, "CALIBRATE", f.status_font, GRect(left, y - (f.big ? 6 : 4), col_w, title_h + 6),
                GTextAlignmentCenter, PBL_IF_COLOR_ELSE(GColorCyan, GColorWhite));
  y += title_h + gap;

  prv_draw_reading(ctx, &f, cx, y, GColorWhite, s_have_power, prv_calibrated(s_fast_dbfs));
  y += f.number_h + gap;

  char offset_text[24];
  if (s_cal_offset == 0) {
    strcpy(offset_text, "Offset 0 dB");
  } else {
    snprintf(offset_text, sizeof(offset_text), "Offset %+d dB", s_cal_offset);
  }
  prv_draw_text(ctx, offset_text, offset_font, GRect(left, y - 4, col_w, offset_h + 6),
                GTextAlignmentCenter, GColorWhite);
  y += offset_h + gap;

  const char *hint;
  if (s_weighting == WeightingA) {
    hint = f.big ? "Match a reference meter set to dBA. Hold Select to zero."
                 : "Match a dBA meter.\nHold Select to zero.";
  } else {
    hint = f.big ? "Match a reference meter set to dBZ. Hold Select to zero."
                 : "Match a dBZ meter.\nHold Select to zero.";
  }
  graphics_context_set_text_color(ctx, grey);
  graphics_draw_text(ctx, hint, hint_font, GRect(left, y - 4, col_w, hint_h + 8),
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, NULL);
}

static void prv_cal_set_offset(int offset) {
  if (offset < -CAL_LIMIT_DB) offset = -CAL_LIMIT_DB;
  if (offset > CAL_LIMIT_DB) offset = CAL_LIMIT_DB;
  s_cal_offset = offset;
  persist_write_int(PERSIST_CAL_OFFSET, s_cal_offset);
  prv_redraw();
}

static void prv_cal_up(ClickRecognizerRef recognizer, void *context) {
  prv_cal_set_offset(s_cal_offset + 1);
}

static void prv_cal_down(ClickRecognizerRef recognizer, void *context) {
  prv_cal_set_offset(s_cal_offset - 1);
}

static void prv_cal_done(ClickRecognizerRef recognizer, void *context) {
  window_stack_pop(true);
}

static void prv_cal_zero(ClickRecognizerRef recognizer, void *context) {
  prv_cal_set_offset(0);
  vibes_short_pulse();
}

static void prv_cal_click_config(void *context) {
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 120, prv_cal_up);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 120, prv_cal_down);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_cal_done);
  window_long_click_subscribe(BUTTON_ID_SELECT, 700, prv_cal_zero, NULL);
}

static void prv_cal_load(Window *window) {
  Layer *root = window_get_root_layer(window);
  s_cal_canvas = layer_create(layer_get_bounds(root));
  layer_set_update_proc(s_cal_canvas, prv_cal_update);
  layer_add_child(root, s_cal_canvas);
}

static void prv_cal_unload(Window *window) {
  layer_destroy(s_cal_canvas);
  s_cal_canvas = NULL;
  // Having been here counts as calibrated, even if the offset stayed at 0.
  s_calibrated = true;
  persist_write_bool(PERSIST_CALIBRATED, true);
  window_destroy(s_cal_window);
  s_cal_window = NULL;
  if (s_menu_layer) {
    menu_layer_reload_data(s_menu_layer);
  }
}

static void prv_open_calibration(void) {
  if (s_cal_window) {
    return;
  }
  if (s_state == MeterStatePaused) {
    prv_mic_start();  // calibrating needs a live reading
  }
  s_cal_window = window_create();
  window_set_background_color(s_cal_window, GColorBlack);
  window_set_click_config_provider(s_cal_window, prv_cal_click_config);
  window_set_window_handlers(s_cal_window, (WindowHandlers) {
    .load = prv_cal_load,
    .unload = prv_cal_unload,
  });
  window_stack_push(s_cal_window, true);
}

// ----------------------------------------------------------------------------- settings menu

enum {
  MenuRowWeighting,
  MenuRowCalibrate,
  MenuRowReset,
  MenuRowCount,
};

static uint16_t prv_menu_num_rows(MenuLayer *menu, uint16_t section, void *context) {
  return MenuRowCount;
}

static void prv_menu_draw_row(GContext *ctx, const Layer *cell, MenuIndex *index, void *context) {
  static char s_subtitle[24];
  switch (index->row) {
    case MenuRowWeighting:
      menu_cell_basic_draw(ctx, cell, "Weighting",
                           s_weighting == WeightingA ? "A: dBA (hearing)" : "Z: flat, unweighted",
                           NULL);
      break;
    case MenuRowCalibrate:
      if (s_cal_offset == 0) {
        strcpy(s_subtitle, "Offset 0 dB");
      } else {
        snprintf(s_subtitle, sizeof(s_subtitle), "Offset %+d dB", s_cal_offset);
      }
      menu_cell_basic_draw(ctx, cell, "Calibrate", s_subtitle, NULL);
      break;
    case MenuRowReset:
      menu_cell_basic_draw(ctx, cell, "Reset session", "Avg, min, max, time", NULL);
      break;
  }
}

static void prv_menu_select(MenuLayer *menu, MenuIndex *index, void *context) {
  switch (index->row) {
    case MenuRowWeighting:
      prv_set_weighting(s_weighting == WeightingA ? WeightingZ : WeightingA);
      menu_layer_reload_data(menu);
      break;
    case MenuRowCalibrate:
      prv_open_calibration();
      break;
    case MenuRowReset:
      prv_reset_session();
      vibes_short_pulse();
      window_stack_pop(true);
      break;
  }
}

static void prv_menu_load(Window *window) {
  Layer *root = window_get_root_layer(window);
  s_menu_layer = menu_layer_create(layer_get_unobstructed_bounds(root));
  menu_layer_set_callbacks(s_menu_layer, NULL, (MenuLayerCallbacks) {
    .get_num_rows = prv_menu_num_rows,
    .draw_row = prv_menu_draw_row,
    .select_click = prv_menu_select,
  });
  menu_layer_set_normal_colors(s_menu_layer, GColorBlack, GColorWhite);
  menu_layer_set_highlight_colors(s_menu_layer, PBL_IF_COLOR_ELSE(GColorCobaltBlue, GColorWhite),
                                  PBL_IF_COLOR_ELSE(GColorWhite, GColorBlack));
  menu_layer_set_click_config_onto_window(s_menu_layer, window);
  layer_add_child(root, menu_layer_get_layer(s_menu_layer));
}

static void prv_menu_unload(Window *window) {
  menu_layer_destroy(s_menu_layer);
  s_menu_layer = NULL;
  window_destroy(s_menu_window);
  s_menu_window = NULL;
}

static void prv_open_menu(void) {
  if (s_menu_window) {
    return;
  }
  s_menu_window = window_create();
  window_set_background_color(s_menu_window, GColorBlack);
  window_set_window_handlers(s_menu_window, (WindowHandlers) {
    .load = prv_menu_load,
    .unload = prv_menu_unload,
  });
  window_stack_push(s_menu_window, true);
}

// ------------------------------------------------------------------------------ main buttons

static void prv_main_up(ClickRecognizerRef recognizer, void *context) {
  if (s_state != MeterStateError) {
    prv_open_menu();
  }
}

static void prv_main_down(ClickRecognizerRef recognizer, void *context) {
  if (s_state != MeterStateError) {
    s_view = (s_view == ViewLive) ? ViewSession : ViewLive;
    prv_redraw();
  }
}

static void prv_main_select(ClickRecognizerRef recognizer, void *context) {
  switch (s_state) {
    case MeterStatePaused:
    case MeterStateError:
    case MeterStateInterrupted:
      prv_mic_start();
      break;
    default:
      mic_capture_stop();
      s_state = MeterStatePaused;
      prv_redraw();
      break;
  }
}

static void prv_main_reset(ClickRecognizerRef recognizer, void *context) {
  prv_reset_session();
  vibes_short_pulse();
}

static void prv_main_click_config(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, prv_main_up);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_main_down);
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_main_select);
  window_long_click_subscribe(BUTTON_ID_SELECT, 700, prv_main_reset, NULL);
}

// --------------------------------------------------------------------------------- app set-up

// The Listening indicator obstructs the bottom of the screen while the mic runs.
__attribute__((unused))
static void prv_unobstructed_change(AnimationProgress progress, void *context) {
  prv_redraw();
  if (s_menu_layer) {
    Layer *root = window_get_root_layer(s_menu_window);
    layer_set_frame(menu_layer_get_layer(s_menu_layer), layer_get_unobstructed_bounds(root));
  }
}

__attribute__((unused))
static void prv_unobstructed_did_change(void *context) {
  prv_unobstructed_change(ANIMATION_NORMALIZED_MAX, context);
}

static void prv_main_load(Window *window) {
  Layer *root = window_get_root_layer(window);
  s_main_canvas = layer_create(layer_get_bounds(root));
  layer_set_update_proc(s_main_canvas, prv_main_update);
  layer_add_child(root, s_main_canvas);

  unobstructed_area_service_subscribe((UnobstructedAreaHandlers) {
    .change = prv_unobstructed_change,
    .did_change = prv_unobstructed_did_change,
  }, NULL);
}

static void prv_main_unload(Window *window) {
  unobstructed_area_service_unsubscribe();
  layer_destroy(s_main_canvas);
  s_main_canvas = NULL;
}

static void prv_init(void) {
  if (persist_exists(PERSIST_CAL_OFFSET)) {
    s_cal_offset = persist_read_int(PERSIST_CAL_OFFSET);
    if (s_cal_offset < -CAL_LIMIT_DB || s_cal_offset > CAL_LIMIT_DB) {
      s_cal_offset = 0;
    }
  }
  s_calibrated = persist_read_bool(PERSIST_CALIBRATED);
  if (persist_exists(PERSIST_WEIGHTING)) {
    s_weighting = (persist_read_int(PERSIST_WEIGHTING) == WeightingZ) ? WeightingZ : WeightingA;
  }

  s_main_window = window_create();
  window_set_background_color(s_main_window, GColorBlack);
  window_set_click_config_provider(s_main_window, prv_main_click_config);
  window_set_window_handlers(s_main_window, (WindowHandlers) {
    .load = prv_main_load,
    .unload = prv_main_unload,
  });
  window_stack_push(s_main_window, true);

  app_focus_service_subscribe(prv_focus_changed);
  prv_mic_start();
}

static void prv_deinit(void) {
  mic_capture_stop();
  app_focus_service_unsubscribe();
  window_destroy(s_main_window);
}

int main(void) {
  prv_init();
  app_event_loop();
  prv_deinit();
}
