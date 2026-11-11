#include "ui.h"

#include <M5Unified.h>

#include <cmath>

namespace ui {
namespace {

M5Canvas s_canvas(&M5.Display);

constexpr int kCx = 120, kCy = 120;

// Palette, all 24-bit RGB888. LovyanGFX reads a uint32_t colour as RGB888
// and a 16-bit one as RGB565, so never mix in the 16-bit TFT_* constants
// here (a ternary would promote them to uint32_t and change their colour).
constexpr uint32_t kBg = 0x000000;
constexpr uint32_t kText = 0xFFF3E0;       // warm white: actual temperature
constexpr uint32_t kSoft = 0xFFC27A;       // light orange: power, progress
constexpr uint32_t kTrack = 0x3A2412;      // dark brown: gauge track, lamp off
constexpr uint32_t kHeat = 0xFF8A00;       // orange: heating, gauge arc
constexpr uint32_t kSetColor = 0xFFC000;   // amber: setpoint, notices
constexpr uint32_t kEditColor = 0xFFF27A;  // pale yellow: setpoint being turned
constexpr uint32_t kOff = 0xFFE08A;        // warm yellow: OFF
constexpr uint32_t kAlarm = 0xFF2A1A;      // red: faults, SSR lamp on

// Ring gauge: 270 degrees, open at the bottom. lgfx arc angles are degrees
// clockwise from 3 o'clock, so 135 is the 7:30 position.
constexpr float kGaugeStart = 135.0f;
constexpr float kGaugeSweep = 270.0f;
constexpr int kRingOuter = 119, kRingInner = 110;

float gaugeAngle(float t, float max_c) {
  float f = t / max_c;
  f = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
  return kGaugeStart + kGaugeSweep * f;
}

void drawGauge(const UiState& s) {
  s_canvas.fillArc(kCx, kCy, kRingOuter, kRingInner, kGaugeStart, kGaugeStart + kGaugeSweep,
                   kTrack);
  if (!std::isnan(s.temp_c) && s.temp_c > 0.0f) {
    s_canvas.fillArc(kCx, kCy, kRingOuter, kRingInner, kGaugeStart,
                     gaugeAngle(s.temp_c, s.setpoint_max_c), s.running ? kHeat : kSoft);
  }
  // Setpoint marker.
  const float a = gaugeAngle(s.setpoint_c, s.setpoint_max_c) * DEG_TO_RAD;
  const float r = (kRingOuter + kRingInner) / 2.0f;
  const uint32_t c = s.editing ? kEditColor : kSetColor;
  s_canvas.fillCircle(kCx + r * cosf(a), kCy + r * sinf(a), 6, c);
  s_canvas.drawCircle(kCx + r * cosf(a), kCy + r * sinf(a), 6, kBg);
}

// Draw "<text>°C" centered at (x, y). The GFX fonts have no degree glyph, so
// the degree sign is drawn as a small circle.
void drawTemp(const char* text, int x, int y, const lgfx::IFont* font, uint32_t color,
              int deg_r) {
  s_canvas.setFont(font);
  s_canvas.setTextColor(color);
  s_canvas.setTextDatum(middle_left);
  const int text_w = s_canvas.textWidth(text);
  const int c_w = s_canvas.textWidth("C");
  const int gap = deg_r * 2 + 4;
  const int left = x - (text_w + gap + c_w) / 2;
  s_canvas.drawString(text, left, y);
  const int top = y - s_canvas.fontHeight() / 2;
  s_canvas.drawCircle(left + text_w + deg_r + 2, top + deg_r + 4, deg_r, color);
  s_canvas.drawString("C", left + text_w + gap, y);
}

}  // namespace

void begin() {
  M5.Display.setRotation(0);
  M5.Display.setBrightness(160);
  s_canvas.setColorDepth(16);
  s_canvas.createSprite(M5.Display.width(), M5.Display.height());
}

void draw(const UiState& s) {
  s_canvas.fillScreen(kBg);
  drawGauge(s);

  // State banner.
  s_canvas.setTextDatum(middle_center);
  s_canvas.setFont(&fonts::FreeSansBold9pt7b);
  if (s.fault) {
    s_canvas.setTextColor(kAlarm);
    s_canvas.drawString(s.fault, kCx, 46);
  } else if (s.toast) {
    s_canvas.setTextColor(kSetColor);
    s_canvas.drawString(s.toast, kCx, 46);
  } else if (s.program) {
    s_canvas.setTextColor(s.waiting ? kSetColor : kHeat);
    s_canvas.drawString(s.waiting ? "WAITING FOR KILN" : "PROGRAM", kCx, 46);
  } else if (s.tuning) {
    s_canvas.setTextColor(kHeat);
    s_canvas.drawString("STEP TEST", kCx, 46);
  } else {
    s_canvas.setTextColor(s.running ? kHeat : kOff);
    s_canvas.drawString(s.running ? "HEATING" : "OFF", kCx, 46);
  }
  if (s.simulated) {
    s_canvas.setFont(&fonts::Font2);
    s_canvas.setTextColor(kSetColor);
    s_canvas.drawString("SIMULATED", kCx, 64);
  }

  // Actual temperature.
  char buf[48];
  if (std::isnan(s.temp_c)) {
    s_canvas.setFont(&fonts::FreeSansBold24pt7b);
    s_canvas.setTextColor(kText);
    s_canvas.setTextDatum(middle_center);
    s_canvas.drawString("---", kCx, 100);
  } else {
    snprintf(buf, sizeof(buf), "%.0f", s.temp_c);
    drawTemp(buf, kCx, 100, &fonts::FreeSansBold24pt7b, kText, 4);
  }

  // Setpoint.
  if (std::isnan(s.setpoint_c)) {
    snprintf(buf, sizeof(buf), "Set ---");
  } else {
    snprintf(buf, sizeof(buf), "Set %.0f", s.setpoint_c);
  }
  drawTemp(buf, kCx, 144, &fonts::FreeSansBold12pt7b, s.editing ? kEditColor : kSetColor, 3);

  // Controller output, program progress and SSR lamp.
  s_canvas.setFont(&fonts::FreeSans9pt7b);
  s_canvas.setTextColor(kSoft);
  s_canvas.setTextDatum(middle_center);
  snprintf(buf, sizeof(buf), "Power %d%%", static_cast<int>(lroundf(s.duty * 100)));
  s_canvas.drawString(buf, kCx, 172);
  if (s.program) {
    const int left_min = static_cast<int>(s.prog_left_s / 60.0f);  // rounded down
    snprintf(buf, sizeof(buf), "Seg %d/%d  %d:%02d left", s.prog_seg, s.prog_segs, left_min / 60,
             left_min % 60);
    s_canvas.setFont(&fonts::Font2);
    s_canvas.drawString(buf, kCx, 192);
  } else if (s.detail) {
    s_canvas.setFont(&fonts::Font2);
    s_canvas.drawString(s.detail, kCx, 192);
  }

  s_canvas.fillCircle(kCx, 213, 7, s.ssr_on ? kAlarm : kTrack);

  s_canvas.pushSprite(0, 0);
}

}  // namespace ui
