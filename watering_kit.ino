#include <Wire.h>
#include "U8glib.h"
#include "Wire.h"
#include "RTClib.h"

// Seems to be 132x64 SH1106 actually.
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);    // I2C

RTC_DS1307 RTC;

#define NFLOWERS  4
struct flower
{
  bool watering;            // are we in a watering session right now ?
  bool valve_open;          // are we actively watering (valve is open) ?
  bool faulted;             // have we detected a fault either with sensor or valve or pipe ?
  int moisture_cur;         // current moisture level
  int moisture_max;         // maximum moisture level seen during current watering session
  unsigned long last_sensor_update;// timestamp of last moisture level check
  unsigned long last_increase_ts;  // timestamp of the last increase in moisture level since watering started
  unsigned long phase_start; // timestamp of when we last opened or closed valve during current watering session
  int relay_pin;
  int sensor_pin;
  int sensor_min_val;
  int sensor_max_val;
} flowers[NFLOWERS];

// Watering is done by periodically opening and closing a valve.
// The pause is to let the water soak into the soil.
// Otheriwse we could be pumping water too fast and put in too much of it
// before a change in the moisture level is detected.
// So, we pump the water in short bursts with longer pauses between them.
// The standard pump in the kit has flow of 2 liters per minute (120 l/h).
// I want to emulate 1 l/h, so the duty cycle should be 1/120.
// Also, I want to limit a single portion of water to no more than 50 ml.
// ActiveWateringPeriod and PauseWateringPeriod are derived from these constraints.
// NB: these calculations assume that the flow of water is unrestricted.
// If there are any drippers, etc, the the flow would be different.

// How long to have a valve open, milliseconds.
// Equivalent to ~ 50 ml at 120 l/h (33 ml/s).
// NB: this does not really account for several valves being open at the same time.
// In practice that should be rare and it should only result in slower watering.
// The pots should still get as much water as they need to reach the moisture levels.
const unsigned long ActiveWateringPeriod = 1500;

// How long to have a valve closed before opening again, milliseconds.
// 1/120 duty cycle to emulate 1 l/h flow.
const unsigned long PauseWateringPeriod = 180000 - ActiveWateringPeriod;

// If, while watering, moisture level does not increase for this long, then declare a fault.
// The value is calculated for 200 ml at (emulated) 1 l/h resulting in 12 minutes.
const unsigned long FaultTimeout = 720000;

const unsigned long IdleUpdatePeriod = 60000;    // period between checking moisture levels while not watering, milliseconds
const unsigned long ActiveUpdatePeriod = 1000;   // period between checking moisture levels while watering, milliseconds

const unsigned long SerialReportPeriod = 600000; // how often to update information on via the serial connection
unsigned long last_report;                       // time of the last screen refresh milliseconds

const unsigned long ScreenRefreshPeriod = 2000;  // how often to update information on the screen,
unsigned long last_refresh;                      // time of the last screen refresh milliseconds
bool force_screen_refresh = false;               // whether to force a screen refresh after the alternative display

const int PumpStartDelay = 100;                  // how long to wait after opening a valve before starting the pump, ms
bool pump_active;

// Watering hysteresis.
const int MoistureLowThreshold = 30;             // start watering when moisture level falls below this threshold
const int MoistureHighThreshold = 50;            // stop watering when moisture level raises above this threshold

const int PumpPin = 4;
const int ButtonPin = 12;

const char DaysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

const int BitmapWidth = 32;
const int BitmapHeight = 30;

// good flower
const unsigned char BitmapGood[] U8G_PROGMEM = {
  0x00, 0x42, 0x4C, 0x00, 0x00, 0xE6, 0x6E, 0x00, 0x00, 0xAE, 0x7B, 0x00, 0x00, 0x3A, 0x51, 0x00,
  0x00, 0x12, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x06, 0x40, 0x00, 0x00, 0x06, 0x40, 0x00,
  0x00, 0x04, 0x60, 0x00, 0x00, 0x0C, 0x20, 0x00, 0x00, 0x08, 0x30, 0x00, 0x00, 0x18, 0x18, 0x00,
  0x00, 0xE0, 0x0F, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0xC1, 0x00, 0x00, 0x0E, 0x61, 0x00,
  0x00, 0x1C, 0x79, 0x00, 0x00, 0x34, 0x29, 0x00, 0x00, 0x28, 0x35, 0x00, 0x00, 0x48, 0x17, 0x00,
  0x00, 0xD8, 0x1B, 0x00, 0x00, 0x90, 0x1B, 0x00, 0x00, 0xB0, 0x09, 0x00, 0x00, 0xA0, 0x05, 0x00,
  0x00, 0xE0, 0x07, 0x00, 0x00, 0xC0, 0x03, 0x00
};

// bad flower
const unsigned char BitmapBad[] U8G_PROGMEM = {
  0x00, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0xE0, 0x0D, 0x00, 0x00, 0xA0, 0x0F, 0x00,
  0x00, 0x20, 0x69, 0x00, 0x00, 0x10, 0x78, 0x02, 0x00, 0x10, 0xC0, 0x03, 0x00, 0x10, 0xC0, 0x03,
  0x00, 0x10, 0x00, 0x01, 0x00, 0x10, 0x80, 0x00, 0x00, 0x10, 0xC0, 0x00, 0x00, 0x30, 0x60, 0x00,
  0x00, 0x60, 0x30, 0x00, 0x00, 0xC0, 0x1F, 0x00, 0x00, 0x60, 0x07, 0x00, 0x00, 0x60, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xC7, 0x1C, 0x00,
  0x80, 0x68, 0x66, 0x00, 0xC0, 0x33, 0x7B, 0x00, 0x40, 0xB6, 0x4D, 0x00, 0x00, 0xE8, 0x06, 0x00,
  0x00, 0xF0, 0x03, 0x00, 0x00, 0xE0, 0x00, 0x00
};

// warning sign '/!\'
const unsigned char BitmapFault[] U8G_PROGMEM = {
#if 0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0xe0, 0x01, 0x00,
  0x00, 0xf0, 0x03, 0x00, 0x00, 0x38, 0x07, 0x00, 0x00, 0x38, 0x07, 0x00, 0x00, 0x1c, 0x0e, 0x00,
  0x00, 0x0c, 0x0c, 0x00, 0x00, 0x0e, 0x1c, 0x00, 0x00, 0xc6, 0x18, 0x00, 0x00, 0xe7, 0x39, 0x00,
  0x80, 0xe3, 0x71, 0x00, 0x80, 0xc3, 0x70, 0x00, 0xc0, 0xc1, 0xe0, 0x00, 0xc0, 0xc0, 0xc0, 0x00,
  0xe0, 0xc0, 0xc0, 0x01, 0x60, 0xc0, 0x80, 0x01, 0x70, 0xc0, 0x80, 0x03, 0x38, 0xc0, 0x00, 0x07,
  0x18, 0x00, 0x00, 0x06, 0x1c, 0xc0, 0x00, 0x0e, 0x0c, 0xc0, 0x00, 0x0c, 0x0e, 0x00, 0x00, 0x1c,
  0x06, 0x00, 0x00, 0x18, 0x07, 0x00, 0x00, 0x38, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x1f,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#else
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x01, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0xc0, 0x07, 0x00, 0x00, 0xe0, 0x07, 0x00,
  0x00, 0x70, 0x0e, 0x00, 0x00, 0x70, 0x0e, 0x00, 0x00, 0x38, 0x1c, 0x00, 0x00, 0x3c, 0x38, 0x00,
  0x00, 0x1c, 0x38, 0x00, 0x00, 0xce, 0x73, 0x00, 0x00, 0xce, 0x73, 0x00, 0x00, 0xc7, 0xe3, 0x00,
  0x80, 0xc3, 0xc3, 0x01, 0x80, 0x83, 0xc1, 0x01, 0xc0, 0x81, 0x81, 0x03, 0xc0, 0x01, 0x80, 0x07,
  0xe0, 0x80, 0x01, 0x07, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00, 0x1c,
  0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
};

// Elecrow Logo
const unsigned char bitmap_logo[] U8G_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xE0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x04, 0xF8, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x08, 0xFE, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x10, 0x1F, 0xE0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xB0, 0x07, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xE0, 0x03, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xC0, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x01, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x60, 0x23, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x70, 0xC7, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x70, 0x9E, 0x0F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x70, 0x3C, 0xFE, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x70, 0x78, 0xF8, 0x7F, 0xF0, 0x9F, 0x07, 0xFE, 0x83, 0x0F, 0xFF, 0x00, 0x77, 0x3C, 0x18, 0x1C,
  0x70, 0xF0, 0xE1, 0x3F, 0xF1, 0x9F, 0x07, 0xFE, 0xE1, 0x1F, 0xFF, 0xC3, 0xF7, 0x3C, 0x38, 0x0C,
  0x70, 0xE0, 0x87, 0x8F, 0xF1, 0xC0, 0x07, 0x1E, 0x70, 0x3C, 0xCF, 0xE3, 0xE1, 0x7D, 0x3C, 0x0E,
  0x70, 0xD0, 0x1F, 0xC0, 0xF1, 0xC0, 0x03, 0x1F, 0x78, 0x3C, 0xCF, 0xE3, 0xE1, 0x7D, 0x3C, 0x06,
  0xF0, 0xB0, 0xFF, 0xF1, 0xF0, 0xC0, 0x03, 0x0F, 0x78, 0x3C, 0xCF, 0xF3, 0xE0, 0x7B, 0x3E, 0x06,
  0xF0, 0x60, 0xFF, 0xFF, 0xF0, 0xC6, 0x03, 0xEF, 0x3C, 0x80, 0xEF, 0xF1, 0xE0, 0x7B, 0x3E, 0x03,
  0xF0, 0xE1, 0xFC, 0xFF, 0xF8, 0xCF, 0x03, 0xFF, 0x3C, 0x80, 0xFF, 0xF0, 0xE0, 0x7B, 0x7B, 0x01,
  0xE0, 0xC3, 0xF9, 0x7F, 0x78, 0xC0, 0x03, 0x0F, 0x3C, 0x80, 0xF7, 0xF1, 0xE0, 0xF9, 0xF9, 0x01,
  0xE0, 0x83, 0xE3, 0x7F, 0x78, 0xE0, 0x03, 0x0F, 0x3C, 0xBC, 0xE7, 0xF1, 0xE0, 0xF9, 0xF9, 0x00,
  0xC0, 0x0F, 0x8F, 0x3F, 0x78, 0xE0, 0x81, 0x0F, 0x3C, 0x9E, 0xE7, 0xF1, 0xE0, 0xF1, 0xF8, 0x00,
  0x80, 0x3F, 0x1E, 0x00, 0x78, 0xE0, 0x81, 0x07, 0x38, 0x9E, 0xE7, 0xF1, 0xF0, 0xF0, 0x78, 0x00,
  0x80, 0xFF, 0xFF, 0x00, 0xF8, 0xEF, 0xBF, 0xFF, 0xF8, 0xCF, 0xE7, 0xE1, 0x7F, 0x70, 0x70, 0x00,
  0x00, 0xFF, 0xFF, 0x0F, 0xF8, 0xEF, 0xBF, 0xFF, 0xE0, 0xC3, 0xE3, 0x81, 0x1F, 0x70, 0x30, 0x00,
  0x00, 0xFC, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xF8, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xE0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void setup()
{
  u8g.firstPage();
  do {
    draw_splash();
  } while (u8g.nextPage());
  delay(3000);

  u8g.firstPage();
  do {
    draw_test();
  } while (u8g.nextPage());
  delay(3000);

  Wire.begin();
  RTC.begin();
  Serial.begin(9600);
  Serial1.begin(115200);

  Serial1.println("Starting...");

  flowers[0].relay_pin = 6;
  flowers[0].sensor_pin = A0;
  flowers[0].sensor_min_val = 270; // based on calibration
  flowers[0].sensor_max_val = 590; // based on calibration

  flowers[1].relay_pin = 8;
  flowers[1].sensor_pin = A1;
  flowers[1].sensor_min_val = 260; // based on calibration
  flowers[1].sensor_max_val = 580; // based on calibration

  flowers[2].relay_pin = 9;
  flowers[2].sensor_pin = A2;
  flowers[2].sensor_min_val = 260; // based on calibration
  flowers[2].sensor_max_val = 590; // based on calibration

  flowers[3].relay_pin = 10;
  flowers[3].sensor_pin = A3;
  flowers[3].sensor_min_val = 280; // based on calibration
  flowers[3].sensor_max_val = 590; // based on calibration

  // declare relays as outputs
  for (int i = 0; i < NFLOWERS; i++) {
    pinMode(flowers[i].relay_pin, OUTPUT);
    digitalWrite(flowers[i].relay_pin, LOW);
  }

  // declare pump as output
  pinMode(PumpPin, OUTPUT);
  digitalWrite(PumpPin, LOW);

  // declare switch as input
  pinMode(ButtonPin, INPUT);
  digitalWrite(ButtonPin, LOW);

  // read values from the moisture sensors
  for (int i = 0; i < NFLOWERS; i++)
    update_moisture(i);
  serial_report_moisture();
}

void loop()
{
  // Also, treat the button press as a signal to refresh moisture readings immediately.
  if (force_screen_refresh) {
    for (int i = 0; i < NFLOWERS; i++)
      update_moisture(i);
  } else {
    for (int i = 0; i < NFLOWERS; i++)
      update_moisture_if_needed(i);
  }

  for (int i = 0; i < NFLOWERS; i++) {
    (void)update_state(i);
  }

  set_controls();

  unsigned long nowMillis = millis();

  if (nowMillis - last_report > SerialReportPeriod)
    serial_report_moisture();

  int button_state = digitalRead(ButtonPin);
  if (button_state == 1) {
    if (force_screen_refresh || nowMillis - last_refresh > ScreenRefreshPeriod) {
      force_screen_refresh = false;
      last_refresh = nowMillis;
      u8g.firstPage();
      do {
        draw_moisture();
        draw_flower();
      } while (u8g.nextPage());
    }
  } else {
    u8g.firstPage();
    do {
      draw_time();
      u8g.drawStr(8, 55 , "www.elecrow.com");
    } while (u8g.nextPage());
    // force screen refresh when the button is released
    force_screen_refresh = true;
  }

  // chill a little bit
  delay(100);
}

void print_serial_preamble(unsigned long nowMillis)
{
      Serial1.print("[");
      Serial1.print(nowMillis, DEC);
      Serial1.print("] ");
}

int read_sensor(int pin, int max_raw, int min_raw)
{
  float raw = analogRead(pin);
  int normalized = map(raw, max_raw, min_raw, 0, 100);
  if (normalized < 0)
    normalized = 0;
  else if (normalized > 100)
    normalized = 100;
  return normalized;
}

void update_moisture(int flower_id)
{
  struct flower *flower = &flowers[flower_id];
  flower->moisture_cur = read_sensor(flower->sensor_pin, flower->sensor_max_val, flower->sensor_min_val);
  flower->last_sensor_update = millis();
}

void update_moisture_if_needed(int flower_id)
{
  struct flower *flower = &flowers[flower_id];
  unsigned long nowMillis = millis();
  unsigned long update_period = flower->watering ? ActiveUpdatePeriod : IdleUpdatePeriod;
  if (nowMillis - flower->last_sensor_update >= update_period)
    update_moisture(flower_id);
}

bool update_state(int flower_id)
{
  struct flower *flower = &flowers[flower_id];

  if (flower->faulted)
    return false;

  unsigned long nowMillis = millis();

  if (!flower->watering) {
    // start watering if too dry
    if (flower->moisture_cur < MoistureLowThreshold) {
      flower->watering = true;
      flower->valve_open = true;
      flower->phase_start = nowMillis;
      flower->moisture_max = flower->moisture_cur;
      flower->last_increase_ts = nowMillis;

      print_serial_preamble(nowMillis);
      Serial1.print(F("Flower "));
      Serial1.print(flower_id, DEC);
      Serial1.println(F(" needs watering"));
      Serial1.print(F("Moisture level is "));
      Serial1.print(flower->moisture_cur, DEC);
      Serial1.println("%");

      return true;
    }
  } else {
    // First, check for a fault: watering is active
    // but there is no increase in moisture level for too long.
    // So, if there is an increase in moisture level, then record it
    // and reset the timestamp, otherwise check again the timeout.
    if (flower->moisture_cur > flower->moisture_max) {
      flower->moisture_max = flower->moisture_cur;
      flower->last_increase_ts = nowMillis;
    } else if (nowMillis - flower->last_increase_ts > FaultTimeout) {
      flower->watering = false;
      flower->valve_open = false;
      flower->last_increase_ts = 0;
      flower->phase_start = 0;
      flower->faulted = true;

      print_serial_preamble(nowMillis);
      Serial1.print(F("Flower "));
      Serial1.print(flower_id, DEC);
      Serial1.println(F(" moisture level is not increasing for too long"));
      Serial1.print(F("Current moisture level is "));
      Serial1.print(flower->moisture_cur, DEC);
      Serial1.println("%");
      Serial1.print(F("Maximum seen moisture level is "));
      Serial1.print(flower->moisture_max, DEC);
      Serial1.println("%");

      return true;
    }

    // stop watering if moist enough, otherwise continue watering
    if (flower->moisture_cur > MoistureHighThreshold) {
      flower->watering = false;
      flower->valve_open = false;
      flower->phase_start = 0;
      flower->moisture_max = 0;
      flower->last_increase_ts = 0;

      print_serial_preamble(nowMillis);
      Serial1.print(F("Flower "));
      Serial1.print(flower_id, DEC);
      Serial1.println(F(" has been watered"));
      Serial1.print(F("Moisture level is "));
      Serial1.print(flower->moisture_cur, DEC);
      Serial1.println("%");

      return true;
    } else {
      // if the current phase is over, switch to the other phase
      unsigned long phase_duration = flower->valve_open ? ActiveWateringPeriod : PauseWateringPeriod;
      if (nowMillis - flower->phase_start >= phase_duration) {
        flower->valve_open = !flower->valve_open;
        flower->phase_start = nowMillis;

        print_serial_preamble(nowMillis);
        Serial1.print(F("Flower "));
        Serial1.print(flower_id, DEC);
        Serial1.print(F(" valve "));
        Serial1.println(flower->valve_open ? F("opened") : F("closed"));

        return true;
      }
    }
  }
  return false;
}

void set_controls(void)
{
  static unsigned long pump_start_time;
  unsigned long nowMillis = millis();
  bool pump_on = false;

  for (int i = 0; i < NFLOWERS; i++) {
    digitalWrite(flowers[i].relay_pin, flowers[i].valve_open ? HIGH : LOW);
    if (flowers[i].valve_open)
      pump_on = true;
  }

  if (pump_active != pump_on) {
    pump_active = pump_on;
    if (pump_active)
      pump_start_time = nowMillis;
    else
      digitalWrite(PumpPin, LOW);

    print_serial_preamble(nowMillis);
    Serial1.print(F("Pump "));
    Serial1.println(pump_on ? F("starting") : F("stopped"));
  }

  if (pump_active && nowMillis - pump_start_time > PumpStartDelay)
    digitalWrite(PumpPin, HIGH);
}

void draw_splash(void) {
  u8g.setFont(u8g_font_gdr9r);
  u8g.drawXBMP(0, 5, 128, 32, bitmap_logo);
  u8g.drawStr(8, 41 , F("agapon@gmail.com"));
  u8g.drawStr(8, 55 , F("www.elecrow.com"));
}

void lcd_print_padded_number(int number, int width, char padding)
{
  int threshold;

  threshold = (width - 1) * 10;
  while (threshold > 0 && number < threshold) {
    u8g.print(padding);
    threshold /= 10;
  }
  u8g.print(number, DEC);
}

// Set this to true to force RTC reset.
bool rtc_reset = false;

void draw_time(void)
{
  if (!RTC.isrunning() || rtc_reset) {
    u8g.setFont(u8g_font_6x10);
    u8g.setPrintPos(5, 20);
    u8g.print(F("RTC is NOT running!"));
    Serial1.println(F("RTC is NOT running!"));
    RTC.adjust(DateTime(__DATE__, __TIME__));
    rtc_reset = false;
  } else {
    DateTime cur_time = RTC.now();

    u8g.setFont(u8g_font_7x13);
    u8g.setPrintPos(5, 11);
    lcd_print_padded_number(cur_time.year(), 4, '0');
    u8g.print("/");
    lcd_print_padded_number(cur_time.month(), 2, '0');
    u8g.print("/");
    lcd_print_padded_number(cur_time.day(), 2, '0');

    u8g.setPrintPos(85, 11);
    u8g.print(DaysOfTheWeek[cur_time.dayOfTheWeek()]);

    u8g.setFont(u8g_font_8x13);
    u8g.setPrintPos(35, 33);
    lcd_print_padded_number(cur_time.hour(), 2, '0');
    u8g.print(":");
    lcd_print_padded_number(cur_time.minute(), 2, '0');
    u8g.print(":");
    lcd_print_padded_number(cur_time.second(), 2, '0');
  }
}

void draw_flower(void)
{
  const int bitmaps_y = 0;

  for (int i = 0; i < NFLOWERS; i++) {
    const unsigned char *bitmap;

    if (flowers[i].faulted)
      bitmap = BitmapFault;
    else if (flowers[i].moisture_cur < MoistureLowThreshold)
      bitmap = BitmapBad;
    else
      bitmap = BitmapGood;

    u8g.drawXBMP(BitmapWidth * i, bitmaps_y, BitmapWidth, BitmapHeight, bitmap);
  }
}

void draw_test(void)
{
  const unsigned char *bitmaps[] = { BitmapGood, BitmapBad, BitmapFault };

  for (int i = 0; i < 3; i++) {
    u8g.drawXBMP((BitmapWidth + 8) * i, 16, BitmapWidth, BitmapHeight, bitmaps[i]);
  }
}

void draw_moisture(void)
{
  const int label_y = 60;
  const int value_y = 45;
  const int label_x_offset = 9;
  const int value_x_offset = 2;

  u8g.setFont(u8g_font_7x14);

  for (int i = 0; i < NFLOWERS; i++) {
    u8g.setPrintPos(BitmapWidth * i + label_x_offset, label_y);
    u8g.print("A");
    u8g.print(i, DEC);

    u8g.setPrintPos(BitmapWidth * i + value_x_offset, value_y);
    lcd_print_padded_number(flowers[i].moisture_cur, 3, ' ');
    u8g.print("%");
  }
}

void serial_report_moisture(void)
{
  unsigned long nowMillis = millis();
  bool faulted = false;

  print_serial_preamble(nowMillis);
  Serial1.print(F("Moisture levels:"));
  for (int f = 0; f < NFLOWERS; f++) {
    Serial1.print(" ");
    Serial1.print(flowers[f].moisture_cur, DEC);
    Serial1.print("%");
    if (flowers[f].faulted)
      faulted = true;
  }
  Serial1.println("");

  if (faulted) {
    Serial1.print(F("FALTED:"));
    for (int f = 0; f < NFLOWERS; f++) {
      if (flowers[f].faulted) {
        Serial1.print(" ");
        Serial1.print(f, DEC);
      }
    }
    Serial1.println("");
  }

  last_report = nowMillis;
}

// vim: ts=2 sw=2 softtabstop=2 expandtab smartindent
