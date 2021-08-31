#include <Wire.h>
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);    // I2C
#include "Wire.h"
#include "RTClib.h"
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
const unsigned long activeWateringPeriod = 5000; // how long to have a valve open, milliseconds
const unsigned long pauseWateringPeriod = 25000; // how long to have a valve closed before opening again, milliseconds

const unsigned long idleUpdatePeriod = 60000;    // period between checking moisture levels while not watering, milliseconds
const unsigned long activeUpdatePeriod = 1000;   // period between checking moisture levels while watering, milliseconds

const unsigned long screenRefreshPeriod = 2000;  // how often to update information on the screen,
unsigned long last_refresh;                      // time of the last screen refresh milliseconds
bool force_screen_refresh = false;               // whether to force a screen refresh after the alternative display

const unsigned long faultTimeout = 120000;       // if, while watering, moisture level does not increase for this long, then declare a fault

// set water pump pin
int pump = 4;

// set button pin
int button = 12;

char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// good flower
const unsigned char bitmap_good[] U8G_PROGMEM = {
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
const unsigned char bitmap_bad[] U8G_PROGMEM = {
  0x00, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0xE0, 0x0D, 0x00, 0x00, 0xA0, 0x0F, 0x00,
  0x00, 0x20, 0x69, 0x00, 0x00, 0x10, 0x78, 0x02, 0x00, 0x10, 0xC0, 0x03, 0x00, 0x10, 0xC0, 0x03,
  0x00, 0x10, 0x00, 0x01, 0x00, 0x10, 0x80, 0x00, 0x00, 0x10, 0xC0, 0x00, 0x00, 0x30, 0x60, 0x00,
  0x00, 0x60, 0x30, 0x00, 0x00, 0xC0, 0x1F, 0x00, 0x00, 0x60, 0x07, 0x00, 0x00, 0x60, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xC7, 0x1C, 0x00,
  0x80, 0x68, 0x66, 0x00, 0xC0, 0x33, 0x7B, 0x00, 0x40, 0xB6, 0x4D, 0x00, 0x00, 0xE8, 0x06, 0x00,
  0x00, 0xF0, 0x03, 0x00, 0x00, 0xE0, 0x00, 0x00
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
  draw_elecrow();
  delay(5000);
  Wire.begin();
  RTC.begin();
  Serial.begin(9600);

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
  pinMode(pump, OUTPUT);
  digitalWrite(pump, LOW);

  // declare switch as input
  pinMode(button, INPUT);
  digitalWrite(button, LOW);

  // read values from the moisture sensors
  for (int i = 0; i < NFLOWERS; i++)
    update_moisture(&flowers[i]);
}

void loop()
{
  // Also, treat the button press as a signal to refresh moisture readings immediately.
  if (force_screen_refresh) {
    for (int i = 0; i < NFLOWERS; i++)
      update_moisture(&flowers[i]);
  } else {
    for (int i = 0; i < NFLOWERS; i++)
      update_moisture_if_needed(&flowers[i]);
  }

  for (int i = 0; i < NFLOWERS; i++) {
    (void)update_state(&flowers[i]);
  }

  set_controls();

  int button_state = digitalRead(button);
  if (button_state == 1) {
    unsigned long nowMillis = millis();
    if (force_screen_refresh || nowMillis - last_refresh > screenRefreshPeriod) {
      force_screen_refresh = false;
      last_refresh = nowMillis;
      u8g.firstPage();
      do {
        drawTH();
        drawflower();
      } while ( u8g.nextPage() );
    }
  } else {
    u8g.firstPage();
    do {
      drawtime();
      u8g.drawStr(8, 55 , "www.elecrow.com");
    } while (u8g.nextPage());
    // force screen refresh when the button is released
    force_screen_refresh = true;
  }

  // chill a little bit
  delay(100);
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

void update_moisture(struct flower *flower)
{
  flower->moisture_cur = read_sensor(flower->sensor_pin, flower->sensor_max_val, flower->sensor_min_val);
  flower->last_sensor_update = millis();
}

void update_moisture_if_needed(struct flower *flower)
{
  unsigned long nowMillis = millis();
  unsigned long update_period = flower->watering ? activeUpdatePeriod : idleUpdatePeriod;
  if (nowMillis - flower->last_sensor_update >= update_period)
    update_moisture(flower);
}

bool update_state(struct flower *flower)
{
  if (flower->faulted)
    return false;

  unsigned long nowMillis = millis();

  if (!flower->watering) {
    // start watering if too dry
    if (flower->moisture_cur < 30) {
      flower->watering = true;
      flower->valve_open = true;
      flower->phase_start = nowMillis;
      flower->moisture_max = flower->moisture_cur;
      flower->last_increase_ts = nowMillis;
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
    } else if (nowMillis - flower->last_increase_ts > faultTimeout) {
      flower->watering = false;
      flower->valve_open = false;
      flower->last_increase_ts = 0;
      flower->phase_start = 0;
      flower->faulted = true;
      return true;
    }

    // stop watering if moist enough, otherwise continue watering
    if (flower->moisture_cur > 55) {
      flower->watering = false;
      flower->valve_open = false;
      flower->phase_start = 0;
      flower->moisture_max = 0;
      flower->last_increase_ts = 0;
      return true;
    } else {
      // if the current phase is over, switch to the other phase
      unsigned long phase_duration = flower->valve_open ? activeWateringPeriod : pauseWateringPeriod;
      if (nowMillis - flower->phase_start >= phase_duration) {
        flower->valve_open = !flower->valve_open;
        flower->phase_start = nowMillis;
        return true;
      }
    }
  }
  return false;
}

void set_controls(void)
{
  bool pump_on = false;
  for (int i = 0; i < NFLOWERS; i++) {
    digitalWrite(flowers[i].relay_pin, flowers[i].valve_open ? HIGH : LOW);
    if (flowers[i].valve_open)
      pump_on = true;
  }

  // Give all opened valves some time to actually open
  if (pump_on)
    delay(50);

  digitalWrite(pump, pump_on ? HIGH : LOW);
}

void draw_elecrow(void) {
  u8g.setFont(u8g_font_gdr9r);
  u8g.drawStr(8, 55 , "www.elecrow.com");
  u8g.drawXBMP(0, 5, 128, 32, bitmap_logo);
}

// Set this to true to force RTC reset.
bool rtc_reset = false;

void drawtime(void)
{
  int x = 5;
  //Serial.print(cur_time.year(), DEC);
  if (!RTC.isrunning() || rtc_reset) {
    u8g.setFont(u8g_font_6x10);
    u8g.setPrintPos(5, 20);
    u8g.print("RTC is NOT running!");
    RTC.adjust(DateTime(__DATE__, __TIME__));
    rtc_reset = false;
  } else {
    DateTime cur_time = RTC.now();

    u8g.setFont(u8g_font_7x13);
    u8g.setPrintPos(x, 11);
    u8g.print(cur_time.year(), DEC);
    u8g.setPrintPos(x + 80, 11);
    u8g.print(daysOfTheWeek[cur_time.dayOfTheWeek()]);
    u8g.setPrintPos(x + 28, 11);
    u8g.print("/");
    u8g.setPrintPos(x + 33, 11);
    if (cur_time.month() < 10)
      u8g.print('0');
    u8g.print(cur_time.month(), DEC);
    u8g.setPrintPos(x + 47, 11);
    u8g.print("/");
    u8g.setPrintPos(x + 53, 11);
    u8g.print(cur_time.day(), DEC);
    u8g.setFont(u8g_font_8x13);
    int x = 35;
    u8g.setPrintPos(x, 33);
    if (cur_time.hour() < 10)
      u8g.print('0');
    u8g.print(cur_time.hour(), DEC);
    u8g.setPrintPos(x + 15, 33);
    u8g.print(":");
    u8g.setPrintPos(x + 21, 33);
    if (cur_time.minute() < 10)
      u8g.print('0');
    u8g.print(cur_time.minute(), DEC);
    u8g.setPrintPos(x + 36, 33);
    u8g.print(":");
    u8g.setPrintPos(x + 42, 33);
    if (cur_time.second() < 10)
      u8g.print('0');
    u8g.print(cur_time.second(), DEC);
  }
}

void drawLogo(uint8_t d)
{
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(8 + d, 30 + d, "E");
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(30 + d, 30 + d, "l");
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(40 + d, 30 + d, "e");
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(55 + d, 30 + d, "c");
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(70 + d, 30 + d, "r");
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(85 + d, 30 + d, "o");
  u8g.setFont(u8g_font_gdr25r);
  u8g.drawStr(100 + d, 30 + d, "w");
}

// Style the flowers:
// bitmap_bad: bad flowers,
// bitmap_good:good  flowers
void drawflower(void)
{
  int moisture1_value = flowers[0].moisture_cur;
  int moisture2_value = flowers[1].moisture_cur;
  int moisture3_value = flowers[2].moisture_cur;
  int moisture4_value = flowers[3].moisture_cur;

  if (flowers[0].faulted) {
    // nothing
  } else if (moisture1_value < 30) {
    u8g.drawXBMP(0, 0, 32, 30, bitmap_bad);
  } else {
    u8g.drawXBMP(0, 0, 32, 30, bitmap_good);
  }

  if (flowers[1].faulted) {
    // nothing
  } else if (moisture2_value < 30) {
    u8g.drawXBMP(32, 0, 32, 30, bitmap_bad);
  } else {
    u8g.drawXBMP(32, 0, 32, 30, bitmap_good);
  }

  if (flowers[2].faulted) {
    // nothing
  } else if (moisture3_value < 30) {
    u8g.drawXBMP(64, 0, 32, 30, bitmap_bad);
  } else {
    u8g.drawXBMP(64, 0, 32, 30, bitmap_good);
  }

  if (flowers[3].faulted) {
    // nothing
  } else if (moisture4_value < 30) {
    u8g.drawXBMP(96, 0, 32, 30, bitmap_bad);
  } else {
    u8g.drawXBMP(96, 0, 32, 30, bitmap_good);
  }
}

void drawTH(void)
{
  int moisture1_value = flowers[0].moisture_cur;
  int moisture2_value = flowers[1].moisture_cur;
  int moisture3_value = flowers[2].moisture_cur;
  int moisture4_value = flowers[3].moisture_cur;
  int A = 0;
  int B = 32;
  int C = 64;
  int D = 96;
  char moisture1_value_temp[4] = {0};
  char moisture2_value_temp[4] = {0};
  char moisture3_value_temp[4] = {0};
  char moisture4_value_temp[4] = {0};

  itoa(moisture1_value, moisture1_value_temp, 10);
  itoa(moisture2_value, moisture2_value_temp, 10);
  itoa(moisture3_value, moisture3_value_temp, 10);
  itoa(moisture4_value, moisture4_value_temp, 10);

  u8g.setFont(u8g_font_7x14);

  u8g.setPrintPos(9, 60);
  u8g.print("A0");
  if (moisture1_value < 10) {
    u8g.drawStr(A + 14, 45, moisture1_value_temp);
  } else if (moisture1_value < 100) {
    u8g.drawStr(A + 7, 45, moisture1_value_temp);
  } else {
    u8g.drawStr(A + 2, 45, moisture1_value_temp);
  }
  u8g.setPrintPos(A + 23, 45 );
  u8g.print("%");

  u8g.setPrintPos(41, 60 );
  u8g.print("A1");
  if (moisture2_value < 10) {
    u8g.drawStr(B + 14, 45, moisture2_value_temp);
  } else if (moisture2_value < 100) {
    u8g.drawStr(B + 7, 45, moisture2_value_temp);
  } else {
    u8g.drawStr(B + 2, 45, moisture2_value_temp);
  }
  u8g.setPrintPos(B + 23, 45);
  u8g.print("%");

  u8g.setPrintPos(73, 60);
  u8g.print("A2");
  if (moisture3_value < 10) {
    u8g.drawStr(C + 14, 45, moisture3_value_temp);
  } else if (moisture3_value < 100) {
    u8g.drawStr(C + 7, 45, moisture3_value_temp);
  } else {
    u8g.drawStr(C + 2, 45, moisture3_value_temp);
  }
  u8g.setPrintPos(C + 23, 45);
  u8g.print("%");

  u8g.setPrintPos(105, 60);
  u8g.print("A3");
  if (moisture4_value < 10) {
    u8g.drawStr(D + 14, 45, moisture4_value_temp);
  } else if (moisture4_value < 100) {
    u8g.drawStr(D + 7, 45, moisture4_value_temp);
  } else {
    u8g.drawStr(D + 2, 45, moisture4_value_temp);
  }
  u8g.setPrintPos(D + 23, 45);
  u8g.print("%");
}

// vim: ts=2 sw=2 softtabstop=2 expandtab smartindent
