#include <Wire.h>
#include "U8glib.h"
#include "Wire.h"
#include "RTClib.h"
#include "OneButton.h"
#include "SerialCommands.h"

// Seems to be 132x64 SH1106 actually.
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);    // I2C

RTC_DS1307 RTC;
const byte RtcMagicReg = 0x00;
const byte RtcMagicVal = 0x5a;

const char RtcStoppedMsg[] PROGMEM = "RTC NOT running!";
const char RtcLostMsg[] PROGMEM =    "RTC lost memory!";

#define NFLOWERS  4
struct flower
{
  int moisture_raw;         // current moisture level
  byte moisture_cur;        // current moisture level
  byte moisture_max;        // maximum moisture level seen during current watering session
  bool watering;            // are we in a watering session right now ?
  bool force_watering_start;
  bool force_watering_stop;
  bool valve_open;          // are we actively watering (valve is open) ?
  bool faulted;             // have we detected a fault either with sensor or valve or pipe ?
  unsigned long last_sensor_update;// timestamp of last moisture level check
  unsigned long last_increase_ts;  // timestamp of the last increase in moisture level since watering started
  unsigned long phase_start; // timestamp of when we last opened or closed valve during current watering session
  byte relay_pin;
  byte sensor_pin;
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
// NB: need to use unsigned long here as UINT_MAX is just 2^16.
const unsigned long ActiveWateringPeriod = 1500;

// How long to have a valve closed before opening again, milliseconds.
// 1/120 duty cycle to emulate 1 l/h flow.
const unsigned long PauseWateringPeriod = 180000 - ActiveWateringPeriod;

// If, while watering, moisture level does not increase for this long, then declare a fault.
// The value is calculated for 200 ml at (emulated) 1 l/h resulting in 12 minutes.
const unsigned long FaultTimeout = 720000;

const unsigned long IdleUpdatePeriod = 60000;    // period between checking moisture levels while not watering, milliseconds
const unsigned long ActiveUpdatePeriod = 1000;   // period between checking moisture levels while watering, milliseconds

const unsigned long SerialIdleReportPeriod = 600000; // how often to update information on via the serial connection
const unsigned long SerialActiveReportPeriod = 30000; // how often to update information on via the serial connection
unsigned long last_serial_report;                // time of the last screen refresh milliseconds

const int PumpStartDelay = 20;                  // how long to wait after opening a valve before starting the pump, ms
bool pump_active;

// Watering hysteresis.
const byte MoistureLowThreshold = 30;            // start watering when moisture level falls below this threshold
const byte MoistureHighThreshold = 50;           // stop watering when moisture level raises above this threshold

const byte PumpPin = 4;
const byte ButtonPin = 12;

OneButton main_button = OneButton(ButtonPin);
enum DisplayMode {
  DM_DEFAULT = 0,
  DM_TECHNICAL,
  DM_TIME
} display_mode;

// How often to update information on the screen in each display mode (ms).
const int ScreenRefreshPeriods[] = {
  2000,
  1000,
  1000
};

unsigned long last_display_refresh;                      // time of the last screen refresh, milliseconds

char serial_command_buffer[64];
SerialCommands serial_commands(&Serial1, serial_command_buffer, sizeof(serial_command_buffer), "\r\n", " ");

void rtc_time_cmd_f(SerialCommands *sender);
void refresh_cmd_f(SerialCommands *sender);
void show_cmd_f(SerialCommands *sender);
void water_cmd_f(SerialCommands *sender);
void clear_cmd_f(SerialCommands *sender);
SerialCommand rtc_time_cmd("rtc_time", rtc_time_cmd_f);
SerialCommand refresh_cmd("refresh", refresh_cmd_f);
SerialCommand show_cmd("show", show_cmd_f);
SerialCommand water_cmd("water", water_cmd_f);
SerialCommand clear_cmd("clear", clear_cmd_f);

char small_printf_buf[12];

const char DaysOfTheWeek[7][4] U8G_PROGMEM = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

const byte BitmapWidth = 32;
const byte BitmapHeight = 30;

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
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x01, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0xc0, 0x07, 0x00, 0x00, 0xe0, 0x07, 0x00,
  0x00, 0x70, 0x0e, 0x00, 0x00, 0x70, 0x0e, 0x00, 0x00, 0x38, 0x1c, 0x00, 0x00, 0x3c, 0x38, 0x00,
  0x00, 0x1c, 0x38, 0x00, 0x00, 0xce, 0x73, 0x00, 0x00, 0xce, 0x73, 0x00, 0x00, 0xc7, 0xe3, 0x00,
  0x80, 0xc3, 0xc3, 0x01, 0x80, 0x83, 0xc1, 0x01, 0xc0, 0x81, 0x81, 0x03, 0xc0, 0x01, 0x80, 0x07,
  0xe0, 0x80, 0x01, 0x07, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00, 0x1c,
  0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xff, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void setup()
{
  u8g.firstPage();
  do {
    draw_splash();
  } while (u8g.nextPage());
  delay(3000);

  Wire.begin();
  RTC.begin();
  Serial.begin(9600);
  Serial1.begin(115200);

  Serial1.println("Starting...");

  flowers[0].relay_pin = 6;
  flowers[0].sensor_pin = A0;
  flowers[0].sensor_min_val = 247; // based on calibration
  flowers[0].sensor_max_val = 585; // based on calibration

  flowers[1].relay_pin = 8;
  flowers[1].sensor_pin = A1;
  flowers[1].sensor_min_val = 249; // based on calibration
  flowers[1].sensor_max_val = 586; // based on calibration

  flowers[2].relay_pin = 9;
  flowers[2].sensor_pin = A2;
  flowers[2].sensor_min_val = 245; // based on calibration
  flowers[2].sensor_max_val = 586; // based on calibration

  flowers[3].relay_pin = 10;
  flowers[3].sensor_pin = A3;
  flowers[3].sensor_min_val = 249; // based on calibration
  flowers[3].sensor_max_val = 582; // based on calibration

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
  serial_report_moisture(&Serial1);

  main_button.attachClick(main_button_click);
  main_button.attachDoubleClick(main_button_doubleclick);

  serial_commands.SetDefaultHandler(unknown_cmd);
  serial_commands.AddCommand(&rtc_time_cmd);
  serial_commands.AddCommand(&refresh_cmd);
  serial_commands.AddCommand(&show_cmd);
  serial_commands.AddCommand(&water_cmd);
  serial_commands.AddCommand(&clear_cmd);

  byte rtc_check_val = RTC.readnvram(RtcMagicReg);
  if (!RTC.isrunning() || rtc_check_val != RtcMagicVal) {
    char buf[17];

    if (rtc_check_val != RtcMagicVal)
      strcpy_P(buf, RtcLostMsg);
    else
      strcpy_P(buf, RtcStoppedMsg);

    Serial1.println(buf);
    set_rtc(DateTime(__DATE__, __TIME__));
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14r);
      u8g.drawStr(5, 25, buf);
    } while (u8g.nextPage());
    delay(3000);
  }
}

void loop()
{
  // Waterning apparatus
  for (int i = 0; i < NFLOWERS; i++)
    update_moisture_if_needed(i);
  for (int i = 0; i < NFLOWERS; i++)
    (void)update_state(i);
  set_controls();

  unsigned long nowMillis = millis();

  // Serial communications
  serial_commands.ReadSerial();

  signed long serial_report_period;
  bool watering = false;
  for (int i = 0; i < NFLOWERS; i++) {
    if (flowers[i].watering) {
      watering = true;
      break;
    }
  }
  serial_report_period = watering ? SerialActiveReportPeriod : SerialIdleReportPeriod;
  if (nowMillis - last_serial_report > serial_report_period)
    serial_report_moisture(&Serial1);

  // Display and input
  if (nowMillis - last_display_refresh > ScreenRefreshPeriods[display_mode]) {
    last_display_refresh = nowMillis;
    u8g.firstPage();
    do {
      switch (display_mode) {
        case DM_DEFAULT:
          draw_moisture();
          draw_flower();
          break;
        case DM_TIME:
          draw_time();
          u8g.drawStr(8, 55 , F("www.elecrow.com"));
          break;
        case DM_TECHNICAL:
          draw_raw_readings();
          draw_debug();
          break;
      };
    } while (u8g.nextPage());
  }
  delay(1);
  main_button.tick();
}

void set_rtc(const DateTime& d)
{
  RTC.adjust(d);
  RTC.writenvram(RtcMagicReg, RtcMagicVal);
}

void main_button_click(void)
{
  if (display_mode != DM_DEFAULT)
    display_mode = DM_DEFAULT;
  else
    display_mode =  DM_TECHNICAL;
}

void main_button_doubleclick(void)
{
  display_mode =  DM_TIME;
}

void print_serial_preamble(Stream *stream, unsigned long nowMillis)
{
      stream->print("[");
      stream->print(RTC.now().timestamp().c_str());
      stream->print(" / ");
      stream->print(nowMillis, DEC);
      stream->print("] ");
}

void update_moisture(byte flower_id)
{
  struct flower *flower = &flowers[flower_id];

  flower->moisture_raw = analogRead(flower->sensor_pin);

  int normalized = map(flower->moisture_raw, flower->sensor_max_val, flower->sensor_min_val, 0, 100);
  if (normalized < 0)
    normalized = 0;
  else if (normalized > 100)
    normalized = 100;
  flower->moisture_cur = normalized;
  flower->last_sensor_update = millis();
}

void update_moisture_if_needed(byte flower_id)
{
  struct flower *flower = &flowers[flower_id];
  unsigned long nowMillis = millis();
  unsigned long update_period = (flower->watering || display_mode == DM_TECHNICAL) ?
    ActiveUpdatePeriod : IdleUpdatePeriod;
  if (nowMillis - flower->last_sensor_update >= update_period)
    update_moisture(flower_id);
}

bool update_state(byte flower_id)
{
  struct flower *flower = &flowers[flower_id];

  if (flower->faulted)
    return false;

  unsigned long nowMillis = millis();

  if (!flower->watering) {
    // start watering if too dry
    if (flower->moisture_cur < MoistureLowThreshold ||
        flower->force_watering_start) {
      flower->watering = true;
      flower->force_watering_start = false;
      flower->valve_open = true;
      flower->phase_start = nowMillis;
      flower->moisture_max = flower->moisture_cur;
      flower->last_increase_ts = nowMillis;

      print_serial_preamble(&Serial1, nowMillis);
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

      print_serial_preamble(&Serial1, nowMillis);
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
    if (flower->moisture_cur > MoistureHighThreshold ||
        flower->force_watering_stop) {
      flower->watering = false;
      flower->force_watering_stop = false;
      flower->valve_open = false;
      flower->phase_start = 0;
      flower->moisture_max = 0;
      flower->last_increase_ts = 0;

      print_serial_preamble(&Serial1, nowMillis);
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

        print_serial_preamble(&Serial1, nowMillis);
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
    if (flowers[i].valve_open) {
      digitalWrite(flowers[i].relay_pin, HIGH);
      pump_on = true;
    }
  }

  if (pump_active != pump_on) {
    pump_active = pump_on;
    if (pump_active)
      pump_start_time = nowMillis;
    else
      digitalWrite(PumpPin, LOW);

    print_serial_preamble(&Serial1, nowMillis);
    Serial1.print(F("Pump "));
    Serial1.println(pump_on ? F("starting") : F("stopped"));
  }

  for (int i = 0; i < NFLOWERS; i++) {
    if (!flowers[i].valve_open)
      digitalWrite(flowers[i].relay_pin, LOW);
  }

  if (pump_active && nowMillis - pump_start_time > PumpStartDelay) {
    print_serial_preamble(&Serial1, nowMillis);
    Serial1.println(F("Pump started"));
    digitalWrite(PumpPin, HIGH);
  }
}

void draw_splash(void)
{
  u8g.setFont(u8g_font_gdr9r);
  u8g.drawStr(8, 17 , F("agapon@gmail.com"));
  u8g.drawStr(8, 38 , F("github.com/avg-I"));
  u8g.drawStr(8, 59 , F("www.elecrow.com"));
}

void draw_time(void)
{
  DateTime cur_time = RTC.now();

  u8g.setFont(u8g_font_7x14r);
  snprintf(small_printf_buf, sizeof(small_printf_buf), "%04d/%02d/%02d",
    cur_time.year(), cur_time.month(), cur_time.day());
  u8g.drawStr(5, 14, small_printf_buf);

  u8g.drawStrP(100, 14, DaysOfTheWeek[cur_time.dayOfTheWeek()]);

  snprintf(small_printf_buf, sizeof(small_printf_buf), "%02d:%02d:%02d",
    cur_time.hour(), cur_time.minute(), cur_time.second());
  u8g.drawStr(35, 37, small_printf_buf);
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

void draw_moisture(void)
{
  const int label_y = 60;
  const int value_y = 45;
  const int label_x_offset = 9;
  const int value_x_offset = 2;

  u8g.setFont(u8g_font_7x14r);

  for (int i = 0; i < NFLOWERS; i++) {
    u8g.setPrintPos(BitmapWidth * i + label_x_offset, label_y);
    u8g.print('A');
    u8g.print(i, DEC);

    snprintf(small_printf_buf, sizeof(small_printf_buf), "%3d%%",
      flowers[i].moisture_cur);
    u8g.drawStr(BitmapWidth * i + value_x_offset, value_y, small_printf_buf);
  }
}

void draw_raw_readings(void)
{
  const byte x_offset = 2;

  u8g.setFont(u8g_font_7x14r);

  for (int i = 0; i < NFLOWERS; i++) {
    u8g.setPrintPos(BitmapWidth * i + 9, 15);
    u8g.print(flowers[i].faulted ? '!' : (flowers[i].watering ? '*' : '-'));
    u8g.print(flowers[i].valve_open ? 'O' : 'X');

    if (flowers[i].watering) {
      snprintf(small_printf_buf, sizeof(small_printf_buf), "%4d",
          (millis() - flowers[i].phase_start) / 1000);
      u8g.drawStr(BitmapWidth * i + 2, 30, small_printf_buf);
    }

    snprintf(small_printf_buf, sizeof(small_printf_buf), "%3d%%",
      flowers[i].moisture_cur);
    u8g.drawStr(BitmapWidth * i + 2, 45, small_printf_buf);

    snprintf(small_printf_buf, sizeof(small_printf_buf), "%4d",
      flowers[i].moisture_raw);
    u8g.drawStr(BitmapWidth * i + 2, 60, small_printf_buf);
  }
}

void draw_debug(void)
{
}

void serial_report_moisture(Stream *stream)
{
  unsigned long nowMillis = millis();
  bool faulted = false;

  print_serial_preamble(stream, nowMillis);
  stream->print(F("Moisture levels:"));
  for (int f = 0; f < NFLOWERS; f++) {
    stream->print(" ");
    stream->print(flowers[f].moisture_cur, DEC);
    stream->print("%");
    if (flowers[f].faulted)
      faulted = true;
  }
  stream->println("");

  if (faulted) {
    stream->print(F("FALTED:"));
    for (int f = 0; f < NFLOWERS; f++) {
      if (flowers[f].faulted) {
        stream->print(" ");
        stream->print(f, DEC);
      }
    }
    stream->println("");
  }

  last_serial_report = nowMillis;
}

void rtc_time_cmd_f(SerialCommands *sender)
{
  const char *subcmd = sender->Next();
  const char *timespec;

  if (subcmd != NULL && strcmp(subcmd, "get") == 0) {
    sender->GetSerial()->println(RTC.now().timestamp().c_str());
  } else if (subcmd != NULL && strcmp(subcmd, "set") == 0 &&
    (timespec = sender->Next()) != NULL) {
    DateTime newTime(timespec);
    if (newTime.isValid())
      set_rtc(newTime);
    else
      sender->GetSerial()->println(F("invalid time specification"));
  } else {
    sender->GetSerial()->println(F("<get|set iso8601-timespec>"));
  }
}

void refresh_cmd_f(SerialCommands *sender)
{
  for (int i = 0; i < NFLOWERS; i++)
    update_moisture(i);
}

void show_cmd_f(SerialCommands *sender)
{
  serial_report_moisture(sender->GetSerial());
}

void water_cmd_f(SerialCommands *sender)
{
  const char *subcmd = sender->Next();
  const char *flower_str = sender->Next();
  bool on;
  byte flower_id;

  if (subcmd == NULL || flower_str == NULL) {
    sender->GetSerial()->println(F("usage error"));
    return;
  }
  if (strcmp(subcmd, "on") == 0)
    on = true;
  else if (strcmp(subcmd, "off") == 0)
    on = false;
  else {
    sender->GetSerial()->println(F("usage error"));
    return;
  }
  flower_id = atoi(flower_str);
  if (flower_id < 0 || flower_id >= NFLOWERS) {
    sender->GetSerial()->println(F("usage error"));
    return;
  }

  if (on && !flowers[flower_id].watering)
    flowers[flower_id].force_watering_start = true;
  else if (!on && flowers[flower_id].watering)
    flowers[flower_id].force_watering_stop = true;
}

void clear_cmd_f(SerialCommands *sender)
{
  for (int i = 0; i < NFLOWERS; i++) {
    if (flowers[i].faulted) {
      flowers[i].faulted = false;
      Serial1.print(F("Flower "));
      Serial1.print(i, DEC);
      Serial1.println(F(" fault cleared"));
    }
  }
}

void unknown_cmd(SerialCommands *sender, const char* cmd)
{
  sender->GetSerial()->print(F("Unrecognized command ["));
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

// vim: ts=2 sw=2 softtabstop=2 expandtab smartindent
