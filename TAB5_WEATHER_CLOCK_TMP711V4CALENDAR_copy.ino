#include <Arduino.h>
#include <M5Unified.h>
#include <M5GFX.h>
#include <SPI.h>
#include <WiFi.h>
#include <TimeLib.h>
#include <lvgl.h>
#include <Preferences.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include "mini100.c"
#include <SparkFun_TMP117.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SD.h>
#include <math.h>

// --- CONFIGURACIONES DE OPTIMIZACIÓN ---
#define FS_READ_BUFFER_SIZE 8192

// --- DRIVER DE SISTEMA DE ARCHIVOS ---
static lv_fs_drv_t fs_drv;
static void *fs_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode);
static lv_fs_res_t fs_close_cb(lv_fs_drv_t *drv, void *file_p);
static lv_fs_res_t fs_read_cb(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br);
static lv_fs_res_t fs_write_cb(lv_fs_drv_t *drv, void *file_p, const void *buf, uint32_t btw, uint32_t *bw);
static lv_fs_res_t fs_seek_cb(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence);
static lv_fs_res_t fs_tell_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p);

void lvgl_fs_init() {
  lv_fs_drv_init(&fs_drv);
  fs_drv.letter = 'S';
  fs_drv.open_cb = fs_open_cb;
  fs_drv.close_cb = fs_close_cb;
  fs_drv.read_cb = fs_read_cb;
  fs_drv.write_cb = fs_write_cb;
  fs_drv.seek_cb = fs_seek_cb;
  fs_drv.tell_cb = fs_tell_cb;
  lv_fs_drv_register(&fs_drv);
}

static void *fs_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) {
  LV_UNUSED(drv);
  String path_str = String(path);
  const char *fs_mode;
  if (mode == LV_FS_MODE_WR) {
    fs_mode = FILE_WRITE;
  } else if (mode == LV_FS_MODE_RD) {
    fs_mode = FILE_READ;
  } else {
    return NULL;
  }
  File f = SD.open(path_str.c_str(), fs_mode);
  if (!f) { return NULL; }
  File *file_p = new File(f);
  return (void *)file_p;
}

static lv_fs_res_t fs_close_cb(lv_fs_drv_t *drv, void *file_p) {
  LV_UNUSED(drv);
  File *fp = (File *)file_p;
  fp->close();
  delete fp;
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_read_cb(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) {
  LV_UNUSED(drv);
  File *fp = (File *)file_p;
  if (btr > 512) {
    uint32_t total_read = 0;
    uint8_t *buffer = (uint8_t *)buf;
    while (total_read < btr) {
      uint32_t to_read = (btr - total_read) > FS_READ_BUFFER_SIZE ? FS_READ_BUFFER_SIZE : (btr - total_read);
      uint32_t chunk_read = fp->read(buffer + total_read, to_read);
      total_read += chunk_read;
      if (chunk_read < to_read) break;
    }
    *br = total_read;
  } else {
    *br = fp->read((uint8_t *)buf, btr);
  }
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_write_cb(lv_fs_drv_t *drv, void *file_p, const void *buf, uint32_t btw, uint32_t *bw) {
  LV_UNUSED(drv);
  File *fp = (File *)file_p;
  *bw = fp->write((const uint8_t *)buf, btw);
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_seek_cb(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence) {
  LV_UNUSED(drv);
  File *fp = (File *)file_p;
  SeekMode mode;
  if (whence == LV_FS_SEEK_SET) mode = SeekSet;
  else if (whence == LV_FS_SEEK_CUR) mode = SeekCur;
  else if (whence == LV_FS_SEEK_END) mode = SeekEnd;
  else return LV_FS_RES_INV_PARAM;
  if (fp->seek(pos, mode)) return LV_FS_RES_OK;
  else return LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t fs_tell_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) {
  LV_UNUSED(drv);
  File *fp = (File *)file_p;
  *pos_p = fp->position();
  return LV_FS_RES_OK;
}

LV_FONT_DECLARE(mini100);
TMP117 tempsensor;

// --- DEFINICIONES DE COLORES ---
#define LV_COLOR_BLANCO lv_color_make(255, 255, 255)
#define LV_COLOR_NEGRO lv_color_make(0, 0, 0)
#define LV_COLOR_AMARILLO_PALIDO lv_color_make(255, 255, 224)
#define LV_COLOR_AZUL_CIELO lv_color_make(135, 206, 235)
#define LV_COLOR_AZUL_ROYAL lv_color_make(65, 105, 225)
#define LV_COLOR_NARANJA_PURO lv_color_make(255, 165, 0)
#define LV_COLOR_MANDARINA lv_color_make(255, 160, 0)
#define LV_COLOR_PLATA lv_color_make(192, 192, 192)
#define LV_COLOR_VERDE_MAR lv_color_make(46, 139, 87)
#define LV_COLOR_TURQUESA lv_color_make(64, 224, 208)

// --- DEFINICIONES GLOBALES ---
#define EXAMPLE_LCD_H_RES 1280
#define EXAMPLE_LCD_V_RES 720
#define LVGL_LCD_BUF_SIZE (EXAMPLE_LCD_H_RES * 40)
#define SDIO2_CLK GPIO_NUM_12
#define SDIO2_CMD GPIO_NUM_13
#define SDIO2_D0 GPIO_NUM_11
#define SDIO2_D1 GPIO_NUM_10
#define SDIO2_D2 GPIO_NUM_9
#define SDIO2_D3 GPIO_NUM_8
#define SDIO2_RST GPIO_NUM_15
#define SD_SPI_CS_PIN 42
#define SD_SPI_SCK_PIN 43
#define SD_SPI_MOSI_PIN 44
#define SD_SPI_MISO_PIN 39

// --- BUFFERS LVGL ---
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

// --- OBJETOS LVGL ---
lv_obj_t *ssid_input, *pass_input, *keyboard, *status_label;
lv_obj_t *main_screen, *wifi_screen, *config_screen;
lv_obj_t *btn_wifi, *btn_config, *btn_back_wifi, *btn_back_config;
lv_obj_t *bat_label, *slider_label;
lv_obj_t *rtc_date_label, *rtc_time_label;
lv_obj_t *scan_btn, *wifi_list = NULL, *list_container;
lv_obj_t *btn_carga, *label_carga;
lv_obj_t *month_dropdown, *year_dropdown;
lv_obj_t *wifi_icon, *bat_icon, *bat_percent_label;
bool wifi_icon_visible = true;
unsigned long lastBlink = 0;
lv_obj_t *temp_screen, *btn_temp_main, *btn_back_temp, *temp_label, *temp_min_display_label, *temp_max_display_label, *temp_chart;
lv_chart_series_t *temp_series;
lv_obj_t *temp_disp_slider, *temp_disp_label, *graph_upd_slider, *graph_upd_label;
lv_obj_t *volume_slider, *volume_label, *temp_max_slider, *temp_max_label, *temp_min_slider, *temp_min_label, *alarm_status_label;
lv_obj_t *moon_icon, *moon_label;

// --- OBJETOS PANTALLA TIEMPO ---
lv_obj_t *weather_screen, *btn_weather_main, *btn_back_weather, *btn_weather_settings;
lv_obj_t *weather_temp_label, *weather_cond_label, *weather_status_label;
lv_obj_t *weather_hum_label, *weather_press_label, *weather_icon;
lv_obj_t *weather_wind_speed_label;
lv_obj_t *weather_wind_dir_label;

// --- OBJETOS PANTALLA AJUSTES TIEMPO ---
lv_obj_t *weather_config_screen;
lv_obj_t *city_input, *country_input, *apikey_input;
lv_obj_t *weather_config_keyboard;
lv_obj_t *weather_config_status_label;

// --- OBJETOS PANTALLA PREVISIÓN 5 DÍAS ---
lv_obj_t *forecast_screen;
lv_obj_t *btn_forecast;
lv_obj_t *day_containers[5];
lv_obj_t *day_labels[5];
lv_obj_t *day_icons[5];
lv_obj_t *day_temp_labels[5];
lv_obj_t *forecast_status_label;

// --- AÑADIDO: OBJETOS PANTALLA CALENDARIO ---
lv_obj_t *calendar_screen;
lv_obj_t *btn_calendar;
lv_obj_t *calendar_widget;

// --- VARIABLES GLOBALES ---
String saved_ssid = "", saved_password = "";
int saved_brightness = 175, saved_temp_display_secs = 1, saved_graph_update_mins = 30, saved_volume = 50;
float saved_temp_max = 35.0, saved_temp_min = 15.0;
bool alarm_active = false;
unsigned long lastAlarmBeep = 0;
float current_temp_min = 999.0, current_temp_max = -999.0;
Preferences preferences;
enum WiFiState { DISCONNECTED,
                 CONNECTING,
                 CONNECTED };
WiFiState wifiState = DISCONNECTED;
static const char ntpServerName[] = "time.nist.gov";
WiFiUDP Udp;
bool udp_active = false;
unsigned int localPort = 8000;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
bool ntpSyncNeeded = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long connectionTimeout = 15000;
unsigned long lastTempUpdate = 0, lastChartUpdate = 0;
float tempAccum = 0.0f;
int tempAccumCount = 0;
// --- VARIABLES DE CONFIGURACIÓN DE OPENWEATHERMAP ---
String weatherCity;
String weatherCountry;
String weatherApiKey;
unsigned long lastWeatherUpdate = 0;
const unsigned long weatherUpdateInterval = 900000;  // 15 minutos
String lastWeatherIconPath = "";
String lastForecastIconPaths[5] = { "", "", "", "", "" };

// --- PROTOTIPOS DE FUNCIONES ---
void startWiFiConnection(const char *ssid, const char *password);
void syncNtpTime();
time_t getNtpTime();
void update_rtc_label();
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read_cb(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void create_status_bar();
void update_weather();
void checkTemperatureAlarm(float temperatura);
void playAlarmBeep();
String traducirCondicion(String mainCond, String descCond);
String get_weather_icon_path(String mainCond, String descCond, bool is_day, bool is_forecast = false);
String traducirDireccionViento(int grados);
int calcularFaseLunar();
void create_main_screen();
void create_wifi_screen();
void create_config_screen();
void create_temp_screen();
void create_weather_screen();
void create_weather_config_screen();
void create_calendar_screen();  // <-- AÑADIDO: Prototipo nueva pantalla
void ui_init();
void create_forecast_screen();
void update_5day_forecast();
void getMoonPhaseInfo(int day, String &iconPath, String &phaseName);
void updateMoonPhaseDisplay();

// --- FUNCIONES DE UTILIDAD ---
bool lv_file_can_read(const char *lv_path) {
  lv_fs_file_t f;
  lv_fs_res_t res = lv_fs_open(&f, lv_path, LV_FS_MODE_RD);
  if (res == LV_FS_RES_OK) {
    lv_fs_close(&f);
    return true;
  }
  return false;
}

void playAlarmBeep() {
  M5.Speaker.tone(1500, 200);
  delay(200);
  M5.Speaker.tone(2000, 200);
}

// --- FUNCIÓN PARA CALCULAR DÍA DEL CICLO LUNAR ---
int calcularFaseLunar() {
  time_t ahora = now();
  const long lunaBase = 947182440;
  const long cicloLunar = 2551443;
  long diff = ahora - lunaBase;
  long fase_seg = diff % cicloLunar;
  int dia_fase = fase_seg / 86400;
  return dia_fase;
}

// --- LÓGICA MEJORADA PARA FASES LUNARES ---
int roundToNearest5(int number) {
  return ((number + 2) / 5) * 5;
}

void getMoonPhaseInfo(int day, String &iconPath, String &phaseName) {
  String basePath = "S:/MOON/";

  if (day == 0) {
    phaseName = "LUNA NUEVA";
    iconPath = basePath + "moon_day_new.bin";
    return;
  }
  if (day >= 14 && day <= 15) {
    phaseName = "LUNA LLENA";
    iconPath = basePath + "moon_day_full.bin";
    return;
  }
  if (day == 7) {
    phaseName = "CUARTO CRECIENTE";
    iconPath = basePath + "moon_day_first.bin";
    return;
  }
  if (day == 22) {
    phaseName = "CUARTO MENGUANTE";
    iconPath = basePath + "moon_day_last.bin";
    return;
  }

  float illumination_percent;
  const float LUNAR_HALF_CYCLE = 29.53 / 2.0;

  if (day < LUNAR_HALF_CYCLE) {
    illumination_percent = (day / LUNAR_HALF_CYCLE) * 100.0;
  } else {
    illumination_percent = 100.0 - ((day - LUNAR_HALF_CYCLE) / LUNAR_HALF_CYCLE * 100.0);
  }

  int file_percent = roundToNearest5((int)illumination_percent);
  if (file_percent == 0) file_percent = 5;
  if (file_percent >= 100) file_percent = 95;

  if (day > 0 && day < 7) {
    phaseName = "CRECIENTE";
    iconPath = basePath + "moon_day_WaxC_" + String(file_percent) + ".bin";
  } else if (day > 7 && day < 15) {
    phaseName = "GIBOSA CRECIENTE";
    iconPath = basePath + "moon_day_WaxG_" + String(file_percent) + ".bin";
  } else if (day > 15 && day < 22) {
    phaseName = "GIBOSA MENGUANTE";
    iconPath = basePath + "moon_day_WanG_" + String(file_percent) + ".bin";
  } else {  // (day > 22)
    phaseName = "MENGUANTE";
    iconPath = basePath + "moon_day_WanC_" + String(file_percent) + ".bin";
  }
}

void updateMoonPhaseDisplay() {
  if (lv_scr_act() != main_screen || !moon_icon || !moon_label) {
    return;
  }

  int moonDay = calcularFaseLunar();
  String iconPath, phaseName;

  getMoonPhaseInfo(moonDay, iconPath, phaseName);

  if (lv_file_can_read(iconPath.c_str())) {
    lv_img_set_src(moon_icon, iconPath.c_str());
  } else {
    // Serial.printf("Error: Icono de luna no encontrado: %s\n", iconPath.c_str());
  }
  lv_obj_set_style_text_font(moon_label, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_label_set_text(moon_label, phaseName.c_str());
}

// --- FUNCIÓN PARA TRADUCIR DIRECCIÓN DEL VIENTO ---
String traducirDireccionViento(int grados) {
  if (grados > 337.5 || grados <= 22.5) return "N";
  if (grados > 22.5 && grados <= 67.5) return "NE";
  if (grados > 67.5 && grados <= 112.5) return "E";
  if (grados > 112.5 && grados <= 157.5) return "SE";
  if (grados > 157.5 && grados <= 202.5) return "S";
  if (grados > 202.5 && grados <= 247.5) return "SO";
  if (grados > 247.5 && grados <= 292.5) return "O";
  if (grados > 292.5 && grados <= 337.5) return "NO";
  return "?";
}

void checkTemperatureAlarm(float temperatura) {
  bool alarm_triggered = false;
  char temp_buf[8];
  char buffer[64];
  dtostrf(temperatura, 4, 1, temp_buf);
  if (temperatura > saved_temp_max) {
    alarm_triggered = true;
    if (alarm_status_label && lv_scr_act() == temp_screen) {
      snprintf(buffer, sizeof(buffer), "ALARMA: TEMPERATURA ALTA (%s °C)", temp_buf);
      lv_label_set_text(alarm_status_label, buffer);
      lv_obj_set_style_text_color(alarm_status_label, LV_COLOR_NARANJA_PURO, LV_PART_MAIN);
    }
  } else if (temperatura < saved_temp_min) {
    alarm_triggered = true;
    if (alarm_status_label && lv_scr_act() == temp_screen) {
      snprintf(buffer, sizeof(buffer), "ALARMA: TEMPERATURA BAJA (%s °C)", temp_buf);
      lv_label_set_text(alarm_status_label, buffer);
      lv_obj_set_style_text_color(alarm_status_label, lv_color_make(0, 100, 255), LV_PART_MAIN);
    }
  } else {
    alarm_active = false;
    if (alarm_status_label && lv_scr_act() == temp_screen) {
      lv_label_set_text(alarm_status_label, "TEMPERATURA NORMAL");
      lv_obj_set_style_text_color(alarm_status_label, lv_color_make(0, 255, 0), LV_PART_MAIN);
    }
  }
  if (alarm_triggered) {
    if (!alarm_active || (millis() - lastAlarmBeep > 2000)) {
      playAlarmBeep();
      lastAlarmBeep = millis();
      alarm_active = true;
    }
  }
}

void rtc_init() {
  if (!M5.Rtc.isEnabled()) {
    //Serial.println("RTC BM8563 init failed!");
  } else {
    //Serial.println("RTC BM8563 initialized successfully");
  }
}

void setRtcTime(int year, int month, int day, int hour, int minute, int second) {
  m5::rtc_datetime_t dt;
  dt.date.year = year;
  dt.date.month = month;
  dt.date.date = day;
  tmElements_t tm;
  tm.Year = year - 1970;
  tm.Month = month;
  tm.Day = day;
  time_t t = makeTime(tm);
  dt.date.weekDay = weekday(t) - 1;
  dt.time.hours = hour;
  dt.time.minutes = minute;
  dt.time.seconds = second;
  M5.Rtc.setDateTime(dt);
}

m5::rtc_datetime_t getRtcTime() {
  auto dt = M5.Rtc.getDateTime();
  if (dt.date.year < 2020 || dt.date.year > 2099) {
    dt.date.year = 2025;
    dt.date.month = 9;
    dt.date.date = 23;
    dt.time.hours = 0;
    dt.time.minutes = 0;
    dt.time.seconds = 0;
  }
  return dt;
}

long getMadridOffset(time_t unixTime) {
  tmElements_t tm;
  breakTime(unixTime, tm);
  auto lastSundayOfMonth = [](int year, int month) -> int {
    int monthLength;
    switch (month) {
      case 1:
      case 3:
      case 5:
      case 7:
      case 8:
      case 10:
      case 12: monthLength = 31; break;
      case 4:
      case 6:
      case 9:
      case 11: monthLength = 30; break;
      case 2:
        {
          bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
          monthLength = leap ? 29 : 28;
          break;
        }
    }
    tmElements_t tml;
    tml.Year = year - 1970;
    tml.Month = month;
    tml.Day = monthLength;
    time_t lastDay = makeTime(tml);
    int wd = weekday(lastDay);
    int offsetDays = (wd == 1) ? 0 : (wd - 1);
    return monthLength - offsetDays;
  };
  long offset = 1 * SECS_PER_HOUR;
  if (tm.Month < 3 || tm.Month > 10) return offset;
  if (tm.Month > 3 && tm.Month < 10) return offset + SECS_PER_HOUR;
  int yearFull = tm.Year + 1970;
  if (tm.Month == 3) {
    int lastSunday = lastSundayOfMonth(yearFull, 3);
    if (tm.Day > lastSunday) return offset + SECS_PER_HOUR;
    if (tm.Day == lastSunday && tm.Hour >= 1) return offset + SECS_PER_HOUR;
    return offset;
  } else {
    int lastSunday = lastSundayOfMonth(yearFull, 10);
    if (tm.Day < lastSunday) return offset + SECS_PER_HOUR;
    if (tm.Day == lastSunday && tm.Hour < 1) return offset + SECS_PER_HOUR;
    return offset;
  }
}

void sendNTPpacket(IPAddress &address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 56;
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

time_t getNtpTime() {
  if (wifiState != CONNECTED) return 0;
  IPAddress ntpServerIP;
  while (Udp.parsePacket() > 0) Udp.flush();
  if (!WiFi.hostByName(ntpServerName, ntpServerIP)) return 0;
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 3000) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      unsigned long secsSince1900 = ((unsigned long)packetBuffer[40] << 24) | ((unsigned long)packetBuffer[41] << 16) | ((unsigned long)packetBuffer[42] << 8) | ((unsigned long)packetBuffer[43]);
      time_t ntpTime = secsSince1900 - 2208988800UL;
      time_t adjustedTime = ntpTime + getMadridOffset(ntpTime);
      return adjustedTime;
    }
    delay(10);
  }
  return 0;
}

void syncNtpTime() {
  if (wifiState != CONNECTED) {
    if (status_label) lv_label_set_text(status_label, "NO HAY Wi-Fi PARA NTP.");
    return;
  }
  if (!Udp.begin(localPort)) {
    if (status_label) lv_label_set_text(status_label, "UDP FALLO INICIALIZACION.");
    return;
  }
  udp_active = true;
  time_t ntpTime = getNtpTime();
  Udp.stop();
  udp_active = false;
  if (ntpTime != 0) {
    setTime(ntpTime);
    if (status_label) lv_label_set_text(status_label, "HORA SINCRONIZADA OK!");
    setRtcTime(year(), month(), day(), hour(), minute(), second());
  } else {
    if (status_label) lv_label_set_text(status_label, "FALLO SINCRONIZACION HORA.");
  }
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  M5.Display.pushImage(area->x1, area->y1, w, h, (uint16_t *)color_p);
  lv_disp_flush_ready(disp);
}

void my_touchpad_read_cb(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint16_t touchX, touchY;
  bool pressed = M5.Display.getTouch(&touchX, &touchY);
  if (pressed) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void update_rtc_label() {
  if (lv_scr_act() != main_screen) return;
  m5::rtc_datetime_t dt = getRtcTime();
  char date_buf[32], time_buf[32];
  sprintf(date_buf, "%02d-%02d-%04d", dt.date.date, dt.date.month, dt.date.year);
  sprintf(time_buf, "%02d:%02d:%02d", dt.time.hours, dt.time.minutes, dt.time.seconds);
  if (rtc_date_label) lv_label_set_text(rtc_date_label, date_buf);
  if (rtc_time_label) lv_label_set_text(rtc_time_label, time_buf);
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START: wifiState = CONNECTING; break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiState = CONNECTED;
      ntpSyncNeeded = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      wifiState = DISCONNECTED;
      if (udp_active) {
        Udp.stop();
        udp_active = false;
      }
      WiFi.reconnect();
      break;
    default: break;
  }
}

void startWiFiConnection(const char *ssid, const char *password) {
  if (!ssid || strlen(ssid) == 0) return;
  if (WiFi.status() == WL_CONNECTED) return;
  if (wifiState == CONNECTING) return;
  WiFi.disconnect(false);
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("Tab5-Fermin");
  if (password && strlen(password) > 0) {
    WiFi.begin(ssid, password);
  } else {
    WiFi.begin(ssid);
  }
  wifiState = CONNECTING;
  lastConnectionAttempt = millis();
}

void select_wifi_cb(lv_event_t *e) {
  lv_obj_t *obj = lv_event_get_target(e);
  lv_obj_t *label = lv_obj_get_child(obj, 1);
  const char *ssid = lv_label_get_text(label);
  lv_textarea_set_text(ssid_input, ssid);
  lv_label_set_text_fmt(status_label, "SSID SELECCIONADO: %s", ssid);
  if (wifi_list) {
    lv_obj_del(wifi_list);
    wifi_list = NULL;
  }
  if (list_container) lv_obj_add_flag(list_container, LV_OBJ_FLAG_HIDDEN);
}

void scan_wifi_cb(lv_event_t *e) {
  lv_label_set_text(status_label, "ESCANEANDO REDES...");
  if (wifi_list) {
    lv_obj_del(wifi_list);
    wifi_list = NULL;
  }
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) {
    lv_label_set_text(status_label, "NO SE ENCONTRARON REDES.");
    WiFi.scanDelete();
    return;
  }
  lv_label_set_text_fmt(status_label, "%d REDES ENCONTRADAS.", n);
  lv_obj_clear_flag(list_container, LV_OBJ_FLAG_HIDDEN);
  wifi_list = lv_list_create(list_container);
  lv_obj_set_size(wifi_list, LV_PCT(100), LV_PCT(100));
  lv_obj_align(wifi_list, LV_ALIGN_CENTER, 0, 0);
  for (int i = 0; i < n; ++i) {
    String ssid = WiFi.SSID(i);
    lv_obj_t *btn = lv_list_add_btn(wifi_list, LV_SYMBOL_WIFI, ssid.c_str());
    lv_obj_add_event_cb(btn, select_wifi_cb, LV_EVENT_CLICKED, NULL);
  }
  lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
  WiFi.scanDelete();
}

void connect_wifi_cb(lv_event_t *e) {
  saved_ssid = lv_textarea_get_text(ssid_input);
  saved_password = lv_textarea_get_text(pass_input);
  saved_ssid.trim();
  saved_password.trim();
  if (saved_ssid.length() == 0) {
    lv_label_set_text(status_label, "SSID VACIO!");
    return;
  }
  preferences.begin("wifi", false);
  preferences.putString("ssid", saved_ssid);
  preferences.putString("pass", saved_password);
  preferences.end();
  lv_label_set_text_fmt(status_label, "CONECTANDO A %s...", saved_ssid.c_str());
  startWiFiConnection(saved_ssid.c_str(), saved_password.c_str());
}

void goto_wifi_cb(lv_event_t *e) {
  lv_textarea_set_text(ssid_input, saved_ssid.c_str());
  lv_textarea_set_text(pass_input, saved_password.c_str());
  lv_scr_load(wifi_screen);
  if (WiFi.status() == WL_CONNECTED) {
    lv_label_set_text_fmt(status_label, "CONECTADO: %s", WiFi.localIP().toString().c_str());
  } else if (wifiState == CONNECTING) {
    lv_label_set_text(status_label, "ESTADO: CONECTANDO...");
  } else {
    lv_label_set_text(status_label, "ESTADO: NO CONECTADO");
  }
}

void goto_config_cb(lv_event_t *e) {
  lv_scr_load(config_screen);
}
void goto_forecast_cb(lv_event_t *e) {
  lv_scr_load(forecast_screen);
  update_5day_forecast();
}
void goto_calendar_cb(lv_event_t *e) {
  lv_scr_load(calendar_screen);
}

void back_to_main_from_wifi(lv_event_t *e) {
  if (list_container) lv_obj_add_flag(list_container, LV_OBJ_FLAG_HIDDEN);
  lv_scr_load(main_screen);
}
void back_to_main_from_config(lv_event_t *e) {
  lv_scr_load(main_screen);
}

// ---- HANDLERS PARA LA PANTALLA DE CONFIGURACIÓN METEO ----
void goto_weather_config_cb(lv_event_t *e) {
  lv_textarea_set_text(city_input, weatherCity.c_str());
  lv_textarea_set_text(country_input, weatherCountry.c_str());
  lv_textarea_set_text(apikey_input, weatherApiKey.c_str());
  lv_label_set_text(weather_config_status_label, "DATOS OpenWeatherMap");
  lv_scr_load(weather_config_screen);
}

void save_weather_config_cb(lv_event_t *e) {
  weatherCity = lv_textarea_get_text(city_input);
  weatherCountry = lv_textarea_get_text(country_input);
  weatherApiKey = lv_textarea_get_text(apikey_input);
  weatherCity.trim();
  weatherCountry.trim();
  weatherApiKey.trim();

  preferences.begin("weather_cfg", false);
  preferences.putString("city", weatherCity);
  preferences.putString("country", weatherCountry);
  preferences.putString("apikey", weatherApiKey);
  preferences.end();

  lv_label_set_text(weather_config_status_label, "CONFIGURACION GUARDADA!");

  lastWeatherUpdate = 0;
}

void slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int value = lv_slider_get_value(slider);
  M5.Display.setBrightness(value);
  saved_brightness = value;
  lv_label_set_text_fmt(slider_label, "BRILLO: %d", value);
  preferences.begin("wifi", false);
  preferences.putInt("brightness", saved_brightness);
  preferences.end();
}

void temp_disp_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int value = lv_slider_get_value(slider);
  saved_temp_display_secs = value;
  if (temp_disp_label) lv_label_set_text_fmt(temp_disp_label, "ACTUALIZACION TEMP: %d SEG", value);
  preferences.begin("wifi", false);
  preferences.putInt("temp_disp_secs", value);
  preferences.end();
}

void graph_upd_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int value = lv_slider_get_value(slider);
  saved_graph_update_mins = value;
  if (graph_upd_label) lv_label_set_text_fmt(graph_upd_label, "ACTUALIZACION GRAFICA: %d MIN", value);
  preferences.begin("wifi", false);
  preferences.putInt("graph_upd_mins", value);
  preferences.end();
}

void volume_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int value = lv_slider_get_value(slider);
  saved_volume = value;
  uint8_t speaker_vol = map(value, 0, 100, 0, 255);
  M5.Speaker.setVolume(speaker_vol);
  if (volume_label) lv_label_set_text_fmt(volume_label, "VOLUMEN: %d%%", value);
  preferences.begin("wifi", false);
  preferences.putInt("volume", saved_volume);
  preferences.end();
}

void temp_max_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int valuemax = lv_slider_get_value(slider);
  saved_temp_max = (float)valuemax;
  if (temp_max_label) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "TEMP MAX ALARMA: %.1f °C", saved_temp_max);
    lv_label_set_text(temp_max_label, buffer);
  }
  preferences.begin("wifi", false);
  preferences.putFloat("temp_max", saved_temp_max);
  preferences.end();
}

void temp_min_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int valuemin = lv_slider_get_value(slider);
  saved_temp_min = (float)valuemin;
  if (temp_min_label) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "TEMP MIN ALARMA: %d C", (int)saved_temp_min);
    lv_label_set_text(temp_min_label, buffer);
  }
  preferences.begin("wifi", false);
  preferences.putFloat("temp_min", saved_temp_min);
  preferences.end();
}

void create_status_bar() {
  lv_obj_t *top_layer = lv_layer_top();
  const int BUTTON_WIDTH = 200;
  const int BUTTON_MARGIN_RIGHT = 20;
  const int ICON_GAP_FROM_BUTTON = 40;
  const int ICON_MARGIN_X = BUTTON_MARGIN_RIGHT + ICON_GAP_FROM_BUTTON + BUTTON_WIDTH;
  wifi_icon = lv_label_create(top_layer);
  lv_label_set_text(wifi_icon, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_font(wifi_icon, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0x888888), LV_PART_MAIN);
  lv_obj_align(wifi_icon, LV_ALIGN_TOP_RIGHT, -(ICON_MARGIN_X), 50);
  bat_icon = lv_label_create(top_layer);
  lv_label_set_text(bat_icon, LV_SYMBOL_BATTERY_FULL);
  lv_obj_set_style_text_font(bat_icon, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_set_style_text_color(bat_icon, lv_color_make(0, 255, 0), LV_PART_MAIN);
  lv_obj_align_to(bat_icon, wifi_icon, LV_ALIGN_OUT_RIGHT_MID, +50, 0);
  bat_percent_label = lv_label_create(top_layer);
  lv_label_set_text(bat_percent_label, "0%");
  lv_obj_set_style_text_font(bat_percent_label, &lv_font_montserrat_34, LV_PART_MAIN);
  lv_obj_set_style_text_color(bat_percent_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align_to(bat_percent_label, bat_icon, LV_ALIGN_OUT_RIGHT_MID, +25, 0);
}

// --- Ajuste de botones en la pantalla principal ---
void create_main_screen() {
  main_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x111111), LV_PART_MAIN);

  const int btn_width = 210;
  const int btn_height = 80;
  const int btn_x = 20;
  const int btn_y_start = 60;
  const int btn_y_gap = 50;

  // Botón WIFI
  btn_wifi = lv_btn_create(main_screen);
  lv_obj_set_size(btn_wifi, btn_width, btn_height);
  lv_obj_align(btn_wifi, LV_ALIGN_TOP_LEFT, btn_x, btn_y_start);
  lv_obj_set_style_bg_color(btn_wifi, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_t *label_wifi = lv_label_create(btn_wifi);
  lv_label_set_text(label_wifi, "WIFI");
  lv_obj_set_style_text_font(label_wifi, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_center(label_wifi);
  lv_obj_add_event_cb(btn_wifi, goto_wifi_cb, LV_EVENT_CLICKED, NULL);

  // Botón CONFIG
  btn_config = lv_btn_create(main_screen);
  lv_obj_set_size(btn_config, btn_width, btn_height);
  lv_obj_align(btn_config, LV_ALIGN_TOP_LEFT, btn_x, btn_y_start + 1 * (btn_height + btn_y_gap));
  lv_obj_set_style_bg_color(btn_config, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_t *label_config = lv_label_create(btn_config);
  lv_label_set_text(label_config, "CONFIG");
  lv_obj_set_style_text_font(label_config, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_center(label_config);
  lv_obj_add_event_cb(btn_config, goto_config_cb, LV_EVENT_CLICKED, NULL);

  // Botón TEMP
  btn_temp_main = lv_btn_create(main_screen);
  lv_obj_set_size(btn_temp_main, btn_width, btn_height);
  lv_obj_align(btn_temp_main, LV_ALIGN_TOP_LEFT, btn_x, btn_y_start + 2 * (btn_height + btn_y_gap));
  lv_obj_set_style_bg_color(btn_temp_main, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_t *label_temp_btn = lv_label_create(btn_temp_main);
  lv_label_set_text(label_temp_btn, "TEMP");
  lv_obj_set_style_text_font(label_temp_btn, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_center(label_temp_btn);

  // Botón METEO
  btn_weather_main = lv_btn_create(main_screen);
  lv_obj_set_size(btn_weather_main, btn_width, btn_height);
  lv_obj_align(btn_weather_main, LV_ALIGN_TOP_LEFT, btn_x, btn_y_start + 3 * (btn_height + btn_y_gap));
  lv_obj_set_style_bg_color(btn_weather_main, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_t *label_weather_btn = lv_label_create(btn_weather_main);
  lv_label_set_text(label_weather_btn, "METEO");
  lv_obj_set_style_text_font(label_weather_btn, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_center(label_weather_btn);

  // --- AÑADIDO: Nuevo botón CALENDARIO ---
  btn_calendar = lv_btn_create(main_screen);
  lv_obj_set_size(btn_calendar, btn_width, btn_height);
  lv_obj_align(btn_calendar, LV_ALIGN_TOP_LEFT, btn_x, btn_y_start + 4 * (btn_height + btn_y_gap));
  lv_obj_set_style_bg_color(btn_calendar, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_t *label_calendar_btn = lv_label_create(btn_calendar);
  lv_label_set_text(label_calendar_btn, "CALENDARIO");
  lv_obj_set_style_text_font(label_calendar_btn, &lv_font_montserrat_28, LV_PART_MAIN);  // Un poco más pequeño para que quepa
  lv_obj_center(label_calendar_btn);
  lv_obj_add_event_cb(btn_calendar, goto_calendar_cb, LV_EVENT_CLICKED, NULL);

  // Reloj y fecha
  rtc_date_label = lv_label_create(main_screen);
  lv_label_set_text(rtc_date_label, "00-00-0000");
  lv_obj_set_style_text_font(rtc_date_label, &mini100, LV_PART_MAIN);
  lv_obj_set_style_text_color(rtc_date_label, LV_COLOR_BLANCO, LV_PART_MAIN);
  lv_obj_align(rtc_date_label, LV_ALIGN_CENTER, 0, -160);

  rtc_time_label = lv_label_create(main_screen);
  lv_label_set_text(rtc_time_label, "00:00:00");
  lv_obj_set_style_text_font(rtc_time_label, &mini100, LV_PART_MAIN);
  lv_obj_set_style_text_color(rtc_time_label, LV_COLOR_BLANCO, LV_PART_MAIN);
  lv_obj_align(rtc_time_label, LV_ALIGN_CENTER, 0, 40);

  // Fase lunar
  moon_icon = lv_img_create(main_screen);
  lv_obj_align(moon_icon, LV_ALIGN_CENTER, 0, 180);

  moon_label = lv_label_create(main_screen);
  lv_label_set_text(moon_label, "CARGANDO FASE...");
  lv_obj_set_style_text_font(moon_label, &lv_font_montserrat_42, LV_PART_MAIN);
  lv_obj_set_style_text_color(moon_label, LV_COLOR_AMARILLO_PALIDO, LV_PART_MAIN);
  lv_obj_align(moon_label, LV_ALIGN_CENTER, 0, 290);
}

void create_wifi_screen() {
  wifi_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_screen, lv_color_black(), LV_PART_MAIN);

  // --- Elementos que ya estaban bien ---
  status_label = lv_label_create(wifi_screen);
  lv_label_set_text(status_label, "ESTADO: NO CONECTADO");
  lv_obj_set_style_text_color(status_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(status_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 110);

  btn_back_wifi = lv_btn_create(wifi_screen);
  lv_obj_set_size(btn_back_wifi, 200, 80);
  lv_obj_align(btn_back_wifi, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back_wifi, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back_wifi = lv_label_create(btn_back_wifi);
  lv_label_set_text(label_back_wifi, "VOLVER");
  lv_obj_set_style_text_font(label_back_wifi, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(label_back_wifi, lv_color_white(), LV_PART_MAIN);
  lv_obj_center(label_back_wifi);
  lv_obj_add_event_cb(btn_back_wifi, back_to_main_from_wifi, LV_EVENT_CLICKED, NULL);

  scan_btn = lv_btn_create(wifi_screen);
  lv_obj_set_size(scan_btn, 200, 55);
  lv_obj_align(scan_btn, LV_ALIGN_TOP_MID, 0, 20);
  lv_obj_set_style_bg_color(scan_btn, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_scan_btn = lv_label_create(scan_btn);
  lv_label_set_text(label_scan_btn, "ESCANEAR");
  lv_obj_set_style_text_font(label_scan_btn, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(label_scan_btn, lv_color_white(), LV_PART_MAIN);
  lv_obj_center(label_scan_btn);
  lv_obj_add_event_cb(scan_btn, scan_wifi_cb, LV_EVENT_CLICKED, NULL);

  // --- MODIFICACIONES AQUÍ ---
  // Ya no creamos el 'container' intermedio.

  // 1. Cambiamos el padre de 'container' a 'wifi_screen'
  ssid_input = lv_textarea_create(wifi_screen);
  lv_obj_set_width(ssid_input, LV_PCT(90));  // Un poco menos de ancho para centrarlo bien
  lv_obj_set_style_text_font(ssid_input, &lv_font_montserrat_30, LV_PART_MAIN);
  lv_textarea_set_placeholder_text(ssid_input, "SSID");
  // 2. Ajustamos la alineación vertical (190 - 10 = 180)
  lv_obj_align(ssid_input, LV_ALIGN_TOP_MID, 0, 180);
  lv_textarea_set_one_line(ssid_input, true);
  lv_obj_set_style_bg_color(ssid_input, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_color(ssid_input, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_border_color(ssid_input, lv_color_white(), LV_PART_MAIN);

  // 1. Cambiamos el padre de 'container' a 'wifi_screen'
  pass_input = lv_textarea_create(wifi_screen);
  lv_obj_set_width(pass_input, LV_PCT(90));
  lv_obj_set_style_text_font(pass_input, &lv_font_montserrat_30, LV_PART_MAIN);
  lv_textarea_set_placeholder_text(pass_input, "PASSWORD");
  // 2. Ajustamos la alineación vertical (190 + 50 = 240)
  lv_obj_align(pass_input, LV_ALIGN_TOP_MID, 0, 240);
  lv_textarea_set_one_line(pass_input, true);
  lv_textarea_set_password_mode(pass_input, true);
  lv_obj_set_style_bg_color(pass_input, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_color(pass_input, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_border_color(pass_input, lv_color_white(), LV_PART_MAIN);

  // 1. Cambiamos el padre de 'container' a 'wifi_screen'
  lv_obj_t *btn_connect = lv_btn_create(wifi_screen);
  lv_obj_set_size(btn_connect, 300, 55);
  // 2. Ajustamos la alineación vertical (190 + 120 = 310)
  lv_obj_align(btn_connect, LV_ALIGN_TOP_MID, 0, 310);
  lv_obj_set_style_bg_color(btn_connect, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_t *label_btn = lv_label_create(btn_connect);
  lv_label_set_text(label_btn, "CONECTAR WiFi");
  lv_obj_set_style_text_font(label_btn, &lv_font_montserrat_30, LV_PART_MAIN);
  lv_obj_set_style_text_color(label_btn, lv_color_white(), LV_PART_MAIN);
  lv_obj_center(label_btn);
  lv_obj_add_event_cb(btn_connect, connect_wifi_cb, LV_EVENT_CLICKED, NULL);

  // --- El resto del código sigue igual ---
  keyboard = lv_keyboard_create(wifi_screen);
  lv_obj_set_size(keyboard, EXAMPLE_LCD_H_RES, 300);
  lv_obj_align(keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_keyboard_set_textarea(keyboard, ssid_input);
  lv_obj_set_style_bg_color(keyboard, lv_color_hex(0x222222), LV_PART_MAIN);
  lv_obj_set_style_border_width(keyboard, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(keyboard, lv_color_hex(0x444444), LV_PART_ITEMS);
  lv_obj_set_style_text_color(keyboard, lv_color_white(), LV_PART_ITEMS);
  lv_obj_set_style_text_font(keyboard, &lv_font_montserrat_32, LV_PART_ITEMS);

  lv_obj_add_event_cb(
    ssid_input, [](lv_event_t *e) {
      lv_keyboard_set_textarea(keyboard, ssid_input);
      lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
      if (list_container) lv_obj_add_flag(list_container, LV_OBJ_FLAG_HIDDEN);
    },
    LV_EVENT_CLICKED, NULL);

  lv_obj_add_event_cb(
    pass_input, [](lv_event_t *e) {
      lv_keyboard_set_textarea(keyboard, pass_input);
      lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
      if (list_container) lv_obj_add_flag(list_container, LV_OBJ_FLAG_HIDDEN);
    },
    LV_EVENT_CLICKED, NULL);

  list_container = lv_list_create(wifi_screen);
  lv_obj_set_size(list_container, 900, 450);
  lv_obj_align(list_container, LV_ALIGN_CENTER, 0, 150);
  lv_obj_set_style_bg_color(list_container, lv_color_hex(0x222222), LV_PART_MAIN);
  lv_obj_set_style_border_width(list_container, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(list_container, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
  lv_obj_add_flag(list_container, LV_OBJ_FLAG_HIDDEN);
}

void create_config_screen() {
  config_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(config_screen, lv_color_hex(0x111111), LV_PART_MAIN);
  btn_back_config = lv_btn_create(config_screen);
  lv_obj_set_size(btn_back_config, 200, 80);
  lv_obj_align(btn_back_config, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back_config, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back_config = lv_label_create(btn_back_config);
  lv_label_set_text(label_back_config, "VOLVER");
  lv_obj_set_style_text_font(label_back_config, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_back_config);
  lv_obj_add_event_cb(btn_back_config, back_to_main_from_config, LV_EVENT_CLICKED, NULL);
  lv_obj_t *slider = lv_slider_create(config_screen);
  lv_obj_set_size(slider, 400, 30);
  lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 60);
  lv_slider_set_range(slider, 0, 255);
  lv_slider_set_value(slider, saved_brightness, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider, LV_COLOR_VERDE_MAR, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider, LV_COLOR_VERDE_MAR, LV_PART_KNOB);
  slider_label = lv_label_create(config_screen);
  lv_label_set_text_fmt(slider_label, "BRILLO: %d", saved_brightness);
  lv_obj_set_style_text_color(slider_label, LV_COLOR_VERDE_MAR, LV_PART_MAIN);
  lv_obj_align(slider_label, LV_ALIGN_TOP_MID, 0, 110);
  lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  volume_slider = lv_slider_create(config_screen);
  lv_obj_set_size(volume_slider, 400, 30);
  lv_obj_align(volume_slider, LV_ALIGN_TOP_MID, 0, 160);
  lv_slider_set_range(volume_slider, 0, 100);
  lv_slider_set_value(volume_slider, saved_volume, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(volume_slider, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(volume_slider, LV_COLOR_MANDARINA, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(volume_slider, LV_COLOR_MANDARINA, LV_PART_KNOB);
  volume_label = lv_label_create(config_screen);
  lv_label_set_text_fmt(volume_label, "VOLUMEN: %d%%", saved_volume);
  lv_obj_set_style_text_color(volume_label, LV_COLOR_MANDARINA, LV_PART_MAIN);
  lv_obj_align(volume_label, LV_ALIGN_TOP_MID, 0, 210);
  lv_obj_add_event_cb(volume_slider, volume_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  temp_disp_slider = lv_slider_create(config_screen);
  lv_obj_set_size(temp_disp_slider, 400, 30);
  lv_obj_align(temp_disp_slider, LV_ALIGN_TOP_MID, 0, 260);
  lv_slider_set_range(temp_disp_slider, 1, 30);
  lv_slider_set_value(temp_disp_slider, saved_temp_display_secs, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(temp_disp_slider, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(temp_disp_slider, LV_COLOR_PLATA, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(temp_disp_slider, LV_COLOR_PLATA, LV_PART_KNOB);
  temp_disp_label = lv_label_create(config_screen);
  lv_obj_set_style_text_color(temp_disp_label, LV_COLOR_PLATA, LV_PART_MAIN);
  lv_label_set_text_fmt(temp_disp_label, "ACTUALIZACION TEMP: %d SEG", saved_temp_display_secs);
  lv_obj_align(temp_disp_label, LV_ALIGN_TOP_MID, 0, 310);
  lv_obj_add_event_cb(temp_disp_slider, temp_disp_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  graph_upd_slider = lv_slider_create(config_screen);
  lv_obj_set_size(graph_upd_slider, 400, 30);
  lv_obj_align(graph_upd_slider, LV_ALIGN_TOP_MID, 0, 360);
  lv_slider_set_range(graph_upd_slider, 1, 60);
  lv_slider_set_value(graph_upd_slider, saved_graph_update_mins, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(graph_upd_slider, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(graph_upd_slider, LV_COLOR_AZUL_CIELO, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(graph_upd_slider, LV_COLOR_AZUL_CIELO, LV_PART_KNOB);
  graph_upd_label = lv_label_create(config_screen);
  lv_obj_set_style_text_color(graph_upd_label, LV_COLOR_AZUL_CIELO, LV_PART_MAIN);
  lv_label_set_text_fmt(graph_upd_label, "ACTUALIZACION GRAFICA: %d MIN", saved_graph_update_mins);
  lv_obj_align(graph_upd_label, LV_ALIGN_TOP_MID, 0, 410);
  lv_obj_add_event_cb(graph_upd_slider, graph_upd_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  temp_max_slider = lv_slider_create(config_screen);
  lv_obj_set_size(temp_max_slider, 400, 30);
  lv_obj_align(temp_max_slider, LV_ALIGN_TOP_MID, 0, 460);
  lv_slider_set_range(temp_max_slider, -20, 60);
  lv_slider_set_value(temp_max_slider, (int)saved_temp_max, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(temp_max_slider, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(temp_max_slider, lv_color_hex(0xFF0000), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(temp_max_slider, lv_color_hex(0xFF0000), LV_PART_KNOB);
  temp_max_label = lv_label_create(config_screen);
  lv_obj_set_style_text_color(temp_max_label, lv_color_hex(0xFF0000), LV_PART_MAIN);
  char max_buf[64];
  snprintf(max_buf, sizeof(max_buf), "TEMP MAX ALARMA: %d °C", (int)saved_temp_max);
  lv_label_set_text(temp_max_label, max_buf);
  lv_obj_align(temp_max_label, LV_ALIGN_TOP_MID, 0, 510);
  lv_obj_add_event_cb(temp_max_slider, temp_max_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  temp_min_slider = lv_slider_create(config_screen);
  lv_obj_set_size(temp_min_slider, 400, 30);
  lv_obj_align(temp_min_slider, LV_ALIGN_TOP_MID, 0, 560);
  lv_slider_set_range(temp_min_slider, -20, 60);
  lv_slider_set_value(temp_min_slider, (int)saved_temp_min, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(temp_min_slider, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(temp_min_slider, lv_color_hex(0x0066FF), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(temp_min_slider, lv_color_hex(0x0066FF), LV_PART_KNOB);
  temp_min_label = lv_label_create(config_screen);
  lv_obj_set_style_text_color(temp_min_label, lv_color_hex(0x0066FF), LV_PART_MAIN);
  char min_buf[64];
  snprintf(min_buf, sizeof(min_buf), "TEMP MIN ALARMA: %d °C", (int)saved_temp_min);
  lv_label_set_text(temp_min_label, min_buf);
  lv_obj_align(temp_min_label, LV_ALIGN_TOP_MID, 0, 610);
  lv_obj_add_event_cb(temp_min_slider, temp_min_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  bat_label = lv_label_create(config_screen);
  lv_obj_align(bat_label, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_set_style_text_color(bat_label, lv_color_white(), LV_PART_MAIN);
  lv_label_set_text(bat_label, "CARGANDO INFO...");
  lv_obj_set_style_text_font(bat_label, &lv_font_montserrat_28, LV_PART_MAIN);
}

void create_temp_screen() {
  temp_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(temp_screen, lv_color_hex(0x222222), LV_PART_MAIN);
  btn_back_temp = lv_btn_create(temp_screen);
  lv_obj_set_size(btn_back_temp, 200, 80);
  lv_obj_align(btn_back_temp, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back_temp, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back_temp = lv_label_create(btn_back_temp);
  lv_label_set_text(label_back_temp, "VOLVER");
  lv_obj_set_style_text_font(label_back_temp, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_back_temp);
  lv_obj_add_event_cb(
    btn_back_temp, [](lv_event_t *e) {
      lv_scr_load(main_screen);
    },
    LV_EVENT_CLICKED, NULL);
  temp_label = lv_label_create(temp_screen);
  lv_label_set_text(temp_label, "TEMPERATURA: --.- °C");
  lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(temp_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(temp_label, LV_ALIGN_TOP_MID, 0, 120);
  temp_min_display_label = lv_label_create(temp_screen);
  lv_label_set_text(temp_min_display_label, "TEMP MIN\n--.- °C");
  lv_obj_set_style_text_font(temp_min_display_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_color(temp_min_display_label, lv_color_hex(0x0066FF), LV_PART_MAIN);
  lv_obj_set_style_text_align(temp_min_display_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(temp_min_display_label, LV_ALIGN_TOP_LEFT, 30, 120);
  temp_max_display_label = lv_label_create(temp_screen);
  lv_label_set_text(temp_max_display_label, "TEMP MAX\n--.- °C");
  lv_obj_set_style_text_font(temp_max_display_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_color(temp_max_display_label, LV_COLOR_NARANJA_PURO, LV_PART_MAIN);
  lv_obj_set_style_text_align(temp_max_display_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(temp_max_display_label, LV_ALIGN_TOP_RIGHT, -30, 120);
  alarm_status_label = lv_label_create(temp_screen);
  lv_label_set_text(alarm_status_label, "TEMPERATURA NORMAL");
  lv_obj_set_style_text_font(alarm_status_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_color(alarm_status_label, lv_color_make(0, 255, 0), LV_PART_MAIN);
  lv_obj_align(alarm_status_label, LV_ALIGN_TOP_MID, 0, 180);
  const int AXIS_LABEL_SPACE = 80, BASE_X_OFFSET = 10, CHART_HEIGHT = 480;
  int CHART_WIDTH = EXAMPLE_LCD_H_RES - (AXIS_LABEL_SPACE + BASE_X_OFFSET + 10);
  temp_chart = lv_chart_create(temp_screen);
  lv_obj_set_size(temp_chart, CHART_WIDTH, CHART_HEIGHT);
  lv_obj_align(temp_chart, LV_ALIGN_TOP_LEFT, BASE_X_OFFSET + AXIS_LABEL_SPACE, 240);
  lv_chart_set_type(temp_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_update_mode(temp_chart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(temp_chart, 144);
  lv_chart_set_range(temp_chart, LV_CHART_AXIS_PRIMARY_Y, -20, 60);
  lv_chart_set_axis_tick(temp_chart, LV_CHART_AXIS_PRIMARY_Y, 5, 2, 9, 10, true, 50);
  lv_obj_set_style_pad_left(temp_chart, AXIS_LABEL_SPACE, LV_PART_MAIN);
  lv_obj_set_style_line_width(temp_chart, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(temp_chart, 0, LV_PART_MAIN);
  lv_obj_set_style_text_font(temp_chart, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_bg_color(temp_chart, lv_color_hex(0x333333), LV_PART_MAIN);
  temp_series = lv_chart_add_series(temp_chart, LV_COLOR_TURQUESA, LV_CHART_AXIS_PRIMARY_Y);
  for (int i = 0; i < 144; ++i) { lv_chart_set_next_value(temp_chart, temp_series, LV_CHART_POINT_NONE); }
}

String traducirCondicion(String mainCond, String descCond) {
  String desc = mainCond + " " + descCond;
  desc.toLowerCase();

  String descProfesional;

  if (desc.indexOf("clear sky") != -1 || desc.indexOf("cielo claro") != -1) descProfesional = "DESPEJADO";
  else if (desc.indexOf("few clouds") != -1 || desc.indexOf("algo de nubes") != -1 || desc.indexOf("pocas nubes") != -1) descProfesional = "POCO NUBOSO";
  else if (desc.indexOf("scattered clouds") != -1 || desc.indexOf("nubes dispersas") != -1) descProfesional = "NUBES DISPERSAS";
  else if (desc.indexOf("broken clouds") != -1 || desc.indexOf("nubes rotas") != -1) descProfesional = "NUBOSO";
  else if (desc.indexOf("overcast clouds") != -1 || desc.indexOf("muy nuboso") != -1) descProfesional = "MUY NUBOSO";
  else if (desc.indexOf("light rain") != -1 || desc.indexOf("lluvia ligera") != -1 || desc.indexOf("lluvia débil") != -1) descProfesional = "LLUVIA DEBIL";
  else if (desc.indexOf("moderate rain") != -1 || desc.indexOf("lluvia moderada") != -1) descProfesional = "LLUVIA MODERADA";
  else if (desc.indexOf("heavy intensity rain") != -1 || desc.indexOf("lluvia fuerte") != -1 || desc.indexOf("lluvia intensa") != -1) descProfesional = "LLUVIA FUERTE";
  else if (desc.indexOf("very heavy rain") != -1 || desc.indexOf("lluvia muy fuerte") != -1) descProfesional = "LLUVIA MUY FUERTE";
  else if (desc.indexOf("extreme rain") != -1 || desc.indexOf("lluvia extrema") != -1) descProfesional = "LLUVIA EXTREMA";
  else if (desc.indexOf("freezing rain") != -1 || desc.indexOf("lluvia helada") != -1) descProfesional = "LLUVIA HELADA";
  else if (desc.indexOf("shower rain") != -1 || desc.indexOf("chubascos") != -1) descProfesional = "CHUBASCOS";
  else if (desc.indexOf("drizzle") != -1) descProfesional = "LLOVIZNA";
  else if (desc.indexOf("light snow") != -1 || desc.indexOf("nieve ligera") != -1 || desc.indexOf("nieve débil") != -1) descProfesional = "NIEVE DEBIL";
  else if (desc.indexOf("heavy snow") != -1 || desc.indexOf("nieve fuerte") != -1) descProfesional = "NIEVE FUERTE";
  else if (desc.indexOf("snow") != -1 || desc.indexOf("nieve") != -1) descProfesional = "NIEVE";
  else if (desc.indexOf("sleet") != -1 || desc.indexOf("aguanieve") != -1) descProfesional = "AGUANIEVE";
  else if (desc.indexOf("thunderstorm") != -1 || desc.indexOf("tormenta") != -1) descProfesional = "TORMENTA";
  else if (desc.indexOf("mist") != -1 || desc.indexOf("neblina") != -1) descProfesional = "NEBLINA";
  else if (desc.indexOf("fog") != -1 || desc.indexOf("niebla") != -1) descProfesional = "NIEBLA";
  else if (desc.indexOf("haze") != -1 || desc.indexOf("calima") != -1) descProfesional = "CALIMA";
  else if (desc.indexOf("smoke") != -1 || desc.indexOf("humo") != -1) descProfesional = "HUMO";
  else if (desc.indexOf("dust") != -1 || desc.indexOf("polvo") != -1) descProfesional = "POLVO";
  else if (desc.indexOf("sand") != -1 || desc.indexOf("arena") != -1) descProfesional = "ARENA";
  else if (desc.indexOf("ash") != -1 || desc.indexOf("ceniza") != -1) descProfesional = "CENIZA";
  else if (desc.indexOf("squalls") != -1 || desc.indexOf("rachas de viento") != -1) descProfesional = "RACHAS";
  else if (desc.indexOf("tornado") != -1) descProfesional = "TORNADO";
  else {
    descProfesional = descCond;
    descProfesional.toUpperCase();
  }

  return descProfesional;
}

String get_weather_icon_path(String mainCond, String descCond, bool is_day, bool is_forecast) {
  mainCond.toLowerCase();
  descCond.toLowerCase();
  const String dir = is_forecast ? "/BINS128/" : "/BINS/";
  String night_prefix = "night_half_moon_";
  if (!is_day) {
    int fase = calcularFaseLunar();
    if (fase >= 12 && fase <= 17) night_prefix = "night_full_moon_";
    else night_prefix = "night_half_moon_";
  }
  if (mainCond == "clear") return is_day ? dir + "day_clear.bin" : dir + night_prefix + "clear.bin";
  if (mainCond == "clouds") {
    if (descCond.indexOf("few") >= 0 || descCond.indexOf("scattered") >= 0) return is_day ? dir + "day_partial_cloud.bin" : dir + night_prefix + "partial_cloud.bin";
    if (descCond.indexOf("broken") >= 0) return dir + "cloudy.bin";
    if (descCond.indexOf("overcast") >= 0) return dir + "overcast.bin";
    return dir + "cloudy.bin";
  }
  if (mainCond == "rain" || mainCond == "drizzle") {
    if (descCond.indexOf("thunder") >= 0) return is_day ? dir + "day_rain_thunder.bin" : dir + night_prefix + "rain_thunder.bin";
    return is_day ? dir + "day_rain.bin" : dir + night_prefix + "rain.bin";
  }
  if (mainCond == "thunderstorm") return dir + "thunder.bin";
  if (mainCond == "snow") return is_day ? dir + "day_snow.bin" : dir + night_prefix + "snow.bin";
  if (descCond.indexOf("sleet") >= 0) return is_day ? dir + "day_sleet.bin" : dir + night_prefix + "sleet.bin";
  if (mainCond == "fog") return dir + "fog.bin";
  if (mainCond == "mist" || mainCond == "haze") return dir + "mist.bin";
  if (mainCond == "tornado") return dir + "tornado.bin";
  return dir + "cloudy.bin";
}

void create_weather_screen() {
  weather_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(weather_screen, lv_color_hex(0x111111), LV_PART_MAIN);

  btn_back_weather = lv_btn_create(weather_screen);
  lv_obj_set_size(btn_back_weather, 200, 80);
  lv_obj_align(btn_back_weather, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back_weather, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back_weather = lv_label_create(btn_back_weather);
  lv_label_set_text(label_back_weather, "VOLVER");
  lv_obj_set_style_text_font(label_back_weather, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_back_weather);
  lv_obj_add_event_cb(
    btn_back_weather, [](lv_event_t *e) {
      lv_scr_load(main_screen);
    },
    LV_EVENT_CLICKED, NULL);

  btn_forecast = lv_btn_create(weather_screen);
  lv_obj_set_size(btn_forecast, 300, 80);
  lv_obj_align_to(btn_forecast, btn_back_weather, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
  lv_obj_set_style_bg_color(btn_forecast, LV_COLOR_AZUL_CIELO, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_forecast, LV_COLOR_AZUL_ROYAL, LV_STATE_PRESSED);
  lv_obj_t *label_forecast_btn = lv_label_create(btn_forecast);
  lv_label_set_text(label_forecast_btn, "PREVISION 5 DIAS");
  lv_obj_set_style_text_color(label_forecast_btn, LV_COLOR_NEGRO, LV_PART_MAIN);
  lv_obj_set_style_text_font(label_forecast_btn, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_forecast_btn);
  lv_obj_add_event_cb(btn_forecast, goto_forecast_cb, LV_EVENT_CLICKED, NULL);

  btn_weather_settings = lv_btn_create(weather_screen);
  lv_obj_set_size(btn_weather_settings, 200, 80);
  lv_obj_align(btn_weather_settings, LV_ALIGN_TOP_LEFT, 240, 20);
  lv_obj_set_style_bg_color(btn_weather_settings, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_settings_btn = lv_label_create(btn_weather_settings);
  lv_label_set_text(label_settings_btn, "AJUSTES");
  lv_obj_set_style_text_font(label_settings_btn, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_settings_btn);
  lv_obj_add_event_cb(btn_weather_settings, goto_weather_config_cb, LV_EVENT_CLICKED, NULL);

  weather_temp_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_temp_label, "--");
  lv_obj_set_style_text_font(weather_temp_label, &mini100, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_temp_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_temp_label, LV_ALIGN_TOP_MID, 0, 100);
  weather_cond_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_cond_label, "---");
  lv_obj_set_style_text_font(weather_cond_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_cond_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_cond_label, LV_ALIGN_TOP_MID, 0, 220);
  weather_hum_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_hum_label, "HUMEDAD: --");
  lv_obj_set_style_text_font(weather_hum_label, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_hum_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_hum_label, LV_ALIGN_TOP_LEFT, 100, 350);
  weather_press_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_press_label, "PRESION: --");
  lv_obj_set_style_text_font(weather_press_label, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_press_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_press_label, LV_ALIGN_TOP_LEFT, 100, 425);
  weather_wind_speed_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_wind_speed_label, "VIENTO: -- km/h");
  lv_obj_set_style_text_font(weather_wind_speed_label, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_wind_speed_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_wind_speed_label, LV_ALIGN_TOP_LEFT, 100, 500);
  weather_wind_dir_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_wind_dir_label, "DIRECCION: --");
  lv_obj_set_style_text_font(weather_wind_dir_label, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_wind_dir_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_wind_dir_label, LV_ALIGN_TOP_LEFT, 100, 575);
  weather_status_label = lv_label_create(weather_screen);
  lv_label_set_text(weather_status_label, "ACTUALIZANDO...");
  lv_obj_set_style_text_font(weather_status_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_color(weather_status_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(weather_status_label, LV_ALIGN_TOP_MID, 0, 20);
  weather_icon = lv_img_create(weather_screen);
  lv_obj_align(weather_icon, LV_ALIGN_TOP_MID, 400, 350);
}

void create_weather_config_screen() {
  weather_config_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(weather_config_screen, lv_color_black(), LV_PART_MAIN);

  weather_config_status_label = lv_label_create(weather_config_screen);
  lv_label_set_text(weather_config_status_label, "DATOS OpenWeatherMap");
  lv_obj_set_style_text_color(weather_config_status_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(weather_config_status_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_align(weather_config_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(weather_config_status_label, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *btn_back = lv_btn_create(weather_config_screen);
  lv_obj_set_size(btn_back, 200, 80);
  lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back = lv_label_create(btn_back);
  lv_label_set_text(label_back, "VOLVER");
  lv_obj_set_style_text_font(label_back, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_back);
  lv_obj_add_event_cb(
    btn_back, [](lv_event_t *e) {
      lv_scr_load(weather_screen);
    },
    LV_EVENT_CLICKED, NULL);

  lv_obj_t *container = lv_obj_create(weather_config_screen);
  lv_obj_set_size(container, LV_PCT(90), 300);
  lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 100);
  lv_obj_set_style_bg_color(container, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
  lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  city_input = lv_textarea_create(container);
  lv_obj_set_width(city_input, LV_PCT(95));
  lv_textarea_set_one_line(city_input, true);
  lv_textarea_set_placeholder_text(city_input, "Ciudad (ej: Durango)");
  lv_obj_set_style_text_font(city_input, &lv_font_montserrat_30, LV_PART_MAIN);

  country_input = lv_textarea_create(container);
  lv_obj_set_width(country_input, LV_PCT(95));
  lv_textarea_set_one_line(country_input, true);
  lv_textarea_set_placeholder_text(country_input, "País (ej: ES)");
  lv_obj_set_style_text_font(country_input, &lv_font_montserrat_30, LV_PART_MAIN);

  apikey_input = lv_textarea_create(container);
  lv_obj_set_width(apikey_input, LV_PCT(95));
  lv_textarea_set_one_line(apikey_input, true);
  lv_textarea_set_password_mode(apikey_input, true);
  lv_textarea_set_placeholder_text(apikey_input, "API OpenWeatherMap");
  lv_obj_set_style_text_font(apikey_input, &lv_font_montserrat_30, LV_PART_MAIN);

  lv_obj_t *btn_save = lv_btn_create(container);
  lv_obj_set_size(btn_save, 300, 60);
  lv_obj_set_style_bg_color(btn_save, lv_color_hex(0x008800), LV_PART_MAIN);
  lv_obj_t *label_save = lv_label_create(btn_save);
  lv_label_set_text(label_save, "GUARDAR");
  lv_obj_center(label_save);
  lv_obj_add_event_cb(btn_save, save_weather_config_cb, LV_EVENT_CLICKED, NULL);

  weather_config_keyboard = lv_keyboard_create(weather_config_screen);
  lv_obj_set_size(weather_config_keyboard, EXAMPLE_LCD_H_RES, 300);
  lv_obj_align(weather_config_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

  lv_obj_add_event_cb(
    city_input, [](lv_event_t *e) {
      lv_keyboard_set_textarea(weather_config_keyboard, city_input);
    },
    LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(
    country_input, [](lv_event_t *e) {
      lv_keyboard_set_textarea(weather_config_keyboard, country_input);
    },
    LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(
    apikey_input, [](lv_event_t *e) {
      lv_keyboard_set_textarea(weather_config_keyboard, apikey_input);
    },
    LV_EVENT_CLICKED, NULL);
}

// --- AÑADIDO: Nueva pantalla de calendario ---
void create_calendar_screen() {
  calendar_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(calendar_screen, lv_color_hex(0x111111), LV_PART_MAIN);

  // Botón para volver al menú principal
  lv_obj_t *btn_back_calendar = lv_btn_create(calendar_screen);
  lv_obj_set_size(btn_back_calendar, 200, 80);
  lv_obj_align(btn_back_calendar, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back_calendar, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back_calendar = lv_label_create(btn_back_calendar);
  lv_label_set_text(label_back_calendar, "VOLVER");
  lv_obj_set_style_text_font(label_back_calendar, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_back_calendar);
  lv_obj_add_event_cb(
    btn_back_calendar, [](lv_event_t *e) {
      lv_scr_load(main_screen);
    },
    LV_EVENT_CLICKED, NULL);

  // --- Widget del Calendario ---
  calendar_widget = lv_calendar_create(calendar_screen);

  lv_obj_set_size(calendar_widget, 1280, 580);
  lv_obj_align(calendar_widget, LV_ALIGN_TOP_MID, 0, 120);

  // Estilos generales del contenedor del calendario
  lv_obj_set_style_bg_color(calendar_widget, lv_color_hex(0x222222), LV_PART_MAIN);
  lv_obj_set_style_border_color(calendar_widget, lv_color_hex(0x666666), LV_PART_MAIN);
  lv_obj_set_style_border_width(calendar_widget, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(calendar_widget, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_all(calendar_widget, 10, LV_PART_MAIN);

  // Aumentamos el tamaño de la fuente de la cabecera del calendario
  lv_obj_set_style_text_color(calendar_widget, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(calendar_widget, &lv_font_montserrat_32, LV_PART_MAIN);

  // Estilo para los días (números y nombres de la semana)
  lv_obj_set_style_text_color(calendar_widget, lv_color_white(), LV_PART_ITEMS);
  lv_obj_set_style_text_font(calendar_widget, &lv_font_montserrat_28, LV_PART_ITEMS);

  // Estilo para el día seleccionado (al hacer clic)
  lv_obj_set_style_bg_color(calendar_widget, LV_COLOR_AZUL_ROYAL, LV_PART_ITEMS);
  lv_obj_set_style_text_color(calendar_widget, lv_color_white(), LV_PART_ITEMS);

  // --- ORDEN DE LOS DÍAS DE LA SEMANA ---
  const char *days_es[] = { "DO", "LU", "MA", "MI", "JU", "VI", "SA" };
  lv_calendar_set_day_names(calendar_widget, days_es);
}

void create_forecast_screen() {
  forecast_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(forecast_screen, lv_color_hex(0x111111), LV_PART_MAIN);
  lv_obj_t *btn_back_forecast = lv_btn_create(forecast_screen);
  lv_obj_set_size(btn_back_forecast, 200, 80);
  lv_obj_align(btn_back_forecast, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_bg_color(btn_back_forecast, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_t *label_back_forecast = lv_label_create(btn_back_forecast);
  lv_label_set_text(label_back_forecast, "VOLVER");
  lv_obj_set_style_text_font(label_back_forecast, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(label_back_forecast);
  lv_obj_add_event_cb(
    btn_back_forecast, [](lv_event_t *e) {
      lv_scr_load(weather_screen);
    },
    LV_EVENT_CLICKED, NULL);
  forecast_status_label = lv_label_create(forecast_screen);
  lv_label_set_text(forecast_status_label, "PREVISION 5 DIAS");
  lv_obj_set_style_text_font(forecast_status_label, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_set_style_text_color(forecast_status_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(forecast_status_label, LV_ALIGN_TOP_MID, 0, 20);
  lv_obj_t *main_container = lv_obj_create(forecast_screen);
  lv_obj_set_size(main_container, EXAMPLE_LCD_H_RES - 40, 600);
  lv_obj_align(main_container, LV_ALIGN_CENTER, 0, 40);
  lv_obj_set_style_bg_color(main_container, lv_color_hex(0x222222), LV_PART_MAIN);
  lv_obj_set_style_border_width(main_container, 0, LV_PART_MAIN);
  lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(main_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(main_container, 10, 0);
  for (int i = 0; i < 5; i++) {
    day_containers[i] = lv_obj_create(main_container);
    lv_obj_set_size(day_containers[i], 220, 500);
    lv_obj_set_style_bg_color(day_containers[i], lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_border_color(day_containers[i], lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_border_width(day_containers[i], 2, LV_PART_MAIN);
    lv_obj_set_style_radius(day_containers[i], 15, LV_PART_MAIN);
    lv_obj_set_flex_flow(day_containers[i], LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(day_containers[i], LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    day_labels[i] = lv_label_create(day_containers[i]);
    lv_label_set_text(day_labels[i], "---");
    lv_obj_set_style_text_font(day_labels[i], &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_set_style_text_color(day_labels[i], LV_COLOR_AZUL_CIELO, LV_PART_MAIN);
    day_icons[i] = lv_img_create(day_containers[i]);
    day_temp_labels[i] = lv_label_create(day_containers[i]);
    lv_label_set_text(day_temp_labels[i], "--° / --°");
    lv_obj_set_style_text_font(day_temp_labels[i], &lv_font_montserrat_40, LV_PART_MAIN);
    lv_obj_set_style_text_color(day_temp_labels[i], lv_color_white(), LV_PART_MAIN);
  }
}

void update_weather() {
  if (wifiState != CONNECTED) {
    if (weather_status_label) lv_label_set_text(weather_status_label, "NO HAY Wi-Fi");
    return;
  }
  if (weatherApiKey.length() < 10 || weatherCity.length() < 2) {
    if (weather_status_label) lv_label_set_text(weather_status_label, "FALTAN AJUSTES METEO");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String location_param = weatherCity + "," + weatherCountry;
  String url = String("https://api.openweathermap.org/data/2.5/weather?q=") + location_param + "&appid=" + weatherApiKey + "&units=metric&lang=es";

  http.begin(client, url);
  http.setTimeout(10000);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      if (weather_status_label) lv_label_set_text(weather_status_label, "ERROR EN JSON");
      http.end();
      return;
    }
    float temperaturew = doc["main"]["temp"];
    String mainCond = doc["weather"][0]["main"];
    String descCond = doc["weather"][0]["description"];
    String iconCode = doc["weather"][0]["icon"];
    bool is_day = (iconCode.endsWith("d"));
    String cond = traducirCondicion(mainCond, descCond);
    int hum = doc["main"]["humidity"];
    int press = doc["main"]["pressure"];
    float windSpeed_ms = doc["wind"]["speed"];
    int windDeg = doc["wind"]["deg"];
    float windSpeed_kmh = windSpeed_ms * 3.6;
    String windDir_str = traducirDireccionViento(windDeg);
    char tempw_buf[16];
    dtostrf(temperaturew, 4, 1, tempw_buf);
    char wind_buf[16];
    dtostrf(windSpeed_kmh, 4, 1, wind_buf);
    if (weather_temp_label) lv_label_set_text_fmt(weather_temp_label, "%s", tempw_buf);
    if (weather_cond_label) lv_label_set_text(weather_cond_label, cond.c_str());
    if (weather_hum_label) lv_label_set_text_fmt(weather_hum_label, "HUMEDAD: %d %%", hum);
    if (weather_press_label) lv_label_set_text_fmt(weather_press_label, "PRESION: %d Mb", press);
    if (weather_wind_speed_label) lv_label_set_text_fmt(weather_wind_speed_label, "VIENTO: %s km/h", wind_buf);
    if (weather_wind_dir_label) lv_label_set_text_fmt(weather_wind_dir_label, "DIRECCION: %s", windDir_str.c_str());
    if (weather_icon) {
      String rel_path = get_weather_icon_path(mainCond, descCond, is_day, false);
      String lv_path = String("S:") + rel_path;
      if (!lv_file_can_read(lv_path.c_str())) {
        if (weather_status_label) lv_label_set_text_fmt(weather_status_label, "ICONO NO ENCONTRADO");
      } else {
        if (lastWeatherIconPath != lv_path) {
          lv_img_set_src(weather_icon, lv_path.c_str());
          lastWeatherIconPath = lv_path;
        }
        if (weather_status_label) lv_label_set_text(weather_status_label, "ACTUALIZADO OK");
      }
    } else {
      if (weather_status_label) lv_label_set_text(weather_status_label, "ACTUALIZADO OK");
    }
  } else {
    if (weather_status_label) lv_label_set_text_fmt(weather_status_label, "ERROR HTTP: %d", httpCode);
  }
  http.end();
}

void update_5day_forecast() {
  if (wifiState != CONNECTED) {
    if (forecast_status_label) lv_label_set_text(forecast_status_label, "NO HAY Wi-Fi");
    return;
  }
  if (weatherApiKey.length() < 10 || weatherCity.length() < 2) {
    if (forecast_status_label) lv_label_set_text(forecast_status_label, "FALTAN AJUSTES METEO");
    return;
  }
  if (forecast_status_label) lv_label_set_text(forecast_status_label, "ACTUALIZANDO PREVISION...");
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String location_param = weatherCity + "," + weatherCountry;
  String url = String("https://api.openweathermap.org/data/2.5/forecast?q=") + location_param + "&appid=" + weatherApiKey + "&units=metric&lang=es";

  http.begin(client, url);
  http.setTimeout(10000);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(16384);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      if (forecast_status_label) lv_label_set_text(forecast_status_label, "ERROR EN JSON");
      http.end();
      return;
    }
    JsonArray list = doc["list"].as<JsonArray>();
    float daily_min[5] = { 999, 999, 999, 999, 999 };
    float daily_max[5] = { -999, -999, -999, -999, -999 };
    String daily_icon_main[5];
    String daily_icon_desc[5];
    time_t daily_ts[5] = { 0, 0, 0, 0, 0 };
    int current_day_index = -1;
    int last_day_num = -1;
    const char *dayNames[] = { "DOM", "LUN", "MAR", "MIE", "JUE", "VIE", "SAB" };
    for (JsonObject item : list) {
      time_t ts = item["dt"];
      tmElements_t tm;
      breakTime(ts, tm);
      int day_num = tm.Day;
      if (day_num != last_day_num) {
        current_day_index++;
        if (current_day_index >= 5) break;
        last_day_num = day_num;
        daily_ts[current_day_index] = ts;
      }
      float temp_min = item["main"]["temp_min"];
      float temp_max = item["main"]["temp_max"];
      if (temp_min < daily_min[current_day_index]) daily_min[current_day_index] = temp_min;
      if (temp_max > daily_max[current_day_index]) daily_max[current_day_index] = temp_max;
      if (tm.Hour == 12 || tm.Hour == 15) {
        daily_icon_main[current_day_index] = item["weather"][0]["main"].as<String>();
        daily_icon_desc[current_day_index] = item["weather"][0]["description"].as<String>();
      }
    }
    for (int i = 0; i < 5; i++) {
      if (daily_ts[i] == 0) continue;
      tmElements_t tm;
      breakTime(daily_ts[i], tm);
      lv_label_set_text(day_labels[i], dayNames[tm.Wday - 1]);
      char temp_buf[32];
      snprintf(temp_buf, sizeof(temp_buf), "%.0f°/%.0f°", daily_max[i], daily_min[i]);
      lv_label_set_text(day_temp_labels[i], temp_buf);
      String rel_path = get_weather_icon_path(daily_icon_main[i], daily_icon_desc[i], true, true);
      String lv_path = "S:" + rel_path;
      if (lastForecastIconPaths[i] != lv_path) {
        if (lv_file_can_read(lv_path.c_str())) {
          lv_img_set_src(day_icons[i], lv_path.c_str());
          lastForecastIconPaths[i] = lv_path;
        } else {
          //Serial.printf("Icono de previsión no encontrado: %s\n", lv_path.c_str());
        }
      }
    }
    if (forecast_status_label) lv_label_set_text(forecast_status_label, "PREVISION ACTUALIZADA");
  } else {
    if (forecast_status_label) lv_label_set_text_fmt(forecast_status_label, "ERROR HTTP: %d", httpCode);
  }
  http.end();
}

void ui_init() {
  create_main_screen();
  create_wifi_screen();
  create_config_screen();
  create_temp_screen();
  create_weather_screen();
  create_forecast_screen();
  create_weather_config_screen();
  create_calendar_screen();  // <-- AÑADIDO: Crear la nueva pantalla de calendario
  create_status_bar();

  if (btn_temp_main && temp_screen) {
    lv_obj_add_event_cb(
      btn_temp_main, [](lv_event_t *e) {
        lv_scr_load(temp_screen);
      },
      LV_EVENT_CLICKED, NULL);
  }
  if (btn_weather_main && weather_screen) {
    lv_obj_add_event_cb(
      btn_weather_main, [](lv_event_t *e) {
        lv_scr_load(weather_screen);
        update_weather();
        lastWeatherUpdate = millis();
      },
      LV_EVENT_CLICKED, NULL);
  }
  lv_scr_load(main_screen);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  //Serial.begin(115200);
  M5.Display.setRotation(3);
  lv_init();
  buf1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * LVGL_LCD_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * LVGL_LCD_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf1 || !buf2) {
    //Serial.println("ERROR: No se pudieron asignar buffers LVGL");
    while (true)
      ;
  }
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LVGL_LCD_BUF_SIZE);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read_cb;
  lv_indev_drv_register(&indev_drv);
  rtc_init();

  preferences.begin("wifi", false);
  saved_ssid = preferences.getString("ssid", "");
  saved_password = preferences.getString("pass", "");
  saved_brightness = preferences.getInt("brightness", 200);
  saved_temp_display_secs = preferences.getInt("temp_disp_secs", 1);
  saved_graph_update_mins = preferences.getInt("graph_upd_mins", 10);
  saved_volume = preferences.getInt("volume", 50);
  saved_temp_max = preferences.getFloat("temp_max", 30.0);
  saved_temp_min = preferences.getFloat("temp_min", 10.0);
  preferences.end();

  preferences.begin("weather_cfg", false);
  weatherCity = preferences.getString("city", "Durango");
  weatherCountry = preferences.getString("country", "ES");
  weatherApiKey = preferences.getString("apikey", "");
  preferences.end();

  WiFi.setPins(SDIO2_CLK, SDIO2_CMD, SDIO2_D0, SDIO2_D1, SDIO2_D2, SDIO2_D3, SDIO2_RST);
  WiFi.setAutoReconnect(true);
  startWiFiConnection(saved_ssid.c_str(), saved_password.c_str());
  WiFi.onEvent(WiFiEvent);
  M5.Display.setBrightness(saved_brightness);
  M5.Speaker.begin();
  uint8_t speaker_vol = map(saved_volume, 0, 100, 0, 255);
  M5.Speaker.setVolume(speaker_vol);
  m5::rtc_datetime_t dt = getRtcTime();
  if (dt.date.year < 2020 || dt.date.year > 2099) {
    if (saved_ssid.length() > 0) { startWiFiConnection(saved_ssid.c_str(), saved_password.c_str()); }
  } else {
    setTime(dt.time.hours, dt.time.minutes, dt.time.seconds, dt.date.date, dt.date.month, dt.date.year);
  }
  SPIFFS.begin();
  M5.Ex_I2C.begin();
  Wire.begin(M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL());
  tempsensor.begin(0x48, Wire);
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  if (!SD.begin(SD_SPI_CS_PIN, SPI, 40000000)) {
    //Serial.println("ERROR: Tarjeta SD no detectada");
  } else {
    //Serial.println("Tarjeta SD montada correctamente a 40MHz");
    lvgl_fs_init();
  }
  ui_init();

  // --- AÑADIDO: Inicialización del calendario con la fecha del RTC ---
  m5::rtc_datetime_t dt_init = getRtcTime();
  if (calendar_widget) {
    lv_calendar_set_today_date(calendar_widget, dt_init.date.year, dt_init.date.month, dt_init.date.date);
    lv_calendar_set_showed_date(calendar_widget, dt_init.date.year, dt_init.date.month);
  }

  update_rtc_label();
  updateMoonPhaseDisplay();
  lastTempUpdate = millis();
  lastChartUpdate = millis();
  tempAccum = 0.0f;
  tempAccumCount = 0;
}

void update_wifi_icon() {
  if (!wifi_icon) return;
  switch (wifiState) {
    case CONNECTED:
      lv_obj_clear_flag(wifi_icon, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0x00FF00), LV_PART_MAIN);
      break;
    case CONNECTING:
      if (millis() - lastBlink > 250) {
        lastBlink = millis();
        wifi_icon_visible = !wifi_icon_visible;
        if (wifi_icon_visible) {
          lv_obj_clear_flag(wifi_icon, LV_OBJ_FLAG_HIDDEN);
          lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0xFFFF00), LV_PART_MAIN);
        } else {
          lv_obj_add_flag(wifi_icon, LV_OBJ_FLAG_HIDDEN);
        }
      }
      break;
    case DISCONNECTED:
    default:
      lv_obj_clear_flag(wifi_icon, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0xFF0000), LV_PART_MAIN);
      break;
  }
}

void loop() {
  M5.update();
  lv_timer_handler();
  update_wifi_icon();
  if (ntpSyncNeeded && wifiState == CONNECTED) {
    ntpSyncNeeded = false;
    syncNtpTime();
  }
  if (wifiState == CONNECTING && (millis() - lastConnectionAttempt) > connectionTimeout) {
    WiFi.disconnect(false);
    wifiState = DISCONNECTED;
    if (status_label) lv_label_set_text(status_label, "FALLO DE CONEXION.");
  }

  static unsigned long lastRtcUpdate = 0;
  static int last_day_checked = -1;

  if (millis() - lastRtcUpdate >= 1000) {
    lastRtcUpdate = millis();
    update_rtc_label();

    m5::rtc_datetime_t dt = getRtcTime();
    if (dt.date.date != last_day_checked) {
      last_day_checked = dt.date.date;
      updateMoonPhaseDisplay();

      // --- AÑADIDO: Lógica de actualización del calendario ---
      if (calendar_widget) {
        lv_calendar_set_today_date(calendar_widget, dt.date.year, dt.date.month, dt.date.date);
        lv_calendar_set_showed_date(calendar_widget, dt.date.year, dt.date.month);
      }
    }
  }

  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate >= 1000) {
    lastStatusUpdate = millis();
    bool bat_ischarging = M5.Power.isCharging();
    int bat_mv = M5.Power.getBatteryVoltage();
    int bat_level = M5.Power.getBatteryLevel();
    if (bat_icon) {
      const char *bat_symbol;
      lv_color_t bat_color;
      if (bat_ischarging) {
        bat_symbol = LV_SYMBOL_CHARGE;
        bat_color = lv_color_make(0, 150, 255);
      } else if (bat_level > 75) {
        bat_symbol = LV_SYMBOL_BATTERY_FULL;
        bat_color = lv_color_make(0, 255, 0);
      } else if (bat_level > 40) {
        bat_symbol = LV_SYMBOL_BATTERY_3;
        bat_color = lv_color_make(255, 255, 0);
      } else if (bat_level > 10) {
        bat_symbol = LV_SYMBOL_BATTERY_1;
        bat_color = lv_color_make(255, 128, 0);
      } else {
        bat_symbol = LV_SYMBOL_BATTERY_EMPTY;
        bat_color = lv_color_make(255, 0, 0);
      }
      lv_label_set_text(bat_icon, bat_symbol);
      lv_obj_set_style_text_color(bat_icon, bat_color, LV_PART_MAIN);
    }
    if (bat_percent_label) {
      lv_label_set_text_fmt(bat_percent_label, "%d%%", bat_level);
      lv_obj_set_style_text_color(bat_percent_label, bat_level < 10 ? lv_color_make(255, 0, 0) : lv_color_white(), LV_PART_MAIN);
    }
    if (bat_label) { lv_label_set_text_fmt(bat_label, "BATERIA: %d.%dV (%d%%) - %s", bat_mv / 1000, (bat_mv % 1000) / 100, bat_level, bat_ischarging ? "CARGANDO" : "DESCARGANDO"); }
  }

  if (millis() - lastTempUpdate >= (unsigned long)saved_temp_display_secs * 1000UL) {
    lastTempUpdate = millis();
    float temperatura = tempsensor.readTempC();
    if (temperatura < current_temp_min) current_temp_min = temperatura;
    if (temperatura > current_temp_max) current_temp_max = temperatura;
    char temp_buf[16];
    dtostrf(temperatura, 4, 1, temp_buf);
    if (temp_label) {
      String tempStr = String("TEMPERATURA: ") + temp_buf + " °C";
      lv_label_set_text(temp_label, tempStr.c_str());
    }
    if (temp_min_display_label && current_temp_min < 999.0) {
      char min_buf[32], min_temp_str[16];
      dtostrf(current_temp_min, 4, 1, min_temp_str);
      snprintf(min_buf, sizeof(min_buf), "TEMP MIN\n%s °C", min_temp_str);
      lv_label_set_text(temp_min_display_label, min_buf);
    }
    if (temp_max_display_label && current_temp_max > -999.0) {
      char max_buf[32], max_temp_str[16];
      dtostrf(current_temp_max, 4, 1, max_temp_str);
      snprintf(max_buf, sizeof(max_buf), "TEMP MAX\n%s °C", max_temp_str);
      lv_label_set_text(temp_max_display_label, max_buf);
    }
    checkTemperatureAlarm(temperatura);
    tempAccum += temperatura;
    tempAccumCount++;
  }

  if (millis() - lastChartUpdate >= (unsigned long)saved_graph_update_mins * 60UL * 1000UL) {
    lastChartUpdate = millis();
    if (tempAccumCount > 0) {
      float avg = tempAccum / tempAccumCount;
      int avgInt = (int)round(avg);
      if (temp_chart && temp_series) {
        lv_chart_set_next_value(temp_chart, temp_series, avgInt);
        lv_chart_refresh(temp_chart);
      }
    }
    tempAccum = 0.0f;
    tempAccumCount = 0;
  }

  if (lv_scr_act() == weather_screen && millis() - lastWeatherUpdate >= weatherUpdateInterval) {
    lastWeatherUpdate = millis();
    update_weather();
  }
  delay(33);
}
