#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 10];
static lv_disp_drv_t disp_drv;

// UI
#include <ui.h>
#include "UI/ui_helpers.h"

#include <SPI.h>

SPIClass &spi = SPI;
uint16_t touchCalibration_x0 = 300, touchCalibration_x1 = 3600, touchCalibration_y0 = 300, touchCalibration_y1 = 3600;
uint8_t touchCalibration_rotate = 1, touchCalibration_invert_x = 2, touchCalibration_invert_y = 0;

int i = 0;
#define TFT_BL 2

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 0 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */
);

// option 1:
// 7寸 50PIN 800*480
Arduino_RPi_DPI_RGBPanel *lcd = new Arduino_RPi_DPI_RGBPanel(
    bus,
    800 /* width */, 1 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 40 /* hsync_back_porch */,
    480 /* height */, 1 /* vsync_polarity */, 13 /* vsync_front_porch */, 1 /* vsync_pulse_width */, 31 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

#include "touch.h"
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

void begin_touch_read_write(void)
{
  digitalWrite(38, HIGH); // Just in case it has been left low
  spi.setFrequency(600000);
  digitalWrite(38, LOW);
}

void end_touch_read_write(void)
{
  digitalWrite(38, HIGH); // Just in case it has been left low
  spi.setFrequency(600000);
}

uint16_t getTouchRawZ(void)
{

  begin_touch_read_write();

  // Z sample request
  int16_t tz = 0xFFF;
  spi.transfer(0xb0);              // Start new Z1 conversion
  tz += spi.transfer16(0xc0) >> 3; // Read Z1 and start Z2 conversion
  tz -= spi.transfer16(0x00) >> 3; // Read Z2

  end_touch_read_write();

  return (uint16_t)tz;
}

uint8_t getTouchRaw(uint16_t *x, uint16_t *y)
{
  uint16_t tmp;

  begin_touch_read_write();

  // Start YP sample request for x position, read 4 times and keep last sample
  spi.transfer(0xd0); // Start new YP conversion
  spi.transfer(0);    // Read first 8 bits
  spi.transfer(0xd0); // Read last 8 bits and start new YP conversion
  spi.transfer(0);    // Read first 8 bits
  spi.transfer(0xd0); // Read last 8 bits and start new YP conversion
  spi.transfer(0);    // Read first 8 bits
  spi.transfer(0xd0); // Read last 8 bits and start new YP conversion

  tmp = spi.transfer(0); // Read first 8 bits
  tmp = tmp << 5;
  tmp |= 0x1f & (spi.transfer(0x90) >> 3); // Read last 8 bits and start new XP conversion

  *x = tmp;

  // Start XP sample request for y position, read 4 times and keep last sample
  spi.transfer(0);    // Read first 8 bits
  spi.transfer(0x90); // Read last 8 bits and start new XP conversion
  spi.transfer(0);    // Read first 8 bits
  spi.transfer(0x90); // Read last 8 bits and start new XP conversion
  spi.transfer(0);    // Read first 8 bits
  spi.transfer(0x90); // Read last 8 bits and start new XP conversion

  tmp = spi.transfer(0); // Read first 8 bits
  tmp = tmp << 5;
  tmp |= 0x1f & (spi.transfer(0) >> 3); // Read last 8 bits

  *y = tmp;

  end_touch_read_write();

  return true;
}

int speedValue = 0;
bool incrementingSpeed = true;
const int speedInterval = 30;
const int SPEED_INCREMENT = 1;
const int MAX_SPEED = 45;
const int MIN_SPEED = 0;

float fraction = 0.0f; // Fraction of the animation cycle, varies from 0 to 1 and back to 0

void updateSpeed()
{
  const float FRACTION_INCREMENT = 0.005f;

  if (incrementingSpeed)
  {
    fraction += FRACTION_INCREMENT;
    if (fraction >= 1.0f)
    {
      fraction = 1.0f;
      incrementingSpeed = false;
    }
  }
  else
  {
    fraction -= FRACTION_INCREMENT;
    if (fraction <= 0.0f)
    {
      fraction = 0.0f;
      incrementingSpeed = true;
    }
  }

  // Applying the sine function for ease-in-out effect over the complete cycle
  speedValue = (int)(MAX_SPEED * (sin((fraction - 0.5f) * M_PI) + 1) / 2);

  lv_arc_set_value(ui_ArcSpeed, speedValue);
  lv_label_set_text_fmt(ui_LabelSpeed, "%d", speedValue);
}

void lvgl_loop(void *parameter)
{
  while (true)
  {
    lv_timer_handler();
  }
  vTaskDelete(NULL);
}

void guiHandler()
{
  xTaskCreatePinnedToCore(
      lvgl_loop,   // Function that should be called
      "LVGL Loop", // Name of the task (for debugging)
      20480,       // Stack size (bytes)
      NULL,        // Parameter to pass
      1,           // Task priority
      NULL,        // Task handle
      1);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200); // Init Display

  lcd->begin();
  lcd->fillScreen(BLACK);
  lcd->setTextSize(2);
  delay(200);

  lv_init();
  touch_init();
  screenWidth = lcd->width();
  screenHeight = lcd->height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);

  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  // Back light
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  lcd->fillScreen(BLACK);

  ui_init();
  guiHandler();
}

unsigned long previousMillis = 0;

void loop()
{
  unsigned long currentMillis = millis();

  // Calculate the time elapsed since the last update
  unsigned long timeElapsed = currentMillis - previousMillis;

  // Update speed based on elapsed time
  while (timeElapsed >= speedInterval)
  {
    updateSpeed();
    timeElapsed -= speedInterval;
    previousMillis += speedInterval;
  }
}
