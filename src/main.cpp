#include <Arduino.h>
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
static int val = 100;
#include <Ticker.h> //Call the ticker. H Library
Ticker ticker1;

int i = 0;
#include <Arduino_GFX_Library.h>
#define TFT_BL 2

#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *lcd = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */

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
    //  800 /* width */, 0 /* hsync_polarity */, 8/* hsync_front_porch */, 2 /* hsync_pulse_width */, 43/* hsync_back_porch */,
    //  480 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 2/* vsync_pulse_width */, 12 /* vsync_back_porch */,
    //  1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

    //  800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    //  480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    //  1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);
    //  800 /* width */, 1 /* hsync_polarity */, 80 /* hsync_front_porch */, 48 /* hsync_pulse_width */,  40/* hsync_back_porch */,
    //  480 /* height */, 1 /* vsync_polarity */, 50 /* vsync_front_porch */, 1 /* vsync_pulse_width */, 31 /* vsync_back_porch */,
    //  0 /* pclk_active_neg */, 30000000 /* prefer_speed */, true /* auto_flush */);
    800 /* width */, 1 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 40 /* hsync_back_porch */,
    480 /* height */, 1 /* vsync_polarity */, 13 /* vsync_front_porch */, 1 /* vsync_pulse_width */, 31 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

#endif /* !defined(DISPLAY_DEV_KIT) */
#include "touch.h"
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  lcd->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  lcd->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

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
      Serial.print("Data x ");
      Serial.println(data->point.x);
      Serial.print("Data y ");
      Serial.println(data->point.y);
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

#define Z_THRESHOLD 350 // Touch pressure threshold for validating touches
#define _RAWERR 20      // Deadband error allowed in successive position samples
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

uint8_t validTouch(uint16_t *x, uint16_t *y, uint16_t threshold)
{
  uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

  // Wait until pressure stops increasing to debounce pressure
  uint16_t z1 = 1;
  uint16_t z2 = 0;
  while (z1 > z2)
  {
    z2 = z1;
    z1 = getTouchRawZ();
    delay(1);
    Serial.print("z1:");
    Serial.println(z1);
  }

  if (z1 <= threshold)
    return false;

  getTouchRaw(&x_tmp, &y_tmp);

  delay(1); // Small delay to the next sample
  if (getTouchRawZ() <= threshold)
    return false;

  delay(2); // Small delay to the next sample
  getTouchRaw(&x_tmp2, &y_tmp2);

  if (abs(x_tmp - x_tmp2) > _RAWERR)
    return false;
  if (abs(y_tmp - y_tmp2) > _RAWERR)
    return false;

  *x = x_tmp;
  *y = y_tmp;

  return true;
}

void calibrateTouch(uint16_t *parameters, uint32_t color_fg, uint32_t color_bg, uint8_t size)
{
  int16_t values[] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t x_tmp, y_tmp;
  uint16_t _width = 800;
  uint16_t _height = 480;

  for (uint8_t i = 0; i < 4; i++)
  {
    lcd->fillRect(0, 0, size + 1, size + 1, color_bg);
    lcd->fillRect(0, _height - size - 1, size + 1, size + 1, color_bg);
    lcd->fillRect(_width - size - 1, 0, size + 1, size + 1, color_bg);
    lcd->fillRect(_width - size - 1, _height - size - 1, size + 1, size + 1, color_bg);

    if (i == 5)
      break; // used to clear the arrows

    switch (i)
    {
    case 0: // up left
      lcd->drawLine(0, 0, 0, size, color_fg);
      lcd->drawLine(0, 0, size, 0, color_fg);
      lcd->drawLine(0, 0, size, size, color_fg);
      break;
    case 1: // bot left
      lcd->drawLine(0, _height - size - 1, 0, _height - 1, color_fg);
      lcd->drawLine(0, _height - 1, size, _height - 1, color_fg);
      lcd->drawLine(size, _height - size - 1, 0, _height - 1, color_fg);
      break;
    case 2: // up right
      lcd->drawLine(_width - size - 1, 0, _width - 1, 0, color_fg);
      lcd->drawLine(_width - size - 1, size, _width - 1, 0, color_fg);
      lcd->drawLine(_width - 1, size, _width - 1, 0, color_fg);
      break;
    case 3: // bot right
      lcd->drawLine(_width - size - 1, _height - size - 1, _width - 1, _height - 1, color_fg);
      lcd->drawLine(_width - 1, _height - 1 - size, _width - 1, _height - 1, color_fg);
      lcd->drawLine(_width - 1 - size, _height - 1, _width - 1, _height - 1, color_fg);
      break;
    }

    // user has to get the chance to release
    if (i > 0)
      delay(1000);

    for (uint8_t j = 0; j < 8; j++)
    {
      while (touch_has_signal())
      {
        if (touch_touched())
        {
          Serial.print("Data x :");
          Serial.println(touch_last_x);
          Serial.print("Data y :");
          Serial.println(touch_last_y);
          break;
        }
      }
    }
  }
}

void touch_calibrate() // 屏幕校准
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;
  Serial.println("屏幕校准");

  Serial.println("按指示触摸角落");
  lv_timer_handler();
  calibrateTouch(calData, MAGENTA, BLACK, 17);
  Serial.println("calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15)");
  Serial.println();
  Serial.println();
  Serial.println("//在setup()中使用此校准代码:");
  Serial.print("uint16_t calData[5] = ");
  Serial.print("{ ");

  for (uint8_t i = 0; i < 5; i++)
  {
    Serial.print(calData[i]);
    if (i < 4)
      Serial.print(", ");
  }

  Serial.println(" };");
}

void setTouch(uint16_t *parameters)
{
  touchCalibration_x0 = parameters[0];
  touchCalibration_x1 = parameters[1];
  touchCalibration_y0 = parameters[2];
  touchCalibration_y1 = parameters[3];

  if (touchCalibration_x0 == 0)
    touchCalibration_x0 = 1;
  if (touchCalibration_x1 == 0)
    touchCalibration_x1 = 1;
  if (touchCalibration_y0 == 0)
    touchCalibration_y0 = 1;
  if (touchCalibration_y1 == 0)
    touchCalibration_y1 = 1;

  touchCalibration_rotate = parameters[4] & 0x01;
  touchCalibration_invert_x = parameters[4] & 0x02;
  touchCalibration_invert_y = parameters[4] & 0x04;
}

int speedValue = 0;
bool incrementingSpeed = true;
const int speedInterval = 30;
const int SPEED_INCREMENT = 1;
const int MAX_SPEED = 45;
const int MIN_SPEED = 0;

void updateSpeed()
{
  if (incrementingSpeed)
  {
    speedValue += SPEED_INCREMENT;
    if (speedValue >= MAX_SPEED)
    {
      incrementingSpeed = false;
    }
  }
  else
  {
    speedValue -= SPEED_INCREMENT;
    if (speedValue <= MIN_SPEED)
    {
      incrementingSpeed = true;
    }
  }
  lv_arc_set_value(ui_ArcSpeed, speedValue);
  lv_label_set_text_fmt(ui_LabelSpeed, "%d", speedValue);
}

void lvgl_loop(void *parameter)
{
  while (true)
  {

    // if (incrementingTemp)
    // {
    //   tempValue += 1;
    //   if (tempValue >= 105)
    //   {
    //     incrementingTemp = false;
    //   }
    // }
    // else
    // {
    //   tempValue -= 1;
    //   if (tempValue <= 0)
    //   {
    //     incrementingTemp = true;
    //   }
    // }
    // lv_slider_set_value(ui_TempSlider, tempValue, LV_ANIM_OFF);
    // lv_label_set_text_fmt(ui_TempLabel, "%d", tempValue);

    // if (incrementingBattery)
    // {
    //   sliderValue++;
    //   if (sliderValue >= 99)
    //   {
    //     incrementingBattery = false;
    //   }
    // }
    // else
    // {
    //   sliderValue--;
    //   if (sliderValue <= 1)
    //   {
    //     incrementingBattery = true;
    //   }
    // }
    // lv_slider_set_value(ui_Slider2, sliderValue, LV_ANIM_OFF);
    // lv_label_set_text_fmt(ui_batteryint, "%d", sliderValue);
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
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  lcd->fillScreen(BLACK);
  Serial.println("Setup done");

  ui_init();
  guiHandler();
}

// void loop()
// {
//   lv_timer_handler();
//   delay(5);
// }

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
