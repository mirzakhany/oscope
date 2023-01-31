#include <Arduino.h>
#include "glcd.h"
// #include <Arduino_GFX_Library.h>
#include "raster.h"
#include "esp_timer.h"

#define GFX_BL DF_GFX_BL
#define SPI_FREQ 60000000

// /* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
// Arduino_DataBus *bus = new Arduino_ESP32SPI(27 /* DC */, 5 /* CS */, 18 /* SCK */, 23 /* MOSI */, 22 /* MISO */, HSPI /* spi_num */);
// Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, DF_GFX_RST /* RST */, 1 /* rotation */, false /* IPS */);

static LGFX display;

#define WAVE_BUFFER_LENGTH 400
int wave_buffer[WAVE_BUFFER_LENGTH];

typedef enum
{
  Stop = 0,
  Run = !Stop
} SamplingStatus;

int8_t current_sampling_status = Run;
char *sampling_status[] = {"Stop", "Run"};

typedef enum
{
  Up = 0,
  Down = !Up
} TriggerMode;

int8_t current_trigger_mode = Up;
char *trigger_mode[] = {"Up", "Down"};

typedef enum
{
  Auto = 0,
  Normal = 1,
  Single = 2
} SamplingMode;

int8_t current_sampling_mode = Normal;
char *sampling_mode[] = {"Auto", "Normal", "Single"};

float current_trigger_value = 2.0;
uint32_t current_time_per_div = 1000;

// TODO fix this values
uint16_t wave_centor_x = 209;
uint16_t wave_centor_y = 159;
uint16_t wave_height = 301;
uint16_t wave_width = 401;

#define SET 1
#define RESET 0
int stop_sampling_flag = RESET;

// cores
static BaseType_t reader_app = 0;
static BaseType_t ui_app = 1;

// Wave reader queue
// static QueueHandle_t reader_status_queue;

uint8_t get_trigger_status(float d0, float d1)
{
  if (current_trigger_mode == Up)
  {
    if ((d0 > current_trigger_value) && (d1 <= current_trigger_value))
    {
      return SET;
    }
  }
  else if (current_trigger_mode == Down)
  {
    if ((d1 >= current_trigger_value) && (d0 < current_trigger_value))
    {
      return SET;
    }
  }
  return RESET;
}

static void plot_wave_thread(void)
{
  uint8_t flag = 0;
  uint8_t i = 0;
  BaseType_t resp;

  // display.setTextColor(TFT_YELLOW);
  // display.drawPixel(80, 80);
  // display.drawLine(10, 10, 40, 40);
  // while (true)
  // {
  // resp = xQueueReceive(reader_status_queue, (void *)&flag, portMAX_DELAY);
  if (stop_sampling_flag == RESET)
  {
    // gfx->draw16bitRGBBitmap(0, 0, (const uint16_t *)ScopeRaster, 480, 320);
    // display.pushImage(0, 0, 480, 320, (const uint16_t *)ScopeRaster);
    for (i = 0; i <= WAVE_BUFFER_LENGTH - 2; i++)
    {
      if ((i % 50 != 0) && ((i + 1) % 50 != 0) && (wave_buffer[i] != 100) && (wave_buffer[i + 1] != 100))
      {

        display.drawLine(wave_centor_x - (wave_width / 2) + i, wave_centor_y - (wave_height / 2) + wave_buffer[i] + 1,
                         wave_centor_x - (wave_width / 2) + i + 1, wave_centor_y - (wave_height / 2) + wave_buffer[i + 1] + 1, TFT_YELLOW);
      }
    }
  }
  // }
}

static void read_wave_thread(void)
{
  uint8_t pre_sample = 100;
  uint8_t flag = 1;
  uint16_t sample_count = 0;

  float d0, d1;
  // while (true)
  //{
  sample_count = 0;

  while (pre_sample--)
  {
    d0 = analogRead(0) / 4096.0 * 3.3;
  }

  if (current_sampling_mode != Auto)
  {
    do
    {
      d0 = analogRead(0) / 4096.0 * 3.3;
      d1 = analogRead(0) / 4096.0 * 3.3;
    } while (get_trigger_status(d0, d1) != SET);
  }

  while (sample_count < WAVE_BUFFER_LENGTH)
  {

    wave_buffer[sample_count] = analogRead(0) * 198 / 4096;
    if ((current_time_per_div / 50 - 7) != 0 && (current_time_per_div / 50 - 7) <= 1000)
    {
      delayMicroseconds(current_time_per_div / 50 - 7);
    }
    else
    {
      delayMicroseconds((current_time_per_div / 50) / 1000);
    }
    sample_count++;
  }
  if (current_sampling_mode == Single)
  {
    // xQueueSend(reader_status_queue, (void *)&flag, sizeof(flag));
    plot_wave_thread();
    current_sampling_status = Stop;
    // CurSamplStatus = SamplStatus[SamplStatusNrb];
    // Setting_Inf_Update(0);
    // vTaskSuspend(NULL);
  }
  plot_wave_thread();
  // xQueueSend(reader_status_queue, (void *)&flag, sizeof(flag));
  // }
}

void setup()
{
  display.init();
  display.setColorDepth(8);
  display.pushImage(0, 0, 480, 320, (const uint16_t *)ScopeRaster);
  display.setTextColor(TFT_YELLOW);
  // display.print("start again");
  //   display.setColorDepth(8);
  //  gfx->begin(SPI_FREQ);
  //  pinMode(GFX_BL, OUTPUT);
  //  digitalWrite(GFX_BL, HIGH);
  //  gfx->draw16bitRGBBitmap(0, 0, (const uint16_t *)ScopeRaster, 480, 320);

  // reader_status_queue = xQueueCreate(5, sizeof(int));
  // xTaskCreatePinnedToCore(read_wave_thread, "read_wave", 4000, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(plot_wave_thread, "plot_wave", 8000, NULL, 1, NULL, 1);
  //  BaseType_t resp;
  //  resp = xTaskCreate(read_wave_thread, "read_wave", 512, NULL, 1, NULL);
  //  if (resp != pdPASS)
  //  {
  //    display.print(resp);
  //  }
  //  if (xTaskCreate(plot_wave_thread, "plot_wave", 512, NULL, 1, NULL) != pdPASS)
  //  {
  //    display.print("create plot_wave task failed");
  //  }

  // reader_status_queue = xQueueCreate(1, sizeof(int));
  // delete the setup thread
  // vTaskDelete(NULL);
}

// void read_input()
// {
//   for (int i = 0; i < WAVE_BUFFER_LENGTH; i++)
//   {
//     wave_buffer[i] = analogRead(0);
//     delayMicroseconds(600);
//   }
// }

// void plot_wave()
// {
//   display.pushImage(0, 0, 480, 320, (const uint16_t *)ScopeRaster);

//   for (int x = 0; x < WAVE_BUFFER_LENGTH; x++)
//   {
//     int y = map(wave_buffer[x], 0, 4095, 309, 9);
//     display.drawPixel(x + 10, y, TFT_YELLOW);
//   }
// }

void loop()
{
  read_wave_thread();
}
