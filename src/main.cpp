#include <Arduino.h>
#include "glcd.h"
// #include <Arduino_GFX_Library.h>
// #include "raster.h"
#include "rs.h"
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

double current_wave_freq = 0.0;
uint8_t wave_len_sum_count = 0;
uint16_t wave_len = 0;

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

void update_info()
{
  display.setTextSize(2);
  display.setCursor(420, 45);
  if (current_wave_freq < 1.0)
  {
    display.printf("%dHz", (uint16_t)(current_wave_freq * 1000));
  }
  else
  {
    display.printf("%.2fkHz", current_wave_freq);
  }

  display.setCursor(420, 128);
  display.printf("%.1fV", current_trigger_value);
  display.setCursor(420, 145);
  display.print(trigger_mode[current_trigger_mode]);

  display.setCursor(420, 210);
  display.printf(sampling_mode[current_sampling_mode]);
  display.setCursor(420, 245);
  display.print(sampling_status[current_sampling_status]);
}

void calculate_freq(void)
{
  uint16_t sample_before_count = wave_centor_x, sample_after_count = wave_centor_x + 1;
  uint8_t sum_count = 4;
  float d0, d1;

  d0 = wave_buffer[sample_before_count - 1] / 200.0 * 3.3;
  d1 = wave_buffer[sample_before_count] / 200.0 * 3.3;
  while ((get_trigger_status(d0, d1) == RESET) && (sample_before_count > 1))
  {
    sample_before_count--;
    d0 = wave_buffer[sample_before_count - 1] / 200.0 * 3.3;
    d1 = wave_buffer[sample_before_count] / 200.0 * 3.3;
  }

  d0 = wave_buffer[sample_after_count] / 200.0 * 3.3;
  d1 = wave_buffer[sample_after_count + 1] / 200.0 * 3.3;
  while ((get_trigger_status(d0, d1) == RESET) && (sample_after_count < (WAVE_BUFFER_LENGTH - 1)))
  {
    sample_after_count++;
    d0 = wave_buffer[sample_after_count] / 200.0 * 3.3;
    d1 = wave_buffer[sample_after_count + 1] / 200.0 * 3.3;
  }

  if (sample_after_count - sample_before_count + 2 < WAVE_BUFFER_LENGTH - 1)
  {
    wave_len += sample_after_count - sample_before_count + 1;
    if (++wave_len_sum_count >= sum_count)
    {
      wave_len_sum_count = 0;
      wave_len = wave_len >> 2;
      current_wave_freq = 1 / (((float)wave_len) * ((double)current_time_per_div) / 50.0 / 1000.0);

      update_info();
      wave_len = 0;
    }
    else
      return;
  }
  else
    return;
}

static void plot_wave_thread(void)
{
  uint8_t flag = 0;
  uint16_t i = 0;
  BaseType_t resp;

  // while (true)
  // {
  // resp = xQueueReceive(reader_status_queue, (void *)&flag, portMAX_DELAY);
  if (stop_sampling_flag == RESET)
  {
    // gfx->draw16bitRGBBitmap(0, 0, (const uint16_t *)ScopeRaster, 480, 320);
    display.pushImage(0, 0, 480, 320, (const uint16_t *)rso);
    // draw_raster();
    for (i = 0; i <= WAVE_BUFFER_LENGTH - 2; i++)
    {
      // Serial.printf("processing  %d\n", i);
      if ((i % 50 != 0) && ((i + 1) % 50 != 0) && (wave_buffer[i] != 100) && (wave_buffer[i + 1] != 100))
      {
        display.drawLine(wave_centor_x - (wave_width / 2) + i, wave_centor_y - (wave_height / 2) + wave_buffer[i] + 1,
                         wave_centor_x - (wave_width / 2) + i + 1, wave_centor_y - (wave_height / 2) + wave_buffer[i + 1] + 1, TFT_YELLOW);
      }
    }
    calculate_freq();
  }
  // }
}

static void read_wave_thread(void)
{
  uint8_t pre_sample = 100;
  uint8_t flag = 1;
  uint16_t sample_count = 0;

  Serial.println("start read wave");
  float d0, d1;
  // while (true)
  //{
  sample_count = 0;

  while (pre_sample--)
  {
    d0 = analogRead(0) / 4096.0 * 3.3;
  }

  Serial.println("wait for trigger");
  if (current_sampling_mode != Auto)
  {
    do
    {
      d0 = analogRead(0) / 4096.0 * 3.3;
      d1 = analogRead(0) / 4096.0 * 3.3;
    } while (get_trigger_status(d0, d1) != SET);
  }

  Serial.println("read signals");
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
  Serial.println("read signals finished");
  if (current_sampling_mode == Single)
  {
    Serial.println("plot single mode");
    // xQueueSend(reader_status_queue, (void *)&flag, sizeof(flag));
    plot_wave_thread();
    current_sampling_status = Stop;
    // CurSamplStatus = SamplStatus[SamplStatusNrb];
    // Setting_Inf_Update(0);
    // vTaskSuspend(NULL);
  }
  Serial.println("plot default");
  plot_wave_thread();
  // xQueueSend(reader_status_queue, (void *)&flag, sizeof(flag));
  // }
}

void setup()
{
  Serial.begin(115200);
  display.init();
  display.setColorDepth(8);
  // draw_raster();
  display.pushImage(0, 0, 480, 320, (const uint16_t *)rso);
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

void loop()
{
  read_wave_thread();
}
