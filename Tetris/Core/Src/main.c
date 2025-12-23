/*
 * PROJECT: STM32 Tetris + Joystick + Clean Stop
 * CONTROLS:
 * - Joystick (PA1/PA2): Move Cursor
 * - Joystick Button (PA3): Reset Cursor to (0,0)
 * - Blue Button (PA0): EMERGENCY OFF (Press before Terminating)
 */

#include "main.h"
#include <string.h>

/* --- HANDLES --- */
SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc1;

/* --- SETTINGS --- */
#define NUM_MODULES     2
#define TOTAL_ROWS      16
#define TOTAL_COLS      8
#define MOVE_SPEED      150
#define JOY_LOW         500
#define JOY_HIGH        3500

/* --- REGISTER MAP --- */
#define REG_DIGIT_0     0x01
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_SCAN_LIMIT  0x0B
#define REG_SHUTDOWN    0x0C
#define REG_DISPLAY_TEST 0x0F

/* --- VARIABLES --- */
uint8_t display_buffer[TOTAL_ROWS];
int8_t cursor_x = 0;
int8_t cursor_y = 0;
uint8_t system_running = 1; // Flag for the Emergency Stop

/* --- PROTOTYPES --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

void Screen_Init(void);
void MAX7219_Flush(void);
void MAX7219_SetPixel(int8_t x, int8_t y, uint8_t state);
void MAX7219_Clear(void);
uint32_t Read_ADC_Channel(uint32_t channel);

/* --- MAIN --- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  Screen_Init();

  while (1)
  {
      // --- EMERGENCY STOP CHECK (Blue Button PA0) ---
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          MAX7219_Clear();      // Turn off LEDs
          system_running = 0;   // Stop Loop
          HAL_Delay(1000);      // Debounce
      }

      if (system_running) {
          // 1. Read Joystick
          uint32_t val_x = Read_ADC_Channel(ADC_CHANNEL_1);
          uint32_t val_y = Read_ADC_Channel(ADC_CHANNEL_2);

          MAX7219_SetPixel(cursor_x, cursor_y, 0);

          // 2. X Movement
          if (val_x < JOY_LOW) {
               cursor_x--;
               if(cursor_x < 0) cursor_x = TOTAL_COLS - 1;
          }
          else if (val_x > JOY_HIGH) {
               cursor_x++;
               if(cursor_x >= TOTAL_COLS) cursor_x = 0;
          }

          // 3. Y Movement
          if (val_y < JOY_LOW) {
               cursor_y--;
               if(cursor_y < 0) cursor_y = TOTAL_ROWS - 1;
          }
          else if (val_y > JOY_HIGH) {
               cursor_y++;
               if(cursor_y >= TOTAL_ROWS) cursor_y = 0;
          }

          MAX7219_SetPixel(cursor_x, cursor_y, 1);
          MAX7219_Flush();

          // 4. Game Reset (Joystick Button PA3)
          if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
              MAX7219_Clear();
              cursor_x = 0; cursor_y = 0;
              HAL_Delay(500);
          }

          HAL_Delay(MOVE_SPEED);
      }
      else {
          // System is Stopped. Safe to Terminate.
          // Blink built-in LED (usually PD12/13/14/15) if available, or just wait.
          HAL_Delay(100);
      }
  }
}

/* --- LOGIC --- */

uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return 2048;
}

void MAX7219_SetPixel(int8_t x, int8_t y, uint8_t state) {
    if (x < 0 || x >= TOTAL_COLS || y < 0 || y >= TOTAL_ROWS) return;

    uint8_t module_offset;
    int8_t local_y;

    if (y < 8) {
        module_offset = 8;
        local_y = y;
    } else {
        module_offset = 0;
        local_y = y - 8;
    }

    uint8_t target_index = module_offset + x;
    uint8_t bit_mask = (1 << local_y);

    if (state) display_buffer[target_index] |= bit_mask;
    else       display_buffer[target_index] &= ~bit_mask;
}

void MAX7219_Flush(void) {
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t addr = i + 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

        HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
        HAL_SPI_Transmit(&hspi1, &display_buffer[8 + i], 1, 10);

        HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
        HAL_SPI_Transmit(&hspi1, &display_buffer[i], 1, 10);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
}

void MAX7219_Clear(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
    MAX7219_Flush();
}

void Screen_Init(void) {
    uint8_t commands[][2] = {
        {REG_DISPLAY_TEST, 0x00}, {REG_SHUTDOWN, 0x01},
        {REG_SCAN_LIMIT, 0x07},   {REG_DECODE_MODE, 0x00},
        {REG_INTENSITY, 0x02}
    };
    for (int c = 0; c < 5; c++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        for(int m=0; m<NUM_MODULES; m++) {
            HAL_SPI_Transmit(&hspi1, &commands[c][0], 1, 10);
            HAL_SPI_Transmit(&hspi1, &commands[c][1], 1, 10);
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
    MAX7219_Clear();
}

/* --- HARDWARE INIT --- */

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_ADC1_Init(void) {
  // Config removed from here to fix warning.
  // Channel config happens in Read_ADC_Channel now.
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  HAL_SPI_Init(&hspi1);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // PA4 (Matrix CS)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA3 (Joystick Button)
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA0 (User Button - Emergency Stop)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA1, PA2 (Joystick Analog)
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
