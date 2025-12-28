#include "main.h"
#include <string.h>

/* --- 1. HARDWARE HANDLES --- */
SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc1;

/* --- 2. CONFIGURATION --- */
#define NUM_MODULES     2
#define TOTAL_ROWS      16
#define TOTAL_COLS      8

// Sensitivity
#define JOY_LOW         1000
#define JOY_HIGH        3500

// Timing Speeds (ms)
#define INPUT_DELAY     100
uint32_t gravity_speed = 200; // The lower the faster it falls

/* --- 3. REGISTERS --- */
#define REG_SHUTDOWN    0x0C
#define REG_SCAN_LIMIT  0x0B
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_DISPLAY_TEST 0x0F

/* --- 4. FSM STATES --- */
typedef enum {
    STATE_INIT,
    STATE_SPAWN,
    STATE_PLAYING,
    STATE_GAMEOVER, // Pending
    STATE_STOPPED
} GameState_t;

/* --- 5. GLOBAL VARIABLES --- */
// BUFFER 1: The "Active" screen sent to SPI
uint8_t display_buffer[TOTAL_ROWS];

// BUFFER 2: The "Background" memory (The Pile)
uint8_t locked_buffer[TOTAL_ROWS];

int8_t cursor_x = 0;
int8_t cursor_y = 0;

GameState_t current_state = STATE_INIT;

// Timers
uint32_t last_input_time = 0;
uint32_t last_gravity_time = 0;

/* --- 6. PROTOTYPES --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

// Logic
void Screen_Init(void);
void MAX7219_Flush(void);
void MAX7219_SetPixel(uint8_t* buffer, int8_t x, int8_t y, uint8_t state);
void MAX7219_Clear(uint8_t* buffer);
uint32_t Read_ADC_Channel(uint32_t channel);
uint8_t Check_Collision(int8_t x, int8_t y);

/* --- 7. MAIN LOOP --- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  while (1)
  {
      // Emergency Stop (Outside FSM)
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          current_state = STATE_STOPPED;
          HAL_Delay(500);
      }

      switch (current_state)
      {
          /* --- STATE: INIT --- */
          case STATE_INIT:
              Screen_Init();
              MAX7219_Clear(locked_buffer);  // Clear Pile
              MAX7219_Clear(display_buffer); // Clear Screen
              current_state = STATE_SPAWN;
              break;

          /* --- STATE: SPAWN NEW DOT --- */
          case STATE_SPAWN:
              cursor_x = 3; // Center X
              cursor_y = 0; // Ceiling Y

              // Immediate Game Over Check:
              // If the spawn point is already blocked, the pile is too high!
              if (Check_Collision(cursor_x, cursor_y)) {
                  // For now, just reset the whole game
                  MAX7219_Clear(locked_buffer);
              }

              last_gravity_time = HAL_GetTick(); // Reset timers
              current_state = STATE_PLAYING;
              break;

          /* --- STATE: PLAYING --- */
          case STATE_PLAYING:
          {
              uint32_t now = HAL_GetTick();

              // --- A. GRAVITY LOGIC ---
              if (now - last_gravity_time > gravity_speed) {
                  int8_t next_y = cursor_y + 1;

                  // 1. Check if we can fall
                  if (!Check_Collision(cursor_x, next_y)) {
                      cursor_y = next_y; // Safe to fall
                  }
                  else {
                      // 2. We hit something (Floor or Pile)!
                      // LOCK IT: Write current position into locked_buffer
                      MAX7219_SetPixel(locked_buffer, cursor_x, cursor_y, 1);

                      // Respawn new dot
                      current_state = STATE_SPAWN;
                  }
                  last_gravity_time = now;
              }

              // --- B. INPUT LOGIC ---
              if (now - last_input_time > INPUT_DELAY) {
                  uint32_t val_x = Read_ADC_Channel(ADC_CHANNEL_1);
                  uint32_t val_y = Read_ADC_Channel(ADC_CHANNEL_2); // Down only

                  // X Movement (Left/Right)
                  int8_t next_x = cursor_x;
                  if (val_x < JOY_LOW) next_x--;
                  else if (val_x > JOY_HIGH) next_x++;

                  // X Ring Wrap Logic
                  if(next_x < 0) next_x = TOTAL_COLS - 1;
                  if(next_x >= TOTAL_COLS) next_x = 0;

                  // Collision Check X
                  if (!Check_Collision(next_x, cursor_y)) {
                      cursor_x = next_x;
                  }

                  // Y Movement (Manual Drop)
                  if (val_y > JOY_HIGH) { // Soft Drop
                      int8_t drop_y = cursor_y + 1;
                      if (!Check_Collision(cursor_x, drop_y)) {
                          cursor_y = drop_y;
                      }
                  }

                  // Reset Button
                  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                      current_state = STATE_INIT;
                  }
                  last_input_time = now;
              }

              // --- C. RENDER FRAME ---
              // 1. Copy the Pile (Locked) to the Display Buffer
              memcpy(display_buffer, locked_buffer, sizeof(locked_buffer));

              // 2. Draw the Active Dot on top
              MAX7219_SetPixel(display_buffer, cursor_x, cursor_y, 1);

              // 3. Send to hardware
              MAX7219_Flush();

              break;
          }

          case STATE_STOPPED:
              MAX7219_Clear(display_buffer);
              MAX7219_Flush();
              break;
      }
  }
}

/* --- 8. COLLISION DETECTION LOGIC --- */

/**
 * @brief Checks if a specific point (x,y) is occupied or out of bounds.
 * @param x, y: Coordinates to test
 * @return 1 if Collision (Blocked), 0 if Empty (Safe)
 */
uint8_t Check_Collision(int8_t x, int8_t y) {
    // 1. Floor Check
    if (y >= TOTAL_ROWS) return 1; // Hit the floor

    // 2. Wall Check (If not using wrapping)
    // if (x < 0 || x >= TOTAL_COLS) return 1;

    // 3. Pile Check (The Object Collision)
    // We check the 'locked_buffer' to see if a bit is already set here.

    // We reuse the mapping logic, but to READ instead of WRITE
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
    uint8_t bit_mask = (1 << local_y); // Matches SetPixel logic

    // Check if bit is 1
    if (locked_buffer[target_index] & bit_mask) {
        return 1; // Collision with Pile
    }

    return 0; // Safe
}

/* --- 9. HELPER FUNCTIONS --- */

/**
 * @brief Sets a pixel in a specific buffer.
 * @param buffer: Pointer to display_buffer OR locked_buffer
 */
void MAX7219_SetPixel(uint8_t* buffer, int8_t x, int8_t y, uint8_t state) {
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

    if (state) buffer[target_index] |= bit_mask;
    else       buffer[target_index] &= ~bit_mask;
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

void MAX7219_Clear(uint8_t* buffer) {
    memset(buffer, 0, TOTAL_ROWS);
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
}

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

/* --- 10. HARDWARE INIT --- */
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
