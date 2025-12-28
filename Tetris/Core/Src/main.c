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

// Speeds (ms)
#define INPUT_DELAY     100
uint32_t gravity_speed = 200;

/* --- 3. REGISTERS --- */
#define REG_SHUTDOWN    0x0C
#define REG_SCAN_LIMIT  0x0B
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_DISPLAY_TEST 0x0F

/* --- 4. SHAPE DEFINITIONS (2D Array) --- */
#define NUM_SHAPES 7
#define COORDS_PER_SHAPE 8 // 4 blocks * 2 (x,y)

// The Library of Shapes (Flat 2D Array)
// Format: {x1, y1, x2, y2, x3, y3, x4, y4}
const int8_t SHAPES[NUM_SHAPES][COORDS_PER_SHAPE] = {
    {-1, 0,  0, 0,  1, 0,  2, 0}, // 0: I-Shape
    {-1,-1, -1, 0,  0, 0,  1, 0}, // 1: J-Shape
    {-1, 0,  0, 0,  1, 0,  1,-1}, // 2: L-Shape
    { 0, 0,  1, 0,  0, 1,  1, 1}, // 3: O-Shape (Square)
    {-1, 1,  0, 1,  0, 0,  1, 0}, // 4: S-Shape
    { 0,-1, -1, 0,  0, 0,  1, 0}, // 5: T-Shape
    {-1, 0,  0, 0,  0, 1,  1, 1}  // 6: Z-Shape
};

/* --- 5. FSM STATES --- */
typedef enum {
    STATE_INIT,
    STATE_SPAWN,
    STATE_PLAYING,
    STATE_CHECK_LINES,
    STATE_STOPPED
} GameState_t;

/* --- 6. GLOBAL VARIABLES --- */
uint8_t display_buffer[TOTAL_ROWS];
uint8_t locked_buffer[TOTAL_ROWS];

int8_t cursor_x = 0;
int8_t cursor_y = 0;

GameState_t current_state = STATE_INIT;

uint32_t last_input_time = 0;
uint32_t last_gravity_time = 0;

/* --- 7. PROTOTYPES --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

void Screen_Init(void);
void MAX7219_Flush(void);
void MAX7219_SetPixel(uint8_t* buffer, int8_t x, int8_t y, uint8_t state);
uint8_t MAX7219_GetPixel(uint8_t* buffer, int8_t x, int8_t y); // New Helper
void MAX7219_Clear(uint8_t* buffer);
uint32_t Read_ADC_Channel(uint32_t channel);

uint8_t Check_Collision(int8_t x, int8_t y);
void Check_Full_Rows(void); // New Logic

/* --- 8. MAIN LOOP --- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  while (1)
  {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          current_state = STATE_STOPPED;
          HAL_Delay(500);
      }

      switch (current_state)
      {
          case STATE_INIT:
              Screen_Init();
              MAX7219_Clear(locked_buffer);
              MAX7219_Clear(display_buffer);
              current_state = STATE_SPAWN;
              break;

          case STATE_SPAWN:
              cursor_x = 3;
              cursor_y = 0;
              if (Check_Collision(cursor_x, cursor_y)) {
                  MAX7219_Clear(locked_buffer); // Game Over -> Reset
              }
              last_gravity_time = HAL_GetTick();
              current_state = STATE_PLAYING;
              break;

          case STATE_PLAYING:
          {
              uint32_t now = HAL_GetTick();

              // GRAVITY
              if (now - last_gravity_time > gravity_speed) {
                  int8_t next_y = cursor_y + 1;
                  if (!Check_Collision(cursor_x, next_y)) {
                      cursor_y = next_y;
                  }
                  else {
                      // Lock Piece
                      MAX7219_SetPixel(locked_buffer, cursor_x, cursor_y, 1);
                      // Go Check for Full Lines
                      current_state = STATE_CHECK_LINES;
                  }
                  last_gravity_time = now;
              }

              // INPUT
              if (now - last_input_time > INPUT_DELAY && current_state == STATE_PLAYING) {
                  uint32_t val_x = Read_ADC_Channel(ADC_CHANNEL_1);
                  uint32_t val_y = Read_ADC_Channel(ADC_CHANNEL_2);

                  int8_t next_x = cursor_x;
                  if (val_x < JOY_LOW) next_x--;
                  else if (val_x > JOY_HIGH) next_x++;

                  if(next_x < 0) next_x = TOTAL_COLS - 1;
                  if(next_x >= TOTAL_COLS) next_x = 0;

                  if (!Check_Collision(next_x, cursor_y)) cursor_x = next_x;

                  // Manual Drop
                  if (val_y > JOY_HIGH) {
                      if (!Check_Collision(cursor_x, cursor_y + 1)) cursor_y++;
                  }

                  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                      current_state = STATE_INIT;
                  }
                  last_input_time = now;
              }

              // RENDER
              memcpy(display_buffer, locked_buffer, sizeof(locked_buffer));
              MAX7219_SetPixel(display_buffer, cursor_x, cursor_y, 1);
              MAX7219_Flush();
              break;
          }

          /* --- NEW STATE: HANDLE LINE CLEARING --- */
          case STATE_CHECK_LINES:
              Check_Full_Rows(); // Run the logic
              current_state = STATE_SPAWN; // Then spawn new dot
              break;

          case STATE_STOPPED:
              MAX7219_Clear(display_buffer);
              MAX7219_Flush();
              break;
      }
  }
}

/* --- 9. LOGIC FUNCTIONS --- */

/**
 * @brief Scans the locked_buffer for full rows and clears them.
 */
void Check_Full_Rows(void) {
    // We scan from Bottom (15) to Top (0)
    for (int y = TOTAL_ROWS - 1; y >= 0; y--) {

        // 1. Check if Row 'y' is full
        uint8_t is_full = 1;
        for (int x = 0; x < TOTAL_COLS; x++) {
            if (MAX7219_GetPixel(locked_buffer, x, y) == 0) {
                is_full = 0; // Found an empty spot
                break;
            }
        }

        // 2. If Full -> Clear and Shift Everything Down
        if (is_full) {
            // Shift Logic:
            // For every row starting at 'y' (the full one),
            // copy the content of the row ABOVE it ('k-1') into it ('k').
            for (int k = y; k > 0; k--) {
                for (int x = 0; x < TOTAL_COLS; x++) {
                    uint8_t pixel_above = MAX7219_GetPixel(locked_buffer, x, k - 1);
                    MAX7219_SetPixel(locked_buffer, x, k, pixel_above);
                }
            }

            // Clear the very top row (Row 0) because it has nothing above it
            for (int x = 0; x < TOTAL_COLS; x++) {
                MAX7219_SetPixel(locked_buffer, x, 0, 0);
            }

            // IMPORTANT:
            // Since we shifted everything down, the current row index 'y'
            // now contains the data that used to be at 'y-1'.
            // We must check this index AGAIN to see if that new line is also full.
            y++;
        }
    }
}

uint8_t Check_Collision(int8_t x, int8_t y) {
    if (y >= TOTAL_ROWS) return 1; // Floor

    // Check Pile
    if (MAX7219_GetPixel(locked_buffer, x, y)) return 1;

    return 0;
}

/* --- 10. BUFFER HELPERS --- */

/** * @brief Reads the state (1/0) of a pixel from memory.
 * This function reverses the logic of SetPixel to find the data.
 */
uint8_t MAX7219_GetPixel(uint8_t* buffer, int8_t x, int8_t y) {
    if (x < 0 || x >= TOTAL_COLS || y < 0 || y >= TOTAL_ROWS) return 0;

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

    return (buffer[target_index] & bit_mask) ? 1 : 0;
}

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

/* --- 11. HARDWARE INIT --- */
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
