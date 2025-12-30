#include "main.h"
#include <string.h>
#include <stdlib.h>

/* --- 1. HARDWARE HANDLES --- */
SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc1;

/* --- 2. REGISTERS --- */
#define REG_SHUTDOWN    0x0C
#define REG_SCAN_LIMIT  0x0B
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_DISPLAY_TEST 0x0F

/* --- 3. CONFIGURATION --- */
// Dimensions
#define NUM_MODULES     2
#define TOTAL_ROWS      16
#define TOTAL_COLS      8

// Scoring
#define DROP_POINTS 1
#define LNS_CLR_NXT_LVL 4 // Lines cleared required to advance to the next level
#define FULLROW_POINTS 40

// Sensitivity [mV]
#define JOY_LOW         200
#define JOY_HIGH        3800

// Delay [ms]
#define INPUT_DELAY     100

/* --- 4. TETRIS SHAPE DEFINITIONS --- */
#define NUM_SHAPES 7
#define COORDS_PER_SHAPE 8

const int8_t SHAPES[NUM_SHAPES][COORDS_PER_SHAPE] = {
// {x0,y0,   x1,y1,   x2,y2,   x3,y3}

	// I-Shape
    {-1, 0,   0, 0,   1, 0,   2, 0},
    // J-Shape
    {-1,-1,  -1, 0,   0, 0,   1, 0},
    // L-Shape
    {-1, 0,   0, 0,   1, 0,   1,-1},
    // O-Shape
	{ 0, 0,   1, 0,   0,-1,   1,-1},
    // S-Shape
	{-1, 0,   0, 0,   0,-1,   1,-1},
    // T-Shape
    { 0,-1,  -1, 0,   0, 0,   1, 0},
    // Z-Shape
	{-1,-1,   0,-1,   0, 0,   1, 0}
};

// 8x8 LED module "X" drawing
const uint8_t GAME_OVER_ICON[8] = {
    0b10000001,
    0b01000010,
    0b00100100,
    0b00011000,
    0b00011000,
    0b00100100,
    0b01000010,
    0b10000001
};

/* --- 5. STATES --- */
typedef enum {
    STATE_INIT,
    STATE_SPAWN,
    STATE_PLAYING,
    STATE_CHECK_LINES,
    STATE_STOPPED,
	STATE_GAMEOVER
} GameState_t;

/* --- 6. GLOBAL VARIABLES --- */
int8_t pivot_x = 0;
int8_t pivot_y = 0;

int8_t active_shape_coords[COORDS_PER_SHAPE];
uint8_t current_shape_id = 3;

uint8_t display_buffer[TOTAL_ROWS];
uint8_t locked_buffer[TOTAL_ROWS];

uint32_t last_input_time = 0;
uint32_t last_gravity_time = 0;
uint32_t gravity_speed = 400; // The lower the faster

GameState_t current_state = STATE_INIT;

// Live expressions variables
volatile uint8_t game_level = 1;
volatile uint32_t game_score = 0;
volatile uint32_t lines_cleared = 0;

/* --- 7. FUNCTIONS DEFINITION LIST --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

void Screen_Init(void);
uint32_t Read_ADC_Channel(uint32_t channel);

void MAX7219_Flush(void);
void MAX7219_Clear(uint8_t* buffer);
void MAX7219_SetPixel(uint8_t* buffer, int8_t x, int8_t y, uint8_t state);
uint8_t MAX7219_GetPixel(uint8_t* buffer, int8_t x, int8_t y);

int8_t Wrap_Coordinate(int8_t x);
uint8_t Check_Pixel_Collision(int8_t x, int8_t y);
void Check_Full_Rows(void);

void Tetromino_Spawn(uint8_t shape_id);
uint8_t Tetromino_CheckCollision(int8_t target_pivot_x, int8_t target_pivot_y);
void Tetromino_Lock(void);
void Tetromino_Draw(uint8_t* buffer);
void Tetromino_Rotate(void);

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

              srand(HAL_GetTick() + Read_ADC_Channel(ADC_CHANNEL_1));

              game_score = 0;
              game_level = 1;
              lines_cleared = 0;
              gravity_speed = 500;

              current_state = STATE_SPAWN;
              break;

          case STATE_SPAWN:
        	  Tetromino_Spawn(rand() % NUM_SHAPES);

              if (Tetromino_CheckCollision(pivot_x, pivot_y)) {
                  // MAX7219_Clear(locked_buffer);
            	  current_state = STATE_GAMEOVER;
              }
              else {
            	  last_gravity_time = HAL_GetTick();
                  current_state = STATE_PLAYING;
              }
              break;

          case STATE_PLAYING:
          {
              uint32_t now = HAL_GetTick();

              // --- GRAVITY ---
              if (now - last_gravity_time > gravity_speed) {
                  if (!Tetromino_CheckCollision(pivot_x, pivot_y + 1)) {
                      pivot_y++;
                      game_score++;
                  }
                  else {
                      Tetromino_Lock();
                      current_state = STATE_CHECK_LINES;
                  }
                  last_gravity_time = now;
              }

              // --- INPUT ---
              if (now - last_input_time > INPUT_DELAY && current_state == STATE_PLAYING) {
                  // Joystick Raw Values Reading
            	  uint32_t val_x = Read_ADC_Channel(ADC_CHANNEL_1); // VRx
                  uint32_t val_y = Read_ADC_Channel(ADC_CHANNEL_2); // VRy

                  int8_t next_x = pivot_x;

                  if (val_x < JOY_LOW) next_x--;
                  else if (val_x > JOY_HIGH) next_x++;

                  // WRAP THE PIVOT
                  if(next_x < 0) next_x = TOTAL_COLS - 1;
                  if(next_x >= TOTAL_COLS) next_x = 0;

                  // Collision Check
                  if (!Tetromino_CheckCollision(next_x, pivot_y)) {
                      pivot_x = next_x;
                  }

                  // Manual Drop
                  if (val_y > JOY_HIGH) {
                      if (!Tetromino_CheckCollision(pivot_x, pivot_y + 1)) {
                          pivot_y++;
                          game_score++;
                      }
                  }

                  // ROTATION LOGIC (Joystick Center Press)
                  static uint8_t rot_pressed = 0; // State flag for button release

                  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                	  if (rot_pressed == 0) {
                		  Tetromino_Rotate();
                          rot_pressed = 1;    // Lock
                      }
                  }
                  else {
                      rot_pressed = 0;        // Unlock when released
                  }

                  last_input_time = now;
              }

              // --- RENDER ---
              memcpy(display_buffer, locked_buffer, sizeof(locked_buffer));
              Tetromino_Draw(display_buffer);
              MAX7219_Flush();
              break;
          }

          case STATE_CHECK_LINES:
              Check_Full_Rows();
              current_state = STATE_SPAWN;
              break;

          case STATE_STOPPED:
              MAX7219_Clear(display_buffer);
              MAX7219_Flush();
              break;

          case STATE_GAMEOVER:
        	  // Game Over "X" Drawing
              memcpy(&display_buffer[0], GAME_OVER_ICON, 8);
              memcpy(&display_buffer[8], GAME_OVER_ICON, 8);
              MAX7219_Flush();

              // Wait for Restart
              if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
            	  HAL_Delay(500);
                  current_state = STATE_INIT;
              }
          break;
      }
  }
}

/* --- 9. HELPER FUNCTIONS DECLARATIONS --- */
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

void MAX7219_Clear(uint8_t* buffer) { memset(buffer, 0, TOTAL_ROWS); }

void MAX7219_SetPixel(uint8_t* buffer, int8_t x, int8_t y, uint8_t state) {
    if (x < 0 || x >= TOTAL_COLS || y < 0 || y >= TOTAL_ROWS) return;
    uint8_t module_offset = (y < 8) ? 8 : 0;
    int8_t local_y = (y < 8) ? y : y - 8;
    if (state) buffer[module_offset + x] |= (1 << local_y);
    else       buffer[module_offset + x] &= ~(1 << local_y);
}

uint8_t MAX7219_GetPixel(uint8_t* buffer, int8_t x, int8_t y) {
    if (x < 0 || x >= TOTAL_COLS || y < 0 || y >= TOTAL_ROWS) return 0;
    uint8_t module_offset = (y < 8) ? 8 : 0;
    int8_t local_y = (y < 8) ? y : y - 8;
    return (buffer[module_offset + x] & (1 << local_y)) ? 1 : 0;
}

int8_t Wrap_Coordinate(int8_t x) {
	// X Axis Ring Wrapping
	// (x % 8 + 8) % 8
	// x = -1 -> x = 7
    return (x % TOTAL_COLS + TOTAL_COLS) % TOTAL_COLS;
}

uint8_t Check_Pixel_Collision(int8_t x, int8_t y) {
    if (y >= TOTAL_ROWS) return 1; // Floor collision
    if (MAX7219_GetPixel(locked_buffer, x, y)) return 1; // Pile collision
    return 0;
}

void Check_Full_Rows(void) {
    for (int y = TOTAL_ROWS - 1; y >= 0; y--) {
        uint8_t is_full = 1;
        for (int x = 0; x < TOTAL_COLS; x++) {
            if (MAX7219_GetPixel(locked_buffer, x, y) == 0) {
                is_full = 0;
                break;
            }
        }
        if (is_full) {
        	game_score += (FULLROW_POINTS * game_level);
        	lines_cleared++;
        	if (lines_cleared % LNS_CLR_NXT_LVL == 0) {
        		game_level++;
        	    if (gravity_speed > 100) {
        	    	gravity_speed -= 50; // Speed up
        	    }
        	}
            for (int k = y; k > 0; k--) {
                for (int x = 0; x < TOTAL_COLS; x++) {
                    uint8_t pixel_above = MAX7219_GetPixel(locked_buffer, x, k - 1);
                    MAX7219_SetPixel(locked_buffer, x, k, pixel_above);
                }
            }
            for (int x = 0; x < TOTAL_COLS; x++) MAX7219_SetPixel(locked_buffer, x, 0, 0);
            y++;
        }
    }
}

void Tetromino_Spawn(uint8_t shape_id) {
    if (shape_id >= NUM_SHAPES) shape_id = 0;
    memcpy(active_shape_coords, SHAPES[shape_id], COORDS_PER_SHAPE);
    pivot_x = 3;
    pivot_y = 0;
    current_shape_id = shape_id;
}

uint8_t Tetromino_CheckCollision(int8_t target_pivot_x, int8_t target_pivot_y) {
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = target_pivot_x + active_shape_coords[i*2];
        int8_t block_y = target_pivot_y + active_shape_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);

        if (Check_Pixel_Collision(block_x, block_y)) {
            return 1;
        }
    }
    return 0;
}

void Tetromino_Lock(void) {
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = pivot_x + active_shape_coords[i*2];
        int8_t block_y = pivot_y + active_shape_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);

        MAX7219_SetPixel(locked_buffer, block_x, block_y, 1);
    }
}

void Tetromino_Draw(uint8_t* buffer) {
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = pivot_x + active_shape_coords[i*2];
        int8_t block_y = pivot_y + active_shape_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);

        MAX7219_SetPixel(buffer, block_x, block_y, 1);
    }
}

void Tetromino_Rotate(void) {
    int8_t temp_coords[COORDS_PER_SHAPE];

    // 90 degrees Anti-Clockwise Rotation
    // Formula: NewX = OldY, NewY = -OldX
    for (int i = 0; i < 4; i++) {
        int8_t old_x = active_shape_coords[i * 2];
        int8_t old_y = active_shape_coords[i * 2 + 1];

        temp_coords[i * 2]     = old_y;
        temp_coords[i * 2 + 1] = -old_x;
    }

    // We check if these new coordinates would hit anything
    uint8_t possible = 1;
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = pivot_x + temp_coords[i*2];
        int8_t block_y = pivot_y + temp_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);

        if (Check_Pixel_Collision(block_x, block_y)) {
            possible = 0; // Collision detected, invalid rotation move
            break;
        }
    }

    if (possible) { // Valid rotation move
        memcpy(active_shape_coords, temp_coords, COORDS_PER_SHAPE);
    }
}

/* --- 10. HARDWARE INIT FUNCTIONS --- */
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
