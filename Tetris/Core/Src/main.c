/* USER CODE BEGIN Header */
/**
  * PROJECT: STM32 Tetris
  * BOARD: STM32F411E-DISCO
  * AUDIO: CS43L22 (I2S3/I2C1)
  * SCREEN: MAX7219 (SPI1/PD0)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_INIT,
    STATE_SPAWN,
    STATE_PLAYING,
    STATE_CHECK_LINES,
    STATE_STOPPED,
    STATE_GAMEOVER
} GameState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- MAX7219 REGISTERS ---
#define REG_SHUTDOWN    0x0C
#define REG_SCAN_LIMIT  0x0B
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_DISPLAY_TEST 0x0F

// --- GAME CONFIG ---
#define NUM_MODULES     2
#define TOTAL_ROWS      16
#define TOTAL_COLS      8
#define INPUT_DELAY     100
#define JOY_LOW         200
#define JOY_HIGH        3800

// --- SCORING ---
#define DROP_POINTS     1
#define FULLROW_POINTS  40
#define LNS_CLR_NXT_LVL 4

// --- AUDIO ADDRESS ---
#define CS43L22_ADDR    0x94

// --- SHAPES ---
#define NUM_SHAPES 7
#define COORDS_PER_SHAPE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// External Handles (Defined in their respective .c files)
extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s3;

// Shapes Database
const int8_t SHAPES[NUM_SHAPES][COORDS_PER_SHAPE] = {
//  {x1,y1,   x2,y2,   x3,y3,   x4,y4}
	{-1, 0,   0, 0,   1, 0,   2, 0}, // I
    {-1,-1,  -1, 0,   0, 0,   1, 0}, // J
    {-1, 0,   0, 0,   1, 0,   1,-1}, // L
	{ 0, 0,   1, 0,   0,-1,   1,-1}, // O
	{-1, 0,   0, 0,   0,-1,   1,-1}, // S
	{ 0,-1,  -1, 0,   0, 0,   1, 0}, // T
	{-1,-1,   0,-1,   0, 0,   1, 0}  // Z
};

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

// Game State Variables
GameState_t current_state = STATE_INIT;
int8_t pivot_x = 0;
int8_t pivot_y = 0;
int8_t active_shape_coords[COORDS_PER_SHAPE];
uint8_t current_shape_id = 3;

// Display Memory
uint8_t display_buffer[TOTAL_ROWS];
uint8_t locked_buffer[TOTAL_ROWS];

// Timing & Physics
uint32_t last_input_time = 0;
uint32_t last_gravity_time = 0;
uint32_t gravity_speed = 500; // The lower the faster

// Stats
volatile uint8_t game_level = 1;
volatile uint32_t game_score = 0;
volatile uint32_t lines_cleared = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Screen_Init(void);
void Audio_Init(void);
void Audio_Beep(uint16_t duration_ms);

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  // NOTE: Initial state is handled in the Loop to ensure clean startup
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // --- EMERGENCY STOP (Blue Button PA0) ---
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          current_state = STATE_STOPPED;
          HAL_Delay(500);
      }

      switch (current_state)
      {
          case STATE_INIT:
              Screen_Init();
              Audio_Init();
              Audio_Beep(350);

              MAX7219_Clear(locked_buffer);
              MAX7219_Clear(display_buffer);

              // Seed random with ADC noise
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
            	  Audio_Beep(350);
            	  current_state = STATE_GAMEOVER;
              } else {
                  last_gravity_time = HAL_GetTick();
                  current_state = STATE_PLAYING;
              }
              break;

          case STATE_PLAYING:
          {
              uint32_t now = HAL_GetTick();

              // 1. Gravity
              if (now - last_gravity_time > gravity_speed) {
                  if (!Tetromino_CheckCollision(pivot_x, pivot_y + 1)) {
                      pivot_y++;
                      game_score += DROP_POINTS;
                  } else {
                      Tetromino_Lock();
                      Audio_Beep(150); // Lock Sound
                      current_state = STATE_CHECK_LINES;
                  }
                  last_gravity_time = now;
              }

              // 2. Input
              if (now - last_input_time > INPUT_DELAY) {
                  uint32_t val_x = Read_ADC_Channel(ADC_CHANNEL_1);
                  uint32_t val_y = Read_ADC_Channel(ADC_CHANNEL_2);
                  int8_t next_x = pivot_x;

                  // X Movement
                  if (val_x < JOY_LOW) next_x--;
                  else if (val_x > JOY_HIGH) next_x++;

                  if(next_x < 0) next_x = TOTAL_COLS - 1;
                  if(next_x >= TOTAL_COLS) next_x = 0;

                  if (!Tetromino_CheckCollision(next_x, pivot_y)) pivot_x = next_x;

                  // Drop
                  if (val_y > JOY_HIGH) {
                      if (!Tetromino_CheckCollision(pivot_x, pivot_y + 1)) {
                          pivot_y++;
                          game_score += DROP_POINTS;
                      }
                  }

                  // Rotation (PA3)
                  static uint8_t rot_pressed = 0;
                  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                      if (rot_pressed == 0) {
                          Tetromino_Rotate();
                          rot_pressed = 1;
                      }
                  } else {
                      rot_pressed = 0;
                  }
                  last_input_time = now;
              }

              // 3. Render
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
        	  memcpy(&display_buffer[0], GAME_OVER_ICON, 8);
              memcpy(&display_buffer[8], GAME_OVER_ICON, 8);
              MAX7219_Flush();

              // Restart Logic (Wait for Release -> Then Wait for Press)
              static uint8_t can_restart = 0;

              // Check if button is released (HIGH)
              if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
            	  can_restart = 1; // Armed and ready
              }

              // Check if button is pressed (LOW) AND we are ready
              if (can_restart && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
            	  can_restart = 0; // Reset flag
                  current_state = STATE_INIT;
              }
              break;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* --- HELPER FUNCTIONS --- */

void Audio_Init(void) {
    uint32_t start_tick;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < 50);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < 50);

    // Init Sequence (Starts Muted)
    uint8_t cmds[][2] = {
        {0x02, 0x01}, // Power Down
        {0x00, 0x99}, {0x47, 0x80}, {0x32, 0xBB}, {0x32, 0x3B}, {0x00, 0x00}, // Required
        {0x04, 0xAF}, // Headphones ON
        {0x05, 0x81}, // Clock Auto
        {0x06, 0x44}, // Slave Mode + MUTE ON (0x44)
        {0x27, 0x00}, // Limiter Disable

        // To change volume:
		// change the four "0x--}" to:
		// 0x18} = MAX
		// 0x10} = High
		// 0x0A} = Medium
        {0x1A, 0x10}, {0x1B, 0x10},
        {0x20, 0x10}, {0x21, 0x10},

        {0x02, 0x9E}  // Power UP
    };

    for (int i = 0; i < 16; i++) {
        HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, cmds[i], 2, 100);
        start_tick = HAL_GetTick();
        while((HAL_GetTick() - start_tick) < 5);
    }
}

void Audio_Beep(uint16_t duration_ms) {
    // Prepare 500Hz Square Wave
    int16_t tone_buffer[192];
    for (int i = 0; i < 192; i++) {
        tone_buffer[i] = (i < 96) ? 32000 : -32000;
    }

    // UNMUTE (Register 0x06 -> 0x04)
    uint8_t unmute_cmd[] = {0x06, 0x04};
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, unmute_cmd, 2, 10);

    // Play Tone
    uint32_t cycles = duration_ms / 2;
    if (cycles == 0) cycles = 1;
    for (uint32_t c = 0; c < cycles; c++) {
        HAL_I2S_Transmit(&hi2s3, (uint16_t*)tone_buffer, 192, 10);
    }

    // Play tiny silence to bring voltage to Zero (Prevents "Pop")
    int16_t zero_buffer[4] = {0, 0, 0, 0};
    HAL_I2S_Transmit(&hi2s3, (uint16_t*)zero_buffer, 4, 10);

    // MUTE (Register 0x06 -> 0x44)
    uint8_t mute_cmd[] = {0x06, 0x44};
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, mute_cmd, 2, 10);
}

void Screen_Init(void) {
    uint8_t commands[][2] = {
        {REG_DISPLAY_TEST, 0x00},
		{REG_SHUTDOWN, 0x01},
        {REG_SCAN_LIMIT, 0x07},
		{REG_DECODE_MODE, 0x00},

		// CHANGE BRIGHTNESS HERE:
		// 0x00 = Min
		// 0x02 = Medium
		// 0x04 = High
        {REG_INTENSITY, 0x01}
    };
    for (int c = 0; c < 5; c++) {
        // NOTE: USING PD0 for CS (Based on latest fix)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        for(int m=0; m<NUM_MODULES; m++) {
            HAL_SPI_Transmit(&hspi1, &commands[c][0], 1, 10);
            HAL_SPI_Transmit(&hspi1, &commands[c][1], 1, 10);
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    }
}

void MAX7219_Flush(void) {
    uint8_t cfg0[] = {REG_DISPLAY_TEST, 0x00};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    for(int m=0; m<NUM_MODULES; m++) {
        HAL_SPI_Transmit(&hspi1, &cfg0[0], 1, 10);
        HAL_SPI_Transmit(&hspi1, &cfg0[1], 1, 10);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    uint8_t cfg1[] = {REG_SCAN_LIMIT, 0x07};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    for(int m=0; m<NUM_MODULES; m++) {
        HAL_SPI_Transmit(&hspi1, &cfg1[0], 1, 10);
        HAL_SPI_Transmit(&hspi1, &cfg1[1], 1, 10);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    uint8_t cfg2[] = {REG_DECODE_MODE, 0x00};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    for(int m=0; m<NUM_MODULES; m++) {
        HAL_SPI_Transmit(&hspi1, &cfg2[0], 1, 10);
        HAL_SPI_Transmit(&hspi1, &cfg2[1], 1, 10);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    // Force Wakeup
    uint8_t cfg3[] = {REG_SHUTDOWN, 0x01};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    for(int m=0; m<NUM_MODULES; m++) {
        HAL_SPI_Transmit(&hspi1, &cfg3[0], 1, 10);
        HAL_SPI_Transmit(&hspi1, &cfg3[1], 1, 10);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    // Pixel Data
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t addr = i + 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
        HAL_SPI_Transmit(&hspi1, &display_buffer[8 + i], 1, 10);
        HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
        HAL_SPI_Transmit(&hspi1, &display_buffer[i], 1, 10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
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
    return (x % TOTAL_COLS + TOTAL_COLS) % TOTAL_COLS;
}

uint8_t Check_Pixel_Collision(int8_t x, int8_t y) {
    if (y >= TOTAL_ROWS) return 1;
    if (MAX7219_GetPixel(locked_buffer, x, y)) return 1;
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
            Audio_Beep(400);
            game_score += (FULLROW_POINTS * game_level);
            lines_cleared++;
            if (lines_cleared % LNS_CLR_NXT_LVL == 0) {
                game_level++;
                if (gravity_speed > 100) gravity_speed -= 50;
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
    pivot_x = 3; pivot_y = 0; current_shape_id = shape_id;
}

uint8_t Tetromino_CheckCollision(int8_t target_pivot_x, int8_t target_pivot_y) {
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = target_pivot_x + active_shape_coords[i*2];
        int8_t block_y = target_pivot_y + active_shape_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);
        if (Check_Pixel_Collision(block_x, block_y)) return 1;
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
    for (int i = 0; i < 4; i++) {
        int8_t old_x = active_shape_coords[i * 2];
        int8_t old_y = active_shape_coords[i * 2 + 1];
        temp_coords[i * 2]     = old_y;
        temp_coords[i * 2 + 1] = -old_x;
    }
    uint8_t possible = 1;
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = pivot_x + temp_coords[i*2];
        int8_t block_y = pivot_y + temp_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);
        if (Check_Pixel_Collision(block_x, block_y)) {
            possible = 0;
            break;
        }
    }
    if (possible) memcpy(active_shape_coords, temp_coords, COORDS_PER_SHAPE);
}

uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
