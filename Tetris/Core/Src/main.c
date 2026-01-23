/* USER CODE BEGIN Header */
/**
  * PROJECT: STM32 TETRIS
  * BOARD: STM32F411E-DISCO
  *
  * COMPONENT REQUIREMENTS:
  * + 2 LED MATRIX DISPLAYS (MAX7219)
  * + 1 ANALOG JOYSTICK
  * + 1 LCD CHARACTER DISPLAY (HD44780)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STATE_INIT,
	STATE_SPAWN,
	STATE_PLAYING,
	STATE_STATS,
	STATE_STOPPED,
	STATE_GAMEOVER
} GameState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// PLAYER CONTROLLERS
#define BTN_PORT        GPIOA
#define BTN_ANTIROT_PIN GPIO_PIN_0    // PA0 (STM Blue Button)
#define BTN_ROT_PIN     GPIO_PIN_3    // PA3 to SW (JoyStick Center Button)
#define JS_RX_CHANNEL   ADC_CHANNEL_1 // PA1 to RX (JS Horizontal Potentiometer)
#define JS_RY_CHANNEL   ADC_CHANNEL_2 // PA2 to RY (JS Vertical Potentiometer)

// MAX7219 LED MATRIX DISPLAY DRIVER
#define MTX_PORT     GPIOA
#define MTX_CS_PIN   GPIO_PIN_6 // PA6 to CS (MAX7219 Chip Select) (Load)
								// PA5 to DIN (MAX7219 Data Input)
								// PA7 to CLK (MAX7219 Clock)

// AUDIO
#define AUDIO_RESET_PORT GPIOD
#define AUDIO_RESET_PIN  GPIO_PIN_4 // PD4 (STM 3.5mm Mini-Jack Port)

// LCD
#define LCD_PORT   GPIOE
#define RS_PIN     GPIO_PIN_7
#define E_PIN      GPIO_PIN_8
#define D4_PIN     GPIO_PIN_9
#define D5_PIN     GPIO_PIN_10
#define D6_PIN     GPIO_PIN_11
#define D7_PIN     GPIO_PIN_12

// REGISTER ADDRESSES
#define REG_SHUTDOWN    0x0C
#define REG_SCAN_LIMIT  0x0B
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_DISPLAY_TEST 0x0F
#define CS43L22_ADDR    0x94

// GAME PARAMETERS
#define NUM_MODULES     2
#define TOTAL_ROWS      16
#define TOTAL_COLS      8
#define DEBOUNCE_DELAY  50
#define JS_LOW          800
#define JS_HIGH         3000
#define DROP_POINTS     1
#define FULLROW_POINTS  40
#define LNS_CLR_NXT_LVL 4
#define NUM_SHAPES      7

// LCD Custom Characters
// 0:I, 1:J, 2:L, 3:O, 4:S, 5:T, 6:Z
const uint8_t TETRIS_LCD_CHARS[NUM_SHAPES][8] = {
	{ // 0: I
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110
    },
    { // 1: J
    	0b00111,
		0b00111,
		0b00111,
		0b00111,
		0b11111,
		0b11111,
		0b11111,
		0b00000
    },
    { // 2: L
    	0b11100,
		0b11100,
		0b11100,
		0b11100,
		0b11111,
		0b11111,
		0b11111,
		0b00000
    },
    { // 3: O
    	0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b00000
    },
    { // 4: S
    	0b01111,
		0b01111,
		0b01111,
		0b11110,
		0b11110,
		0b11110,
		0b00000,
		0b00000
    },
    { // 5: T
    	0b11111,
		0b11111,
		0b11111,
		0b00100,
		0b00100,
		0b00100,
		0b00100,
		0b00000
    },
    { // 6: Z
    	0b11110,
		0b11110,
		0b11110,
		0b01111,
		0b01111,
		0b01111,
		0b00000,
		0b00000
    }
};
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

// MAX7219 LED Matrix Display Custom Shapes
//	0:I, 1:J, 2:L, 3:O, 4:S, 5:T, 6:Z
const int8_t SHAPES[NUM_SHAPES][8] = {
//  {x1,y1,   x2,y2,   x3,y3,   x4,y4}
	{-1, 0,   0, 0,   1, 0,   2, 0}, // I
    {-1,-1,  -1, 0,   0, 0,   1, 0}, // J
    {-1, 0,   0, 0,   1, 0,   1,-1}, // L
	{ 0, 0,   1, 0,   0,-1,   1,-1}, // O
	{-1, 0,   0, 0,   0,-1,   1,-1}, // S
	{ 0,-1,  -1, 0,   0, 0,   1, 0}, // T
	{-1,-1,   0,-1,   0, 0,   1, 0}  // Z
};
const uint8_t START_ICON_1[8] = {
    0b00001111,
    0b00001001,
    0b11111001,
    0b10000001,
    0b10000001,
    0b11111001,
    0b00001001,
    0b00001111
};
const uint8_t START_ICON_2[8] = {
    0b00000000,
    0b00000110,
    0b00000110,
    0b01111110,
    0b01111110,
    0b00000110,
    0b00000110,
    0b00000000
};
const uint8_t GAMEOVER_ICON_1[8] = {
    0b10000001,
    0b01000010,
    0b00100100,
    0b00011000,
    0b00011000,
    0b00100100,
    0b01000010,
    0b10000001
};
const uint8_t GAMEOVER_ICON_2[8] = {
    0b01000010,
    0b10100101,
    0b01011010,
    0b00100100,
    0b00100100,
    0b01011010,
    0b10100101,
    0b01000010
};

// Game State variables
GameState_t current_state = STATE_INIT;
int8_t pivot_x = 0;
int8_t pivot_y = 0;
int8_t active_shape_coords[8];
uint8_t current_shape_id = 3;

// Display Memory variables
uint8_t display_buffer[TOTAL_ROWS];
uint8_t locked_buffer[TOTAL_ROWS];
int16_t dma_tone_buffer[192];

// Timing variables
uint32_t last_input_time = 0;
uint32_t last_gravity_time = 0;
uint32_t gravity_speed = 500; // The lower the faster
uint32_t beep_start_time = 0;
uint16_t beep_total_duration = 0;

// Stats variables
volatile uint8_t game_level = 1;
volatile uint8_t next_shape_id = 0;
volatile uint32_t game_score = 0;
volatile uint32_t lines_cleared = 0;
volatile uint8_t beeping = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// MAX7219 LED Matrix Display Functions
void Screen_Init(void);
void MAX7219_Flush(void);
void MAX7219_Clear(uint8_t* buffer);
void MAX7219_SetPixel(uint8_t* buffer, int8_t x, int8_t y, uint8_t state);
uint8_t MAX7219_GetPixel(uint8_t* buffer, int8_t x, int8_t y);
uint32_t Read_ADC_Channel(uint32_t channel);

// Helper Functions
int8_t Wrap_Coordinate(int8_t x);
uint8_t Check_Pixel_Collision(int8_t x, int8_t y);
uint8_t Check_Full_Rows(void);

// Tetromino Functions
void Tetromino_Spawn(uint8_t shape_id);
uint8_t Tetromino_CheckCollision(int8_t target_pivot_x, int8_t target_pivot_y);
void Tetromino_Lock(void);
void Tetromino_Draw(uint8_t* buffer);
void Tetromino_Rotate(int8_t direction); // +1 or -1

// LCD Display Functions
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_LoadCustomChars(void);
void LCD_UpdateStats(void);

// Audio Functions
void Audio_Init(void);
void Audio_Beep(uint16_t duration_ms);
void Audio_Update(void);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  // Hardware Initialization (Run once)
  Screen_Init();
  Audio_Init();
  LCD_Init();
  LCD_LoadCustomChars();

  // Seed random
  srand(HAL_GetTick() + Read_ADC_Channel(JS_RX_CHANNEL));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  uint32_t now = HAL_GetTick();
	  Audio_Update();

  	  // Emergency Stop = Clockwise + Anti-clockwise rotation buttons pressed together
	  if (current_state != STATE_STOPPED &&
  	  HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_SET &&
  	  HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_RESET) {
  		  current_state = STATE_STOPPED;
  	      uint32_t stop_time = HAL_GetTick();
  	      while((HAL_GetTick() - stop_time) < DEBOUNCE_DELAY);
  	  }
      switch (current_state)
      {
          case STATE_INIT:
          {
        	  // Start Animation
        	  uint8_t frame_index = (now >> 9) & 1;
        	  static uint32_t last_frame_time = 0;

        	  if (now - last_frame_time > 100) {
        		  const uint8_t* current_icon = (frame_index) ? START_ICON_2 : START_ICON_1;
        		  memcpy(&display_buffer[0], current_icon, 8);
        		  memcpy(&display_buffer[8], current_icon, 8);
        		  MAX7219_Flush();
        		  last_frame_time = now;
        	  }
        	  static uint8_t start_ok = 0;

        	  if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_RESET &&
        	  HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_SET) {
        	      start_ok = 1;
        	  }
        	  if (start_ok) {
        		  if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_SET ||
        		  HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_RESET) {

        			  Audio_Beep(500);
        			  uint32_t tick = HAL_GetTick();
        			  while((HAL_GetTick() - tick) < DEBOUNCE_DELAY);

        			  // Reset Game Variables
        			  game_score = 0;
        			  game_level = 1;
        			  lines_cleared = 0;
        			  gravity_speed = 500;
        			  next_shape_id = rand() % NUM_SHAPES;

        			  MAX7219_Clear(locked_buffer);
        			  MAX7219_Clear(display_buffer);
        			  LCD_UpdateStats();

        			  start_ok = 0;
        			  current_state = STATE_SPAWN;
        		  }
        	  }
              break;
      	  }

          case STATE_SPAWN:
          {
              Tetromino_Spawn(next_shape_id);
              if (Tetromino_CheckCollision(pivot_x, pivot_y)) {
            	  Audio_Beep(300);
            	  current_state = STATE_GAMEOVER;
              } else {
                  last_gravity_time = HAL_GetTick();
                  current_state = STATE_PLAYING;
              }
              break;
          }

          case STATE_PLAYING:
          {
              // Gravity Logic
              if (now - last_gravity_time > gravity_speed) {
            	  if (!Tetromino_CheckCollision(pivot_x, pivot_y + 1)) {
            		  pivot_y++;
                      game_score += DROP_POINTS;
                  } else {
                	  // Lock Piece
                	  Tetromino_Lock();
                	  Audio_Beep(50);
                	  current_state = STATE_STATS;
                  }
            	  last_gravity_time = now;
              }
              // Joystick Input
              if (now - last_input_time > DEBOUNCE_DELAY) {
                  uint32_t val_x = Read_ADC_Channel(JS_RX_CHANNEL);
                  uint32_t val_y = Read_ADC_Channel(JS_RY_CHANNEL);
                  int8_t next_x = pivot_x;

                  // X Movement
                  if (val_x < JS_LOW) next_x--;
                  else if (val_x > JS_HIGH) next_x++;
                  if(next_x < 0) next_x = TOTAL_COLS - 1;
                  if(next_x >= TOTAL_COLS) next_x = 0;
                  if (!Tetromino_CheckCollision(next_x, pivot_y)) pivot_x = next_x;

                  // Y Movement (Drop)
                  if (val_y > JS_HIGH) {
                      if (!Tetromino_CheckCollision(pivot_x, pivot_y + 1)) {
                          pivot_y++;
                          game_score += DROP_POINTS;
                      }
                  }
                  last_input_time = now;
              }
              // Anti-Clockwise Rotation
              static uint8_t antirot_latch = 0;
              if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_SET) {
            	  if (antirot_latch == 0) {
            		  Tetromino_Rotate(-1);
                      antirot_latch = 1;
                  }
              } else {
            	  antirot_latch = 0;
              }
              // Clockwise Rotation
              static uint8_t rot_latch = 0;
              if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_RESET) {
            	  if (rot_latch == 0) {
            		  Tetromino_Rotate(1);
                      rot_latch = 1;
                  }
              } else {
            	  rot_latch = 0;
              }
              // LED Matrix Render
              memcpy(display_buffer, locked_buffer, sizeof(locked_buffer));
              Tetromino_Draw(display_buffer);
              MAX7219_Flush();
              break;
          }

          case STATE_STATS:
          {
        	  uint8_t cleared_lines = Check_Full_Rows();
        	  if (cleared_lines > 0) {
        		  game_score += (cleared_lines * FULLROW_POINTS * game_level);
        		  lines_cleared += cleared_lines;
        		  // Level Up Logic
                  if (lines_cleared >= (game_level * LNS_CLR_NXT_LVL)) {
                	  game_level++;
                	  if (gravity_speed > 100) {
                		  gravity_speed -= 50;
                	  }
                	  Audio_Beep(600);
                  }
                  else {
                	  Audio_Beep(300);
                  }
        	  }
              // We update the next_shape_id variable which LCD_UpdateStats reads
              next_shape_id = rand() % NUM_SHAPES;
              // Update the LCD with the new score, level and next shape
              LCD_UpdateStats();

              current_state = STATE_SPAWN;
              break;
          }

          case STATE_STOPPED:
          {
        	  MAX7219_Clear(display_buffer);
        	  MAX7219_Flush();

              static uint8_t restart_ok = 0;

              if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_RESET &&
              HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_SET) {
            	  restart_ok = 1;
              }
              if (restart_ok) {
            	  if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_SET ||
            	  HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_RESET) {

            		  uint32_t tick = HAL_GetTick();
            		  while((HAL_GetTick() - tick) < DEBOUNCE_DELAY);

            		  restart_ok = 0;
            		  current_state = STATE_INIT;
            	  }
              }
              break;
          }

          case STATE_GAMEOVER:
          {
              // GameOver Animation
              uint8_t frame_index = (now >> 9) & 1;
              static uint32_t last_frame_time = 0;

              if (now - last_frame_time > 100) {
            	  const uint8_t* current_icon = (frame_index) ? GAMEOVER_ICON_2 : GAMEOVER_ICON_1;
                  memcpy(&display_buffer[0], current_icon, 8);
                  memcpy(&display_buffer[8], current_icon, 8);
                  MAX7219_Flush();
                  last_frame_time = now;
              }
              static uint8_t restart_ok = 0;

              if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_RESET &&
              HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_SET) {
            	  restart_ok = 1;
              }
              if (restart_ok) {
            	  if (HAL_GPIO_ReadPin(BTN_PORT, BTN_ANTIROT_PIN) == GPIO_PIN_SET ||
            	  HAL_GPIO_ReadPin(BTN_PORT, BTN_ROT_PIN) == GPIO_PIN_RESET) {

            		  uint32_t tick = HAL_GetTick();
            		  while((HAL_GetTick() - tick) < DEBOUNCE_DELAY);

            		  restart_ok = 0;
            		  current_state = STATE_INIT;
            	  }
              }
              break;
          }
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

/* MAX7219 LED MATRIX DISPLAY FUNCTIONS */
void Screen_Init(void) {
    uint8_t commands[][2] = {
    	{REG_DISPLAY_TEST, 0x00},
		{REG_SHUTDOWN, 0x01},
		{REG_SCAN_LIMIT, 0x07},
		{REG_DECODE_MODE, 0x00},

		// To change brightness,
		// change the next "0x--}" to:
		// 0x00} = Min.
		// 0x02} = Medium
		// 0x04} = High
		{REG_INTENSITY, 0x01} // Brightness level
    };
    for (int c = 0; c < 5; c++) {
    	HAL_GPIO_WritePin(MTX_PORT, MTX_CS_PIN, GPIO_PIN_RESET);
        for(int m=0; m<NUM_MODULES; m++) {
        	HAL_SPI_Transmit(&hspi1, &commands[c][0], 1, 10);
        	HAL_SPI_Transmit(&hspi1, &commands[c][1], 1, 10);
        }
        HAL_GPIO_WritePin(MTX_PORT, MTX_CS_PIN, GPIO_PIN_SET);
    }
}

void MAX7219_Flush(void) {
    uint8_t packet[4]; // Buffer for [Addr2, Data2, Addr1, Data1]

    for (uint8_t i = 0; i < 8; i++) {
    	uint8_t addr = i + 1;

    	// Module 2 Top-Left shifted out first
        packet[0] = addr;
        packet[1] = display_buffer[8 + i];
        // Module 1 Bottom-Right shifted out last
        packet[2] = addr;
        packet[3] = display_buffer[i];

        HAL_GPIO_WritePin(MTX_PORT, MTX_CS_PIN, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, packet, 4, 10);
        HAL_GPIO_WritePin(MTX_PORT, MTX_CS_PIN, GPIO_PIN_SET);
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

uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
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

/* BASE GAME LOGIC FUNCTIONS */
int8_t Wrap_Coordinate(int8_t x) {
    return (x % TOTAL_COLS + TOTAL_COLS) % TOTAL_COLS;
}

uint8_t Check_Pixel_Collision(int8_t x, int8_t y) {
    if (y >= TOTAL_ROWS) return 1;
    if (MAX7219_GetPixel(locked_buffer, x, y)) return 1;
    return 0;
}

uint8_t Check_Full_Rows(void) {
    uint8_t fr_count = 0;

    for (int8_t y = TOTAL_ROWS - 1; y >= 0; y--) {
        uint8_t row_full = 1;

        for (uint8_t x = 0; x < TOTAL_COLS; x++) {
            if (MAX7219_GetPixel(locked_buffer, x, y) == 0) {
                row_full = 0;
                break;
            }
        }
        if (row_full) {
            fr_count++;
            // Move all rows above this one down
            for (int8_t k = y; k > 0; k--) {
                for (uint8_t x = 0; x < TOTAL_COLS; x++) {
                    uint8_t pixel_above = MAX7219_GetPixel(locked_buffer, x, k - 1);
                    MAX7219_SetPixel(locked_buffer, x, k, pixel_above);
                }
            }
            // Clear top row
            for (uint8_t x = 0; x < TOTAL_COLS; x++) {
                MAX7219_SetPixel(locked_buffer, x, 0, 0);
            }
            y++;
        }
    }
    return fr_count;
}

/* TETROMINO FUNCTIONS */
void Tetromino_Spawn(uint8_t shape_id) {
    if (shape_id >= NUM_SHAPES) shape_id = 0;
    memcpy(active_shape_coords, SHAPES[shape_id], 8);
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

void Tetromino_Rotate(int8_t direction) {
	if (current_shape_id == 3) return;
	int8_t temp_coords[8];
    for (int i = 0; i < 4; i++) {
        int8_t old_x = active_shape_coords[i * 2];
        int8_t old_y = active_shape_coords[i * 2 + 1];

        if (direction > 0) {
            // Clockwise: (x, y) to (-y, x)
            temp_coords[i * 2]     = old_y;
            temp_coords[i * 2 + 1] = -old_x;
        } else {
            // Anti-Clockwise: (x, y) to (y, -x)
            temp_coords[i * 2]     = -old_y;
            temp_coords[i * 2 + 1] = old_x;
        }
    }

    // Check collisions to validate the possible new rotation
    uint8_t rot_ok = 1;
    for (int i = 0; i < 4; i++) {
        int8_t raw_x = pivot_x + temp_coords[i*2];
        int8_t block_y = pivot_y + temp_coords[i*2 + 1];
        int8_t block_x = Wrap_Coordinate(raw_x);
        if (Check_Pixel_Collision(block_x, block_y)) {
            rot_ok = 0;
            break;
        }
    }
    if (rot_ok) memcpy(active_shape_coords, temp_coords, 8);
}

/* LCD FUNCTIONS */
void LCD_Send4Bits(uint8_t val) {
    HAL_GPIO_WritePin(LCD_PORT, D4_PIN, (val >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_PORT, D5_PIN, (val >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_PORT, D6_PIN, (val >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_PORT, D7_PIN, (val >> 3) & 0x01);
}

void LCD_EnablePulse(void) {
    HAL_GPIO_WritePin(LCD_PORT, E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_PORT, E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_RESET);
    LCD_Send4Bits(cmd >> 4);
    LCD_EnablePulse();
    LCD_Send4Bits(cmd);
    LCD_EnablePulse();
}

void LCD_SendData(uint8_t data) {
    HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_SET);
    LCD_Send4Bits(data >> 4);
    LCD_EnablePulse();
    LCD_Send4Bits(data);
    LCD_EnablePulse();
}

void LCD_Init(void) {
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_PORT, RS_PIN, GPIO_PIN_RESET);

    LCD_Send4Bits(0x03); LCD_EnablePulse(); HAL_Delay(5);
    LCD_Send4Bits(0x03); LCD_EnablePulse(); HAL_Delay(1);
    LCD_Send4Bits(0x03); LCD_EnablePulse(); HAL_Delay(1);
    LCD_Send4Bits(0x02); LCD_EnablePulse();

    LCD_SendCommand(0x28);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06);
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}

void LCD_LoadCustomChars(void) {
    LCD_SendCommand(0x40);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 8; j++) {
            LCD_SendData(TETRIS_LCD_CHARS[i][j]);
        }
    }
    LCD_SendCommand(0x80);
}

void LCD_SendString(char *str) {
    while (*str) LCD_SendData(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendCommand(address);
}

void LCD_UpdateStats(void) {
    char buffer[16];
    LCD_SetCursor(0, 0);
    sprintf(buffer, "Score:%lu   ", game_score);
    LCD_SendString(buffer);

    LCD_SetCursor(1, 0);
    sprintf(buffer, "Lvl:%d Next:", game_level);
    LCD_SendString(buffer);
    LCD_SendData(next_shape_id);
    LCD_SendString(" ");
}

/* AUDIO FUNCTIONS */
void Audio_Init(void) {
	uint32_t start_tick;

    HAL_GPIO_WritePin(AUDIO_RESET_PORT, AUDIO_RESET_PIN, GPIO_PIN_RESET);
	start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < 10);
    HAL_GPIO_WritePin(AUDIO_RESET_PORT, AUDIO_RESET_PIN, GPIO_PIN_SET);
	start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < 10);

    // Init Sequence (Starts Muted)
    uint8_t cmds[15][2] = {
        {0x02, 0x01}, // Power DOWN
        {0x00, 0x99}, {0x47, 0x80}, {0x32, 0xBB}, {0x32, 0x3B}, {0x00, 0x00},
        {0x04, 0x92}, {0x05, 0x00}, {0x06, 0x44}, {0x27, 0x00},

        // To change volume,
		// change the four next "0x--}" to:
		// 0x18} = MAX
		// 0x10} = High
		// 0x0A} = Medium
        {0x1A, 0x18}, {0x1B, 0x18}, {0x20, 0x18}, {0x21, 0x18}, // Volume level

        {0x02, 0x9E}  // Power UP
    };

    for (int i = 0; i < 15; i++) {
        HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, cmds[i], 2, 100);
        start_tick = HAL_GetTick();
        while((HAL_GetTick() - start_tick) < 5);
    }
}

void Audio_Beep(uint16_t duration_ms) {
    if (beeping) {
        HAL_I2S_DMAStop(&hi2s3);
    }
    for (int i = 0; i < 192; i++) {
        dma_tone_buffer[i] = (i < 96) ? 32000 : -32000;
    }
    uint8_t unmute_cmd[] = {0x06, 0x04};
    HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, unmute_cmd, 2, 10);
    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)dma_tone_buffer, 192);

    beeping = 1;
    beep_start_time = HAL_GetTick();
    beep_total_duration = duration_ms;
}

void Audio_Update(void) {
    // If we are beeping passed the time we mute it
    if (beeping && (HAL_GetTick() - beep_start_time >= beep_total_duration)) {

        HAL_I2S_DMAStop(&hi2s3);
        uint8_t mute_cmd[] = {0x06, 0x44};
        HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDR, mute_cmd, 2, 10);
        beeping = 0;
    }
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
