#include "main.h"
#include <string.h>

/* --- 1. HARDWARE HANDLES --- */
/* ID/Nametags for our hardware peripherals. */
SPI_HandleTypeDef hspi1; // Handle for the Screen (SPI)
ADC_HandleTypeDef hadc1; // Handle for the Joystick (ADC)

/* --- 2. CONFIGURATION & SETTINGS --- */
#define NUM_MODULES     2       // Two 8x8 matrices chained together
#define TOTAL_ROWS      16      // Total height (0 to 15)
#define TOTAL_COLS      8       // Total width (0 to 7)

// Joystick Sensitivity Thresholds (Range 0 to 4095)
#define JOY_LOW         1000    // Threshold for Left/Up input
#define JOY_HIGH        3500    // Threshold for Right/Down input

// Speeds (in milliseconds)
#define INPUT_DELAY     100     // How fast you can move the dot manually
uint32_t gravity_speed = 980;   // How fast the dot falls automatically

/* --- 3. MAX7219 REGISTER MAP --- */
/* These are the command addresses the LED chip understands. */
#define REG_DIGIT_0     0x01
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_SCAN_LIMIT  0x0B
#define REG_SHUTDOWN    0x0C
#define REG_DISPLAY_TEST 0x0F

/* --- 4. FINITE STATE MACHINE (FSM) DEFINITIONS --- */
/* An Enumeration (enum) gives names to numbers.
 * It defines the valid "States" our game logic can be in. */
typedef enum {
    STATE_INIT,     // Startup: Clear screen, setup variables
    STATE_PLAYING,  // Active: Gravity on, Input enabled
    STATE_STOPPED   // Emergency Stop: LEDs off, System halted
} GameState_t;

/* --- 5. GLOBAL VARIABLES --- */
uint8_t display_buffer[TOTAL_ROWS]; // Memory map of the screen
int8_t cursor_x = 0;                // Current X position (0-7)
int8_t cursor_y = 0;                // Current Y position (0-15)

// State Variable
GameState_t current_state = STATE_INIT; // Start in INIT state

// Timing Variables (For non-blocking delays)
uint32_t last_input_time = 0;       // When did we last move the cursor?
uint32_t last_gravity_time = 0;     // When did the dot last fall?

/* --- 6. FUNCTION PROTOTYPES --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

// Game Logic Functions
void Screen_Init(void);
void MAX7219_Flush(void);
void MAX7219_SetPixel(int8_t x, int8_t y, uint8_t state);
void MAX7219_Clear(void);
uint32_t Read_ADC_Channel(uint32_t channel);

/* --- 7. MAIN FUNCTION --- */
int main(void)
{
  /* --- A. HARDWARE SETUP --- */
  HAL_Init();             // Initialize the STM32 "Brain"
  SystemClock_Config();   // Setup the Heartbeat (Clock)
  MX_GPIO_Init();         // Setup Pins (Buttons, LED CS)
  MX_SPI1_Init();         // Setup SPI (Screen Communication)
  MX_ADC1_Init();         // Setup ADC (Joystick Eyes)

  /* --- B. INFINITE LOOP (The FSM Engine) --- */
  while (1)
  {
      /* --- EMERGENCY STOP CHECK --- */
      /* If Blue Button (PA0) is pressed, force state to STOPPED. */
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          current_state = STATE_STOPPED; // Forced stop.
          HAL_Delay(500); // Simple debounce.
      }

      /* --- FSM SWITCH --- */
      /* The brain checks "What mode am I in?" and runs only that code block. */
      switch (current_state)
      {
          /* --- STATE 1: INITIALIZATION --- */
          case STATE_INIT:
              Screen_Init();          // Wake up MAX7219 chips
              MAX7219_Clear();        // Wipe the buffer
              cursor_x = 3;           // Spawn in middle X
              cursor_y = 0;           // Spawn at Ceiling Y
              last_input_time = HAL_GetTick();   // Reset timers
              last_gravity_time = HAL_GetTick(); // Reset timers

              current_state = STATE_PLAYING; // Transition immediately to Play
              break;

          /* --- STATE 2: PLAYING GAME --- */
          case STATE_PLAYING:
          {
              uint32_t current_time = HAL_GetTick(); // Get current clock time in ms

              // --- 1. Clear Old Cursor Position ---
              // We erase the dot *before* calculating new moves
              MAX7219_SetPixel(cursor_x, cursor_y, 0);

              // --- 2. GRAVITY LOGIC (Downward Pull) ---
              // Check if enough time has passed (e.g., 800ms) to drop the dot
              if (current_time - last_gravity_time > gravity_speed) {
                  // Check Floor Limit: Can we go deeper?
                  if (cursor_y < (TOTAL_ROWS - 1)) {
                      cursor_y++; // Move down
                  }
                  // If we hit bottom (cursor_y == 15), we do nothing (Hit Wall)

                  last_gravity_time = current_time; // Reset gravity timer
              }

              // --- 3. INPUT LOGIC (Joystick & Buttons) ---
              // Check if enough time has passed (e.g., 100ms) to accept input
              if (current_time - last_input_time > INPUT_DELAY) {

                  // Read Joystick Values
                  uint32_t val_x = Read_ADC_Channel(ADC_CHANNEL_1);
                  uint32_t val_y = Read_ADC_Channel(ADC_CHANNEL_2);

                  // X-AXIS MOVEMENT (Left/Right)
                  // We Keep Ring Buffer for X (Pac-man effect)
                  if (val_x < JOY_LOW) {
                      cursor_x--;
                      if(cursor_x < 0) cursor_x = TOTAL_COLS - 1; // Wrap Left -> Right
                  }
                  else if (val_x > JOY_HIGH) {
                      cursor_x++;
                      if(cursor_x >= TOTAL_COLS) cursor_x = 0;    // Wrap Right -> Left
                  }

                  // Y-AXIS MOVEMENT (Manual Up/Down)
                  // We remove Ring Buffer. Implement Walls.
                  if (val_y < JOY_LOW) {
                       // Move Up? Only if not at Ceiling (0)
                       if (cursor_y > 0) {
                           cursor_y--;
                       }
                  }
                  else if (val_y > JOY_HIGH) {
                       // Move Down? Only if not at Floor (15)
                       if (cursor_y < (TOTAL_ROWS - 1)) {
                           cursor_y++;
                       }
                  }

                  // Joystick Button (Reset Game)
                  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                      current_state = STATE_INIT; // Go back to Init State
                  }

                  last_input_time = current_time; // Reset input timer
              }

              // --- 4. Draw New Cursor Position ---
              MAX7219_SetPixel(cursor_x, cursor_y, 1);
              MAX7219_Flush(); // Send to screen

              break;
          }

          /* --- STATE 3: STOPPED (Emergency) --- */
          case STATE_STOPPED:
              MAX7219_Clear(); // Ensure screen is dark
              // We stay here forever until reset, or we could check a restart button
              // For now, the loop just spins safely.
              break;
      }
  }
}

/* --- 8. HELPER FUNCTIONS --- */

/**
 * @brief Reads a specific ADC channel cleanly.
 * @param channel: ADC_CHANNEL_1 (PA1) or ADC_CHANNEL_2 (PA2)
 * @return 0-4095 value (2048 is center)
 */
uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    // Apply Config
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    // Start & Poll
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return 2048; // Return "Center" if read fails
}

/**
 * @brief Maps X/Y to the physical screen buffer.
 * @param x: 0-7
 * @param y: 0-15 (0=Top, 15=Bottom)
 * @param state: 1=ON, 0=OFF
 */
void MAX7219_SetPixel(int8_t x, int8_t y, uint8_t state) {
    if (x < 0 || x >= TOTAL_COLS || y < 0 || y >= TOTAL_ROWS) return;

    uint8_t module_offset;
    int8_t local_y;

    // Logic: Top screen (Module 2) handles rows 0-7
    //        Bottom screen (Module 1) handles rows 8-15
    if (y < 8) {
        module_offset = 8; // Top Module Buffer Index
        local_y = y;
    } else {
        module_offset = 0; // Bottom Module Buffer Index
        local_y = y - 8;
    }

    uint8_t target_index = module_offset + x;
    uint8_t bit_mask = (1 << local_y);

    if (state) display_buffer[target_index] |= bit_mask;
    else       display_buffer[target_index] &= ~bit_mask;
}

/**
 * @brief Sends the entire display buffer to the LED chips via SPI.
 */
void MAX7219_Flush(void) {
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t addr = i + 1; // Registers are 1 to 8

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Chip Select Low (Start)

        // Send to Top Module (Module 2) - Data pushed out first
        HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
        HAL_SPI_Transmit(&hspi1, &display_buffer[8 + i], 1, 10);

        // Send to Bottom Module (Module 1) - Data pushed out last
        HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
        HAL_SPI_Transmit(&hspi1, &display_buffer[i], 1, 10);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Chip Select High (Latch)
    }
}

void MAX7219_Clear(void) {
    memset(display_buffer, 0, sizeof(display_buffer)); // Fill buffer with 0s
    MAX7219_Flush();
}

/**
 * @brief Sends configuration commands to wake up the MAX7219 chips.
 */
void Screen_Init(void) {
    uint8_t commands[][2] = {
        {REG_DISPLAY_TEST, 0x00}, // Test Mode OFF
        {REG_SHUTDOWN, 0x01},     // Normal Operation (Wake Up)
        {REG_SCAN_LIMIT, 0x07},   // Scan all 8 digits
        {REG_DECODE_MODE, 0x00},  // Matrix Mode (No BCD decode)
        {REG_INTENSITY, 0x02}     // Brightness (0x00 to 0x0F)
    };

    for (int c = 0; c < 5; c++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        for(int m=0; m<NUM_MODULES; m++) {
            // Send same config to both modules
            HAL_SPI_Transmit(&hspi1, &commands[c][0], 1, 10);
            HAL_SPI_Transmit(&hspi1, &commands[c][1], 1, 10);
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
    MAX7219_Clear();
}

/* --- 9. HARDWARE INIT (Boilerplate) --- */

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
  // Config handled in Read_ADC_Channel
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

  // PA4: Output for CS
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA3: Input for Joystick Button
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA0: Input for Emergency Stop Blue Button
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA1, PA2: Analog Inputs for Joystick
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
