#include "stm32f0xx.h"
#include <stdio.h>//"i2c.h"
#include <math.h>   // for M_PI
#include <stdint.h>

//#include "gpio.h"
//#include "tim.h"
// #include "dac.h"
// #include "dma.h"

// Definitions for game parameters and hardware setup
#define LED_MATRIX_WIDTH 64
#define LED_MATRIX_HEIGHT 32
#define NOTE_DROP_SPEED 5                // Speed of note fall in pixels per update
#define OLED_ADDRESS 0x3C                // SOC1602A OLED I2C address
#define EEPROM_AUDIO_ADDRESS 0x50        // EEPROM address for audio samples
#define EEPROM_HIGH_SCORE_ADDRESS 0x52   // EEPROM address for high score
#define I2C_TIMING 0x00B01A4B            // Timing for 400kHz with 48MHz clock
#define TIMING_WINDOW 5                  // Timing window (in ms) for scoring

// Pin definitions for RGB LED matrix
// Bit Banging Bus Pins
#define A1_PIN (1 << 7)
#define A2_PIN (1 << 5) 
#define A3_PIN (1 << 3)
#define A4_PIN (1 << 1)
#define B1_PIN (1 << 0)
#define B2_PIN (1 << 2)
#define B3_PIN (1 << 4) 
#define B4_PIN (1 << 6)
#define OE_PIN (1 << 8)
#define CLK_PIN (1 << 9)
#define C_PIN (1 << 10)
#define A_PIN (1 << 11)
#define LAT_PIN (1 << 12)
#define BUTTON_PIN (1 << 4)
#define BUTTON_PORT GPIOB
#define BIT_BANGING_PORT GPIOB
#define TARGET_POSITION 0 // Replace with the desired target position for the note

// Game variables
volatile uint16_t score = 0;
volatile uint16_t high_score = 0;
volatile uint8_t note_positions[LED_MATRIX_WIDTH];  // Array to track note positions
uint8_t oled_data_buffer[16];                       // Buffer for OLED display data
uint8_t audio_data_buffer[128];                     // Buffer for audio data
uint8_t current_note_index = 0;                     // Tracks the index of the current note
uint32_t note_timing[LED_MATRIX_WIDTH];             // Array to track expected timing for each note

// Function Prototypes
//void SystemClock_Config(void);
void LED_Matrix_Init(void);
void sendBit(uint8_t bit);
void latchData(void);
void updateMatrix(uint8_t *framebuffer, size_t size);
void LED_Matrix_Update(void);
void initButton(void);
int isButtonPressed(void);
void checkButtonHit(uint8_t notePosition);
void DAC_Audio_Init(void);
void Play_Note_Sound(int hit);
void I2C_Init(void);
void OLED_Display_Score_DMA(uint16_t score);
void Start_Audio_DMA(void);
void Play_Audio_Track(void);
void Detect_Note_Hit(uint32_t current_time);
void Game_Reset(void);
int Game_Over(void);
uint16_t I2C_EEPROM_Read_HighScore(void);
void I2C_EEPROM_Write_HighScore(uint16_t score);
void Display_High_Score(void);

// Main Function
int main(void) {
    HAL_Init();                      // Initialize the HAL library
    internal_clock();//SystemClock_Config();            // Configure system clock
    I2C_Init();                      // Initialize I2C for OLED and EEPROM
    LED_Matrix_Init();               // Initialize RGB LED Matrix
    Button_Input_Init();             // Initialize GPIO buttons
    DAC_Audio_Init();                // Initialize DAC for sound playback

    high_score = I2C_EEPROM_Read_HighScore();  // Retrieve saved high score

    while (1) {
        LED_Matrix_Update();                 // Update falling notes
        uint32_t current_time = HAL_GetTick();  // Get current time in ms
        Detect_Note_Hit(current_time);       // Check for user input and hits
        Play_Audio_Track();                  // Play background music
        
        // Display current score on OLED
        OLED_Display_Score_DMA(score);

        if (Game_Over()) {
            if (score > high_score) {
                I2C_EEPROM_Write_HighScore(score);  // Update high score in EEPROM
                high_score = score;
            }
            Display_High_Score();  // Show high score on OLED
            Game_Reset();          // Restart the game
        }
    }
}

// System Clock Configuration
//void SystemClock_Config(void) {
    // Configure system clock based on STM32 model
//}

// Initialize RGB LED Matrix
void LED_Matrix_Init(void) {
    // Enable clock for GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure bit banging pins as outputs
    GPIOB->MODER |= (GPIO_MODER_MODER7_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER1_0 |
                    GPIO_MODER_MODER0_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER6_0 |
                    GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 |
                    GPIO_MODER_MODER12_0);

    // Set high speed
    GPIOB->OSPEEDR |= 0xFFFFFFFF;
}

void sendBit(uint8_t bit) {
    // Write DATA line based on bit
    if (bit) {
        GPIOB->BSRR = (A1_PIN | A2_PIN | A3_PIN | A4_PIN | B1_PIN | B2_PIN | B3_PIN | B4_PIN);
    } else {
        GPIOB->BSRR = (A1_PIN | A2_PIN | A3_PIN | A4_PIN | B1_PIN | B2_PIN | B3_PIN | B4_PIN);
    }

    // Pulse the clock
    GPIOA->BSRR = CLK_PIN;  // Set CLK high
    GPIOA->BRR = CLK_PIN;   // Set CLK low
}

void latchData(void) {
    // Pulse the latch line
    GPIOA->BSRR = LAT_PIN; // Set LAT high
    GPIOA->BRR = LAT_PIN;  // Set LAT low
}

void updateMatrix(uint8_t *framebuffer, size_t size) {
    // Disable the display during update (OE high)
    GPIOA->BSRR = OE_PIN;

    for (size_t i = 0; i < size; i++) {
        sendBit(framebuffer[i]);
    }

    // Latch the data to the matrix
    latchData();

    // Enable the display (OE low)
    GPIOA->BRR = OE_PIN;
}

// Update LED Matrix to display falling notes
void LED_Matrix_Update(void) {
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        note_positions[i] += NOTE_DROP_SPEED;  // Move notes down

        // Assign timing for note drop (example timing logic)
        if (note_positions[i] == 0) {
            note_timing[i] = HAL_GetTick() + 1000; // Expect note to hit bottom in 1 second
        }

        // Reset note if it falls off the bottom
        if (note_positions[i] >= LED_MATRIX_HEIGHT) {
            note_positions[i] = 0;
        }
    }
}

// Initialize GPIO Pins for Button Inputs
void initButton(void) {
    // Enable clock for GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure BUTTON_PIN as input
    GPIOB->MODER &= ~GPIO_MODER_MODER4_Msk; // Input mode
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;   // Pull-up
}

int isButtonPressed(void) {
    // Check if button is pressed (active low)
    return !(GPIOB->IDR & BUTTON_PIN); // Returns 1 if pressed
}

void checkButtonHit(uint8_t notePosition) {
    static uint32_t lastPressTime = 0;
    uint32_t currentTime = HAL_GetTick(); // Get current system time in ms

    if (isButtonPressed()) {
        // Debounce button: Ensure at least 200ms between presses
        if (currentTime - lastPressTime > 200) {
            lastPressTime = currentTime; // Update last press time

            if (notePosition == TARGET_POSITION) { // Synchronize with note
                printf("Hit!\n");
            } else {
                printf("Miss.\n");
            }
        }
    }
}

// Initialize DAC for Audio Playback
void DAC_Audio_Init(void) {
    // Initialize DAC channels for music playback and note sound effects
}

// Play Sound for Note Hit or Miss
void Play_Note_Sound(int hit) {
    if (hit) {
        // Generate sound for successful hit
    } else {
        // Generate sound for missed note
    }
}

// Initialize I2C peripheral with DMA
void I2C_Init(void) {
    I2C2->TIMINGR = I2C_TIMING;               // Set timing
    I2C2->CR1 = I2C_CR1_PE;                   // Enable I2C peripheral
    I2C2->CR1 |= I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN;  // Enable DMA for TX and RX
}

// Display score on SOC1602A OLED using DMA
void OLED_Display_Score_DMA(uint16_t score) {
    snprintf((char*)oled_data_buffer, sizeof(oled_data_buffer), "Score: %u", score);
    I2C2->CR2 = (OLED_ADDRESS << 1) | (sizeof(oled_data_buffer) << 16) | I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    DMA1_Stream6->M0AR = (uint32_t)oled_data_buffer;
    DMA1_Stream6->PAR = (uint32_t)&I2C2->TXDR;
    DMA1_Stream6->NDTR = sizeof(oled_data_buffer);
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

// Start receiving audio data from EEPROM using DMA
void Start_Audio_DMA(void) {
    I2C2->CR2 = I2C_CR2_RD_WRN | (sizeof(audio_data_buffer) << 16) | (EEPROM_AUDIO_ADDRESS << 1) | I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    DMA1_Stream3->M0AR = (uint32_t)audio_data_buffer;
    DMA1_Stream3->PAR = (uint32_t)&I2C2->RXDR;
    DMA1_Stream3->NDTR = sizeof(audio_data_buffer);
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

// Play audio track from received data buffer using DAC
void Play_Audio_Track(void) {
    for (int i = 0; i < sizeof(audio_data_buffer); i++) {
        DAC->DHR8R1 = audio_data_buffer[i];
        micro_wait(2500);  // Adjust delay for playback rate
    }
}

// Detect Button Press to Check for Note Hits with Timing-Based Scoring
void Detect_Note_Hit(uint32_t current_time) {
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        if (note_positions[i] >= LED_MATRIX_HEIGHT - 1) { // Note reached bottom
            if (GPIO_ReadInputDataBit(GPIO_PORT, GPIO_PIN)) {
                // Check timing window
                int timing_difference = abs((int)(current_time - note_timing[i]));
                
                // Scoring based on timing accuracy
                if (timing_difference <= TIMING_WINDOW) {
                    score += 10;  // Perfect hit
                    Play_Note_Sound(1);  // Hit sound
                } else if (timing_difference <= TIMING_WINDOW * 2) {
                    score += 5;  // Good hit
                    Play_Note_Sound(1);  // Hit sound
                } else {
                    score += 2;  // Okay hit
                    Play_Note_Sound(1);  // Hit sound
                }
            } else {
                Play_Note_Sound(0);   // Missed sound
            }
            note_positions[i] = 0;    // Reset note position
        }
    }
}

// Check if game is over (e.g., time limit or max misses)
int Game_Over(void) {
    // Define game over condition, such as time or misses
    return 0; // Placeholder
}

// Reset Game State
void Game_Reset(void) {
    score = 0;
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        note_positions[i] = 0;
        note_timing[i] = 0;
    }
}

// Read High Score from EEPROM
uint16_t I2C_EEPROM_Read_HighScore(void) {
    uint16_t high_score = 0;
    I2C2->CR2 = (EEPROM_HIGH_SCORE_ADDRESS << 1) | (sizeof(high_score) << 16) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;
    while (!(I2C2->ISR & I2C_ISR_RXNE));
    high_score = I2C2->RXDR;
    return high_score;
}

// Write High Score to EEPROM
void I2C_EEPROM_Write_HighScore(uint16_t score) {
    I2C2->CR2
