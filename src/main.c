#include "stm32f0xx.h"    // CMSIS header for STM32F0 (which should be part of CMSIS)
#include "core_cm0.h"     // CMSIS Core header for ARM Cortex-M0
#include "stdio.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include "whitestripes.h"
#include <stdlib.h> // for abs()
#include "tty.h"

// Definitions for game parameters and hardware setup
#define LED_MATRIX_WIDTH 64
#define LED_MATRIX_HEIGHT 32
#define NOTE_DROP_SPEED 5                // Speed of note fall in pixels per update
#define OLED_ADDRESS 0x3C                // SOC1602A OLED I2C address
#define EEPROM_AUDIO_ADDRESS 0x50        // EEPROM address for audio samples
#define EEPROM_HIGH_SCORE_ADDRESS 0x52   // EEPROM address for high score
#define I2C_TIMING 0x00B01A4B            // Timing for 400kHz with 48MHz clock
#define TIMING_WINDOW 5                  // Timing window (in ms) for scoring
#define TARGET_POSITION 0                // Replace with desired target position for the note
#define MAX_MISSES 5                     // Maximum number of missed notes allowed

// Pin definitions for RGB LED matrix
#define B2_PIN (1 << 7)
#define R2_PIN (1 << 5)
#define B1_PIN (1 << 3)
#define R1_PIN (1 << 1)
#define G1_PIN (1 << 0)
#define G2_PIN (1 << 2)
#define B_PIN (1 << 4)
#define D_PIN (1 << 6)
#define OE_PIN (1 << 8)
#define CLK_PIN (1 << 9)
#define C_PIN (1 << 10)
#define A_PIN (1 << 11)
#define LAT_PIN (1 << 12)
#define BUTTON_PIN (1 << 4)
#define BUTTON_PORT GPIOB
#define BIT_BANGING_PORT GPIOB

// Game variables
volatile uint16_t score = 0;
volatile uint16_t high_score = 0;
volatile uint8_t note_positions[LED_MATRIX_WIDTH];  // Array to track note positions
uint8_t oled_data_buffer[16];                       // Buffer for OLED display data
uint8_t audio_data_buffer[128];                     // Buffer for audio data
uint8_t current_note_index = 0;                     // Tracks the index of the current note
uint32_t note_timing[LED_MATRIX_WIDTH];             // Array to track expected timing for each note
volatile uint8_t missed_notes = 0;
volatile uint32_t msTicks = 0;                      // Millisecond tick counter

// SysTick Handler to increment msTicks
void SysTick_Handler(void) {
    msTicks++;
}

// Function to get current tick count in ms
uint32_t GetTick(void) {
    return msTicks;
}

// Function Prototypes
void LED_Matrix_Init(void);
void sendBit(uint8_t red, uint8_t green, uint8_t blue);
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
    SystemInit();                    // CMSIS System Initialization
    SysTick_Config(SystemCoreClock / 1000);
    init_usart5();                  // Initialize USART1 for printf
    initc();
    initb();
    
    // I2C_Init();                      // Initialize I2C for OLED and EEPROM
    
    // LED_Matrix_Init();               // Initialize RGB LED Matrix
    // DAC_Audio_Init();                // Initialize DAC for sound playback

    // high_score = I2C_EEPROM_Read_HighScore();  // Retrieve saved high score

    while (1) {
        LED_Matrix_Update();                 // Update falling notes
        // uint32_t current_time = SysTick->VAL;  // Get current time in ms
        // Detect_Note_Hit(current_time);       // Check for user input and hits
        // Play_Audio_Track();                  // Play background music

        // Display current score on OLED
        // OLED_Display_Score_DMA(score);

        // if (Game_Over()) {
        //     if (score > high_score) {
        //         I2C_EEPROM_Write_HighScore(score);  // Update high score in EEPROM
        //         high_score = score;
        //     }
        //     Display_High_Score();  // Show high score on OLED
        //     Game_Reset();          // Restart the game
        // }
    }
}


// System Clock Configuration
//void SystemClock_Config(void) {
    // Configure system clock based on STM32 model
//}

void initc(void) {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= 0xfffffff0;
    GPIOC->PUPDR &= 0xfffffff0;
}

void initb() {
  RCC->AHBENR |=RCC_AHBENR_GPIOBEN;

  // Set pins PB0-PB4 as outputs
  GPIOB->MODER &= 0x00000000;

  GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0
                    |GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0
                    | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0
                    | GPIO_MODER_MODER12_0);

    GPIOB->ODR = 0x00000084;

}




void init_usart5() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    // PC12 to be routed to USART5_TX
    GPIOC->MODER |= GPIO_MODER_MODER12_1;
    GPIOC->MODER &= ~GPIO_MODER_MODER12_0;
    // PD2 to be routed to USART5_RX
    GPIOD->MODER |= GPIO_MODER_MODER2_1;
    GPIOD->MODER &= ~GPIO_MODER_MODER2_0;
    
    //AFR bits
    GPIOC->AFR[1] |= 0x00020000;
    GPIOD->AFR[0] |= 0x00000200;

    // Activate RCC of USART5
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    // Disable USART
    USART5->CR1 &= ~USART_CR1_UE;
    // USART5 set to 8 bits
    USART5->CR1 &= ~USART_CR1_M0;
    USART5->CR1 &= ~USART_CR1_M1;
    // One stop bit
    USART5->CR2 &= ~USART_CR2_STOP_0;
    USART5->CR2 &= ~USART_CR2_STOP_1;
    //No parity Control
    USART5->CR1 &= ~USART_CR1_PCE;
    // X16 Oversampling
    USART5->CR1 &= ~USART_CR1_OVER8;
    // Baud rate to 115200
    USART5->BRR = 0x1A1;
    // Enable RE and TE
    USART5->CR1 |= USART_CR1_TE;
    USART5->CR1 |= USART_CR1_RE;
    // Enable the USART
    USART5->CR1 |= USART_CR1_UE;
}

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    USART5->CR1 |= USART_CR1_RXNEIE;
    USART5->CR3 |= USART_CR3_DMAR;
    NVIC->ISER[0] |= (1<<29); 

    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off

    DMA2_Channel2->CMAR = (uint32_t)&serfifo; //Sending address
    DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR); //Receiving adress
    DMA2_Channel2->CNDTR = FIFOSIZE; //Number of data to be transferred (CDNTR)
    DMA2_Channel2->CCR &= ~DMA_CCR_DIR; //Direction (DIR) READ FROM PERIPHERAL
    DMA2_Channel2->CCR &= ~DMA_CCR_MSIZE; //Memory Size (MSIZE) and Peripheral Size (PSIZE)
    DMA2_Channel2->CCR &= ~DMA_CCR_PSIZE; //Peripheral Size (PSIZE)
    DMA2_Channel2->CCR &= ~DMA_CCR_HTIE; //Half Completion Disabled (HTIE)
    DMA2_Channel2->CCR &= ~DMA_CCR_TCIE; //Total Completion Disabled (TCIE)
    DMA2_Channel2->CCR |= DMA_CCR_MINC; //Memory Increment (MINC)
    DMA2_Channel2->CCR &= ~DMA_CCR_PINC; //Memory Increment (MINC)
    DMA2_Channel2->CCR |= DMA_CCR_CIRC; //Circular operation (CIRC)
    DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM; //Memory to memory disabled (MINC)
    NVIC_SetPriority(USART3_8_IRQn, 0);

    DMA2_Channel2->CCR |= DMA_CCR_EN; //Enable DMA2

}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    USART_TypeDef *u = USART5;
    // Wait for a newline to complete the buffer.
    while(fifo_newline(&input_fifo) == 0) {
        asm volatile ("wfi"); // wait for an interrupt
        // insert_echo_char(u->RDR);
    }
    
    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_putchar(int c) {
     if(c=='\n'){
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = '\r';
    }

    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    return interrupt_getchar();
}

void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}


// Initialize RGB LED Matrix
// void LED_Matrix_Init(void) {
//     // Enable clock for GPIOB
//     RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//     // Configure bit banging pins as outputs
//     GPIOB->MODER |= (GPIO_MODER_MODER7_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER1_0 |
//                     GPIO_MODER_MODER0_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER6_0 |
//                     GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 |
//                     GPIO_MODER_MODER12_0);

//     // Set high speed
//     GPIOB->OSPEEDR |= 0xFFFFFFFF;
// }

// Send bit to LED matrix
void sendBit(uint8_t red, uint8_t green, uint8_t blue) {
    if (red) {
        GPIOB->BSRR = R1_PIN | R2_PIN ;
    } else {
        GPIOB->BRR = R1_PIN | R2_PIN;
    }

    if (green) {
        GPIOB->BSRR = G1_PIN | G2_PIN;
    } else {
        GPIOB->BRR = G1_PIN | G2_PIN;
    }

    if (blue) {
        GPIOB->BSRR = B1_PIN | B2_PIN;
    } else {
        GPIOB->BRR = B1_PIN | B2_PIN;
    }
    GPIOA->BSRR = CLK_PIN;  // Set CLK high
    GPIOA->BRR = CLK_PIN;   // Set CLK low
}

// Pulse the latch line
void latchData(void) {
    GPIOA->BSRR = LAT_PIN; // Set LAT high
    GPIOA->BRR = LAT_PIN;  // Set LAT low
}

// Update matrix
void updateMatrix(uint8_t *framebuffer, size_t size) {
    GPIOA->BSRR = OE_PIN; // Disable the display during update (OE high)

    for (size_t i = 0; i < size; i++) {
        uint8_t red = framebuffer[i] & 0xFF;
        uint8_t green = (framebuffer[i] >> 8) & 0xFF;
        uint8_t blue = (framebuffer[i] >> 16) & 0xFF;
        sendBit(red, green, blue);
    }

    latchData(); // Latch the data to the matrix
    GPIOA->BRR = OE_PIN;   // Enable the display (OE low)
}

// Rest of the code remains the same as it was provided above

/// Update LED Matrix to display falling notes
void LED_Matrix_Update(void) {
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        note_positions[i] += NOTE_DROP_SPEED;  // Move notes down

        // Assign timing for note drop (example timing logic)
        if (note_positions[i] == 0) {
            note_timing[i] = SysTick->VAL + 1000; // Expect note to hit bottom in 1 second
        }

        // Reset note if it falls off the bottom
        if (note_positions[i] >= LED_MATRIX_HEIGHT) {
            note_positions[i] = 0;
        }
    }
}


/* int __io_putchar(int ch) {
    // Implement this based on your UART configuration, for example:
    ITM_SendChar(ch);
    return ch;
}
*/
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
    uint32_t currentTime = SysTick->VAL; // Get current system time in ms

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
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;  // Enable DAC clock
    DAC->CR |= DAC_CR_EN1;              // Enable DAC channel 1
}

// Play Sound for Note Hit or Miss
void Play_Note_Sound(int hit) {
    if (hit) {
        DAC->DHR8R1 = 0xFF;  // Example max amplitude
    } else {
        DAC->DHR8R1 = 0x80;  // Example lower amplitude
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

    DMA1_Channel2->CMAR = (uint32_t)oled_data_buffer;
    DMA1_Channel2->CPAR = (uint32_t)&I2C2->TXDR;
    DMA1_Channel2->CNDTR = sizeof(oled_data_buffer);
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}
// page 205 & 943


// Start receiving audio data from EEPROM using DMA
void Start_Audio_DMA(void) {
    I2C2->CR2 = I2C_CR2_RD_WRN | (sizeof(audio_data_buffer) << 16) | (EEPROM_AUDIO_ADDRESS << 1) | I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    DMA1_Channel3->CMAR = (uint32_t)audio_data_buffer;
    DMA1_Channel3->CPAR = (uint32_t)&I2C2->RXDR;
    DMA1_Channel3->CNDTR = sizeof(audio_data_buffer);
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

/// Play audio track from received data buffer using DAC
void Play_Audio_Track(void) {
    // Add a correct length for whitestripes_audio_data_len if it's not already defined
    for (unsigned int i = 0; i < whitestripes_audio_data_len && i < sizeof(whitestripes_audio_data); i++) {
        while (!(TIM2->SR & TIM_SR_UIF)); // Wait for timer overflow
        TIM2->SR &= ~TIM_SR_UIF;           // Clear update interrupt flag
        DAC->DHR8R1 = whitestripes_audio_data[i];  // Set DAC output to current sample value
    }
}


// Detect Button Press to Check for Note Hits with Timing-Based Scoring
void Detect_Note_Hit(uint32_t current_time) {
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        if (note_positions[i] >= LED_MATRIX_HEIGHT - 1) { // Note reached bottom
            if (isButtonPressed()) {
                int timing_difference = abs((int)(current_time - note_timing[i]));
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
                missed_notes++;
            }
            note_positions[i] = 0;    // Reset note position
        }
    }
}


// Check if game is over (e.g., time limit or max misses)
int Game_Over(void) {
    printf("Game over, you lose!");
    return missed_notes >= MAX_MISSES; // Placeholder condition
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

// Write high score to EEPROM
void I2C_EEPROM_Write_HighScore(uint16_t score) {
    I2C1->CR2 = (EEPROM_HIGH_SCORE_ADDRESS << 1) | (2 << 16) | I2C_CR2_AUTOEND;
    I2C1->CR2 |= I2C_CR2_START;
    I2C1->TXDR = (score & 0xFF);  // Write low byte
    while (!(I2C1->ISR & I2C_ISR_TXE));
    I2C1->TXDR = (score >> 8);    // Write high byte
}

// Reset game state
/* void Game_Reset(void) {
    score = 0;
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        note_positions[i] = 0;
    }
}
*/
// Check if game is over


// Display high score
void Display_High_Score(void) {
    printf("High Score: %d\n", high_score);  // Replace with OLED update logic
}

