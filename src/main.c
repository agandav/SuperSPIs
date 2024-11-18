#include "stm32f0xx.h"    // CMSIS header for STM32F0 (which should be part of CMSIS)
#include "core_cm0.h"     // CMSIS Core header for ARM Cortex-M0
#include "stdio.h"
#include <math.h>   // for M_PI
#include <stdint.h>
// #include "whitestripes.h"
#include <stdlib.h> // for abs()
#include "tty.h"
#include <projectsong.h>

// Definitions for game parameters and hardware setup
#define LED_MATRIX_WIDTH 64
#define LED_MATRIX_HEIGHT 32
#define NOTE_DROP_SPEED 5                // Speed of note fall in pixels per update
#define OLED_ADDRESS 0x3C                // SOC1602A OLED I2C address
#define EEPROM_AUDIO_ADDRESS 0x50        // EEPROM address for audio samples
#define EEPROM_HIGH_SCORE_ADDRESS 0x52   // EEPROM address for high score
#define I2C_TIMING 0x00B01A4B            // Timing for 400kHz with 48MHz clock
#define TIMING_WINDOW 10                  // Timing window (in ms) for scoring
#define TARGET_POSITION 0                // Replace with desired target position for the note
#define MAX_MISSES 5                     // Maximum number of missed notes allowed
#define M_PI 3.1415
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

#define MATRIX_WIDTH 64
#define MATRIX_HEIGHT 32
#define FRAMEBUFFER_BYTES (MATRIX_WIDTH*MATRIX_HEIGHT/2)
uint8_t framebuffer[FRAMEBUFFER_BYTES]; // [0 0 R2 G2 B2 R1 G1 B1]
volatile int row;
volatile int portc;
volatile int block_position;
volatile int color_index;
// color 0 = black, 1 = blue, 2 = green, 3 = cyan, 4 = red, 5 = magenta, 6 = yellow, 7 = white
uint8_t blockColors[4] = {6, 5, 4, 7};



// Game variables
int score = 0;
volatile uint32_t msTicks = 0;                      // Millisecond tick counter
// int g_miss, g_hit;
volatile int missed_notes;


//===========================================================================
// 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
//===========================================================================
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'S', 0x200+'C', 0x200+'O', 0x200+'R', 0x200+'E', + 0x200+' ', 0x200+':', 0x200+' ',
        0x200+'5', 0x200+'0', 0x200+'0', 0x200+'0', + 0x200+'0', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'G', 0x200+'o', 0x200+'o', 0x200+'d', 0x200+' ', + 0x200+'g', 0x200+'a', 0x200+'m',
        0x200+'e', 0x200+'!', 0x200+' ', 0x200+' ', + 0x200+' ', 0x200+' ', 0x200+' ', 0x200+' ',
};

uint32_t volume = 2048;

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 25050
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;


// Function Prototypes
void setup_bb(void);
void init_usart5(void);
void initc(void);
void initb(void);
void LED_Matrix_Update(void);
void sendRGB1(uint8_t red, uint8_t green, uint8_t blue);
void latchData(void);
void updateMatrix(uint8_t *framebuffer, size_t size);
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
void setup_tim7();
void changeRow(uint8_t row);
void USER_input_init();



// Main Function
int main(void) {
    internal_clock();

    missed_notes=0;
    // Initialize other peripherals
    init_usart5();  // Initialize USART5 for printf
    enable_tty_interrupt();
    // These turn off buffering.
    setbuf(stdin,0); 
    setbuf(stdout,0);
    setbuf(stderr,0);

    #define USER_input_subsystem
    #define SPI_subsystem
    #define DAC_subsystem
    #define LED_Matrix_subsystem
    
    //User Input GPIO initialization
    #if defined(USER_input_subsystem)
    USER_input_init();
    init_exti();
    #endif
    // OLED SPI initialization
    #if defined(SPI_subsystem)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
    init_tim2();
    #endif
    //Setup DAC for music reproduction
    #if defined(DAC_subsystem)
    init_wavetable();
    // offset0=89142*12;
    setup_dac();
    init_tim6();
    float f = 460.5;
    set_freq(0,f);
    #endif
    //Initialize GPIO Matrix and TIM7 for switching rows
    #if defined(LED_Matrix_subsystem)
    clearFramebuffer();
    // RedFramebuffer();
    // setPixel(28, 16, 1);
    LED_Matrix_init();
    setup_tim7();
    // for(;;){
    //     for(int i = 0; i<64; i++){
    //     sendRGB1(1,0,0);
    //     sendRGB2(1,0,0);
    //     GPIOB->BSRR = CLK_PIN;  // Set CLK high
    //     GPIOB->BRR = CLK_PIN;   // Set CLK low
    // }    

    // GPIOB->BSRR |= OE_PIN;
    // changeRow(row);
    // row ++;
    // if (row > 16) {
    //     row = 0;
    // }
    // latchData();
    // GPIOB->BRR |= OE_PIN;
    // }
    setup_tim14();
    #endif


    // while(1) {
    //     // printf("Missed notes: %d", missed_notes);
    //     if(missed_notes >= 5) {
    //         printf("into missed notes if");
    //         // Disable all interrupts
    //         NVIC_DisableIRQ(EXTI0_1_IRQn);    // Button interrupts
    //         NVIC_DisableIRQ(EXTI2_3_IRQn);
    //         NVIC_DisableIRQ(EXTI4_15_IRQn);
            
    //         // Disable all timers
    //         TIM2->CR1 &= ~TIM_CR1_CEN;  // SPI timer
    //         TIM6->CR1 &= ~TIM_CR1_CEN;  // DAC timer
    //         TIM7->CR1 &= ~TIM_CR1_CEN;  // LED Matrix timer
    //         TIM14->CR1 &= ~TIM_CR1_CEN; // Other LED timer
            
    //         // Reset game variables
    //         missed_notes = 0;
            
    //         // Re-enable all interrupts
    //         // NVIC_EnableIRQ(EXTI0_1_IRQn);
    //         // NVIC_EnableIRQ(EXTI2_3_IRQn);
    //         // NVIC_EnableIRQ(EXTI4_15_IRQn);
            
    //         // Re-enable all timers
    //         // TIM2->CR1 |= TIM_CR1_CEN;
    //         // TIM6->CR1 |= TIM_CR1_CEN;
    //         // TIM7->CR1 |= TIM_CR1_CEN;
    //         // TIM14->CR1 |= TIM_CR1_CEN;
    //     }
    // }


}

void togglexn(GPIO_TypeDef *port, int n) {
  uint16_t portvalue=port->ODR;
    if (portvalue & (1<<n)){
      port->BRR |= (1<<n);
    }else{
      port->BSRR |= (1<<n);
    }
}
//-------------------------------
// Timer 7 for Bit banging
//-------------------------------
// uint8_t framebuffer[FRAMEBUFFER_BYTES]; // [0 0 R2 G2 B2 R1 G1 B1]
void TIM7_IRQHandler(){
    
    TIM7->SR &= ~TIM_SR_UIF;
    clearFramebuffer();
    // GreenFramebuffer();
    setBlock(block_position, 0, 8, 32, blockColors[color_index]);
    block_position++;
    if(block_position>64){
        block_position = 0;
        color_index++;
        if(color_index>=3){
            color_index = 0;
        }
    }
}


void TIM14_IRQHandler(){
    TIM14->SR &= ~TIM_SR_UIF; 
    uint8_t*pRowData = &framebuffer[(row)*MATRIX_WIDTH];
    for(int i = 0; i<64; i++){
        uint8_t pixelData = pRowData[i];
        int r1 = (pixelData & 0x04) >> 2;
        int g1 = (pixelData & 0x02) >> 1;
        int b1 = (pixelData & 0x01);
        
        int r2 = (pixelData & 0x20) >> 5;
        int g2 = (pixelData & 0x10) >> 4;
        int b2 = (pixelData & 0x08) >> 3;
        sendRGB1(r1,g1,b1);
        sendRGB2(r2,g2,b2);
        GPIOB->BSRR = CLK_PIN;  // Set CLK high
        GPIOB->BRR = CLK_PIN;   // Set CLK low
    }    

    GPIOB->BSRR |= OE_PIN;
    changeRow(row);
    row ++;
    if (row > 16) {
        row = 0;
    }
    latchData();
    GPIOB->BRR |= OE_PIN;

}

void setup_tim14() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 4800-1;
    TIM14->ARR = 10-1;
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= (1<<19);
    TIM14->CR1 |= TIM_CR1_CEN;
}

void setup_tim7() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 4800-1;
    TIM7->ARR = 500-1;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= (1<<18);
    TIM7->CR1 |= TIM_CR1_CEN;
}

void changeRow(uint8_t row){
    if (row & 0x1){
        GPIOB->BSRR = A_PIN;
    }else{
        GPIOB->BRR = A_PIN ;
    }
    if (row & 0x2){
        GPIOB->BSRR = B_PIN;
    }else{
        GPIOB->BRR = B_PIN ;
    }
    if (row & 0x4){
        GPIOB->BSRR = C_PIN;
    }else{
        GPIOB->BRR = C_PIN ;
    }
    if (row & 0x8){
        GPIOB->BSRR = D_PIN;
    }else{
        GPIOB->BRR = D_PIN ;
    }

}

void initFramebufferForX() {
    // Clear the framebuffer
    for (int i = 0; i < FRAMEBUFFER_BYTES; i++) {
        framebuffer[i] = 0;
    }

    // Draw an "X" in the middle
    for (int row = 0; row < MATRIX_HEIGHT; row++) {
        int col1 = row;               // Top-left to bottom-right diagonal
        int col2 = MATRIX_WIDTH - 1 - row; // Top-right to bottom-left diagonal

        // Calculate the byte and bit positions for the pixels
        int byteIndex1 = (row * MATRIX_WIDTH + col1) / 2;
        int byteIndex2 = (row * MATRIX_WIDTH + col2) / 2;

        int isFirstPixel1 = (col1 % 2 == 0);
        int isFirstPixel2 = (col2 % 2 == 0);

        // Set R1, G1, B1 for col1 and col2 to 1 to make the pixel white
        if (isFirstPixel1) {
            framebuffer[byteIndex1] |= 0x07; // R1=1, G1=1, B1=1
        } else {
            framebuffer[byteIndex1] |= 0x70; // R2=1, G2=1, B2=1
        }

        if (isFirstPixel2) {
            framebuffer[byteIndex2] |= 0x07; // R1=1, G1=1, B1=1
        } else {
            framebuffer[byteIndex2] |= 0x70; // R2=1, G2=1, B2=1
        }
    }
}
void setPixel(uint8_t x, uint8_t y, uint8_t color)
{
	if (x > MATRIX_WIDTH || y > MATRIX_HEIGHT)
		return;

	// color 0 = black, 1 = blue, 2 = green, 3 = cyan, 4 = red, 5 = magenta, 6 = yellow, 7 = white
	if (y < MATRIX_HEIGHT/2)
	{
		// top half of matrix, color value is in bits 0-2
		uint16_t addr = y*MATRIX_WIDTH + x;
		framebuffer[addr] &= ~0x7;
		framebuffer[addr] |= (color & 0x7);
	}
	else
	{
		// bottom half of matrix, color value is in bits 3-5
		uint16_t addr = (y-MATRIX_HEIGHT/2)*MATRIX_WIDTH + x;
		framebuffer[addr] &= ~0x38;
		framebuffer[addr] |= ((color & 0x7) << 3);
	}
}


void clearFramebuffer() {
    // Clear the framebuffer
    for (int i = 0; i < FRAMEBUFFER_BYTES; i++) {
        framebuffer[i] = 0;
    }
}

void setBlock(uint8_t startX, uint8_t startY, uint8_t width, uint8_t height, uint8_t color) {
    // Ensure the block does not exceed the matrix boundaries
    if (startX >= MATRIX_WIDTH || startY >= MATRIX_HEIGHT) {
        return;
    }

    // Calculate the effective width and height to avoid overflow
    uint8_t effectiveWidth = (startX + width > MATRIX_WIDTH) ? (MATRIX_WIDTH - startX) : width;
    uint8_t effectiveHeight = (startY + height > MATRIX_HEIGHT) ? (MATRIX_HEIGHT - startY) : height;

    // Loop through each pixel in the block
    for (uint8_t y = startY; y < startY + effectiveHeight; y++) {
        for (uint8_t x = startX; x < startX + effectiveWidth; x++) {
            setPixel(x, y, color);
        }
    }
}

void setBlockColors() {
    // Define the colors for each block
    uint8_t blockColors[4] = {1, 2, 4, 7}; // Blue, Green, Red, White

    for (int block = 0; block < 4; block++) {
        // Calculate the starting and ending rows for the current block
        uint8_t startRow = block * 8;
        uint8_t endRow = startRow + 8;

        // Set the pixels for the current block
        for (uint8_t y = startRow; y < endRow; y++) {
            for (uint8_t x = 0; x < 8; x++) { // Each block spans 8 columns (x = 0 to 7)
                setPixel(x, y, blockColors[block]);
            }
        }
    }
}

void RedFramebuffer() {
    // Clear the framebuffer
    for (int i = 0; i < FRAMEBUFFER_BYTES; i++) {
        framebuffer[i] = 18; //Green
    }
}
void GreenFramebuffer() {
    // Clear the framebuffer
    for (int i = 0; i < FRAMEBUFFER_BYTES; i++) {
        framebuffer[i] = 18; //Green
    }
}

void USER_input_init() {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= 0xfffffff0;
    GPIOC->PUPDR &= 0xfffffff0;

    GPIOC->MODER |= GPIO_MODER_MODER6_0| GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
}

void init_exti() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
  // Setting Port B 0, 2, 3
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2;
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
  SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC;

  //Setting pins to generate interrupt ont eh rising edge
  EXTI->RTSR |= EXTI_RTSR_TR0;
  EXTI->RTSR |= EXTI_RTSR_TR2;
  EXTI->RTSR |= EXTI_RTSR_TR3;
  EXTI->RTSR |= EXTI_RTSR_TR4;

  // Unmask pins
  EXTI->IMR |= EXTI_IMR_IM0;
  EXTI->IMR |= EXTI_IMR_IM2;
  EXTI->IMR |= EXTI_IMR_IM3;
  EXTI->IMR |= EXTI_IMR_IM4;
  
  // Enable interrupts
  NVIC->ISER[0] |= (1<<5);
  NVIC->ISER[0] |= (1<<6);
  NVIC->ISER[0] |= (1<<7);
}

void EXTI0_1_IRQHandler(){
    EXTI->PR = EXTI_PR_PR0;

    if (block_position >= 55 && color_index==0){
        score = score + 50;
        printf("Hit!\n");
        // g_hit = 1;
        // g_miss = 0;
    } else{
        missed_notes++;
        printf("Missed!\n");
        printf("%d\n", missed_notes);    
        // g_hit = 0;
        // g_miss = 1;
    }
    
    togglexn(GPIOC, 6);
}

void EXTI2_3_IRQHandler(){
    if (EXTI->PR & EXTI_PR_PR2) {
        togglexn(GPIOC, 7);      // Toggle pin PC7

    if (block_position >= 55 && color_index==1){
        score = score + 50;
        printf("Hit!\n");    
        // g_hit = 1;
        // g_miss = 0;
    } else{
        missed_notes++;
        printf("Missed!\n");
        printf("%d\n", missed_notes);  
        // g_hit = 0;
        // g_miss = 1;  
    }

        EXTI->PR = EXTI_PR_PR2;  // Clear the interrupt pending flag for EXTI line 2
    }

    // Check if the interrupt was triggered by EXTI line 3
    if (EXTI->PR & EXTI_PR_PR3) {
        togglexn(GPIOC, 8); // Toggle pin PC8
        if (block_position >= 55 && color_index==2){
            score = score + 50;
            printf("Hit!\n");
            // g_hit = 1;
            // g_miss = 0;      
        } else{
            missed_notes++;
            printf("Missed!\n"); 
            printf("%d\n", missed_notes);
            // g_hit = 0;
            // g_miss = 1;   
        }
        
        EXTI->PR = EXTI_PR_PR3;  // Clear the interrupt pending flag for EXTI line 3
    }
}

void EXTI4_15_IRQHandler(){
    EXTI->PR = EXTI_PR_PR4;

    if (block_position >= 55 && color_index==3){
                score = score + 50;
                printf("Hit!\n");    
                // g_hit = 1;
                // g_miss=0;
            } else{
                missed_notes++;
                printf("Missed!\n");
            printf("%d\n", missed_notes);
                // g_hit = 0;
                // g_miss = 1;    
            }
            
    togglexn(GPIOC, 9);
}

void LED_Matrix_init() {
  RCC->AHBENR |=RCC_AHBENR_GPIOBEN;

  // Set pins PB0-PB4 as outputs
    GPIOB->MODER &= 0x00000000;

    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0
                    |GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0
                    | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0
                    | GPIO_MODER_MODER12_0);
    GPIOB->OSPEEDR |= 0xFFFFFFFF;
    GPIOB->BRR = A_PIN | B_PIN | C_PIN | D_PIN ;
    GPIOB->BRR = R1_PIN | R2_PIN | B1_PIN | B2_PIN | G1_PIN | G2_PIN;
    GPIOB->BRR = CLK_PIN;
    GPIOB->BRR = LAT_PIN;
    GPIOB->BRR = OE_PIN;
    row = 0;
}


//===========================================================================
// UART and Command Shell
//===========================================================================
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

// TODO Copy the content for the USART5 ISR here
void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

//===========================================================================
// BIT banging LED Matrix
//===========================================================================
// Send bit to LED matrix
void sendRGB1(uint8_t red, uint8_t green, uint8_t blue) {
    if (red==1) {
        GPIOB->BSRR = R1_PIN;
    } else {
        GPIOB->BRR = R1_PIN;
    }

    if (green==1) {
        GPIOB->BSRR = G1_PIN;
    } else {
        GPIOB->BRR = G1_PIN;
    }

    if (blue==1) {
        GPIOB->BSRR = B1_PIN;
    } else {
        GPIOB->BRR = B1_PIN;
    }

}

void sendRGB2(uint8_t red, uint8_t green, uint8_t blue) {
    if (red==1) {
        GPIOB->BSRR = R2_PIN ;
    } else {
        GPIOB->BRR = R2_PIN;
    }

    if (green==1) {
        GPIOB->BSRR = G2_PIN;
    } else {
        GPIOB->BRR =  G2_PIN;
    }

    if (blue==1) {
        GPIOB->BSRR = B2_PIN;
    } else {
        GPIOB->BRR =  B2_PIN;
    }
    // GPIOB->BSRR = CLK_PIN;  // Set CLK high
    // GPIOB->BRR = CLK_PIN;   // Set CLK low
}

// Pulse the latch line
void latchData(void) {
    GPIOB->BSRR = LAT_PIN; // Set LAT high
    GPIOB->BRR = LAT_PIN;  // Set LAT low
}

//===========================================================================
// SPI OLED Display
//===========================================================================
void init_spi1() {

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~GPIO_MODER_MODER15 & ~GPIO_MODER_MODER5 & ~GPIO_MODER_MODER7 ;
    GPIOA->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 ;

    GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH7;

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_3; 
    SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;
}
void spi_cmd(unsigned int data) {
    while((SPI1->SR & SPI_SR_TXE) == 0); // wait for the transmit buffer to be empty
    SPI1->DR = data; 
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);   
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
    
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0'){
        spi_data(*string);
        string++;
    }    

}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    while(*string != '\0'){
        spi_data(*string);
        string++;
    }
}

void spi1_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMAEN; //Clock enable

    DMA1_Channel3->CCR &= ~0X00000001; // Disable channel
    DMA1_Channel3->CMAR = (uint32_t)&display; //Sending address
    DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR); //Receiving adress
    DMA1_Channel3->CNDTR = 0x00000022; //Number of data to be transferred (CDNTR)
    DMA1_Channel3->CCR |= 0x00000080; //Memory Increment (MINC)
    DMA1_Channel3->CCR |= 0x00000010; //Direction (DIR)
    DMA1_Channel3->CCR |= 0x00000500; //Memory Size (MSIZE) and Peripheral Size (PSIZE)
    DMA1_Channel3->CCR |= 0x00000020; //Circular operation (CIR)
}

void spi1_enable_dma(void) {
    SPI1->CR2 |= SPI_CR2_TXEIE;
    DMA1_Channel3->CCR |= 0X00000001; // Enable channel
}

void updateScore(uint32_t internal_score) {
    // Indices for the score section in the display array
    const int scoreStartIndex = 9;
    const int scoreEndIndex = 13;

    // Ensure score fits within the range (5 digits max)
    if (internal_score > 99999) {
        internal_score = 99999; // Clamp the score to the maximum displayable value
    }

    // Fill score digits into the display array
    for (int i = scoreEndIndex; i >= scoreStartIndex; i--) {
        display[i] = 0x200 + ('0' + (internal_score % 10)); // Extract the last digit and convert to display format
        internal_score /= 10;
    }

    // Fill leading spaces if the score has fewer than 5 digits
    for (int i = scoreStartIndex; i <= scoreEndIndex && internal_score == 0; i++) {
        if (display[i] == 0x200) {
            display[i] = 0x200 + ' ';
        }
    }
}

void updateDisplay(uint32_t internal_score, int hit, int miss) {
    // Indices for the score section in the display array
    const int scoreStartIndex = 9;
    const int scoreEndIndex = 13;

    // Ensure score fits within the range (5 digits max)
    if (internal_score > 99999) {
        internal_score = 99999; // Clamp the score to the maximum displayable value
    }

    // Fill score digits into the display array
    for (int i = scoreEndIndex; i >= scoreStartIndex; i--) {
        display[i] = 0x200 + ('0' + (internal_score % 10)); // Extract the last digit and convert to display format
        internal_score /= 10;
    }

    // Fill leading spaces if the score has fewer than 5 digits
    for (int i = scoreStartIndex; i <= scoreEndIndex && internal_score == 0; i++) {
        if (display[i] == 0x200) {
            display[i] = 0x200 + ' ';
        }
    }

    // Modify the second line message based on hit or miss flags
    const int line2StartIndex = 17; // Start index for line 2 in the display array
    if (hit) {
        display[line2StartIndex + 0] = 0x200 + 'H';
        display[line2StartIndex + 1] = 0x200 + 'I';
        display[line2StartIndex + 2] = 0x200 + 'T';
        display[line2StartIndex + 3] = 0x200 + ' ';
        display[line2StartIndex + 4] = 0x200 + ' ';
        display[line2StartIndex + 5] = 0x200 + ' ';
        display[line2StartIndex + 6] = 0x200 + ' ';
        display[line2StartIndex + 7] = 0x200 + ' ';
    } else if (miss) {
        display[line2StartIndex + 0] = 0x200 + 'M';
        display[line2StartIndex + 1] = 0x200 + 'I';
        display[line2StartIndex + 2] = 0x200 + 'S';
        display[line2StartIndex + 3] = 0x200 + 'S';
        display[line2StartIndex + 4] = 0x200 + ' ';
        display[line2StartIndex + 5] = 0x200 + ' ';
        display[line2StartIndex + 6] = 0x200 + ' ';
        display[line2StartIndex + 7] = 0x200 + ' ';
    } else {
        // Default "Good game" message
        display[line2StartIndex + 0] = 0x200 + 'G';
        display[line2StartIndex + 1] = 0x200 + 'o';
        display[line2StartIndex + 2] = 0x200 + 'o';
        display[line2StartIndex + 3] = 0x200 + 'd';
        display[line2StartIndex + 4] = 0x200 + ' ';
        display[line2StartIndex + 5] = 0x200 + 'g';
        display[line2StartIndex + 6] = 0x200 + 'a';
        display[line2StartIndex + 7] = 0x200 + 'm';
        display[line2StartIndex + 8] = 0x200 + 'e';
        display[line2StartIndex + 9] = 0x200 + '!';
    }
}


//============================================================================
// Timer 2 ISR
//============================================================================
void TIM2_IRQHandler(void){
    TIM2->SR &= ~TIM_SR_UIF;
    updateScore(score);
    // updateDisplay(score, g_miss, g_hit);
    // printf("Score: %d\n", score);
}

void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 4800-1;
    TIM2->ARR = 10000-1;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= (1<<15);
    TIM2->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// Setting up the DAC for music reproduction
//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++)
         wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x00000300; //PA4 to analog mode

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    DAC->CR &= ~DAC_CR_TSEL1; // Select TRGO to TIM6
    DAC->CR |= DAC_CR_TEN1; // Trigger Enable
    DAC->CR |= DAC_CR_EN1; // DAC Enable   

}

//============================================================================
// Timer 6 ISR
//============================================================================
// void TIM6_DAC_IRQHandler(void){
//     TIM6->SR &= ~TIM_SR_UIF;
//     offset0 += step0;
//     offset1 += step1;
//     if (offset0 >= (N<<16)){
//         offset0 = offset0 - (N<<16);
//     }
//     if (offset1 >= (N<<16)){
//         offset1 = offset1 - (N<<16);
//     }

//     int samp = wavetable[offset0>>16] + wavetable[offset1>>16];
//     samp = samp*volume;
//     samp = (samp>>17);
//     samp += 2048;
//     DAC->DHR12R1 = samp;
// }


void TIM6_DAC_IRQHandler(void){// Implemented for music
     TIM6->SR &= ~TIM_SR_UIF; // Clear interrupt flag

    offset0++;
    if (offset0 >= 209273) {
        offset0 = 0; // Loop audio data
    }

    int samp = projectsong_audio_data[offset0]; 
    samp = (samp * volume);  // Apply volume scaling
    samp = (samp * 4095) / 255; // Scale 8-bit to 12-bit
    DAC->DHR12R1 = samp; // Output to DAC
}

void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48-1;
    TIM6->ARR = (1000000/RATE)-1;
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = (1<<17);
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
}