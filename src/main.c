#include "stm32f0xx.h"    // CMSIS header for STM32F0 (which should be part of CMSIS)
#include "core_cm0.h"     // CMSIS Core header for ARM Cortex-M0
#include "stdio.h"
#include <math.h>   // for M_PI
#include <stdint.h>
// #include "whitestripes.h"
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

#define MATRIX_WIDTH 64
#define MATRIX_HEIGHT 32
#define FRAMEBUFFER_BYTES (MATRIX_WIDTH*MATRIX_HEIGHT/2)
uint8_t framebuffer[FRAMEBUFFER_BYTES]; // [0 0 R2 G2 B2 R1 B1 G1]
int row;
int portc;

// Game variables
uint8_t score = 10;
volatile uint16_t high_score = 0;
volatile uint8_t note_positions[LED_MATRIX_WIDTH];  // Array to track note positions
uint8_t oled_data_buffer[16];                       // Buffer for OLED display data
uint8_t audio_data_buffer[128];                     // Buffer for audio data
uint8_t current_note_index = 0;                     // Tracks the index of the current note
uint32_t note_timing[LED_MATRIX_WIDTH];             // Array to track expected timing for each note
volatile uint8_t missed_notes = 0;
volatile uint32_t msTicks = 0;                      // Millisecond tick counter

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

//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;
uint32_t volume = 2048;

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

// SysTick Handler to increment msTicks
void SysTick_Handler(void) {
    msTicks++;
}

// Function to get current tick count in ms
uint32_t GetTick(void) {
    return msTicks;
}

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
    SystemInit();                    // CMSIS System Initialization
    SysTick_Config(SystemCoreClock / 1000);  // 1ms Systick for timing

    // Initialize other peripherals
    init_usart5();  // Initialize USART5 for printf
    // I2C_Init();  // Initialize I2C for EEPROM/OLED

    //Initialize GPIO Matrix and TIM7 for switching rows
    // #define LED_Matrix_Subsytem
    #if defined(LED_Matrix_Subsytem)
    LED_Matrix_init();
    setup_tim7();
    #endif
    //User Input GPIO initialization
    #define USER_input_subsystem
    #if defined(USER_input_subsystem)
    USER_input_init();
    init_exti();
    #endif
    // OLED SPI initialization
    #define SPI_subsystem
    #if defined(SPI_subsystem)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
    setup_tim14();
    #endif
    //Setup ADC for Volume controller
    // #define ADC_subsystem
    #if defined(ADC_subsystem)
    setup_adc();
    init_tim2();
    #endif
    //Setup DAC for music reproduction
    // #define DAC_subsystem
    #if defined(DAC_subsystem)
    init_wavetable();
    // offset0=89142*12;
    setup_dac();
    init_tim6();
    float f = 460.5;
    set_freq(0,f);
    #endif

    // for(;;) {
    //     int portc = GPIOC->IDR;
    //     if (portc&1){
    //     togglexn(GPIOC, 6);
    //     }
    //     // nano_wait(500000000);
    // }
    // Main loop
    // while (1) {
        // Bit-banging LED matrix update
        // LED_Matrix_Update();             // Update falling notes
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

void TIM7_IRQHandler(){
  TIM7->SR &= ~TIM_SR_UIF;
  row ++;
  if (row>32) row = 0;
    for(int i = 0; i<64; i++){
        sendRGB1(0,1,1);
        sendRGB2(0,0,1);
    }    
    GPIOB->BSRR |= OE_PIN;
    changeRow(row);
    latchData();
    GPIOB->BRR |= OE_PIN;
}


void setup_tim7() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 480-1;
    TIM7->ARR = 10-1;
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
    score = score + 20;
    togglexn(GPIOC, 6);
}

void EXTI2_3_IRQHandler(){
    if (EXTI->PR & EXTI_PR_PR2) {
        togglexn(GPIOC, 7);      // Toggle pin PC7
        score = score + 30;
        EXTI->PR = EXTI_PR_PR2;  // Clear the interrupt pending flag for EXTI line 2
    }

    // Check if the interrupt was triggered by EXTI line 3
    if (EXTI->PR & EXTI_PR_PR3) {
        togglexn(GPIOC, 8); // Toggle pin PC8
        score = score + 40;      
        EXTI->PR = EXTI_PR_PR3;  // Clear the interrupt pending flag for EXTI line 3
    }
}

void EXTI4_15_IRQHandler(){
    EXTI->PR = EXTI_PR_PR4;
    score = score + 50;      
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
    GPIOB->BRR = CLK_PIN;
    GPIOB->BRR = LAT_PIN;
    GPIOB->BRR = OE_PIN;
    row = 0;
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
    GPIOB->BSRR = CLK_PIN;  // Set CLK high
    GPIOB->BRR = CLK_PIN;   // Set CLK low
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
    GPIOB->BSRR = CLK_PIN;  // Set CLK high
    GPIOB->BRR = CLK_PIN;   // Set CLK low
}

// Pulse the latch line
void latchData(void) {
    GPIOB->BSRR = LAT_PIN; // Set LAT high
    GPIOB->BRR = LAT_PIN;  // Set LAT low
}

// Update LED Matrix to display falling notes
void LED_Matrix_Update(void) {
    static uint8_t current_row = 0;

    GPIOA->BSRR = OE_PIN; // Disable all LEDs
    GPIOA->BRR = LAT_PIN | CLK_PIN; // Ensure LAT and CLK are low

    // Update each row of the matrix
    for (uint8_t col = 0; col < LED_MATRIX_WIDTH; col++) {
        uint8_t red = (note_positions[col] & 0x01) ? 1 : 0;
        uint8_t green = (note_positions[col] & 0x02) ? 1 : 0;
        uint8_t blue = (note_positions[col] & 0x04) ? 1 : 0;
        sendRGB1(red, green, blue);
    }

    latchData();

    // Set row selection lines
    GPIOB->BSRR = ((current_row & 0x01) ? A_PIN : 0) | ((current_row & 0x02) ? B_PIN : 0) | ((current_row & 0x04) ? C_PIN : 0);

    GPIOA->BRR = OE_PIN; // Re-enable LEDs

    // Increment row
    current_row = (current_row + 1) % (LED_MATRIX_HEIGHT / 2);
    // Move falling notes
    for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
        note_positions[i] += NOTE_DROP_SPEED;
        if (note_positions[i] >= LED_MATRIX_HEIGHT) {
            note_positions[i] = 0;
        }
    }
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

void updateScore(uint32_t score) {
    // Indices for the score section in the display array
    const int scoreStartIndex = 9;
    const int scoreEndIndex = 13;

    // Ensure score fits within the range (5 digits max)
    if (score > 99999) {
        score = 99999; // Clamp the score to the maximum displayable value
    }

    // Fill score digits into the display array
    for (int i = scoreEndIndex; i >= scoreStartIndex; i--) {
        display[i] = 0x200 + ('0' + (score % 10)); // Extract the last digit and convert to display format
        score /= 10;
    }

    // Fill leading spaces if the score has fewer than 5 digits
    for (int i = scoreStartIndex; i <= scoreEndIndex && score == 0; i++) {
        if (display[i] == 0x200) {
            display[i] = 0x200 + ' ';
        }
    }
}

//-------------------------------
// Timer 14 ISR goes here
//-------------------------------
void TIM14_IRQHandler(){
  TIM14->SR &= ~TIM_SR_UIF;
  updateScore(score);
//   score ++;
}

void setup_tim14() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 12000-1;
    TIM14->ARR = 1000-1;
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= (1<<19);
    TIM14->CR1 |= TIM_CR1_CEN;
}



//=============================================================================
// Analog-to-digital conversion for a volume level.
//=============================================================================
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x0000000C; //PA1 to analog mode

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Clk enable
    RCC->CR2 |= RCC_CR2_HSI14ON; //Clk turned on
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {
        //Wait for HSI14 oscillator to be ready
    }
    // ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; //Clk selection

    ADC1->CR |= ADC_CR_ADEN; //ADC enable
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
    // Wait for the ADC to be ready    
    }
    ADC1->CHSELR |= 0x00000002; // Channel Selection
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
    // Wait for the ADC to be ready
    }
}

//============================================================================
// Timer 2 ISR
//============================================================================
void TIM2_IRQHandler(void){
    TIM2->SR &= ~TIM_SR_UIF;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
    // Wait for End of conversion (EOC)
    }
    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    volume = bcsum / BCSIZE;
}

void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 4800-1;
    TIM2->ARR = 1000-1;
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
void TIM6_DAC_IRQHandler(void){
    TIM6->SR &= ~TIM_SR_UIF;
    offset0 += step0;
    offset1 += step1;
    if (offset0 >= (N<<16)){
        offset0 = offset0 - (N<<16);
    }
    if (offset1 >= (N<<16)){
        offset1 = offset1 - (N<<16);
    }

    int samp = wavetable[offset0>>16] + wavetable[offset1>>16];
    samp = samp*volume;
    samp = (samp>>17);
    samp += 2048;
    DAC->DHR12R1 = samp;
}


// void TIM6_DAC_IRQHandler(void){// Implemented for music
//     TIM6->SR &= ~TIM_SR_UIF;
//     offset0 ++;

//     int samp = myfile_audio_data[offset0];
//     samp = samp*volume;
//     samp = (samp>>17);
//     samp += 2048;
//     DAC->DHR12R1 = samp;
// }

void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48-1;
    TIM6->ARR = (1000000/RATE)-1;
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = (1<<17);
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
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
// void Play_Audio_Track(void) {
//     // Add a correct length for whitestripes_audio_data_len if it's not already defined
//     for (unsigned int i = 0; i < whitestripes_audio_data_len && i < sizeof(whitestripes_audio_data); i++) {
//         while (!(TIM2->SR & TIM_SR_UIF)); // Wait for timer overflow
//         TIM2->SR &= ~TIM_SR_UIF;           // Clear update interrupt flag
//         DAC->DHR8R1 = whitestripes_audio_data[i];  // Set DAC output to current sample value
//     }
// }


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

