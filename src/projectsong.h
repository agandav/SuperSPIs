#ifndef PROJECTSONG_H
#define PROJECTSONG_H

extern const unsigned int projectsong_audio_data_len;
extern const unsigned char projectsong_audio_data[];

// Declare the beat points and their count
extern const float beat_points[];
extern const unsigned int num_beat_points;

#endif // PROJECTSONG_H



// // What to implement in main file:

// #define BLOCK_DROP_DELAY_MS 50  // A small delay to make sure the block falls slightly before the exact beat
// static uint8_t current_beat_index = 0; // Tracks the current beat in the array

// void syncFallingBlocks(uint32_t current_time_ms) {
//     // Check if it's time to drop the block based on the beat points
//     if (current_beat_index < NUM_BEAT_POINTS &&
//         current_time_ms >= (uint32_t)(beat_points[current_beat_index] * 1000) - BLOCK_DROP_DELAY_MS) {
        
//         // Action at each beat point (e.g., drop a block)
//         dropBlock();  // Function that initiates the block falling action
        
//         // Move to the next beat point
//         current_beat_index++;
//     }
// }
//         //Example of main loop
//             int main(void) {
//     SystemInit();                    // CMSIS System Initialization
//     SysTick_Config(SystemCoreClock / 1000);  // 1ms Systick for timing

//     // Initialize other peripherals
//     init_usart5();  // Initialize USART5 for printf

//     // GPIO Matrix and row switch initialization
//     #if defined(LED_Matrix_Subsytem)
//     LED_Matrix_init();
//     setup_tim7();
//     #endif

//     // User Input GPIO initialization
//     #if defined(USER_input_subsystem)
//     USER_input_init();
//     init_exti();
//     #endif

//     // SPI and OLED initialization
//     #if defined(SPI_subsystem)
//     init_spi1();
//     spi1_init_oled();
//     spi1_setup_dma();
//     spi1_enable_dma();
//     setup_tim14();
//     #endif

//     // Setup ADC for Volume controller
//     #if defined(ADC_subsystem)
//     setup_adc();
//     init_tim2();
//     #endif

//     // Setup DAC for music reproduction
//     #if defined(DAC_subsystem)
//     init_wavetable();
//     setup_dac();
//     init_tim6();
//     float f = 460.5;
//     set_freq(0, f);
//     #endif

//     // Main game loop
//     unsigned int current_beat_index = 0;

    // while (1) {
    //     uint32_t current_time_ms = GetTick(); // Get current time in milliseconds

    //     // Synchronize block falling with beats
    //     if (current_beat_index < num_beats && 
    //         current_time_ms >= (unsigned int)(beat_timings[current_beat_index] * 1000)) {

    //         updateFallingBlocks();  // Trigger a block fall when a beat is reached
    //         current_beat_index++;   // Move to the next beat
    //     }

    //     // Other game logic like updating the display, detecting note hits, etc.
    //     LED_Matrix_Update();
    //     Detect_Note_Hit(current_time_ms);
    //     Play_Audio_Track();
    //     OLED_Display_Score_DMA(score);

    //     if (Game_Over()) {
    //         if (score > high_score) {
    //             I2C_EEPROM_Write_HighScore(score);
    //             high_score = score;
    //         }
    //         Display_High_Score();
    //         Game_Reset();
    //         current_beat_index = 0;  // Reset beat index when game resets
    //     }
    // }

// }


//             // I should have a block falling function but if not:
//                 void dropBlock(void) {
//                     static uint8_t current_x_position = 0;

//                     setPixel(current_x_position, 0, 7);  // Place a white block at the top row (color = 7)
//                     updateFallingBlocks();  // Move all blocks down

//                     // Move the x position to the next column, wrap if necessary
//                     current_x_position = (current_x_position + 1) % LED_MATRIX_WIDTH;
//                 }
