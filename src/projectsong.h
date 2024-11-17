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



// Dude i fogot we said we would have audio feedback for hit and misses

    //update these if needed:
        //   void init_tim6(void) {
         // RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable Timer 6 clock
        // TIM6->PSC = 48 - 1;                 // Set prescaler to divide system clock to 1 MHz
        // TIM6->ARR = 1000 - 1;               // Set ARR to generate an interrupt at 1 kHz
        // TIM6->DIER |= TIM_DIER_UIE;         // Enable update interrupt
        // NVIC->ISER[0] = (1 << 17);          // Enable Timer 6 interrupt in NVIC
        // TIM6->CR1 |= TIM_CR1_CEN;           // Enable Timer 6
        // }

        // void TIM6_DAC_IRQHandler(void) {
        // TIM6->SR &= ~TIM_SR_UIF; // Clear the interrupt flag

        // static int phase = 0;
        // static int step = 0;

        // if (step == 0) {
        //     return;  // No tone is currently playing
        // }

        // phase += step;
        // if (phase >= 1000) {
        //     phase -= 1000; // Wrap phase around for sine wave calculation
        // }

        // // Simple sine wave approximation
        // int amplitude = 2048 + (int)(2048 * sin(2 * M_PI * phase / 1000));
        // DAC->DHR12R1 = amplitude; // Output to DAC
        // }


     //   hit or miss sounds
        //     void play_hit_sound(void) {
        //     set_sound_frequency(880);  // Set a high frequency for hits
        //     delay(100);                // Play the sound for 100 ms
        //     stop_sound();              // Stop the sound after duration
        // }

        // void play_miss_sound(void) {
        //     set_sound_frequency(440);  // Set a lower frequency for misses
        //     delay(100);                // Play the sound for 100 ms
        //     stop_sound();              // Stop the sound after duration
        // }

        // void set_sound_frequency(int frequency) {
        //     step = (frequency * 1000 / 1000) * (1 << 16);
        // }

        // void stop_sound(void) {
        //     step = 0; // Stop generating any sound
        // }


          //Modify detect note hit
              //void Detect_Note_Hit(uint32_t current_time) {
              //     for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
              //     if (framebuffer[i] >= LED_MATRIX_HEIGHT - 1) { // Note reached bottom
              //         if ((BUTTON_PORT->IDR & BUTTON_PIN) == 0) {  // Button pressed (active low)
              //             int timing_difference = abs((int)(current_time - note_timing[i]));
              //             if (timing_difference <= TIMING_WINDOW) {
              //                 score += 10;  // Perfect hit
              //                 play_hit_sound();  // Play hit sound
              //             } else if (timing_difference <= TIMING_WINDOW * 2) {
              //                 score += 5;  // Good hit
              //                 play_hit_sound();  // Play hit sound
              //             } else {
              //                 score += 2;  // Okay hit
              //                 play_hit_sound();  // Play hit sound
              //             }
              //         } else {
              //             play_miss_sound();   // Play miss sound
              //             missed_notes++;
              //         }
              //         framebuffer[i] = 0;    // Reset note position
              //     }
              // }
//}

