#ifndef PROJECTSONG_H
#define PROJECTSONG_H

// Existing declarations for audio data
extern const unsigned int projectsong_audio_data_len;
extern const unsigned char projectsong_audio_data[];

// Declare the beat points and their count
#define NUM_BEAT_POINTS 20
extern const float beat_points[NUM_BEAT_POINTS];

#endif // PROJECTSONG_H

// what to add in main file

// 1. Add this after the function setBlock()
// Function to move the block down when a beat point is reached
    // void updateBlockPosition() {
    //     if (block_position >= LED_MATRIX_WIDTH) {
    //         block_position = 0;  // Reset the position once it reaches the bottom
    //     } else {
    //         block_position++;  // Move the block down
    //     }

    //     // Optional: Update the color or any other parameter if needed
    //     color_index = (color_index + 1) % 4;  // Example for cycling colors
    //     setBlock(block_position, 0, 8, 32, blockColors[color_index]);
    // }

// 2. Add this in the main while loop before the MAX_MISSES part
    //  if (current_beat_index < NUM_BEAT_POINTS) {
    //         float current_time_seconds = msTicks / 1000.0;  // Convert msTicks to seconds

    //         if (current_time_seconds >= beat_points[current_beat_index]) {
    //             updateBlockPosition();  // Move the block down

    //             // Move to the next beat point
    //             current_beat_index++;
    //         }

// 3. Add this after: volatile uint32_t msTicks = 0;  // Global millisecond tick counter
    // unsigned int current_beat_index = 0;  // Index to keep track of the current beat

// Also just make sure that new function is defined yadayadaya
