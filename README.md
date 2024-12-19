This project is a rhythm-based game where the player must match their button inputs (connected to GPIO) with falling notes displayed on an RGB LED matrix. The notes are synchronized to a background music track. Audio cues, generated via the DAC, will play to indicate whether the user has successfully hit or missed a note. The game will also track and store the player's score using an I2C-based memory system.

Objectives:
- Implement an RGB LED matrix connected via bit-banging protocols to show falling notes synchronized with background music using timers.
- Capture user inputs through GPIO-connected buttons to match with the falling notes.
- Generate and play music through the DAC, and provide audio feedback for hits or misses.
- Store and display the user’s score using SOC1602A OLED display and manage audio file access through the I2C protocol.

Main Features
- 64x32 RGB LED Matrix - 4mm pitch: ID 2278
- LM324
- TRRS Jack
- Speaker
- SOC1602A OLED
- Pushbuttons
- 24AA32AF I2C EEPROM
- Buzzer

Role of Code in Project Implementation:
- Initializing bit-banging communication with the RGB LED matrix; handling all graphical rendering, menu navigation, and song selection
- Monitoring the GPIO pins to detect button presses (user input), and checking if the button press is synchronized with falling notes on the screen, registering hit or miss
- Handling loading and playing the music track, ensuring it is synchronized with the falling notes on the screen
- Providing audio feedback when the player hits or misses a note
- Calculating scores based on how close the player’s input is to the correct timing
- Writing the player’s score to the EEPROM after each game and displaying the high scores at the end of each round

Group Members: David Mendoza (Hardware setup, Timer Functions, & Display), Sai Gandavarapu (DAC audio & GPIO implementation), Tanvi Dhawade (I2C implementation), Suhani Mathur (Video Submission)
