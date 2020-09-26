# Whack-a-Mole
Whack a Mole game developed using ARM assembly language on the Cortex-M3 architecture. It demonstrates low-level representation and manipulation of data.
![me](https://github.com/ammaralvi21/Whack-a-Mole/blob/master/Images/Whackamole2.gif)

# What the Game is?

The game is called whack-a-mole, which is a popular arcade game.
I have implemented a modified version of that game. I used LEDs to represent
the moles and the button press to indicate the mole has been whacked. Instead
of having multiple random moles apearing at the same time, I have only one
mole appear at a time. Instead of using a hammer, the user simply presses the 
corresponding pushbutton to hit the mole. There are maximum of 16 cycles. 


# How to play the Game?

When the user first powers up the board, the four LEDs display a moving pattern. 
This indicates the Waiting For the playermode. If the user presses any of the 4 
pushbuttons, it starts to the actual game. One of the four LEDs is randomly lit 
up, and the user has to press the corresponding button in a specific time called 
reaction time. The reaction time first is maximum 2 seconds, and it decreases by 
100ms every cycle until it reaches minimum of 500ms. Reaction time cannot go lower
than 500ms. If the user fails to press the button in the specified reaction time 
or presses the wrong button, the player loses. If the player loses the game then,
the score out of 16 is displayed on the LEDs in a blinking pattern for one minute max.
The score is in binary form with most significant bit on LED1 and least significant 
bit on LED4. The user can press any button to exit out of that mode and go into waiting for player mode. 
If the player however presses the correct button and doesn't lose, it then continues and lights up 
another random LED. This repeats for a maximum of 16 cycles. If the user presses the 
correct button for all 16 cycles, the user wins the game. An LED pattern indicating a winning
signal is displayed for one minute max. The user can then press any button to return to the
waiting for player mode and restart. 


# Extra Features and information?

I used the four LEDs to display the score, in binary, if the user loses the game.
If the score was zero, the blue LED on the STM32 descovery board flashes. If the user 
makes the number cycles greater than 16, then to indicate a score greater than 15, I flash all four
LEDs as well as the orange LED on the STM32 descovery board.
I used the RTC clock as the seed input to the psudo random number generating algorithm.
RTC clock counter does not loose its state from last power up and it is seperate from the 
system clock so therefore it provides an element of randomnes to the algorithm. 
I also used RTC clock to also generate accurate delays. I prescaled the frequency 
of LSE clock 32.768KHz down to 1024Hz. This means that RTC count of 1024 corresponds to exactly one second. 
I also have a mechanism for when the user presses the button for too long. So lets say if the 
user wants to exit the waiting for player mode by pressing a button, my code will detect a button press
and it will only move on the next mode when the user releases the button. This ensures that multiple
button presses are not detected from one long button press.
One of the extra features I could impliment in the future is use the switches on the left of the pushbuttons
as a way to select the difficulty level or interface with the LCD screen to display game information.


# How the user can adjust the game parameters?

The user can adjust the preliminary wait time (PrelimWait), reaction time (ReactTime), 
number of cycles to complete (NumCycles), time for when the winning signal is displayed (WinningSignalTime),
and the time for when the score is displayed when the game is lost (LosingSignalTime). The user can
adjust these paratemeters in the main assembly file called Game.s, these are the equates located on line 17 to 21. 
They can be changed to get the desired difficulty level.
