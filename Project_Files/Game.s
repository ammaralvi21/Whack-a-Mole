;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ENSE 352 Wack a mole term project
;Author: Ammer Alvi (200374049)
;file: Game.s
;By interfacing with the STM32 descovery board, this program replicates 
;the popular wack a mole game. Moles are represented by LEDs and corresponding pushbutton
;must be pressed at the right time to succeed. No more than 1 LED appearing at the same time.
;LEDs are randomly lit. 

;;; Directives
            PRESERVE8
            THUMB       

        		 
;;; Equates
;These are the adjustable paramaters for the game, for time constants 1024 value = 1 second
LosingSignalTime 	EQU 122880		; Time for when to display the score after the user has lost, equal to 1 minute
WinningSignalTime 	EQU 122880		; Time for when to display the winning LED pattern after winning, equal to 1 minute
PrelimWaitTime		EQU 300			; Time to wait before a random LED turns on. Equal to approx. 300ms
ReactTime			EQU 2048		; The maximum time given to the user to press the proper button. Equal to 2 seconds.
NumCycles			EQU	16			; The maximum number of cycles completed

INITIAL_MSP	EQU		0x20001000	; Initial Main Stack Pointer Value

;PORT A GPIO - Base Addr: 0x40010800
GPIOA_CRL	EQU		0x40010800	; (0x00) Port Configuration Register for Px7 -> Px0
GPIOA_CRH	EQU		0x40010804	; (0x04) Port Configuration Register for Px15 -> Px8
GPIOA_IDR	EQU		0x40010808	; (0x08) Port Input Data Register
GPIOA_ODR	EQU		0x4001080C	; (0x0C) Port Output Data Register
GPIOA_BSRR	EQU		0x40010810	; (0x10) Port Bit Set/Reset Register
GPIOA_BRR	EQU		0x40010814	; (0x14) Port Bit Reset Register
GPIOA_LCKR	EQU		0x40010818	; (0x18) Port Configuration Lock Register

;PORT B GPIO - Base Addr: 0x40010C00
GPIOB_CRL	EQU		0x40010C00	; (0x00) Port Configuration Register for Px7 -> Px0
GPIOB_CRH	EQU		0x40010C04	; (0x04) Port Configuration Register for Px15 -> Px8
GPIOB_IDR	EQU		0x40010C08	; (0x08) Port Input Data Register
GPIOB_ODR	EQU		0x40010C0C	; (0x0C) Port Output Data Register
GPIOB_BSRR	EQU		0x40010C10	; (0x10) Port Bit Set/Reset Register
GPIOB_BRR	EQU		0x40010C14	; (0x14) Port Bit Reset Register
GPIOB_LCKR	EQU		0x40010C18	; (0x18) Port Configuration Lock Register

;The onboard LEDS are on port C bits 8 and 9
;PORT C GPIO - Base Addr: 0x40011000
GPIOC_CRL	EQU		0x40011000	; (0x00) Port Configuration Register for Px7 -> Px0
GPIOC_CRH	EQU		0x40011004	; (0x04) Port Configuration Register for Px15 -> Px8
GPIOC_IDR	EQU		0x40011008	; (0x08) Port Input Data Register
GPIOC_ODR	EQU		0x4001100C	; (0x0C) Port Output Data Register
GPIOC_BSRR	EQU		0x40011010	; (0x10) Port Bit Set/Reset Register
GPIOC_BRR	EQU		0x40011014	; (0x14) Port Bit Reset Register
GPIOC_LCKR	EQU		0x40011018	; (0x18) Port Configuration Lock Register

;Registers for configuring and enabling the clocks
;RCC Registers - Base Addr: 0x40021000
RCC_CR		EQU		0x40021000	; Clock Control Register
RCC_CFGR	EQU		0x40021004	; Clock Configuration Register
RCC_CIR		EQU		0x40021008	; Clock Interrupt Register
RCC_APB2RSTR	EQU	0x4002100C	; APB2 Peripheral Reset Register
RCC_APB1RSTR	EQU	0x40021010	; APB1 Peripheral Reset Register
RCC_AHBENR	EQU		0x40021014	; AHB Peripheral Clock Enable Register
RCC_APB2ENR	EQU		0x40021018	; APB2 Peripheral Clock Enable Register  -- Used
RCC_APB1ENR	EQU		0x4002101C	; APB1 Peripheral Clock Enable Register
RCC_BDCR	EQU		0x40021020	; Backup Domain Control Register
RCC_CSR		EQU		0x40021024	; Control/Status Register
RCC_CFGR2	EQU		0x4002102C	; Clock Configuration Register 2
	
RTC_CRL		EQU		0x40002804	; RTC control register low
RTC_PRLL	EQU		0x4000280C	; RTC prescaler load register
RTC_CNTH	EQU		0x40002818	; RTC counter register high
RTC_CNTL	EQU		0x4000281C	; RTC counter register low
PWR_CR		EQU		0x40007000	; Power control register

; Times for delay routines
        
DELAYTIME	EQU		1600000		; (200 ms/24MHz PLL)


; Vector Table Mapped to Address 0 at Reset
            AREA    RESET, Data, READONLY
            EXPORT  __Vectors

__Vectors	DCD		INITIAL_MSP			; stack pointer value when stack is empty
        	DCD		Reset_Handler		; reset vector
			
            AREA    MYCODE, CODE, READONLY
			EXPORT	Reset_Handler
			ENTRY

Reset_Handler		PROC

		BL GPIO_ClockInit	; initializing the clocks to all the ports that will be used
		BL GPIO_init		; initializing the general purpose pins to specified configuration
		bl LED_OFF			; turing the LED's off initially
		bl RTC_init			; initializing the Real Time Clock (RTC) counter to start counting	

mainLoop
		ldr r12,=NumCycles		; Getting the number of turns to be played by the user and storing it in r12
		ldr r11,=ReactTime		; Getting the reaction time limit and storing it in r11
		mov r9, #0				; initializing the score to be equal to zero
		bl  WaitingForPlayer	; Entering the waiting for player state
		bl PlayGame				; When the user is ready, they start the actual game
		B	mainLoop			; Once the game is played, branch back to the mainLoop to start again
		
		ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;---------------------------------------Subroutines-------------------------------------------------
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; The waiting for player subroutine, generates a waiting menu in which
; an LED pattern is displayed until the user presses any button indicating
; that they are ready to exit the menu.
; The subroutine has no inputs or output registers, it is simply a state that the user can stay
; in for as long as they wish

	align
WaitingForPlayer	proc
	push {lr}					; store link register in stack to get back to mainloop
Wait_loop						; start the loop for the waiting pattern
	bl LED_OFF					; turn the LED's off
	mov r3,#100					; Delay for approx. 100 ms
	bl Delay
					
	ldr r2,=0x1c00				; Turn on the first LED
	bl TurnOnLED			
	bl Get_Buttons				; Check if there is any input from the buttons
	cmp r0, #0x1e00				; Check if its in normal state(0x1e00 becasue active low)..
	bne buttonPressed			; ..if button is pressed, branch forword to 'buttonPressed' label
	mov r3,#100					; Delay for approx. 100 ms
	bl Delay
	
	ldr r2,=0x1800				; Turing on the second LED
	bl TurnOnLED		
	bl Get_Buttons				; Checking for any input from the buttons
	cmp r0, #0x1e00
	bne buttonPressed			; When pressed branch out
	mov r3,#100					; Delay for approx. 100 ms
	bl Delay			
	
	ldr r2,=0x1200				; Turning off first LED and turn on third LED
	bl TurnOnLED	
	bl Get_Buttons				; Get the button state
	cmp r0, #0x1e00
	bne buttonPressed			; if pressed, branch out
	mov r3,#100					; 100 ms delay
	bl Delay
	
	ldr r2,=0x600				; Turn off the second LED, turn on fourth LED
	bl TurnOnLED	
	bl Get_Buttons				; Get button state
	cmp r0, #0x1e00
	bne buttonPressed			; if pressed, branch out
	mov r3,#100					; 100ms delay
	bl Delay
	
	ldr r2,=0xe00				; Turn off third LED
	bl TurnOnLED
	bl Get_Buttons				; Get button state
	cmp r0, #0x1e00		
	bne buttonPressed			; branch out if button pressed
	mov r3,#100					; 100ms delay
	bl Delay
	
	bl LED_OFF					; Turn LEDs off
	mov r3,#100					; 100ms delay
	bl Delay
	
	bl Get_Buttons				; Get button state one last time
	cmp r0, #0x1e00				; if no button is pressed branch back to wait_loop..
	beq Wait_loop				; ... and repeat the LED pattern
buttonPressed
	bl LED_OFF					; Turn LED OFF
	bl WaitForButtonRelease		; Wait until pressed button has been released, and then continue
	mov r3,#512					; 500ms delay
	bl Delay			
	pop{lr}						; Get the previously stored link register value
	bx lr						; branch back
	LTORG	; Used to avoid problems with ldr statements
	endp

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
;The PlayGame subroutine is when the playing of the actual game happens.
; This subroutine defines the logic of wack a mole game by first having a 
; preliminary delay, then a random led turns on, then determine what action
; took place i.e the correspodning button pressed, wrong button pressed, or 
; time ran out. From those decisions, it is determined if game was lost or to continue
; playing. Once all the cycles are completed, game has been won
	align
PlayGame proc
	push{lr}					; Store the link register in stack
	ldr r3,=PrelimWaitTime
	bl Delay					; Wait for a specified period of time before turning on LED
	bl Random_LED				; Light a random LED
	bl GetState					; Based on user input, determine if user has lost or should continue playing
	bl LED_OFF					; Turn the LED off
	pop{lr}						; Pop the link register form the stack
	cmp r11,#512				; Determine if the activation time has reached minimum of 500 ms
	blo MinimumTimeReached		; If the active time of LED's is reached less than 500 ms, skip the next bit of code
	sub r11,#96					; If the active time of LED's is greater than 500 ms, subtract the active time by 100 ms
MinimumTimeReached
	sub r12,#1					; Decrement the number of cycles counter
	cmp r12,#0					; If the counter has reached zero...
	bne PlayGame				; ...the player has won, or else branch back to the PlayGame subroutine
	push{lr}					; Store link register into stack
	bl LED_OFF					; Turn the LEDs off
	mov r3,#512					; Delay for 500ms
	bl Delay					
	bl GameWon					; Display the Game winning LED pattern
	pop{lr}						; Get the stored link register value from the stack
	bx lr						; branch back to main loop
	endp

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; GetState subroutine determines how the user has responded to the random LED lit up.
; If the user presses the corresponding button branch to next, and continue to the next stage.
; If the user presses the any other wrong button, branch to game lost
; If the user doesn't press any button and time expires, branch to game lost
	align
GetState proc
	push{lr}					; Store link register in stack
	bl GetRTCCountVal			; Getting the current RTC register counter value
	add r3, r0, r11				; Add the reaction time to the current RTC counter value
loop_delay						; Entering the react time delay loop
	bl Get_Buttons				; Getting the state of the buttons
	cmp r0,r10					; Compare whether the button pressed corresponds to the LED that is lit
	addeq r9,#1					; If its true then increment the score count by 1
	beq next					; If its true then branch to the label called next to exit the loop and continue to next cycle
	cmp r0, #0x1e00				; compare to determine if no button was pressed..
	beq continue				; If its true no button was pressed then branch to continue
	b GameLost					; If a button was pressed but it was a wrong one, then the user has lost the game, branching to GameLost
continue						; Continue branch that continues on with the loop if there is no button pressed
	bl GetRTCCountVal			; Getting the current RTC counter value
	cmp r0, r3					; Compare the RTC counter value to the see if it has reached the reaction time
	blo loop_delay				; If the RTC counter value is less than the reaction time value then loop again
	b	GameLost				; If the reaction time has expired then the user has lost, branch to GameLost
next							; next label used to get to the next cycle if the user has pressed the right button
	bl LED_OFF					; turn the led off
	pop{lr}						; Get the stored link register value from stack
	bx lr						; branch back to PlayGame
	LTORG
	endp

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; GameWon subroutine displays the winning LED pattern which are 2 LED's on left or right going
; back and forth in a clapping motion. ex: 1001 => 0110 => 1001 ...
	align
GameWon proc
	push{lr}					; store link register in stack
	ldr r4,=WinningSignalTime	; Getting the winning signal time and store in r4
	bl GetRTCCountVal			; Getting the current RTC counter value
	add r4, r0, r4				; Adding the current RTC counter value to winning signal time						
WinningSignal_loop				; Entering a loop
	ldr r2,=0xc00				; Turn on first LED and fourth LED
	bl TurnOnLED	
	bl Get_Buttons				; Get current button state
	cmp r0, #0x1e00				; Check to see if it has been pressed
	bne ExitingState			; If it has been pressed branch outside the loop
	mov r3,#130					; Delay for aboout 130 ms
	bl Delay				
	ldr r2,=0x1200				; Turn on second and third LED
	bl TurnOnLED	
	bl Get_Buttons				; Get the current button state
	cmp r0, #0x1e00				; Check to see if it has been pressed
	bne ExitingState			; Exit the loop if a button has been pressed
	mov r3,#130					; Delay for another 130 ms
	bl Delay
	bl GetRTCCountVal			; Get the current RTC counter value
	cmp r0, r4					; Checking if the counter value has reached winning signal time
	blo WinningSignal_loop		; If it the counter value is less than the winning signal loop back
ExitingState					; Label used to go outside the signal 
	bl LED_OFF					; Turn LEDs off
	bl WaitForButtonRelease		; Wait until button has been released 
	pop{lr}						; Get the stored link register 
	bx lr						; branch back to the main loop and start game again
	endp
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; GameLost subroutines displays the score the user got before losing
; The score is displayed in a flashing LEDs using binary values from most significant bit on left
	align
GameLost proc
	bl LED_OFF			   		; Turn off the LEDs
	bl WaitForButtonRelease		; Wait until button has been released 
	mov r3,#512					; Delay for 500 ms
	bl Delay 
	cmp r9, #0					; Check if the score was zero
	beq LED_Blink1				; if the score is zero display the LED that shows a score of zero
	cmp r9, #15					; Check if the score scored is higher than 15 (maximum binary value that can be 4bits)
	bhi LED_Blink2				; Display the LED that indicates a score higher than 15
	bl ChangeOrderOfBits		; Change the order of bits of the score that is accumulated to display properly on LEDs
	mvn r9,r9					; invert the score bits beacuse the LEDs are active low
	and r9,#0xf					; Get the first 4 bits
	lsl r9, #9					; Shift left by 9, ready to be outputed to LEDs
	ldr r4,=LosingSignalTime	; Get the losing signal time 
	bl GetRTCCountVal			; Get the current RTC value
	add r4, r0, r4				; Add the current RTC count to losing signal time
losingSignal_loop			
	mov r2, r9					; Output the score to the LEDs
	bl TurnOnLED					
	bl Get_Buttons				; Get the button state
	cmp r0, #0x1e00				; Check if the button has been pressed
	bne ButtonPressed2			; Exit the loop if a button presseed
	mov r3,#100					; Delay for 100ms
	bl Delay
	bl LED_OFF					; Turn LEDs off
	bl Get_Buttons				; Get button state
	cmp r0, #0x1e00				; Check if the button has been pressed
	bne ButtonPressed2			; Exit the loop if a button has been pressed
	mov r3,#100					; Delay for 100ms
	bl Delay
	bl GetRTCCountVal			; Get the current RTC counter value
	cmp r0, r4					; Compare with losing signal time
	blo losingSignal_loop		; Loop if its lower
	
ButtonPressed2					; label used to exit loop
	bl LED_OFF					; turn leds off
	bl WaitForButtonRelease		; wait until the button pressed is released
	pop{lr}						; get the stored link register from the stack and exit..
	pop{lr}						; ..to the mainloop
			
	bx lr
	endp

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; LED_Blink2 blink all 4 red LEDs and the orange LED indicating score greater than 15
	ALIGN
LED_Blink2  PROC

	ldr r4,=LosingSignalTime	; Get the losing signal time
	bl GetRTCCountVal			; get rtc counter value
	add r4, r0, r4				; add the counter value to losing signal time
losingSignal_loop3
	ldr r1,=GPIOC_ODR			; turn on the orange LED
	mov r0,#0x200
	str r0,[r1]
	mov r2, #0				; turn on the red LEDs
	bl TurnOnLED
	bl Get_Buttons				; Get the button state
	cmp r0, #0x1e00	
	bne ButtonPressed3			; if pressed, branch out the loop
	mov r3,#100
	bl Delay					; delay for 100ms
	bl LED_OFF					; turn the 4 red leds off
	ldr r1,=GPIOC_ODR			; turn off the orange LED
	ldr r0,[r1]
	mvn r2,#0x200
	and r0,r2
	str r0,[r1]
	mov r3, #100				; delay for 100ms
	bl Delay
	bl GetRTCCountVal			; get the rtc counter value
	cmp r0, r4					; check if it has reached the losing signal time
	blo losingSignal_loop3		; loop if the counter value is lower
ButtonPressed3
	ldr r1,=GPIOC_ODR			; clear the LEDs
	mov r0,#0
	str r0,[r1]
	bl WaitForButtonRelease		; Wait until button pressed has been released
	pop{lr}						; branch back to mainloop
	pop{lr}

	BX LR
	ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; LED_Blink1 blinks the blue LED indicating score of zero. 
; The subroutine follows the exact same procedure as LED_Blink2 subroutine except we are only
; flashing the blue LED on the STM board

	ALIGN
LED_Blink1	PROC
	ldr r4,=LosingSignalTime
	bl GetRTCCountVal
	add r4, r0, r4
losingSignal_loop2
	ldr r1,=GPIOC_ODR				; Output to port C for blue LED
	mov r0,#0x100					
	str r0,[r1]
	
	bl Get_Buttons			
	cmp r0, #0x1e00
	bne ButtonPressed4
	mov r3, #100
	bl Delay
	
	ldr r1,=GPIOC_ODR				; Clear the blue LED on the STM board
	ldr r0,[r1]
	mvn r2,#0x100
	and r0,r2
	str r0,[r1]
	
	mov r3, #100
	bl Delay
	bl GetRTCCountVal
	cmp r0, r4
	blo losingSignal_loop2
ButtonPressed4
	ldr r1,=GPIOC_ODR
	mov r0,#0
	str r0,[r1]
	bl WaitForButtonRelease
	pop{lr}
	pop{lr}
	BX LR
	ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; WaitForButtonRelease is a general subroutine that is used by the major subroutines above.
; The main purpose of it is that it keeps looping until the buttons are in their neutral state.
; This written to ensure that the button press by the user does not affect the next stages that 
; also rely on button input

	align
WaitForButtonRelease proc
	push{lr}						; store link register in stack
	bl Get_Buttons					; Get the button state
	pop{lr}		
	cmp r0, #0x1e00					; check to see if no button is pressed (0x1e00 because its active low)
	bne WaitForButtonRelease		; If a button is still being pressed, the keep looping
	bx lr							; Exit the subroutine once a button has been released
	endp
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Get_Buttons subroutine basically reads the input data registers for the pushbuttons on the
; pcb. It detects input at each pushbutton and does bit manipulation to combine the values into 
; the corresponding and equivalent form of the LED ouput registers. The result is ouputed to R0.
	align
Get_Buttons proc
	ldr r1,=GPIOB_IDR				; reading the red and black pushbuttons in port B
	ldr r0,[r1]						
	and r0,#0x300					; isolating the values for PB8 (red) and PB9 (black)
	lsl r0,#1						; shift left by 1 to place the bits in corresponding LED ouput register format
	ldr r1,=GPIOC_IDR				; reading the blue push button in port C	
	ldr r2,[r1]					
	and r2,#0x1000					; isolating the value for PC12 (blue)
	lsr r2,#1						; shift right by 1 to place it in corresponding LED ouput register format
	orr r0,r2						; combine the data for red & black push buttons and blue push button by OR logic
	ldr r1,=GPIOA_IDR				; reading the input of greed pushbutton in port A
	ldr r2,[r1]
	and r2,#0x20					; isolating the vlaue for PA5
	lsl r2,#7						; shift left by 7 to place it in the corresponding LED ouput register format
	orr r0,r2						; Combine the data for all pushbuttons ainto single regsiter R0 ready for output
	bx lr							; exit the subroutine
	endp		

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; GetRTCCountVal subroutine gets the counter value from the RTC clock counter and ouputs it to
; R0.
	align
GetRTCCountVal proc
	ldr r1,=RTC_CNTH			 	; Reading the RTC high counter
	ldr r0,[r1]						
	lsl r0, #16						; Shift the high counter value left by 16 bits 
	ldr r2,[r1,#4]					; Reading the RTC low counter value
	orr r0,r2						; Combine the high counter value and low counter value using OR logic
	bx lr							; Exit the subroutine 
	endp

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Random_LED subroutine turns on one of the four red LEDs randomly. 
; I used an algorithm found online called Linear Congruential Generator that generates  
; psudorandom numbers using a simple method.
;___________________________________________________________________________________________
; ------  https://en.wikipedia.org/wiki/Linear_congruential_generator  ---------------------
;--------------------------------------------------------------------------------------------
; The generator is defined by recurrence relation:
;    X(n + 1) = ( a*X(n) + c )  mod  m
;	 where X is the sequence of pseudorandom values, and
;    m , 0 < m — the "modulus"
;    a , 0 < a < m — the "multiplier"
;    c , 0 = c < m — the "increment"
;    X0, 0 = X0 < m — the "seed" or "start value"
; As the seed(X0), I used the RTC counter value as the input. I figured this would provide a fairly 
; random and unpredictable input.
; For modulous I chose m = 134456, a = 1664525, c = 1013904223. These were one of the recommended values on the
; wikipedia page. I chose to iterate the relation 9 times.
	ALIGN
Random_LED  PROC
	push {lr}
	bl GetRTCCountVal				; Get the current RTC counter value for initial seed input
	ldr r4,=1013904223				; Selecting R4 as the "c" value
	mov r5,#9						; Iteration counter
	ldr r6,=1664525					; Selecting R6 as the "a" value	 
randloop							; Entering loop that will iterate 9 times over the relation
	sub r5, #1						; Decrement the iteration counter
	mla r0, r0, r6,r4				; r0 = r0(seed) * r6(multiplier) + r4(increment)					
	cmp r5, #0						; Check if the iteration counter has reached 0
	bne randloop					; If not equal to zero, countinue looping				
	and r0, #3						; Doing a modulous of r0 mod 4, to get a value somewhere between 0 to 3
	;Encoding the randomly generated value of 0 to 3, to the corresponding LED data register ouput
	cmp r0, #0						; If its 0, then turn on 1st LED
	moveq r10, #0x1c00
	cmp r0, #1						; If its 1, then turn on 2nd LED
	moveq r10, #0x1a00	
	cmp r0, #2						; If its 2, then turn on 3rd LED
	moveq r10, #0x1600
	cmp r0, #3						; If its 3, then turn on 4th LED
	moveq r10, #0xe00
					 
	mov r2, r10						; Writing randomly generated encoded value to LED registers
	bl TurnOnLED	
	pop{lr}
	BX LR
	ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; TurnOnLED simply takes R2 as the encoded input and ouputs that to 
; the ouput data ragisters for the LEDs
	align
TurnOnLED proc
	ldr r1,=GPIOA_ODR			; Writing to the output data register on port A			
	str r2,[r1]					; Store the input value to the register memory location
	bx lr						; exit subroutine
	endp
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; LED_OFF simply turns off all the LEDs on the board.
	align
LED_OFF	proc
	mov r0, #0x1e00				; LEDs are active low and require 0x1e00 to turn off
	ldr r1,=GPIOA_ODR			; loading the memory location of data register for LEDs
	str r0,[r1]					; Store the encoded value to memory location
	bx lr						; exit subroutine
	endp
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Delay is a general purpose delay sub routine that takes R0 as the the input register and
; provides a delay of that amount in approx. milliseconds. To be precise 1024 input is equal to
; one second. 512 input is equal to 500 ms.
	ALIGN
Delay PROC
	push{lr}
	bl GetRTCCountVal			; Read the RTC count value				
	add r3, r0, r3				; Add the delay amount to the counter
loop_delay2
	bl GetRTCCountVal			; Get the current count value
	cmp r0, r3					; check if it has reached that added delay amount
	blo loop_delay2				; If the current count is less, then continue looping
	pop {lr}					

	BX LR						; Exiting subroutine
	ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;This routine will enable the clock for the Ports required
	ALIGN
GPIO_ClockInit PROC
	; ENEL 384 Pushbuttons: SW2(Red): PB8, SW3(Black): PB9, SW4(Blue): PC12 *****NEW for 2015**** SW5(Green): PA5
	; ENEL 384 board LEDs: D1 - PA9, D2 - PA10, D3 - PA11, D4 - PA12
	ldr r1,=RCC_APB2ENR
	ldr r0,[r1]
	orr r0,#0x1c	
	str r0,[r1]	
	BX LR
	ENDP
		
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
; This routine enables the GPIO for the LED's.  By default the I/O lines are input so we only need to configure for ouptut.
; Using general pupose output push-pull configuration with output max speed 50 MHz
	ALIGN
GPIO_init  PROC
	; ENEL 384 board LEDs: D1 - PA9, D2 - PA10, D3 - PA11, D4 - PA12
	ldr r1,=GPIOA_CRH  ;------> For LEDs on the STM32 descovery board 
	ldr r0,[r1]
	ldr r5,=0x33330				
	orr r0, r5					
	ldr r5,=0xcccc0
	mvn r2, r5
	and r0,r2	
	str r0,[r1]
	ldr r1,=GPIOC_CRH  ;------> For the LEDs on the PCB 
	ldr r0,[r1]
	orr r0, #0x33
	mvn r2, #0xcc
	and r0,r2	
	str r0,[r1]
    BX LR
	ENDP
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Initializing Real Time Clock (RTC) counter
; To active the RTC, we first have to enable the RCC clock for power interfaces, backup interfaces.
; Using the power interface, we use the power control register to get RTC write access.
; Then we use the backup domain register in RCC to enable the LSE oscillator clock for the RTC.
; Once we have acess to RTC, we use the RTC peripheral registers to read or write to it. 
; For RTC write operations, we have to enter configuration mode using the configuration flag
; bit. After entering configuration mode, I prescaled the clock frequency of 32.768KHz
; down to 1024 Hz using the RTC prescaler load register. Now we are done with configuration 
; mode so we exit the configuration mode and wait for RTC operation OFF bit in control register
; to be set to '1' meaning "Last write on RTC regsiters terminated". Our RTC counter register 
; now starts counting based on the prescaled frequency. RTC has been fully configured
	ALIGN
RTC_init  PROC
	
	ldr r1,=RCC_APB1ENR		; Activate the clock for power interface and backup interface
	ldr r0,[r1]
	ldr r2,=0x18000000
	orr r0,r2	
	str r0,[r1]

	ldr r1,=PWR_CR			; Using the power control register to get RTC write access
	ldr r0,[r1]
	orr r0,#0x100	
	str r0,[r1]
	
	ldr r1,=RCC_BDCR		; Use backup domain register to enable RTC clock and use LSE oscillator as the clock
	ldr r0,[r1]
	ldr r2,=0x8101
	orr r0,r2	
	str r0,[r1]
	
	ldr r1,=RTC_CRL			; Using the control register for the RTC
CRL_Loop
	ldr r0,[r1]				; Reading the RTC operation OFF bit and checking to see if last write operation has been.. 
	and r0, #0x20			; ..terminated
	cmp r0, #0x20
	bne CRL_Loop
	
	ldr r1,=RTC_CRL			; Writing to the configuration flag, to enter configuration mode
	ldr r0,[r1]
	orr r0,#0x10	
	str r0,[r1]
	
	ldr r1,=RTC_PRLL  		; Writing to RTC prescaler load register. I set the prescaler to 31.
	ldr r0,[r1]				; it uses the formula f(TR_CLK) = 32.768 kHz/(prll_val +1) which gives us a frequency of 1024Hz.
	mov r0,#31
	str r0,[r1]
	
	ldr r1,=RTC_CRL			; Exiting configuration mode
	ldr r0,[r1]
	mvn r2,#0x10
	and r0,r2	
	str r0,[r1]
	
	ldr r1,=RTC_CRL
CRL_Loop2					; Wating for the last write operation to get terminiated 
	ldr r0,[r1]
	and r0, #0x20
	cmp r0, #0x20
	bne CRL_Loop2
	
	BX LR
	ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; This subroutine changes the order of 4 bits. This was used because when displaying the score
; on the PCB LEDs, we want our most significant bit to be first LED and fourth LED to be least
; significant. To do that, we have to flip the order of the bits. I hard coded the logic.
; ex: 1010 => 0101,    1110 => 0111,     0100 => 0010
; Input is r9, ouput is r9
	align
ChangeOrderOfBits proc
	mov r0,#0
	and r1,r9,#1		;Get the first bit
	lsl r1,#3			;shift it left by 3
	orr r0,r1			
	and r1,r9,#2		;Get the second bit
	lsl r1,#1			;Shift it left by 1
	orr r0,r1
	and r1,r9,#4		;Get the thrid bit
	lsr r1,#1			;Shift it right by 1
	orr r0,r1
	and r1,r9,#8		;Get the fourth bit
	lsr r1,#3			;Shift it right by 3 
	orr r0,r1
	mov r9, r0
	bx lr
	endp

	ALIGN
	END
		

