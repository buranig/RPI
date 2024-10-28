#!/usr/bin/env python

from time import sleep           # Allows us to call the sleep function to slow down our loop
import RPi.GPIO as GPIO           # Allows us to call our GPIO pins and names it just GPIO
 
GPIO.setmode(GPIO.BCM)           # Set's GPIO pins to BCM GPIO numbering
INPUT_PIN = 4           # Sets our input pin, in this example I'm connecting our button to pin 4. Pin 0 is the SDA pin so I avoid using it for sensors/buttons
SWITCH_PIN = 23
GPIO.setup(INPUT_PIN, GPIO.IN)           # Set our input pin to be an input

# Define debounce time in milliseconds
DEBOUNCE_TIME_MS = 200  # 200 milliseconds
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#switch_state = GPIO.input(SWITCH_PIN)
#prev_switch_state = switch_state

# Define a function to handle button presses
def button_callback(channel):
    global switch_state
    switch_state = GPIO.input(SWITCH_PIN)

# Add an event listener for the button press
#GPIO.add_event_detect(SWITCH_PIN, GPIO.BOTH, callback=button_callback, bouncetime=DEBOUNCE_TIME_MS)

try:
    # Main loop
    while True:
        if GPIO.input(SWITCH_PIN) == GPIO.HIGH:
            print("BUTTON NOT PRESSED")
        else:
            print("BUTTON PRESSED")
        #if switch_state != prev_switch_state:
         #   if switch_state == GPIO.HIGH:
          #      print("The limit switch: TOUCHED -> UNTOUCHED")
           # else:
            #    print("The limit switch: UNTOUCHED -> TOUCHED")

            #prev_switch_state = switch_state


        #if switch_state == GPIO.HIGH:
         #   print("The limit switch: UNTOUCHED")
        #else:
        #    print("The limit switch: TOUCHED")

except KeyboardInterrupt:
    # Clean up GPIO on exit
    GPIO.cleanup()

while True: 
           if (GPIO.input(INPUT_PIN) == True): # Physically read the pin now
                    print('3.3')
           else:
                    print('0')
           sleep(1);           # Sleep for a full second before restarting our loop

