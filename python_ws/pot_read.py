import board
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO

# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an ADS1115 object
ads = ADS.ADS1115(i2c)
 
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)

#GPIO.setmode(GPIO.BCM)  # Set's GPIO pins to BCM GPIO numbering
#INPUT_PIN = 4           # Sets our input pin, in this example I'm connecting our button to pin 4. Pin 0 is the SDA pin so I avoid using it for sensors/buttons
#in1 = 17
#in2 = 27
#en = 22
#GPIO.setup(INPUT_PIN, GPIO.IN)  # Set our input pin to be an input
#GPIO.setup(in1,GPIO.OUT)
#GPIO.setup(in2,GPIO.OUT)
#GPIO.setup(en,GPIO.OUT)
#GPIO.output(in1,GPIO.LOW)
#GPIO.output(in2,GPIO.LOW)
#p=GPIO.PWM(en,1000)
#p.start(25)

# Loop to read the analog input continuously
while True:
    print("Analog Value: ", channel.value, "Voltage: ", channel.voltage)
