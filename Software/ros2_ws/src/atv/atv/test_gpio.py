import Jetson.GPIO as GPIO
import time

BUTTON_PIN = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN)  # Using external pull-up resistor; no internal pull-up here

try:
    while True:
        state = GPIO.input(BUTTON_PIN)
        print("Button state:", "LOW (pressed)" if state == GPIO.LOW else "HIGH (not pressed)")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
