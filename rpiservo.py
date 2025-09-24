import RPi.GPIO as GPIO
import time

servo_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz (20 ms period)
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)   # maps 0–180° to 2–12% duty
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

set_angle(0)
set_angle(90)
set_angle(180)

pwm.stop()
GPIO.cleanup()
