import lgpio
import time

# Open the GPIO chip
h = lgpio.gpiochip_open(4)

servo_pin = 12

lgpio.gpio_claim_output(h,servo_pin)
print('opened chip')

# Set up PWM at 50 Hz
try:
    lgpio.tx_pwm(h, servo_pin, 50, 2.5)  # 7.5% duty = ~1.5 ms pulse (center)
    print('first worked')
except Exception as e:
    print(f"error:{e}")
time.sleep(2)

# Move to 0 degrees (~1 ms pulse, 5% duty)
lgpio.tx_pwm(h, servo_pin, 50, 7.5)
time.sleep(2)

# Move to 180 degrees (~2 ms pulse, 10% duty)
lgpio.tx_pwm(h, servo_pin, 50, 12.5)
time.sleep(2)

# Stop PWM output
lgpio.tx_pwm(h, servo_pin, 0, 0)

# Close GPIO chip
lgpio.gpiochip_close(h)

