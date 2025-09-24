import lgpio
import time

# Open the GPIO chip
h = lgpio.gpiochip_open(0)

servo_pin = 12

# Set up PWM at 50 Hz
lgpio.tx_pwm(h, servo_pin, 50, 7.5)  # 7.5% duty = ~1.5 ms pulse (center)

time.sleep(2)

# Move to 0 degrees (~1 ms pulse, 5% duty)
lgpio.tx_pwm(h, servo_pin, 50, 5.0)
time.sleep(2)

# Move to 180 degrees (~2 ms pulse, 10% duty)
lgpio.tx_pwm(h, servo_pin, 50, 10.0)
time.sleep(2)

# Back to center
lgpio.tx_pwm(h, servo_pin, 50, 7.5)
time.sleep(2)

# Stop PWM output
lgpio.tx_pwm(h, servo_pin, 0, 0)

# Close GPIO chip
lgpio.gpiochip_close(h)

