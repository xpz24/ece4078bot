import time
import csv
import RPi.GPIO as GPIO  # type: ignore


# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# Globals
prev_left_state, prev_right_state = None, None
left_count, right_count = 0, 0

# Config
PWM_RANGE = range(-40, 41, 1)  # sweep -40% → +40%
HOLD_TIME = 0.6  # hold each PWM for 0.6s
SAMPLE_DT = 0.02  # log every 20ms
TRANSIENT_T = 0.2  # duration of transient period
PWM_FREQUENCY = 60


def setup_gpio():
    global prev_left_state, prev_right_state

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)

    # This prevents slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)

    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)

    # Init encoder state
    prev_left_state = GPIO.input(LEFT_ENCODER)
    prev_right_state = GPIO.input(RIGHT_ENCODER)

    # Initialize PWM (frequency: 60Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, PWM_FREQUENCY)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, PWM_FREQUENCY)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)


def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)

    # Check for actual state change. Without this, false positive happens due to electrical noise
    # After testing, debouncing not needed
    if prev_left_state is not None and current_state != prev_left_state:
        left_count += 1
        prev_left_state = current_state


def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)

    if prev_right_state is not None and current_state != prev_right_state:
        right_count += 1
        prev_right_state = current_state


def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0


def get_encoder_counts(side: str):
    if side == "left":
        return left_count
    else:
        return right_count


def set_motors(left, right):

    # # ---- Normal Drive + Steady State Coasting Braking ----
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(0)

    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(0)


def calibrate_wheel(side, filename="cal_log.csv"):
    print(f"Starting {side} wheel calibration...")
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "pwm_left", "pwm_right", "enc_left", "enc_right"])

        reset_encoder()
        for pwm in PWM_RANGE:
            print(f"PWM = {pwm}")
            t0 = time.time()

            if side == "left":
                set_motors(pwm, 0)
            else:
                set_motors(0, pwm)

            while time.time() - t0 < HOLD_TIME:
                ts = time.time()
                elapsed = ts - t0
                if elapsed > TRANSIENT_T:
                    encL = get_encoder_counts("left")
                    encR = get_encoder_counts("right")
                    writer.writerow(
                        [
                            ts,
                            pwm if side == "left" else 0,
                            pwm if side == "right" else 0,
                            encL,
                            encR,
                        ]
                    )
                time.sleep(SAMPLE_DT)

            set_motors(0, 0)
            if pwm == 0:
                time.sleep(1)
            else:
                time.sleep(0.3)
            reset_encoder()

    print(f"{side} calibration finished → saved to {filename}")


if __name__ == "__main__":
    try:
        setup_gpio()
        calibrate_wheel("left", f"left_calib_{PWM_FREQUENCY}Hz_{int(time.time())}.csv")
        calibrate_wheel(
            "right", f"right_calib_{PWM_FREQUENCY}Hz_{int(time.time())}.csv"
        )
    except KeyboardInterrupt:
        print("Stopping....")
    finally:
        set_motors(0, 0)
        GPIO.cleanup()
