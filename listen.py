import socket
import struct
import io
import threading
import time
import math
import pigpio
import RPi.GPIO as GPIO  # type: ignore
from picamera2 import Picamera2  # type: ignore

# from scipy.signal import savgol_filter

# Network Configuration
HOST = "0.0.0.0"
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 22
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16
COUNTER_PIN = 18

# PID Constants (default values, will be overridden by client)
use_PID = 0
KP, Ki, KD, rKP, rKI, rKD = 0, 0, 0, 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value
MAX_INTEGRAL = MAX_CORRECTION

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
sL = 0.0
sR = 0.0
ds = 0.0
dth = 0.0
sign_L, sign_R = 1, 1
packet_id = 0
packet_ready = False
counter = 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE_ACC = 100  # PWM units per second (adjust this value to tune ramp speed)
RAMP_RATE_DEC = 100
MIN_RAMP_THRESHOLD = 5  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 5
current_movement, prev_movement = "stop", "stop"
TICKS_PER_REV = 40
RADIUS = 0.033
BASELINE = 0.115
RAD_PER_TICK = 2 * math.pi / TICKS_PER_REV
M_PER_TICK = RAD_PER_TICK * RADIUS
TARGET_REV_PER_SEC = 1
INTERVAL = 1 / (TARGET_REV_PER_SEC * TICKS_PER_REV)
LINEAR_PRIMING = 0.05
ROTATION_PRIMING = 0.03
POWER_BRAKING_DUTY = 40  # ! Be careful not to burn the motors, do not set to 100
POWER_BRAKING_TIME_ROT = 0.04
POWER_BRAKING_TIME_LIN = 0.08
DISABLE_ODM_PB = True
pb_mode = False
disable_brake = False
BRAKE_DISABLE_THRESHOLD = 0.2

# locks
encoder_lock = threading.Lock()
pwm_lock = threading.Lock()
movement_lock = threading.Lock()


def reference_counter():
    global counter
    next_tick = time.perf_counter()
    while running:
        now = time.perf_counter()
        if now >= next_tick:
            counter += 1
            next_tick += INTERVAL
        time.sleep(max(0, next_tick - time.perf_counter()))

def right_pwm_compensator(req_pwm):
    if req_pwm > 0:
        cmd_pwm = 1.241809 * req_pwm + -3.539788
    elif req_pwm < 0:
        cmd_pwm = 1.272179 * req_pwm + 3.640242
    else:
        return req_pwm
    return cmd_pwm


def clamp(x: int | float, minimum: int | float, maximum: int | float):
    X = max(minimum, min(x, maximum))
    return X


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
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 60)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 60)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)


def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)

    # Check for actual state change. Without this, false positive happens due to electrical noise
    # After testing, debouncing not needed
    if prev_left_state is not None and current_state != prev_left_state:
        with encoder_lock:
            left_count += 1
        prev_left_state = current_state


def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)

    if prev_right_state is not None and current_state != prev_right_state:
        with encoder_lock:
            right_count += 1
        prev_right_state = current_state


def reset_encoder():
    global left_count, right_count
    with encoder_lock:
        left_count, right_count = 0, 0


def set_motors(left, right):
    global prev_movement, current_movement, sign_L, sign_R, pb_mode
    with movement_lock:
        p_movement = prev_movement
        c_movement = current_movement

    # # ------------------- MOTOR PRIMING ----------------------
    if p_movement == "stop" and c_movement in ["forward", "backward"]:
        if DISABLE_ODM_PB:
            with encoder_lock:
                pb_mode = True
        if c_movement == "forward":
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif c_movement == "backward":
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(LINEAR_PRIMING)

    if p_movement == "stop" and c_movement in ["rotate_left", "rotate_right"]:
        # brief symmetrical 100% kick to overcome static friction
        if DISABLE_ODM_PB:
            with encoder_lock:
                pb_mode = True
        if c_movement == "rotate_left":
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        else:  # rotate_right
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)

        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(ROTATION_PRIMING)  # shorter than linear kick (rotation needs less)

    # # ------- Transient Power Braking -------
    # detect transition from movement to stop
    if (
        p_movement in ["forward", "backward", "rotate_left", "rotate_right"]
        and c_movement == "stop"
        and not disable_brake
    ):
        if DISABLE_ODM_PB:
            with encoder_lock:
                pb_mode = True
        brake_duty = POWER_BRAKING_DUTY  # % reverse torque
        if p_movement in ["rotate_left", "rotate_right"]:
            brake_time = POWER_BRAKING_TIME_ROT
        else:
            brake_time = POWER_BRAKING_TIME_LIN

        if p_movement == "forward":
            # reverse direction briefly
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
            with pwm_lock:
                sign_R, sign_L = -1, -1
        elif p_movement == "backward":
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
            with pwm_lock:
                sign_R, sign_L = 1, 1
        elif p_movement == "rotate_left":
            # briefly reverse rotation
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
            with pwm_lock:
                sign_R, sign_L = -1, 1
        elif p_movement == "rotate_right":
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
            with pwm_lock:
                sign_R, sign_L = 1, -1

        left_motor_pwm.ChangeDutyCycle(brake_duty)
        right_motor_pwm.ChangeDutyCycle(brake_duty)
        time.sleep(brake_time)  # brief pulse

        # then fall into steady-state active brake
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        return  # early exit to avoid reapplying below
    # # ---- Normal Drive + Steady State Active Braking ----
    with encoder_lock:
        pb_mode = False
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # when pwm = 0, implement Active Braking mode, better than putting duty cycle to 0 which may cause uneven stopping
        if not disable_brake:
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            right_motor_pwm.ChangeDutyCycle(100)
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
        if not disable_brake:
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
            left_motor_pwm.ChangeDutyCycle(100)
        else:
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
            left_motor_pwm.ChangeDutyCycle(0)


def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        # Boost small values to minimum threshold, preserving direction
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value


def pid_control():
    global left_pwm, right_pwm, use_PID, KP, KI, KD, rKP, rKI, rKD, prev_movement, current_movement, sign_L, sign_R, disable_brake

    integral = 0.0
    last_error = 0.0
    last_time = time.monotonic()
    switch_mode = [False, False]
    ramp_left_pwm = 0
    ramp_right_pwm = 0
    signL, signR = 1, 1
    move_start_time = None

    while running:
        with encoder_lock:
            l_count = left_count
            r_count = right_count
        with pwm_lock:
            l_pwm = left_pwm
            r_pwm = right_pwm

        current_time = time.monotonic()
        dt = current_time - last_time
        last_time = current_time

        prev_movement = current_movement

        if l_pwm > 0 and r_pwm > 0:
            requested_movement = "forward"
            signL, signR = 1, 1
        elif l_pwm < 0 and r_pwm < 0:
            requested_movement = "backward"
            signL, signR = -1, -1
        elif l_pwm == 0 and r_pwm == 0:
            requested_movement = "stop"
            signL, signR = signL, signR
        elif l_pwm < 0 and r_pwm > 0:
            requested_movement = "rotate_left"
            signL, signR = -1, 1
        else:
            requested_movement = "rotate_right"
            signL, signR = 1, -1

        # --- Measure move duration for brake decision ---
        if requested_movement in ("rotate_left", "rotate_right", "forward", "backward"):
            # Start timer if entering move
            if move_start_time is None:
                move_start_time = time.monotonic()
        elif requested_movement == "stop":
            # move just ended
            if move_start_time is not None:
                move_duration = time.monotonic() - move_start_time
                # Decide if braking should be disabled
                disable_brake = (
                    move_duration < BRAKE_DISABLE_THRESHOLD
                )  # e.g. 0.25s threshold
                if not disable_brake:
                    move_start_time = None
            else:
                disable_brake = False

        # Only switch once both ramps have finished braking/accelerating
        if not any(switch_mode):  # means [False, False]
            current_movement = requested_movement
            with pwm_lock:
                sign_L, sign_R = signL, signR
        else:
            current_movement = prev_movement

        if not use_PID:
            target_left_pwm = l_pwm
            target_right_pwm = r_pwm
        else:
            if current_movement in [
                "forward",
                "backward",
                "rotate_left",
                "rotate_right",
            ]:
                error = l_count - r_count
                if current_movement in ["forward", "backward"]:
                    if prev_movement in ["rotate_left", "rotate_right"]:
                        integral = 0.0  # ? Decay instead of reset?
                        last_error = 0.0
                        reset_encoder()

                    proportional = KP * error
                    derivative = KD * (error - last_error) / dt if dt > 0 else 0
                    integral += KI * error * dt
                else:
                    if prev_movement in ["forward", "backward"]:
                        integral = 0.0
                        last_error = 0.0
                        reset_encoder()
                    proportional = rKP * error
                    derivative = rKD * (error - last_error) / dt if dt > 0 else 0
                    integral += rKI * error * dt

                integral = clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL)
                correction = clamp(
                    proportional + integral + derivative,
                    -MAX_CORRECTION,
                    MAX_CORRECTION,
                )
                last_error = error
                if current_movement in ["backward", "rotate_left"]:
                    correction *= -1
                if current_movement in ["forward", "backward"]:
                    target_left_pwm = l_pwm - correction
                    target_right_pwm = r_pwm + correction
                elif current_movement in ["rotate_right", "rotate_left"]:
                    target_left_pwm = l_pwm - correction
                    target_right_pwm = r_pwm - correction
            else:
                reset_encoder()
                integral = 0.0
                last_error = 0.0
                target_left_pwm = l_pwm
                target_right_pwm = r_pwm

        if use_ramping and use_PID:
            # ensure dt>0 to avoid zero step
            effective_dt = dt
            max_a = max(1e-9, RAMP_RATE_ACC * effective_dt)
            max_d = max(1e-9, RAMP_RATE_DEC * effective_dt)

            def signed_step(curr, tgt, step):
                delta = tgt - curr  # delta is SIGNED
                if abs(delta) > step:
                    return curr + (delta / abs(delta)) * step
                return tgt

            def ramp_one(curr, tgt, switch_mode_list, index):
                # Stop: decelerate to 0 (signed)
                if tgt == 0.0:
                    return signed_step(curr, 0.0, max_d)

                # Sign change: brake to 0 first (only decel case besides stop)
                if curr != 0.0 and (curr * tgt < 0.0):
                    switch_mode_list[index] = True
                    return signed_step(curr, 0.0, max_d)

                # Same sign or starting from 0: ALWAYS accelerate toward signed target
                switch_mode_list[index] = False
                return signed_step(curr, tgt, max_a)

            ramp_left_pwm = ramp_one(ramp_left_pwm, target_left_pwm, switch_mode, 0)
            ramp_right_pwm = ramp_one(ramp_right_pwm, target_right_pwm, switch_mode, 1)
        else:
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm

        final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)
        # print(f"Set motors: L={final_left_pwm:.2f}, R={final_right_pwm:.2f}")
        # if ramp_left_pwm != 0: # print for debugging purpose
        #     print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")

        time.sleep(0.005)


def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start()

    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")

            while running:
                # Capture frame and convert to bytes
                stream = io.BytesIO()
                picam2.capture_file(stream, format="jpeg")
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)

                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break

                # Small delay to avoid hogging CPU
                time.sleep(0.01)

        except Exception as e:
            print(f"Camera stream server error: {str(e)}")

        if "client_socket" in locals() and client_socket:
            client_socket.close()

    server_socket.close()
    picam2.stop()


def pid_config_server():
    global use_PID, KP, KI, KD, rKP, rKI, rKD

    # Create socket for receiving PID configuration
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")

            try:
                # Receive PID constants (7 floats)
                data = client_socket.recv(28)
                if data and len(data) == 28:
                    use_PID, KP, KI, KD, rKP, rKI, rKD = struct.unpack("!fffffff", data)
                    if use_PID:
                        print(
                            f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}, rKP={rKP}, rKI={rKI}, rKD = {rKD}"
                        )
                    else:
                        print("The robot is not using PID.")

                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    # Send failure response
                    response = struct.pack("!i", 0)

                client_socket.sendall(response)

            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass

            client_socket.close()

        except Exception as e:
            print(f"PID config server error: {str(e)}")

    server_socket.close()


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)


def wheel_server():
    global left_pwm, right_pwm, running, sL, sR, dth, ds, packet_id, packet_ready

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Wheel client connected")

            while running:
                try:
                    data = recv_exact(client_socket, 8)

                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    with pwm_lock:
                        left_pwm, right_pwm = (left_speed * 100, right_speed * 100)

                    # Send sL, sR, ds and dth back
                    with encoder_lock:
                        if packet_ready:
                            response = struct.pack("!ffffI", sL, sR, ds, dth, packet_id)
                            packet_ready = False
                            sL = sR = ds = dth = 0.0
                            packet_id += 1
                        else:
                            response = struct.pack(
                                "!ffffI", 0.0, 0.0, 0.0, 0.0, packet_id
                            )
                    client_socket.sendall(response)

                except Exception as e:
                    print(f"Wheel client disconnected")
                    break

        except Exception as e:
            print(f"Wheel server error: {str(e)}")

        if "client_socket" in locals() and client_socket:
            client_socket.close()

    server_socket.close()


def measure_displacement():
    global sL, sR, ds, dth, packet_ready

    baseline = BASELINE
    mPerTick = M_PER_TICK
    signL, signR = 1, 1
    with encoder_lock:
        last_Lc = left_count
        last_Rc = right_count

    while running:
        with pwm_lock:
            signL = sign_L
            signR = sign_R

        with encoder_lock:
            Lc = left_count
            Rc = right_count

        dLc = Lc - last_Lc
        dRc = Rc - last_Rc
        last_Lc = Lc
        last_Rc = Rc

        if dLc > 0 or dRc > 0:
            with encoder_lock:
                if not pb_mode:
                    sL += signL * dLc * mPerTick
                    sR += signR * dRc * mPerTick
                    ds = (sL + sR) / 2
                    dth = (sR - sL) / baseline
                    packet_ready = True

        time.sleep(0.005)


def main():
    try:
        setup_gpio()

        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()

        # Start camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()

        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()

        # Start velocity measurement thread
        velocity_thread = threading.Thread(target=measure_displacement)
        velocity_thread.daemon = True
        velocity_thread.start()

        # Start wheel server (main thread)
        wheel_server()

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
