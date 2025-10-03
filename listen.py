import socket
import struct
import io
import threading
import time
import math
from time import monotonic
from collections import deque
import RPi.GPIO as GPIO  # type: ignore
from picamera2 import Picamera2  # type: ignore

# Network Configuration
HOST = "0.0.0.0"
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0
KP, Ki, KD, rKP, rKI, rKD = 0, 0, 0, 0, 0, 0
MAX_CORRECTION = 80  # Maximum PWM correction value
MAX_INTEGRAL = MAX_CORRECTION

# Global variables
WINDOW_SIZE = 5
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
omegaL_f, omegaR_f = 0.0, 0.0
omegaL_q = deque(maxlen=WINDOW_SIZE)
omegaR_q = deque(maxlen=WINDOW_SIZE)
vL_f, vR_f = 0.0, 0.0
Wq = deque(maxlen=WINDOW_SIZE)
Vq = deque(maxlen=WINDOW_SIZE)
W, V = 0.0, 0.0
last_L = 0
last_R = 0
last_time = None
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE_ACC = 180  # PWM units per second (adjust this value to tune ramp speed)
RAMP_RATE_DEC = 180
RIGHT_WHEEL_OFFSET = 1  # 5% boost for weaker wheel
MIN_RAMP_THRESHOLD = 30  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 30
current_movement, prev_movement = "stop", "stop"

# locks
encoder_lock = threading.Lock()
pwm_lock = threading.Lock()
movement_lock = threading.Lock()


def clamp(x: int, minimum: int, maximum: int):
    X = max(minimum, min(x, maximum))
    return X


def setup_gpio():
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

    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
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

    elif prev_left_state is None:
        # First reading
        prev_left_state = current_state


def right_encoder_callback(channel):
    global right_count, prev_right_state, prev_right_time
    current_state = GPIO.input(RIGHT_ENCODER)

    if prev_right_state is not None and current_state != prev_right_state:
        with encoder_lock:
            right_count += 1
        prev_right_state = current_state

    elif prev_right_state is None:
        prev_right_state = current_state


def reset_encoder():
    global left_count, right_count
    with encoder_lock:
        left_count, right_count = 0, 0


def set_motors(left, right):
    global prev_movement, current_movement
    with movement_lock:
        p_movement = prev_movement
        c_movement = current_movement

    # Pre-Start Kick (Motor Priming), to reduce initial jerk and slight orientation change
    if p_movement == "stop" and c_movement in ["forward", "backward"]:
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
        time.sleep(0.05)

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
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)

    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)


def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        # Boost small values to minimum threshold, preserving direction
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value


def pid_control():
    # Only applies for forward/backward, not turning
    global left_pwm, right_pwm, use_PID, KP, KI, KD, rKP, rKI, rKD, prev_movement, current_movement
    with pwm_lock:
        l_pwm = left_pwm
        r_pwm = right_pwm

    integral_rotation_left = 0.0
    integral_rotation_right = 0.0
    integral_linear_forward = 0.0
    integral_linear_back = 0.0
    last_error_rotation = 0.0
    last_error_linear = 0.0
    last_time = monotonic()
    last_derivative_linear = 0.0
    last_derivative_rotation = 0.0
    alpha = 0.1
    switch_mode = [False, False]  # [left, right]

    # Ramping variables & params
    ramp_left_pwm = 0
    ramp_right_pwm = 0

    while running:
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time

        prev_movement = current_movement

        if l_pwm > 0 and r_pwm > 0:
            requested_movement = "forward"
        elif l_pwm < 0 and r_pwm < 0:
            requested_movement = "backward"
        elif l_pwm == 0 and r_pwm == 0:
            requested_movement = "stop"
        elif l_pwm < 0 and r_pwm > 0:
            requested_movement = "rotate_left"
        else:
            requested_movement = "rotate_right"

        # Only switch once both ramps have finished braking/accelerating
        if not any(switch_mode):  # means [False, False]
            current_movement = requested_movement
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

                if current_movement in ["forward", "backward"]:
                    with encoder_lock:
                        error = omegaL_f - omegaR_f
                    # print(f"linear! leftV {left_v}, rightV{right_v}")
                    derivative = KD * (error - last_error_linear) / dt if dt > 0 else 0
                    derivative = (
                        alpha * last_derivative_linear + (1 - alpha) * derivative
                    )
                    last_derivative_linear = derivative
                    if current_movement == "forward":
                        integral_linear_forward += KI * error * dt
                        integral_linear_forward = clamp(
                            integral_linear_forward, -MAX_INTEGRAL, MAX_INTEGRAL
                        )
                    else:
                        integral_linear_back += KI * error * dt
                        integral_linear_back = clamp(
                            integral_linear_back, -MAX_INTEGRAL, MAX_INTEGRAL
                        )

                    I = (
                        integral_linear_forward
                        if current_movement == "forward"
                        else integral_linear_back
                    )
                    last_error_linear = error
                    proportional = KP * error
                    correction = clamp(
                        proportional + I + derivative, -MAX_CORRECTION, MAX_CORRECTION
                    )
                else:
                    error = omegaL_f + omegaR_f
                    # print(f"Rotation! leftV {left_v}, rightV{right_v}")
                    derivative = (
                        rKD * (error - last_error_rotation) / dt if dt > 0 else 0
                    )
                    derivative = (
                        alpha * last_derivative_rotation + (1 - alpha) * derivative
                    )
                    last_derivative_rotation = derivative
                    if current_movement == "rotate_left":
                        integral_rotation_left += rKI * error * dt
                        integral_rotation_left = clamp(
                            integral_rotation_left, -MAX_INTEGRAL, MAX_INTEGRAL
                        )
                    else:
                        integral_rotation_right += rKI * error * dt
                        integral_rotation_right = clamp(
                            integral_rotation_right, -MAX_INTEGRAL, MAX_INTEGRAL
                        )

                    I = (
                        integral_rotation_left
                        if current_movement == "rotate_left"
                        else integral_rotation_right
                    )
                    last_error_rotation = error
                    proportional = rKP * error
                    correction = clamp(
                        proportional + I + derivative, -MAX_CORRECTION, MAX_CORRECTION
                    )

                if current_movement in ["forward", "backward"]:
                    target_left_pwm = l_pwm - correction
                    target_right_pwm = r_pwm + correction
                elif current_movement in ["rotate_left", "rotate_right"]:
                    target_left_pwm = l_pwm - correction
                    target_right_pwm = r_pwm - correction
            else:
                # Reset when stopped
                # integral_linear_forward = 0
                # integral_linear_back = 0
                # integral_rotation_right = 0
                # integral_rotation_left = 0
                last_error_linear = 0
                last_error_rotation = 0
                reset_encoder()
                target_left_pwm = l_pwm
                target_right_pwm = r_pwm
                # print(f"Stopped! leftV {left_v}, rightV{right_v}")

        # (Optional for debugging symmetry: disable offset first)
        # target_right_pwm *= RIGHT_WHEEL_OFFSET

        if use_ramping and use_PID:
            # ensure dt>0 to avoid zero step
            max_a = max(1e-9, RAMP_RATE_ACC * dt)  # accel step (>0)
            max_d = max(1e-9, RAMP_RATE_DEC * dt)  # decel step (>0)

            def signed_step(curr, tgt, step):
                delta = tgt - curr  # delta is SIGNED
                if abs(delta) > step:
                    return curr + (delta / abs(delta)) * step
                return tgt

            def ramp_one(curr, tgt, switch_mode_list, index):
                # Stop: decelerate to 0 (signed)
                if tgt == 0.0:
                    switch_mode_list[index] = True if curr != 0.0 else False
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
        # if ramp_left_pwm != 0: # print for debugging purpose
        #     print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")

        # with encoder_lock:
        #     print(
        #         f"left_count={left_count}, right_count={right_count}, diff={left_count-right_count}, correction={correction if use_PID else 0}"
        #     )
        time.sleep(0.01)


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


def wheel_server():
    global left_pwm, right_pwm, running, vL_f, vR_f, V, W

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")

            while running:
                try:
                    # Receive speed (4 bytes for each value)
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break

                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    # print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
                    with pwm_lock:
                        left_pwm, right_pwm = (
                            left_speed * 100,
                            right_speed * 100 * RIGHT_WHEEL_OFFSET,
                        )

                    # Send vL_f, vR_f, V, W back
                    with encoder_lock:
                        response = struct.pack("!ffff", vL_f, vR_f, V, W)
                    client_socket.sendall(response)
                    time.sleep(0.001)

                except Exception as e:
                    print(f"Wheel client disconnected")
                    break

        except Exception as e:
            print(f"Wheel server error: {str(e)}")

        if "client_socket" in locals() and client_socket:
            client_socket.close()

    server_socket.close()


def measure_velocities():
    global last_L, last_R, last_time, omegaL_f, omegaR_f, vL_f, vR_f, V, W

    ticks_per_rev = 20
    r = 0.033
    alpha = 0.1  # tau = T/alpha -> 0.005/0.1 = 50ms
    max_omega = 2  # Big jump protection
    baseline = 0.115
    time2stop = 0.5

    while running:
        with pwm_lock:
            sL = left_pwm / abs(left_pwm) if left_pwm != 0 else 0
            sR = right_pwm / abs(right_pwm) if right_pwm != 0 else 0

        with encoder_lock:
            L = left_count
            R = right_count

        now = time.monotonic()
        if last_L is None or last_R is None or last_time is None:
            last_L = L
            last_R = R
            last_time = now
            return

        dt = now - last_time
        last_time = now
        dL = L - last_L
        dR = R - last_R
        last_L = L
        last_R = R

        if dL != 0:
            omegaL = sL * 2 * math.pi * (dL / ticks_per_rev) / dt
            last_tick_L = now
        else:
            t_since = now - last_tick_L
            if t_since < time2stop:
                omegaL = sL * 2 * math.pi / (ticks_per_rev * t_since)
            else:
                omegaL = 0.0

        if dR != 0:
            omegaR = sR * 2 * math.pi * (dR / ticks_per_rev) / dt
            last_tick_R = now
        else:
            t_since = now - last_tick_R
            if t_since < time2stop:
                omegaR = sR * 2 * math.pi / (ticks_per_rev * t_since)
            else:
                omegaR = 0.0

        # This is ok since big jump can only occur for one cycle
        if abs(omegaL) > max_omega:
            omegaL = 0.0
        if abs(omegaR) > max_omega:
            omegaR = 0.0

        vL = omegaL * r
        vR = omegaR * r
        # omegaL_q.append(omegaL)
        # omegaR_q.append(omegaR)

        Vq.append((vL + vR) / 2)
        Wq.append((vR - vL) / baseline)

        with encoder_lock:
            # omegaL_f = sum(omegaL_q)/len(omegaL_q) if len(omegaL_q) > 0 else 0.0
            # omegaR_f = sum(omegaR_q)/len(omegaR_q) if len(omegaR_q) > 0 else 0.0
            # V = sum(Vq)/len(Vq) if len(Vq) > 0 else 0.0
            # W = sum(Wq)/len(Wq) if len(Wq) > 0 else 0.0
            V = (1 - alpha) * V + alpha * ((vL + vR) / 2)
            W = (1 - alpha) * W + alpha * ((vR - vL) / baseline)
            omegaL_f = (1 - alpha) * omegaL_f + alpha * omegaL
            omegaR_f = (1 - alpha) * omegaR_f + alpha * omegaR
            vL_f = omegaL_f * r
            vR_f = omegaR_f * r

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
        velocity_thread = threading.Thread(target=measure_velocities)
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
