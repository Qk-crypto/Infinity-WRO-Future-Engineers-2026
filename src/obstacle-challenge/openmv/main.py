# Infinity WRO Future Engineers 2026
# Obstacle Challenge - OpenMV H7 Plus
#
# Hardware:
#   BNO08X UART-RVC TX/SDA -> OpenMV P0 (UART1 RX)
#   OpenMV P4 (UART3 TX)   -> Arduino Uno D10
#   Common GND
#
# UART message:
# Y:<yaw>,D:<C/A/U>,LE:<line_event>,LC:<B/O/N>,
# CE:<cube_event>,EC:<R/G/N>,C:<R/G/N>,S:<-100..100>,H:<height>,X:<cx>

import sensor
import time
from pyb import UART, LED

# ============================================================
# UART
# ============================================================

imu_uart = UART(
    1,
    115200,
    bits=8,
    parity=None,
    stop=1,
    timeout=50,
    timeout_char=10
)

arduino_uart = UART(
    3,
    38400,
    bits=8,
    parity=None,
    stop=1,
    timeout_char=10
)

# ============================================================
# CAMERA
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160 x 120

# Camera is mounted upside down.
sensor.set_vflip(True)
sensor.set_hmirror(True)

# Fixed values are required for stable LAB colour tracking.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=10000)

sensor.set_brightness(0)
sensor.set_contrast(1)
sensor.skip_frames(time=2000)

clock = time.clock()

# ============================================================
# COLOUR THRESHOLDS
#
# Blue/orange values are based on the values already tested
# on this robot. Red/green values are initial values adapted
# from Nerdvana's OpenMV implementation and MUST be calibrated
# on the real WRO pillars under competition lighting.
# ============================================================

BLUE_THRESHOLD = (10, 75, -25, 35, -90, -8)
ORANGE_THRESHOLD = (20, 85, 8, 55, 25, 90)

RED_THRESHOLD = (18, 80, 5, 60, -10, 65)
GREEN_THRESHOLD = (20, 85, -60, -5, -5, 55)

# ============================================================
# ROIs
# ============================================================

LINE_ROI = (0, 68, 160, 52)
CUBE_ROI = (0, 45, 160, 75)

# ============================================================
# OPTIONAL DIRECTION OVERRIDE
#
#  0 = automatic
# +1 = force clockwise for bench testing
# -1 = force counter-clockwise for bench testing
#
# Keep 0 for competition.
# ============================================================

FORCE_DIRECTION = 0

# ============================================================
# LINE DETECTION
# ============================================================

LINE_MIN_PIXELS = 30
LINE_MIN_AREA = 55
LINE_MIN_WIDTH = 28
LINE_MAX_HEIGHT = 28
LINE_MIN_ASPECT = 1.35

DIRECTION_MIN_BOTTOM = 88
LINE_TRIGGER_BOTTOM = 94

DIRECTION_HISTORY_SIZE = 7
DIRECTION_REQUIRED = 4

LINE_HISTORY_SIZE = 5
LINE_REQUIRED = 3

LINE_CLEAR_TIME_MS = 250
LINE_EVENT_COOLDOWN_MS = 850

direction_history = [0] * DIRECTION_HISTORY_SIZE
direction_history_index = 0

line_history = [0] * LINE_HISTORY_SIZE
line_history_index = 0

# U = unknown, C = clockwise, A = anticlockwise
locked_direction = "U"

line_event_counter = 0
line_event_armed = True
line_clear_started = 0
last_line_event_time = 0

# ============================================================
# CUBE DETECTION
# ============================================================

CUBE_MIN_PIXELS = 35
CUBE_MIN_AREA = 40
CUBE_MIN_HEIGHT = 4
CUBE_MIN_DENSITY = 0.48

# Initial proximity thresholds. Tune from camera logs.
RED_CLOSE_HEIGHT = 18
GREEN_CLOSE_HEIGHT = 16
CUBE_CLOSE_BOTTOM = 91

CUBE_CLOSE_REQUIRED = 3
CUBE_CLEAR_TIME_MS = 450
CUBE_EVENT_COOLDOWN_MS = 900

# Camera steering output.
# Positive output means steer right on Arduino.
CAMERA_KP = 0.010
CAMERA_KD = 0.020
CAMERA_STEER_LIMIT = 0.75

previous_cube_error = 0.0
cube_close_score = 0
cube_event_counter = 0
cube_event_color = "N"
cube_event_armed = True
cube_clear_started = 0
last_cube_event_time = 0

# ============================================================
# IMU
# ============================================================

imu_buffer = bytearray()
latest_yaw = 0.0
yaw_available = False
imu_packet_count = 0
imu_checksum_errors = 0

# ============================================================
# TIMERS
# ============================================================

last_send_time = 0
last_print_time = 0

# ============================================================
# HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def signed_int16(low_byte, high_byte):
    value = low_byte | (high_byte << 8)
    if value & 0x8000:
        value -= 65536
    return value


def read_rvc_packet():
    global imu_buffer
    global imu_checksum_errors

    available = imu_uart.any()
    if available:
        data = imu_uart.read(available)
        if data:
            imu_buffer.extend(data)

    while len(imu_buffer) >= 2:
        header_position = -1

        for i in range(len(imu_buffer) - 1):
            if imu_buffer[i] == 0xAA and imu_buffer[i + 1] == 0xAA:
                header_position = i
                break

        if header_position < 0:
            if len(imu_buffer) and imu_buffer[-1] == 0xAA:
                imu_buffer = bytearray([0xAA])
            else:
                imu_buffer = bytearray()
            return None

        if header_position > 0:
            imu_buffer = bytearray(imu_buffer[header_position:])

        if len(imu_buffer) < 19:
            return None

        packet = imu_buffer[0:19]
        calculated_checksum = sum(packet[2:18]) & 0xFF
        received_checksum = packet[18]

        if calculated_checksum != received_checksum:
            imu_checksum_errors += 1
            imu_buffer = bytearray(imu_buffer[1:])
            continue

        imu_buffer = bytearray(imu_buffer[19:])

        yaw_raw = signed_int16(packet[3], packet[4])
        pitch_raw = signed_int16(packet[5], packet[6])
        roll_raw = signed_int16(packet[7], packet[8])

        return (
            yaw_raw / 100.0,
            pitch_raw / 100.0,
            roll_raw / 100.0
        )

    return None


def largest_blob(blobs):
    best = None
    for blob in blobs:
        if best is None or blob.area() > best.area():
            best = blob
    return best


def valid_line_blob(blob):
    if blob is None:
        return False
    if blob.w() < LINE_MIN_WIDTH:
        return False
    if blob.h() <= 0 or blob.h() > LINE_MAX_HEIGHT:
        return False
    if (blob.w() / blob.h()) < LINE_MIN_ASPECT:
        return False
    return True


def best_line_blob(img, threshold):
    blobs = img.find_blobs(
        [threshold],
        roi=LINE_ROI,
        pixels_threshold=LINE_MIN_PIXELS,
        area_threshold=LINE_MIN_AREA,
        merge=True,
        margin=5
    )

    best = None
    for blob in blobs:
        if not valid_line_blob(blob):
            continue
        if best is None or blob.pixels() > best.pixels():
            best = blob
    return best


def nearest_line(blue_blob, orange_blob):
    if blue_blob is None and orange_blob is None:
        return "N", None

    if blue_blob is not None and orange_blob is None:
        return "B", blue_blob

    if orange_blob is not None and blue_blob is None:
        return "O", orange_blob

    blue_bottom = blue_blob.y() + blue_blob.h()
    orange_bottom = orange_blob.y() + orange_blob.h()

    if blue_bottom > orange_bottom:
        return "B", blue_blob
    if orange_bottom > blue_bottom:
        return "O", orange_blob

    if blue_blob.pixels() >= orange_blob.pixels():
        return "B", blue_blob
    return "O", orange_blob


def center_inside_blob(inner_blob, outer_blob):
    if inner_blob is None or outer_blob is None:
        return False

    x = inner_blob.cx()
    y = inner_blob.cy()

    return (
        x >= outer_blob.x() and
        x <= outer_blob.x() + outer_blob.w() and
        y >= outer_blob.y() and
        y <= outer_blob.y() + outer_blob.h()
    )


def valid_cube_blob(blob, confusing_line_blob):
    if blob is None:
        return False
    if blob.h() < CUBE_MIN_HEIGHT:
        return False
    if blob.density() < CUBE_MIN_DENSITY:
        return False
    if center_inside_blob(blob, confusing_line_blob):
        return False
    return True


def best_cube_blob(img, threshold, confusing_line_blob):
    blobs = img.find_blobs(
        [threshold],
        roi=CUBE_ROI,
        pixels_threshold=CUBE_MIN_PIXELS,
        area_threshold=CUBE_MIN_AREA,
        merge=True,
        margin=4
    )

    best = None
    for blob in blobs:
        if not valid_cube_blob(blob, confusing_line_blob):
            continue
        if best is None or blob.area() > best.area():
            best = blob
    return best


# ============================================================
# STARTUP LED
# ============================================================

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

green_led.on()
blue_led.on()
time.sleep_ms(350)
green_led.off()
blue_led.off()

if FORCE_DIRECTION > 0:
    locked_direction = "C"
elif FORCE_DIRECTION < 0:
    locked_direction = "A"

print("INFINITY OBSTACLE CAMERA READY")
print("RED pillar: pass on RIGHT")
print("GREEN pillar: pass on LEFT")

# ============================================================
# MAIN LOOP
# ============================================================

while True:
    clock.tick()
    now = time.ticks_ms()

    # --------------------------------------------------------
    # IMU
    # --------------------------------------------------------

    while True:
        imu_result = read_rvc_packet()
        if imu_result is None:
            break

        latest_yaw = imu_result[0]
        yaw_available = True
        imu_packet_count += 1

    # --------------------------------------------------------
    # CAMERA FRAME
    # --------------------------------------------------------

    img = sensor.snapshot()

    blue_blob = best_line_blob(img, BLUE_THRESHOLD)
    orange_blob = best_line_blob(img, ORANGE_THRESHOLD)
    line_color, line_blob = nearest_line(blue_blob, orange_blob)

    # --------------------------------------------------------
    # DIRECTION LOCK
    # --------------------------------------------------------

    if locked_direction == "U":
        direction_vote = 0

        if line_blob is not None:
            line_bottom = line_blob.y() + line_blob.h()

            if line_bottom >= DIRECTION_MIN_BOTTOM:
                if line_color == "B":
                    direction_vote = 1
                elif line_color == "O":
                    direction_vote = -1

        direction_history[direction_history_index] = direction_vote
        direction_history_index += 1
        if direction_history_index >= DIRECTION_HISTORY_SIZE:
            direction_history_index = 0

        blue_votes = 0
        orange_votes = 0

        for vote in direction_history:
            if vote == 1:
                blue_votes += 1
            elif vote == -1:
                orange_votes += 1

        if blue_votes >= DIRECTION_REQUIRED:
            locked_direction = "C"
            print("DIRECTION LOCKED: CLOCKWISE")
        elif orange_votes >= DIRECTION_REQUIRED:
            locked_direction = "A"
            print("DIRECTION LOCKED: ANTICLOCKWISE")

    # --------------------------------------------------------
    # LINE EVENT
    # --------------------------------------------------------

    line_visible = False

    if line_blob is not None:
        line_bottom = line_blob.y() + line_blob.h()
        line_visible = line_bottom >= LINE_TRIGGER_BOTTOM

    line_history[line_history_index] = 1 if line_visible else 0
    line_history_index += 1
    if line_history_index >= LINE_HISTORY_SIZE:
        line_history_index = 0

    line_score = sum(line_history)

    if not line_visible:
        if line_clear_started == 0:
            line_clear_started = now

        if time.ticks_diff(now, line_clear_started) >= LINE_CLEAR_TIME_MS:
            line_event_armed = True
    else:
        line_clear_started = 0

    if (
        line_event_armed and
        locked_direction != "U" and
        line_score >= LINE_REQUIRED and
        (
            last_line_event_time == 0 or
            time.ticks_diff(now, last_line_event_time) >= LINE_EVENT_COOLDOWN_MS
        )
    ):
        line_event_counter += 1
        last_line_event_time = now
        line_event_armed = False

        for i in range(LINE_HISTORY_SIZE):
            line_history[i] = 0

        print(
            "LINE EVENT:%d COLOR:%s DIR:%s"
            % (line_event_counter, line_color, locked_direction)
        )

    # --------------------------------------------------------
    # CUBE DETECTION
    #
    # Red can be confused with orange line.
    # Green can be confused with blue line.
    # --------------------------------------------------------

    red_blob = best_cube_blob(img, RED_THRESHOLD, orange_blob)
    green_blob = best_cube_blob(img, GREEN_THRESHOLD, blue_blob)

    cube_blob = None
    cube_color = "N"

    if red_blob is not None:
        cube_blob = red_blob
        cube_color = "R"

    if green_blob is not None:
        if cube_blob is None or green_blob.area() > cube_blob.area():
            cube_blob = green_blob
            cube_color = "G"

    cube_x = 0
    cube_h = 0
    cube_steer = 0
    cube_close = False

    if cube_blob is not None:
        cube_x = cube_blob.cx()
        cube_h = cube_blob.h()
        cube_bottom = cube_blob.y() + cube_blob.h()

        # Slight green target offset follows Nerdvana's idea.
        target_x = 80 if cube_color == "R" else 75

        cube_error = cube_x - target_x
        derivative = cube_error - previous_cube_error
        previous_cube_error = cube_error

        steering = (
            cube_error * CAMERA_KP +
            derivative * CAMERA_KD
        )
        steering = clamp(
            steering,
            -CAMERA_STEER_LIMIT,
            CAMERA_STEER_LIMIT
        )

        cube_steer = int(steering * 100)

        close_height = (
            RED_CLOSE_HEIGHT
            if cube_color == "R"
            else GREEN_CLOSE_HEIGHT
        )

        cube_close = (
            cube_h >= close_height and
            cube_bottom >= CUBE_CLOSE_BOTTOM
        )
    else:
        previous_cube_error = 0.0

    if cube_close:
        cube_close_score += 1
    else:
        cube_close_score = 0

    if cube_blob is None:
        if cube_clear_started == 0:
            cube_clear_started = now

        if time.ticks_diff(now, cube_clear_started) >= CUBE_CLEAR_TIME_MS:
            cube_event_armed = True
    else:
        cube_clear_started = 0

    if (
        cube_event_armed and
        cube_close_score >= CUBE_CLOSE_REQUIRED and
        (
            last_cube_event_time == 0 or
            time.ticks_diff(now, last_cube_event_time) >= CUBE_EVENT_COOLDOWN_MS
        )
    ):
        cube_event_counter += 1
        cube_event_color = cube_color
        cube_event_armed = False
        last_cube_event_time = now
        cube_close_score = 0

        print(
            "CUBE EVENT:%d COLOR:%s H:%d X:%d"
            % (cube_event_counter, cube_event_color, cube_h, cube_x)
        )

    # --------------------------------------------------------
    # DEBUG DRAWING
    # --------------------------------------------------------

    img.draw_rectangle(LINE_ROI, color=(255, 255, 0))
    img.draw_rectangle(CUBE_ROI, color=(255, 255, 255))

    if line_blob is not None:
        img.draw_rectangle(line_blob.rect(), color=(0, 0, 255))

    if cube_blob is not None:
        draw_color = (255, 0, 0) if cube_color == "R" else (0, 255, 0)
        img.draw_rectangle(cube_blob.rect(), color=draw_color)
        img.draw_cross(cube_blob.cx(), cube_blob.cy(), color=draw_color)

    # --------------------------------------------------------
    # SEND ONE STRUCTURED MESSAGE TO ARDUINO
    # --------------------------------------------------------

    if (
        yaw_available and
        time.ticks_diff(now, last_send_time) >= 40
    ):
        last_send_time = now

        message = (
            "Y:%.2f,D:%s,LE:%d,LC:%s,"
            "CE:%d,EC:%s,C:%s,S:%d,H:%d,X:%d\n"
            % (
                latest_yaw,
                locked_direction,
                line_event_counter,
                line_color,
                cube_event_counter,
                cube_event_color,
                cube_color,
                cube_steer,
                cube_h,
                cube_x
            )
        )

        arduino_uart.write(message)

    # --------------------------------------------------------
    # SERIAL DEBUG
    # --------------------------------------------------------

    if time.ticks_diff(now, last_print_time) >= 150:
        last_print_time = now

        print(
            "Y:%7.2f D:%s LE:%d L:%s "
            "CE:%d EC:%s C:%s S:%d H:%d X:%d "
            "FPS:%.1f ERR:%d"
            % (
                latest_yaw,
                locked_direction,
                line_event_counter,
                line_color,
                cube_event_counter,
                cube_event_color,
                cube_color,
                cube_steer,
                cube_h,
                cube_x,
                clock.fps(),
                imu_checksum_errors
            )
        )

    time.sleep_ms(1)
