
import math
import threading
import time

import pygame
import serial

# ============ SETTINGS ============
PORT = "COM11"
BAUD = 9600

MAX_DISTANCE = 100
WARNING_DISTANCE = 40
DANGER_DISTANCE = 20

# Object tracking
MAX_MATCH_DISTANCE = 25
MAX_MISSED_SWEEPS = 2
MIN_CONFIRMATION_SWEEPS = 2

# Stricter noise filtering
MIN_GROUP_READINGS = 3
MAX_GROUP_ANGLE_GAP = 4
MAX_GROUP_DISTANCE_GAP = 12

# Approach-speed warning
APPROACH_SPEED_WARNING = 12  # cm/second
SPEED_SMOOTHING = 0.35

WIDTH = 1000
HEIGHT = 720
FPS = 60
# =================================

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mini Radar System - Object Tracking")
clock = pygame.time.Clock()

FONT = pygame.font.SysFont("consolas", 19)
SMALL = pygame.font.SysFont("consolas", 15)
BIG = pygame.font.SysFont("consolas", 26, bold=True)

BLACK = (5, 12, 18)
GREEN = (40, 230, 100)
DARK_GREEN = (20, 90, 50)
WHITE = (240, 240, 240)
YELLOW = (255, 210, 40)
RED = (255, 55, 55)
CYAN = (60, 220, 240)
GREY = (140, 150, 155)

CX = WIDTH // 2
CY = HEIGHT - 65
RADIUS = 455

scan_points = {}
angle_now = 90
distance_now = -1
status = "Connecting..."
connection = None
running = True

try:
    import winsound
except ImportError:
    winsound = None

siren_level = 0


# ============ SIREN ============

def siren():
    while True:
        level = siren_level

        if winsound is None or level == 0:
            time.sleep(0.1)
            continue

        try:
            if level == 2:
                winsound.Beep(1400, 180)
                time.sleep(0.05)
                winsound.Beep(900, 180)
            else:
                winsound.Beep(800, 120)
                time.sleep(0.5)
        except Exception:
            time.sleep(0.2)


threading.Thread(target=siren, daemon=True).start()


# ============ GROUP SENSOR READINGS ============

def group_obstacles(data):
    readings = sorted(data.items())
    groups = []
    current = []

    for angle, distance in readings:
        if not 2 <= distance <= MAX_DISTANCE:
            continue

        if not current:
            current = [(angle, distance)]
            continue

        old_angle, old_distance = current[-1]

        if (
            angle - old_angle <= MAX_GROUP_ANGLE_GAP
            and abs(distance - old_distance)
            <= MAX_GROUP_DISTANCE_GAP
        ):
            current.append((angle, distance))
        else:
            if len(current) >= MIN_GROUP_READINGS:
                groups.append(current)

            current = [(angle, distance)]

    if len(current) >= MIN_GROUP_READINGS:
        groups.append(current)

    obstacles = []

    for group in groups:
        avg_angle = sum(p[0] for p in group) / len(group)
        avg_distance = sum(p[1] for p in group) / len(group)

        obstacles.append({
            "angle": avg_angle,
            "distance": avg_distance
        })

    return obstacles


# ============ MULTIPLE-OBJECT TRACKING ============

targets = []
next_target_id = 1


def polar_to_xy(angle, distance):
    radians = math.radians(angle)

    return (
        distance * math.cos(radians),
        distance * math.sin(radians)
    )


def update_tracking(detections):
    global targets, next_target_id

    now = time.monotonic()

    for obj in detections:
        obj["x"], obj["y"] = polar_to_xy(
            obj["angle"], obj["distance"]
        )

    # Find possible matches between detections and tracked targets.
    candidates = []

    for ti, target in enumerate(targets):
        for di, obj in enumerate(detections):
            separation = math.hypot(
                target["x"] - obj["x"],
                target["y"] - obj["y"]
            )

            if separation <= MAX_MATCH_DISTANCE:
                candidates.append((separation, ti, di))

    candidates.sort()

    matched_targets = set()
    matched_detections = set()

    # Match each target and detection at most once.
    for _, ti, di in candidates:
        if ti in matched_targets or di in matched_detections:
            continue

        target = targets[ti]
        obj = detections[di]

        dt = now - target["last_update"]

        if dt >= 0.15:
            measured_speed = (
                target["distance"] - obj["distance"]
            ) / dt

            target["speed"] = (
                SPEED_SMOOTHING * measured_speed
                + (1 - SPEED_SMOOTHING) * target["speed"]
            )

        target["angle"] = obj["angle"]
        target["distance"] = obj["distance"]
        target["x"] = obj["x"]
        target["y"] = obj["y"]
        target["last_update"] = now
        target["missed"] = 0

        # Count consecutive successful detections.
        target["hits"] += 1

        matched_targets.add(ti)
        matched_detections.add(di)

    # Age targets missing from this sweep.
    for ti, target in enumerate(targets):
        if ti not in matched_targets:
            target["missed"] += 1
            target["hits"] = 0
            target["speed"] *= 0.5

    # Remove targets that have disappeared.
    targets = [
        t for t in targets
        if t["missed"] <= MAX_MISSED_SWEEPS
    ]

    # Create tentative tracks for new detections.
    for di, obj in enumerate(detections):
        if di in matched_detections:
            continue

        targets.append({
            "id": next_target_id,
            "angle": obj["angle"],
            "distance": obj["distance"],
            "x": obj["x"],
            "y": obj["y"],
            "speed": 0.0,
            "last_update": now,
            "missed": 0,
            "hits": 1
        })

        next_target_id += 1

    # Only show targets confirmed in multiple sweeps.
    return [
        t for t in targets
        if t["hits"] >= MIN_CONFIRMATION_SWEEPS
        and t["missed"] == 0
    ]


# ============ RADAR DISPLAY ============

def radar_point(angle, distance):
    r = min(distance / MAX_DISTANCE, 1) * RADIUS
    a = math.radians(angle)

    x = CX + r * math.cos(a)
    y = CY - r * math.sin(a)

    return int(x), int(y)


def text(message, x, y, color=WHITE, font=FONT):
    image = font.render(str(message), True, color)
    screen.blit(image, (x, y))


def draw_radar(confirmed_targets):
    screen.fill(BLACK)

    text("MINI RADAR SYSTEM", 25, 15, GREEN, BIG)
    text(f"Angle: {angle_now} degrees", 25, 55)

    text(
        "Distance: "
        + (
            f"{distance_now:.1f} cm"
            if distance_now >= 0
            else "No echo"
        ),
        25, 83
    )

    # Range semicircles
    for d in (20, 40, 60, 80, 100):
        r = int(RADIUS * d / MAX_DISTANCE)

        rect = pygame.Rect(
            CX - r, CY - r, 2 * r, 2 * r
        )

        pygame.draw.arc(
            screen, DARK_GREEN, rect,
            math.pi, 2 * math.pi, 2
        )

        text(
            f"{d}cm", CX + 8, CY - r - 18,
            GREY, SMALL
        )

    # Angle lines
    for a in range(0, 181, 30):
        x, y = radar_point(a, MAX_DISTANCE)

        pygame.draw.line(
            screen, DARK_GREEN, (CX, CY), (x, y), 1
        )

        text(f"{a}°", x - 12, y - 20, GREY, SMALL)

    pygame.draw.line(
        screen, DARK_GREEN,
        (CX - RADIUS, CY), (CX + RADIUS, CY), 2
    )

    # Latest sensor readings
    for a, d in scan_points.items():
        if 2 <= d <= MAX_DISTANCE:
            x, y = radar_point(a, d)
            pygame.draw.circle(screen, GREEN, (x, y), 3)

    # Scanner line
    x, y = radar_point(angle_now, MAX_DISTANCE)

    pygame.draw.line(
        screen, CYAN, (CX, CY), (x, y), 3
    )

    # Confirmed targets with persistent IDs
    for target in confirmed_targets:
        d = target["distance"]
        a = target["angle"]
        speed = target["speed"]

        if d <= DANGER_DISTANCE:
            color = RED
        elif (
            d <= WARNING_DISTANCE
            or speed >= APPROACH_SPEED_WARNING
        ):
            color = YELLOW
        else:
            color = GREEN

        x, y = radar_point(a, d)

        pygame.draw.circle(screen, color, (x, y), 8)
        pygame.draw.circle(screen, WHITE, (x, y), 9, 1)

        if speed >= 2:
            motion = "APPROACH"
        elif speed <= -2:
            motion = "RECEDING"
        else:
            motion = "STEADY"

        text(
            f"O{target['id']} {d:.0f}cm",
            x + 10, y - 24, color, SMALL
        )

        text(
            f"{speed:+.1f}cm/s {motion}",
            x + 10, y - 7, color, SMALL
        )

    text(
        f"Confirmed objects: {len(confirmed_targets)}",
        25, 125, WHITE
    )

    y_info = 155

    for target in confirmed_targets:
        d = target["distance"]
        speed = target["speed"]

        if d <= DANGER_DISTANCE:
            state, color = "CLOSE", RED
        elif speed >= APPROACH_SPEED_WARNING:
            state, color = "APPROACHING FAST", YELLOW
        elif d <= WARNING_DISTANCE:
            state, color = "NEARBY", YELLOW
        elif speed <= -2:
            state, color = "MOVING AWAY", GREEN
        else:
            state, color = "TRACKING", GREEN

        text(
            f"O{target['id']}: Angle {target['angle']:.0f}° | "
            f"{d:.1f} cm | {speed:+.1f} cm/s | {state}",
            25, y_info, color, SMALL
        )

        y_info += 22

        if y_info > CY - 10:
            break

    text("Serial: " + status, 25, HEIGHT - 30, GREEN, SMALL)
    pygame.display.flip()


# ============ MAIN PROGRAM ============

def main():
    global running, angle_now, distance_now
    global connection, status, siren_level

    try:
        connection = serial.Serial(PORT, BAUD, timeout=0.05)
        time.sleep(2)
        connection.reset_input_buffer()
        status = f"Connected to {PORT}"

    except serial.SerialException as error:
        connection = None
        status = f"Connection failed: {error}"

    sweep_readings = {}
    previous_angle = None
    previous_direction = 0

    confirmed_targets = []

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if (
                event.type == pygame.KEYDOWN
                and event.key == pygame.K_ESCAPE
            ):
                running = False

        if connection is not None:
            try:
                for _ in range(100):
                    if connection.in_waiting <= 0:
                        break

                    line = connection.readline().decode(
                        "utf-8", errors="ignore"
                    ).strip()

                    parts = line.split(",")

                    if len(parts) != 2:
                        continue

                    try:
                        a = int(float(parts[0]))
                        d = float(parts[1])
                    except ValueError:
                        continue

                    if not 0 <= a <= 180:
                        continue

                    angle_now = a
                    distance_now = d

                    # Detect the servo reversing direction.
                    direction = 0

                    if previous_angle is not None:
                        delta = a - previous_angle

                        if delta > 0:
                            direction = 1
                        elif delta < 0:
                            direction = -1

                    if (
                        direction != 0
                        and previous_direction != 0
                        and direction != previous_direction
                    ):
                        # Finish processing the previous sweep.
                        obstacles = group_obstacles(
                            dict(sweep_readings)
                        )

                        confirmed_targets = update_tracking(
                            obstacles
                        )

                        sweep_readings.clear()

                    if direction != 0:
                        previous_direction = direction

                    previous_angle = a

                    if 2 <= d <= MAX_DISTANCE:
                        scan_points[a] = d
                        sweep_readings[a] = d
                    else:
                        scan_points.pop(a, None)
                        sweep_readings.pop(a, None)

            except (serial.SerialException, OSError) as error:
                status = f"Serial error: {error}"
                connection = None

        # Siren uses confirmed targets only.
        closest = min(
            (t["distance"] for t in confirmed_targets),
            default=float("inf")
        )

        fast_approach = any(
            t["speed"] >= APPROACH_SPEED_WARNING
            for t in confirmed_targets
        )

        if closest <= DANGER_DISTANCE:
            siren_level = 2
        elif closest <= WARNING_DISTANCE or fast_approach:
            siren_level = 1
        else:
            siren_level = 0

        draw_radar(confirmed_targets)
        clock.tick(FPS)

    siren_level = 0

    if connection is not None:
        connection.close()

    pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        siren_level = 0
        pygame.quit()
