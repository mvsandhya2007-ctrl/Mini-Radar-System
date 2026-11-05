
import math
import time
from dataclasses import dataclass

# Tune these after testing with real objects.
MAX_DISTANCE_CM = 100
MAX_MATCH_DISTANCE_CM = 25
MAX_MISSED_SCANS = 3
MIN_SPEED_INTERVAL_S = 0.15
SPEED_SMOOTHING = 0.4


@dataclass
class Target:
    target_id: int
    x: float
    y: float
    distance: float
    angle: float
    radial_speed: float = 0.0
    missed_scans: int = 0
    last_update: float = 0.0


class RadarTracker:
    def __init__(self):
        self.targets = []
        self.next_id = 1

    @staticmethod
    def xy(angle, distance):
        radians = math.radians(angle)
        return (
            distance * math.cos(radians),
            distance * math.sin(radians)
        )

    def update(self, readings):
        """
        readings: list of (angle_degrees, distance_cm)
        Call once after each complete servo sweep.
        """
        now = time.monotonic()

        # Convert this sweep into Cartesian points.
        points = []
        for angle, distance in readings:
            if 2 <= distance <= MAX_DISTANCE_CM:
                x, y = self.xy(angle, distance)
                points.append((angle, distance, x, y))

        # Group nearby readings into approximate target clusters.
        clusters = []
        for point in points:
            angle, distance, x, y = point

            if clusters:
                previous = clusters[-1][-1]
                angle_gap = angle - previous[0]
                distance_gap = abs(distance - previous[1])

                if angle_gap <= 4 and distance_gap <= 15:
                    clusters[-1].append(point)
                    continue

            clusters.append([point])

        detections = []
        for cluster in clusters:
            # Ignore isolated single readings to reduce noise.
            if len(cluster) < 2:
                continue

            count = len(cluster)
            angle = sum(p[0] for p in cluster) / count
            distance = sum(p[1] for p in cluster) / count
            x = sum(p[2] for p in cluster) / count
            y = sum(p[3] for p in cluster) / count

            detections.append((angle, distance, x, y))

        # Associate detections with existing tracks.
        # Greedy nearest-neighbour matching, once per track.
        candidates = []
        for ti, target in enumerate(self.targets):
            for di, detection in enumerate(detections):
                _, _, x, y = detection
                separation = math.hypot(
                    target.x - x, target.y - y
                )
                if separation <= MAX_MATCH_DISTANCE_CM:
                    candidates.append((separation, ti, di))

        candidates.sort()
        used_tracks = set()
        used_detections = set()

        for _, ti, di in candidates:
            if ti in used_tracks or di in used_detections:
                continue

            target = self.targets[ti]
            angle, distance, x, y = detections[di]
            dt = now - target.last_update

            if dt >= MIN_SPEED_INTERVAL_S:
                # Positive speed = approaching.
                # Negative speed = moving away.
                measured_speed = (
                    target.distance - distance
                ) / dt

                target.radial_speed = (
                    SPEED_SMOOTHING * measured_speed
                    + (1 - SPEED_SMOOTHING)
                    * target.radial_speed
                )

            target.angle = angle
            target.distance = distance
            target.x = x
            target.y = y
            target.missed_scans = 0
            target.last_update = now

            used_tracks.add(ti)
            used_detections.add(di)

        # Age tracks that were not matched this sweep.
        for ti, target in enumerate(self.targets):
            if ti not in used_tracks:
                target.missed_scans += 1

        self.targets = [
            t for t in self.targets
            if t.missed_scans <= MAX_MISSED_SCANS
        ]

        # Create IDs for newly detected targets.
        for di, detection in enumerate(detections):
            if di in used_detections:
                continue

            angle, distance, x, y = detection
            self.targets.append(Target(
                target_id=self.next_id,
                x=x,
                y=y,
                distance=distance,
                angle=angle,
                last_update=now
            ))
            self.next_id += 1

        return self.targets
