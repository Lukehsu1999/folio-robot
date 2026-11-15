
#!/usr/bin/env python3
"""
Folio quadruped walking controller.

Example:
    python3 walk.py --pose neutral.json --cycles 3 \
        --period 2.5 --stride 16 --lift 30 --duty 0.8
"""

import argparse
import json
import math
import time
from pathlib import Path

from pylx16a.lx16a import LX16A


# ============================================
# HARDWARE CONFIGURATION
# ============================================

PORT = "/dev/ttyUSB0"

LEGS = {
    "LF": {"hip": 2, "knee": 1},
    "RF": {"hip": 6, "knee": 5},
    "LB": {"hip": 7, "knee": 8},
    "RB": {"hip": 4, "knee": 3},
}

SERVO_IDS = set(
    sid
    for leg in LEGS.values()
    for sid in leg.values()
)

# Positive logical direction = forward.
HIP_DIRECTION = {
    2: +1,
    4: -1,
    6: +1,
    7: -1,
}

# Based on your original forward-bend calibration.
# Reverse individual signs if the physical knee
# bends in the wrong direction for foot lifting.
KNEE_DIRECTION = {
    1: +1,
    3: -1,
    5: +1,
    8: -1,
}

# Sequential crawl: each leg swings at a
# different point in the gait cycle.
PHASE_OFFSETS = {
    "LF": 0.80,
    "RB": 0.55,
    "RF": 0.30,
    "LB": 0.05,
}

MIN_ANGLE = 0.0
MAX_ANGLE = 240.0


# ============================================
# POSE MANAGEMENT
# ============================================

def load_pose(filename):
    path = Path(filename)

    if not path.is_absolute() and not path.exists():
        path = (
            Path(__file__).resolve().parent
            / "poses"
            / filename
        )

    with open(path, "r") as f:
        data = json.load(f)

    pose = {
        int(sid): float(angle)
        for sid, angle in data.items()
    }

    if set(pose) != SERVO_IDS:
        raise ValueError(
            "Pose must contain exactly servo IDs 1-8"
        )

    for sid, angle in pose.items():
        if not math.isfinite(angle):
            raise ValueError(
                f"Servo {sid}: non-finite angle"
            )

        if not MIN_ANGLE <= angle <= MAX_ANGLE:
            raise ValueError(
                f"Servo {sid}: angle out of range"
            )

    return pose


def connect_servos():
    LX16A.initialize(PORT)

    return {
        sid: LX16A(sid)
        for sid in sorted(SERVO_IDS)
    }


def move_to_pose(servos, pose, duration_ms=2000):
    for sid in sorted(SERVO_IDS):
        servos[sid].move(
            pose[sid],
            time=duration_ms
        )

    time.sleep(duration_ms / 1000 + 0.3)


def read_angles(servos):
    return {
        sid: servo.get_physical_angle()
        for sid, servo in servos.items()
    }


def verify_pose(servos, pose, tolerance=5.0):
    actual = read_angles(servos)

    for sid in sorted(SERVO_IDS):
        error = abs(actual[sid] - pose[sid])

        print(
            f"Servo {sid}: "
            f"actual={actual[sid]:.1f}, "
            f"target={pose[sid]:.1f}, "
            f"error={error:.1f}"
        )

        if error > tolerance:
            raise RuntimeError(
                f"Servo {sid} did not reach pose"
            )


# ============================================
# GAIT MATHEMATICS
# ============================================

def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def leg_trajectory(phase, duty):
    """
    Returns normalized hip and knee motion.

    Stance:
        Foot remains down.
        Hip moves from forward to backward.

    Swing:
        Knee lifts the foot.
        Hip moves from backward to forward.
    """

    if phase < duty:
        progress = phase / duty

        hip = 1.0 - 2.0 * smoothstep(progress)
        knee = 0.0

    else:
        progress = (phase - duty) / (1.0 - duty)

        hip = -1.0 + 2.0 * smoothstep(progress)
        knee = math.sin(math.pi * progress) ** 2

    return hip, knee


def gait_targets(
    elapsed,
    neutral,
    period,
    stride,
    lift,
    duty,
    direction,
):
    targets = {}

    # Gradually increase motion over first cycle.
    amplitude = min(1.0, elapsed / max(period, 1.0))

    for leg_name, joints in LEGS.items():
        phase = (
            elapsed / period
            + PHASE_OFFSETS[leg_name]
        ) % 1.0

        hip_norm, knee_norm = leg_trajectory(
            phase, duty
        )

        hip_id = joints["hip"]
        knee_id = joints["knee"]

        targets[hip_id] = (
            neutral[hip_id]
            + direction
            * HIP_DIRECTION[hip_id]
            * stride
            * amplitude
            * hip_norm
        )

        targets[knee_id] = (
            neutral[knee_id]
            + KNEE_DIRECTION[knee_id]
            * lift
            * amplitude
            * knee_norm
        )

    return targets


# ============================================
# SAFETY VALIDATION
# ============================================

def validate_targets(targets):
    for sid, angle in targets.items():
        if not math.isfinite(angle):
            raise ValueError(
                f"Servo {sid}: invalid angle"
            )

        if not MIN_ANGLE <= angle <= MAX_ANGLE:
            raise ValueError(
                f"Servo {sid}: "
                f"target {angle:.1f} out of range"
            )


def validate_motion_envelope(neutral, stride, lift):
    """
    Check the full theoretical angle envelope
    before starting the gait.
    """

    for leg in LEGS.values():
        hip_id = leg["hip"]
        knee_id = leg["knee"]

        hip_min = neutral[hip_id] - stride
        hip_max = neutral[hip_id] + stride

        knee_end = (
            neutral[knee_id]
            + KNEE_DIRECTION[knee_id] * lift
        )

        knee_min = min(neutral[knee_id], knee_end)
        knee_max = max(neutral[knee_id], knee_end)

        for sid, low, high in [
            (hip_id, hip_min, hip_max),
            (knee_id, knee_min, knee_max),
        ]:
            if low < MIN_ANGLE or high > MAX_ANGLE:
                raise ValueError(
                    f"Servo {sid} exceeds angle limits: "
                    f"{low:.1f} to {high:.1f}"
                )


# ============================================
# WALKING CONTROLLER
# ============================================

def walk(servos, neutral, args):
    total_time = args.cycles * args.period
    interval = 1.0 / args.hz

    print("\n=== FOLIO WALKING ===")
    print(f"Cycles:      {args.cycles}")
    print(f"Period:      {args.period:.2f} s")
    print(f"Stride:      {args.stride:.1f} deg")
    print(f"Lift:        {args.lift:.1f} deg")
    print(f"Duty factor: {args.duty:.2f}")
    print(f"Direction:   {args.direction}")
    print(f"Update rate: {args.hz:.1f} Hz")

    start = time.monotonic()
    next_update = start

    while True:
        now = time.monotonic()
        elapsed = now - start

        if elapsed >= total_time:
            break

        targets = gait_targets(
            elapsed=elapsed,
            neutral=neutral,
            period=args.period,
            stride=args.stride,
            lift=args.lift,
            duty=args.duty,
            direction=args.direction,
        )

        validate_targets(targets)

        for sid in sorted(SERVO_IDS):
            servos[sid].move(
                targets[sid],
                time=args.command_ms
            )

        next_update += interval

        delay = next_update - time.monotonic()

        if delay > 0:
            time.sleep(delay)
        else:
            # Skip missed update slots.
            next_update = time.monotonic()

    print("\nWalking sequence complete.")


# ============================================
# MAIN
# ============================================

def main():
    parser = argparse.ArgumentParser(
        description="Folio quadruped gait controller"
    )

    parser.add_argument(
        "--pose",
        default="neutral.json",
        help="Neutral pose JSON file"
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=2
    )

    parser.add_argument(
        "--period",
        type=float,
        default=4.0
    )

    parser.add_argument(
        "--stride",
        type=float,
        default=8.0,
        help="Hip amplitude in degrees"
    )

    parser.add_argument(
        "--lift",
        type=float,
        default=20.0,
        help="Knee lift amplitude in degrees"
    )

    parser.add_argument(
        "--duty",
        type=float,
        default=0.8
    )

    parser.add_argument(
        "--direction",
        type=int,
        choices=[-1, 1],
        default=1
    )

    parser.add_argument(
        "--hz",
        type=float,
        default=10.0
    )

    parser.add_argument(
        "--command-ms",
        type=int,
        default=150
    )

    args = parser.parse_args()

    if args.cycles < 1:
        parser.error("--cycles must be >= 1")

    if not math.isfinite(args.period) or args.period < 1.5:
        parser.error("--period must be >= 1.5")

    if not math.isfinite(args.stride) or not 0 <= args.stride <= 20:
        parser.error("--stride must be between 0 and 20")

    if not math.isfinite(args.lift) or not 0 <= args.lift <= 45:
        parser.error("--lift must be between 0 and 45")

    if not math.isfinite(args.duty) or not 0.75 <= args.duty <= 0.85:
        parser.error("--duty must be between 0.75 and 0.85")

    if not math.isfinite(args.hz) or not 2 <= args.hz <= 15:
        parser.error("--hz must be between 2 and 15")

    if not 100 <= args.command_ms <= 1000:
        parser.error("--command-ms must be 100-1000")

    neutral = load_pose(args.pose)

    validate_motion_envelope(
        neutral,
        args.stride,
        args.lift
    )

    print("\nFolio walking controller")
    print("Neutral pose loaded successfully.")
    print("All target angles are within servo limits.")

    print("\nWARNING:")
    print("Ensure Folio is supported for initial testing.")
    print("Check joint clearance and servo directions.")
    print("Stop other programs controlling the servos.")

    answer = input(
        "\nConnect and move to neutral? (y/n): "
    ).strip().lower()

    if answer != "y":
        print("Cancelled.")
        return

    servos = connect_servos()

    try:
        move_to_pose(servos, neutral)
        verify_pose(servos, neutral)

        answer = input(
            "\nStart walking? (y/n): "
        ).strip().lower()

        if answer != "y":
            print("Walking cancelled.")
            return

        walk(servos, neutral, args)

        answer = input(
            "\nReturn to neutral? (y/n): "
        ).strip().lower()

        if answer == "y":
            move_to_pose(servos, neutral)
            verify_pose(servos, neutral)
            print("\nReturned to neutral.")

        else:
            print(
                "\nRobot remains at its last "
                "commanded position."
            )

    except KeyboardInterrupt:
        print("\nWalking interrupted.")
        print("No automatic reset was attempted.")
        print("Support Folio and inspect its position.")

    except Exception as exc:
        print(f"\nERROR: {exc}")
        print("No automatic reset was attempted.")
        print("Inspect Folio before moving again.")
        raise


if __name__ == "__main__":
    main()
