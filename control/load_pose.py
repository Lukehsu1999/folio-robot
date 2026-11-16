
import argparse
import json
import time
from pathlib import Path
from pylx16a.lx16a import LX16A

PORT = "/dev/ttyUSB0"
POSE_DIR = Path(__file__).resolve().parent / "poses"

SERVO_MAP = {
    1: "Left Front Knee",
    2: "Left Front Hip",
    3: "Right Back Knee",
    4: "Right Back Hip",
    5: "Right Front Knee",
    6: "Right Front Hip",
    7: "Left Back Hip",
    8: "Left Back Knee",
}


def load_pose(filename):
    """Read and validate a saved pose."""
    path = POSE_DIR / filename

    if not path.is_file():
        raise FileNotFoundError(f"Pose not found: {path}")

    with open(path) as f:
        data = json.load(f)

    angles = {
        int(sid): float(angle)
        for sid, angle in data.items()
    }

    if set(angles) != set(SERVO_MAP):
        raise ValueError("Pose must contain all 8 servos")

    for sid, angle in angles.items():
        if not 0 <= angle <= 240:
            raise ValueError(
                f"Servo {sid}: invalid angle {angle}"
            )

    return angles


def move_to_pose(angles, duration=2000):
    """Move all eight servos to the target pose."""
    LX16A.initialize(PORT)

    servos = {}

    print("\n=== FOLIO POSE TRANSITION ===")

    for sid, name in SERVO_MAP.items():
        servo = LX16A(sid)
        servos[sid] = servo

        current = servo.get_physical_angle()
        target = angles[sid]

        print(
            f"Servo {sid} ({name}): "
            f"{current:.2f} -> {target:.2f} deg"
        )

    print("\nWARNING: All eight servos will move.")
    print("Support the robot before transitioning.")

    input("\nPress Enter to move to pose...")

    for sid, servo in servos.items():
        servo.move(angles[sid], time=duration)

    time.sleep(duration / 1000 + 0.5)

    print("\n=== Final Servo Positions ===")

    for sid, servo in servos.items():
        actual = servo.get_physical_angle()
        error = actual - angles[sid]

        print(
            f"Servo {sid}: "
            f"{actual:.2f} deg "
            f"(error: {error:+.2f} deg)"
        )

    print("\nPose transition completed!")


def main():
    parser = argparse.ArgumentParser(
        description="Move Folio to a saved pose"
    )

    parser.add_argument(
        "pose",
        help="Pose filename, e.g. neutral.json"
    )

    parser.add_argument(
        "--duration",
        type=int,
        default=2000,
        help="Movement duration in milliseconds"
    )

    args = parser.parse_args()

    if Path(args.pose).name != args.pose:
        parser.error("Provide a filename, not a path")

    if not args.pose.endswith(".json"):
        parser.error("Pose filename must end with .json")

    if args.duration <= 0:
        parser.error("Duration must be positive")

    angles = load_pose(args.pose)
    move_to_pose(angles, duration=args.duration)


if __name__ == "__main__":
    main()
