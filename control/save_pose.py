
import argparse
import json
import re
from pathlib import Path
from pylx16a.lx16a import LX16A

PORT = "/dev/ttyUSB0"

# Save poses relative to this script
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


def save_pose(name, overwrite=False):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise ValueError(
            "Pose name must contain only letters, "
            "numbers, underscores, or hyphens."
        )

    POSE_DIR.mkdir(parents=True, exist_ok=True)
    output_file = POSE_DIR / f"{name}.json"

    if output_file.exists() and not overwrite:
        raise FileExistsError(
            f"{output_file} already exists. "
            "Use --overwrite to replace it."
        )

    LX16A.initialize(PORT)

    print(f"\n=== FOLIO: SAVE POSE '{name}' ===")
    print("Position all servos in the desired pose.")
    input("Press Enter to capture...")

    angles = {}

    for sid, label in SERVO_MAP.items():
        servo = LX16A(sid)
        angle = servo.get_physical_angle()

        if not 0 <= angle <= 240:
            raise ValueError(
                f"Servo {sid}: invalid angle {angle}"
            )

        angles[str(sid)] = round(angle, 2)

        print(f"Servo {sid} ({label}): {angle:.2f}°")

    print("\nCaptured pose:")
    print(json.dumps(angles, indent=4))

    confirm = input("\nSave this pose? (y/n): ").strip().lower()

    if confirm != "y":
        print("Cancelled.")
        return

    # Write atomically to avoid partial JSON files
    temporary_file = output_file.with_suffix(".tmp")

    with open(temporary_file, "w") as f:
        json.dump(angles, f, indent=4)
        f.write("\n")

    temporary_file.replace(output_file)

    print(f"\nPose saved: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Save Folio's current servo pose"
    )

    parser.add_argument(
        "name",
        help="Name of the pose, e.g. neutral"
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite an existing pose"
    )

    args = parser.parse_args()

    save_pose(args.name, args.overwrite)


if __name__ == "__main__":
    main()
