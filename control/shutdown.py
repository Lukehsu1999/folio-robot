
import argparse
import json
import math
import subprocess
import time
from pathlib import Path

from pylx16a.lx16a import LX16A


# ==========================================
# FOLIO - SAFE SHUTDOWN
# ==========================================

PORT = "/dev/ttyUSB0"
POSE_FILE = (
    Path(__file__).resolve().parent
    / "poses"
    / "neutral.json"
)

MOVE_TIME = 2500       # milliseconds
ANGLE_TOLERANCE = 5.0  # degrees

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


# ==========================================
# SYSTEM HEALTH
# ==========================================

def run_command(command):
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )

        if result.returncode == 0:
            return result.stdout.strip()

        return "Unavailable"

    except (OSError, subprocess.TimeoutExpired):
        return "Unavailable"


def print_pi_status():
    print("\n=== RASPBERRY PI STATUS ===")

    temperature = run_command(
        ["vcgencmd", "measure_temp"]
    )

    throttled = run_command(
        ["vcgencmd", "get_throttled"]
    )

    uptime = run_command(["uptime", "-p"])

    print(f"CPU temperature: {temperature}")
    print(f"Power/throttle flags: {throttled}")
    print(f"Uptime: {uptime}")

    if throttled.startswith("throttled="):
        try:
            flags = int(throttled.split("=")[1], 16)

            if flags & 0x1:
                print("WARNING: Undervoltage detected now")

            if flags & 0x10000:
                print("WARNING: Undervoltage occurred earlier")

        except ValueError:
            pass


# ==========================================
# POSE MANAGEMENT
# ==========================================

def load_neutral():
    with open(POSE_FILE, "r") as f:
        data = json.load(f)

    pose = {
        int(sid): float(angle)
        for sid, angle in data.items()
    }

    if set(pose) != set(SERVO_MAP):
        raise ValueError(
            "Neutral pose must contain all 8 servos"
        )

    for sid, angle in pose.items():
        if not math.isfinite(angle) or not 0 <= angle <= 240:
            raise ValueError(
                f"Invalid neutral angle for servo {sid}"
            )

    return pose


def connect_servos():
    LX16A.initialize(PORT)

    servos = {}

    for sid in SERVO_MAP:
        servos[sid] = LX16A(sid)

    return servos


def read_angles(servos):
    angles = {}

    for sid, servo in servos.items():
        angles[sid] = servo.get_physical_angle()

    return angles


def print_servo_status(angles, neutral):
    print("\n=== SERVO STATUS ===")

    print(
        f"{'ID':<4} {'Joint':<20} "
        f"{'Current':>9} {'Neutral':>9} {'Error':>9}"
    )

    for sid, name in SERVO_MAP.items():
        current = angles[sid]
        target = neutral[sid]
        error = current - target

        print(
            f"{sid:<4} {name:<20} "
            f"{current:>8.2f}° "
            f"{target:>8.2f}° "
            f"{error:>+8.2f}°"
        )


def move_to_neutral(servos, neutral):
    print("\n=== RETURNING TO NEUTRAL ===")

    # Validate all targets before sending commands.
    for sid, angle in neutral.items():
        if not 0 <= angle <= 240:
            raise ValueError(
                f"Servo {sid}: invalid target"
            )

    for sid, servo in servos.items():
        servo.move(
            neutral[sid],
            time=MOVE_TIME
        )

    time.sleep(MOVE_TIME / 1000.0 + 0.5)


def verify_neutral(servos, neutral):
    print("\n=== VERIFYING NEUTRAL POSE ===")

    actual = read_angles(servos)
    print_servo_status(actual, neutral)

    errors = {
        sid: abs(actual[sid] - neutral[sid])
        for sid in SERVO_MAP
    }

    failed = [
        sid for sid, error in errors.items()
        if error > ANGLE_TOLERANCE
    ]

    if failed:
        print(
            "\nWARNING: Some servos did not "
            "reach neutral within tolerance."
        )
        print(f"Servo IDs: {failed}")
        return False

    print("\nAll servos reached neutral tolerance.")
    return True


# ==========================================
# SHUTDOWN PROCEDURE
# ==========================================

def main():
    parser = argparse.ArgumentParser(
        description="Folio safe shutdown"
    )

    parser.add_argument(
        "--no-poweroff",
        action="store_true",
        help="Return to neutral without shutting down Pi"
    )

    args = parser.parse_args()

    print("\n================================")
    print("       FOLIO SHUTDOWN")
    print("================================")

    print_pi_status()

    try:
        neutral = load_neutral()
        servos = connect_servos()
        current = read_angles(servos)

        print_servo_status(current, neutral)

    except Exception as exc:
        print(f"\nERROR: Pre-shutdown check failed: {exc}")
        print("No movement commands were sent.")
        print("Pi shutdown was not requested.")
        return

    print("\nIMPORTANT:")
    print("Support Folio before moving the joints.")
    print("Check that all legs have clear movement paths.")

    answer = input(
        "\nReturn to neutral pose? (y/n): "
    ).strip().lower()

    if answer != "y":
        print("Cancelled. No movement or shutdown.")
        return

    try:
        move_to_neutral(servos, neutral)
        success = verify_neutral(servos, neutral)

    except Exception as exc:
        print(f"\nERROR during pose transition: {exc}")
        print("Do not assume the robot is at neutral.")
        print("No Pi shutdown requested.")
        return

    if not success:
        print("\nInspect the servos before disconnecting power.")
        print("No Pi shutdown requested.")
        return

    print("\n=== ROBOT READY FOR POWER-DOWN ===")
    print("1. Keep Folio physically supported.")
    print("2. Stop any other robot-control programs.")
    print("3. Prepare to disconnect servo power.")
    print("4. Shut down the Raspberry Pi cleanly.")

    if args.no_poweroff:
        print("\n--no-poweroff selected.")
        print("Raspberry Pi will remain running.")
        return

    answer = input(
        "\nShut down Raspberry Pi now? (y/n): "
    ).strip().lower()

    if answer != "y":
        print("Pi remains running.")
        return

    print("\nRequesting Raspberry Pi shutdown...")
    print("Your SSH session may disconnect.")

    try:
        subprocess.run(
            ["sudo", "-n", "shutdown", "-h", "now"],
            check=True,
            timeout=10,
        )

    except subprocess.CalledProcessError:
        print(
            "Shutdown command failed. "
            "Try: sudo shutdown -h now"
        )

    except subprocess.TimeoutExpired:
        print(
            "Shutdown command timed out. "
            "Verify Pi state before disconnecting power."
        )

    except OSError as exc:
        print(f"Shutdown could not start: {exc}")


if __name__ == "__main__":
    main()
