
import os
import time
import subprocess
from datetime import datetime
from pylx16a.lx16a import LX16A

PORT = "/dev/ttyUSB0"

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

# Initial warning thresholds; adjust after validation
CPU_TEMP_WARN = 75.0
SERVO_TEMP_WARN = 65.0


def run_command(command):
    try:
        return subprocess.check_output(
            command,
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=3
        ).strip()
    except Exception:
        return None


def check_raspberry_pi():
    print("\n=== Raspberry Pi Health ===")

    raw_temp = run_command(["vcgencmd", "measure_temp"])
    throttled = run_command(["vcgencmd", "get_throttled"])

    print(f"CPU Temperature: {raw_temp or 'N/A'}")
    print(f"Power Status: {throttled or 'N/A'}")

    if raw_temp:
        try:
            temp = float(raw_temp.split("=")[1].replace("'C", ""))
            if temp >= CPU_TEMP_WARN:
                print("WARNING: High CPU temperature")
        except (ValueError, IndexError):
            pass

    if throttled and "=" in throttled:
        try:
            flags = int(throttled.split("=")[1], 16)

            if flags & 0x1:
                print("WARNING: Undervoltage detected NOW")

            if flags & 0x10000:
                print("WARNING: Undervoltage occurred since boot")

            if flags & 0x4:
                print("WARNING: CPU currently throttled")

            if flags & 0x40000:
                print("WARNING: CPU throttled since boot")

            if flags == 0:
                print("Power flags: OK")

        except ValueError:
            print("Could not decode power flags")

    mem = run_command(["free", "-h"])
    if mem:
        print("\nMemory:")
        print("\n".join(mem.splitlines()[:2]))


def safe_read(servo, method_name):
    method = getattr(servo, method_name, None)

    if not callable(method):
        return None

    try:
        return method()
    except Exception:
        return None


def check_servos():
    print("\n=== Servo Health ===")

    if not os.path.exists(PORT):
        print(f"ERROR: Serial port {PORT} not found")
        return

    try:
        LX16A.initialize(PORT)
    except Exception as e:
        print(f"ERROR: Controller connection failed: {e}")
        return

    connected = 0
    warnings = []

    for sid, name in SERVO_MAP.items():
        try:
            servo = LX16A(sid)

            # Angle read confirms communication
            angle = servo.get_physical_angle()
            connected += 1

            # Optional telemetry
            temp = safe_read(servo, "get_temp")
            voltage = safe_read(servo, "get_vin")

            status = "OK"

            if temp is not None and temp >= SERVO_TEMP_WARN:
                status = "HOT"
                warnings.append(f"Servo {sid}: high temperature")

            temp_str = (
                f"{temp:.1f} C"
                if isinstance(temp, (int, float))
                else "N/A"
            )

            voltage_str = (
                str(voltage)
                if voltage is not None
                else "N/A"
            )

            print(
                f"Servo {sid} ({name})\n"
                f"  Angle: {angle:.2f} deg\n"
                f"  Temp: {temp_str}\n"
                f"  Voltage (raw): {voltage_str}\n"
                f"  Status: {status}"
            )

        except Exception as e:
            warnings.append(f"Servo {sid}: communication failed")
            print(f"Servo {sid} ({name}): OFFLINE ({e})")

    print(f"\nConnected: {connected}/8")

    if warnings:
        print("\n=== Warnings ===")
        for warning in warnings:
            print(f"- {warning}")
    else:
        print("No detected servo warnings")


def main():
    print("=" * 45)
    print("ROBOT HEALTH CHECK")
    print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 45)

    check_raspberry_pi()
    check_servos()

    print("\nHealth check completed.")


if __name__ == "__main__":
    main()
