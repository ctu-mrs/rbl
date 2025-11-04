import math

def main():
    n = int(input("Enter number of UAVs (n): "))
    r = float(input("Enter radius of circle (r): "))
    z = float(input("Enter altitude (z): "))

    print("\n--- UAV Spawns ---\n")
    for i in range(n):
        angle = 2 * math.pi * i / n
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        heading = math.radians(math.atan2(-y, -x))  # towards center

        print(f"uav{i+1}:")
        print(f"  type: \"x500\"")
        print(f"  spawn:")
        print(f"    x: {x:.2f}")
        print(f"    y: {y:.2f}")
        print(f"    z: {z:.2f}")
        print(f"    heading: {heading:.2f}\n")

    print("\n--- First rosservice calls after takeoff to get to correct height ---\n")
    for i in range(n):
        angle = 2 * math.pi * i / n
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        print(f"- 'history -s rosservice call /uav{i+1}/rbl_controller/goto \\\"goal: \\[{x:.2f}, {y:.2f}, {z:.2f}, 0.0\\]\\\"'")
    print("\n--- Second rosservice calls goal ---\n")
    for i in range(n):
        angle = 2 * math.pi * i / n + math.pi
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        print(f"- 'history -s rosservice call /uav{i+1}/rbl_controller/goto \\\"goal: \\[{x:.2f}, {y:.2f}, {z:.2f}, 0.0\\]\\\"'")

if __name__ == "__main__":
    main()
