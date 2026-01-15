#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Land, Takeoff, GoTo
from builtin_interfaces.msg import Duration


class LandTakeoffTester(Node):
    def __init__(self, robot_num: int):
        super().__init__("land_takeoff_tester")
        self.robot_num = robot_num
        self.ns = f"cf_{robot_num}"
        
        self.takeoff_cli = self.create_client(Takeoff, f"/{self.ns}/takeoff")
        self.land_cli = self.create_client(Land, f"/{self.ns}/land")
        self.go_to_cli = self.create_client(GoTo, f"/{self.ns}/go_to")
        
        print(f"\n[Robot {robot_num}]")
        print(f"  Takeoff service: /{self.ns}/takeoff")
        print(f"  Land service: /{self.ns}/land")
        print(f"  GoTo service: /{self.ns}/go_to")

    def wait_for_service(self, client, service_name: str) -> bool:
        print(f"\nWaiting for {service_name}...", end="", flush=True)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f" TIMEOUT!")
            return False
        print(f" OK")
        return True

    def duration_msg(self, seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        d = Duration()
        d.sec = sec
        d.nanosec = nanosec
        return d

    def test_takeoff(self, height: float = 0.5, duration: float = 2.0):
        if not self.wait_for_service(self.takeoff_cli, "takeoff"):
            return False
        
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = float(height)
        req.duration = self.duration_msg(duration)
        
        print(f"Sending TakeOff request: height={height}, duration={duration}s")
        future = self.takeoff_cli.call_async(req)
        
        print("Waiting for response...", end="", flush=True)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                result = future.result()
                print(f" SUCCESS")
                print(f"  Result: {result}")
                return True
            except Exception as e:
                print(f" FAILED: {e}")
                return False
        else:
            print(f" TIMEOUT")
            return False

    def test_land(self, height: float = 0.05, duration: float = 3.0):
        if not self.wait_for_service(self.land_cli, "land"):
            return False
        
        req = Land.Request()
        req.group_mask = 0
        req.height = float(height)
        req.duration = self.duration_msg(duration)
        
        print(f"Sending Land request: height={height}, duration={duration}s")
        future = self.land_cli.call_async(req)
        
        print("Waiting for response...", end="", flush=True)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                result = future.result()
                print(f" SUCCESS")
                print(f"  Result: {result}")
                return True
            except Exception as e:
                print(f" FAILED: {e}")
                return False
        else:
            print(f" TIMEOUT")
            return False

    def test_go_to(self, x: float, y: float, z: float, duration: float = 5.0):
        if not self.wait_for_service(self.go_to_cli, "go_to"):
            return False
        
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal.x = float(x)
        req.goal.y = float(y)
        req.goal.z = float(z)
        req.yaw = 0.0
        req.duration = self.duration_msg(duration)
        
        print(f"Sending GoTo request: ({x}, {y}, {z}), duration={duration}s")
        future = self.go_to_cli.call_async(req)
        
        print("Waiting for response...", end="", flush=True)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                result = future.result()
                print(f" SUCCESS")
                print(f"  Result: {result}")
                return True
            except Exception as e:
                print(f" FAILED: {e}")
                return False
        else:
            print(f" TIMEOUT")
            return False


def main():
    rclpy.init()
    
    print("\n" + "="*60)
    print("  CRAZYFLIE LAND/TAKEOFF SERVICE TESTER")
    print("="*60)
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 test_land_takeoff.py <robot_num> [command] [args...]")
        print("\nCommands:")
        print("  takeoff [height] [duration]  - Test takeoff (default: 0.5m, 2.0s)")
        print("  land [height] [duration]     - Test landing (default: 0.05m, 3.0s)")
        print("  go_to x y z [duration]       - Test go_to (default: 5.0s)")
        print("\nExamples:")
        print("  python3 test_land_takeoff.py 1 takeoff")
        print("  python3 test_land_takeoff.py 1 takeoff 1.0 3.0")
        print("  python3 test_land_takeoff.py 1 land")
        print("  python3 test_land_takeoff.py 1 land 0.1 2.0")
        print("  python3 test_land_takeoff.py 1 go_to 0 0 0.5")
        sys.exit(1)
    
    # robot_num = int(sys.argv[1])
    # node = LandTakeoffTester(robot_num)
    
    # command = sys.argv[2] if len(sys.argv) > 2 else None
    
    # try:
    #     if command == "takeoff":
    #         height = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    #         duration = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    #         node.test_takeoff(height, duration)
            
    #     elif command == "land":
    #         height = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    #         duration = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
    #         node.test_land(height, duration)
            
    #     elif command == "go_to":
    #         if len(sys.argv) < 5:
    #             print("Error: go_to requires x, y, z arguments")
    #             sys.exit(1)
    #         x = float(sys.argv[3])
    #         y = float(sys.argv[4])
    #         z = float(sys.argv[5])
    #         duration = float(sys.argv[6]) if len(sys.argv) > 6 else 5.0
    #         node.test_go_to(x, y, z, duration)
            
    #     else:
    #         print(f"Unknown command: {command}")
    #         sys.exit(1)
            
    # except Exception as e:
    #     print(f"\nError: {e}")
    #     import traceback
    #     traceback.print_exc()
    #     sys.exit(1)
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
    
    # print()


if __name__ == "__main__":
    main()
