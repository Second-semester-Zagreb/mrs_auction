#!/usr/bin/env python3
import json
import os
from datetime import datetime
from mrs_auction.auctioneer import Auction


def main():
    auction = Auction(
        tasks_yaml="/root/ros2_ws/src/mrs_auction/config/tasks.yaml",
        robots_yaml="/root/ros2_ws/src/mrs_auction/config/robots.yaml",
    )
    sol = auction.make_solution()
    print(f"Final schedule: {sol}")

    d = "/root/ros2_ws/src/mrs_auction/schedule"
    os.makedirs(d, exist_ok=True)
    ts = datetime.now().strftime("%H%M%d%m")
    filepath = os.path.join(d, f"schedule_{ts}.json")
    with open(filepath, "w") as f:
        json.dump(sol, f, indent=2)
    print(f"\nSchedule saved to: {filepath}")


if __name__ == "__main__":
    main()
