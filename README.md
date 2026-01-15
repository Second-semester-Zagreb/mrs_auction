# MRS Auction

Multi-Robot System task allocation using auction-based scheduling with ROS 2.

## Features

- Task auction algorithm for multi-robot coordination
- Support for Single-Robot (SR) and Multi-Robot (MR) tasks
- Precedence constraints between tasks
- Simple executor for task execution
- Body-frame velocity control for Crazyflies

## Structure

```
mrs_auction/
├── config/           # Task and robot configurations
├── launch/           # Launch files
├── mrs_auction/      # Python package
│   ├── auction_def.py
│   ├── auctioneer.py
│   ├── bidder.py
│   ├── mission_executor.py
│   └── simple_executor.py
├── scripts/          # Utility scripts
└── schedule/         # Generated schedules (JSON)
```

## Usage

### 1. Generate Schedule

```bash
python3 /root/ros2_ws/src/mrs_auction/scripts/run_auction.py
```

### 2. Execute Schedule

```bash
# Build first
cd ~/ros2_ws && colcon build --packages-select mrs_auction

# Launch all robot executors
ros2 launch mrs_auction execute_schedule.launch.py
```

### 3. Single Robot Execution

```bash
ros2 run mrs_auction simple_executor_node R1
```

## Configuration

Edit `config/tasks.yaml` and `config/robots.yaml` to define your mission.

## Dependencies

- ROS 2 Humble
- Python 3.10+
- geometry_msgs
- NetworkX

## License

See LICENSE file.
