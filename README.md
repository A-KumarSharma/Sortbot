# sortbot_ws — colour-sorting Panda arm (ROS 2 Jazzy + Gazebo + MoveIt 2)

A simulated Franka Panda arm looks down at a table with red, green and blue cubes,
finds them with an overhead camera, picks each one up and drops it in the bin of its colour.

## 1. Requirements
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros_gz`), `gz_ros2_control`, MoveIt 2, `xacro`, `cv_bridge`, OpenCV.
```bash
sudo apt install ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit-resources-panda-description \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-xacro python3-opencv
```

## 2. Build and run
```bash
pkill -f "gz sim"; pkill -f ruby; pkill -f parameter_bridge   # kill stale Gazebo from earlier runs
cd ~/sortbot_ws
rm -rf build install log                                      # clean build (avoids stale-file errors)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sortbot_sim sortbot.launch.py
```
In a second terminal (`source install/setup.bash` first), choose what to sort:
```bash
ros2 topic pub --once /sorter/command std_msgs/msg/String "{data: ALL}"   # R, G, B, RG, ALL, STOP
```
Check controllers: `ros2 control list_controllers` (all three should be `active`).
## 3. File structure
```
sortbot_ws/
├── README.md
└── src/
    ├── sortbot/                  # the "brain" (Python nodes)
    │   ├── sortbot/detector.py   # camera image -> 3D cube poses
    │   ├── sortbot/sorter.py     # pick-and-place state machine using MoveIt
    │   └── setup.py, package.xml # entry points: `detector`, `sorter`
    └── sortbot_sim/              # the "world" (simulation + robot + MoveIt config)
        ├── launch/sortbot.launch.py   # starts everything
        ├── sortbot_sim/world.py       # generates the Gazebo world (table, bins, cubes, camera)
        ├── urdf/sortbot.urdf.xacro    # Panda arm + hand + ros2_control + Gazebo plugin
        └── config/
            ├── ros2_controllers.yaml  # arm / hand / joint-state controllers
            ├── moveit-controllers.yaml# tells MoveIt which controller runs trajectories
            ├── sortbot.srdf           # planning group `panda_arm`, collision exclusions
            ├── kinematics.yaml        # KDL inverse kinematics
            ├── ompl_planning.yaml     # OMPL planners (RRTConnect)
            └── joint_limits.yaml      # velocity / acceleration limits
```
## 4. What happens when you launch
1. **Gazebo** loads a generated world (`/tmp/sortbot_world.sdf`): table, three bins, six random cubes
   and a static overhead camera (640x480, 15 Hz, looking straight down from 1 m).
2. **robot_state_publisher** publishes the Panda URDF; **`ros_gz_sim create`** spawns it in Gazebo.
   The `gz_ros2_control` plugin inside the URDF starts the `controller_manager`.
3. **Spawners** start `joint_state_broadcaster`, `panda_arm_controller`, `panda_hand_controller`.
4. **ros_gz_bridge** forwards `/clock`, `/camera/image_raw`, `/camera/camera_info` to ROS.
5. Two **static TF** publishers give the camera pose (`world -> camera_link -> camera_optical_frame`).
6. **move_group** (MoveIt) starts with the OMPL planner and the controller config.
7. **detector** and **sorter** nodes start (after a delay so the rest is ready).

## 5. The two nodes
**detector.py** — turns pixels into cube poses
- Blurs the image, thresholds red / green / blue in HSV, finds blobs, rejects wrong sizes.
- Casts each blob outline onto the cube's mid-height plane using the camera pose from TF, then fits a
  rectangle in world coordinates -> cube centre (x, y) and yaw (cubes are symmetric every 90 degrees).
- Tracks cubes across frames (smooths position, needs 5 sightings) and publishes
  `/detections` (`vision_msgs/Detection3DArray`, frame `panda_link0`, `class_id` = colour).

**sorter.py** — the pick-and-place loop (runs in its own thread)
1. Waits for MoveIt / controllers, adds the table to the planning scene, opens the hand, goes HOME.
2. Picks the nearest cube of an active colour, then runs these steps (each is logged if it fails):
   `open -> pre-grasp -> descend -> close -> grip check -> lift -> to bin -> lower -> release -> raise -> home`
3. Free-space moves use OMPL pose goals; straight-line moves (descend, lift, lower) use MoveIt's
   Cartesian path service at low speed. The hand is driven directly via its trajectory controller.
4. **Grip check:** adds both finger positions. A 40 mm cube gives about 40 mm; closing on nothing
   gives about 24 mm. The cube blocks the fingers, so the controller's "goal not reached" error is
   expected and ignored — the measured width decides.
5. A failed cube is skipped for 20 s, the hand reopens and the arm goes HOME.

## 6. Useful tuning (top of `sorter.py` / `detector.py` / `world.py`)
| What | Where | Value |
|---|---|---|
| Bin positions | `BINS` in `sorter.py` and `world.py` (keep identical) | x, y in metres |
| Approach / drop heights | `PRE, DROP, TRAVEL` in `sorter.py` | 0.12 / 0.14 / 0.25 |
| Closing target | `CLOSED` in `sorter.py` | 0.012 (must be < 0.02) |
| Workspace limits, cube size | `XR, YR, C` in `detector.py` | |
| Number of cubes, random seed | `build(seed, n)` in `world.py` | 6, 3 |

## 7. Troubleshooting
| Symptom | Likely cause / fix |
|---|---|
| `can't copy ... .yaml` on build | Stale build folder: `rm -rf build install log` |
| Arm not visible, spawner waits forever | Model failed to load; read the first `[gazebo-1] Error` lines |
| `Controller already loaded` / `Failed to configure` | Old Gazebo still running: run the `pkill` line above |
| Sorter prints `step "..." failed` | The step name shows where; `grip width` shows if the cube was held |
| `/sorter/command` waits for a subscriber | `sorter` crashed or is not running: check the launch log |
| `No 3D sensor plugin(s)` from MoveIt | Harmless (no octomap sensor is used) |

Debug: `ros2 topic echo /detections`, `ros2 run rqt_image_view rqt_image_view /camera/image_raw`.
