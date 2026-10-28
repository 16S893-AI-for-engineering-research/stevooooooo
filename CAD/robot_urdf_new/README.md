# Three-Arm Cell

A URDF model of a work cell with three robot arms on a shared base:

- **Two SO-101 arms** (`left_` and `right_`), each with a custom gripper on the wrist roll and a camera mount on the wrist.
- **One Rotrics DexArm** with its gripper module and a laser camera on its end effector (`EE`).
- **A base mount** (`baseforreal`) that the arms sit on.

All units are meters and radians. Every mesh the model needs is in `meshes/`, so the folder works on its own.

## What's in the folder

| Path | What it is |
|---|---|
| `three_arm_cell.urdf` | The model, with mesh paths relative to this folder. Use this one outside ROS. |
| `urdf/three_arm_cell.urdf` | The same model with `package://three_arm_cell_description/...` mesh paths, for ROS 2. |
| `meshes/so101/` | SO-101 parts |
| `meshes/dexarm/` | DexArm parts |
| `meshes/custom/` | Base mount, gripper, camera mount, laser camera |
| `package.xml`, `CMakeLists.txt`, `launch/` | ROS 2 package files |
| `config/three_arm_cell.srdf` | Collision pairs for MoveIt to ignore: the SO-101 bases on the base mount, and the SO-101 grippers inside the tube |
| `licenses/` | Licenses for the third-party robot models |

## How to open it

**In a browser or viewer, no install needed.** Open `three_arm_cell.urdf` together with the `meshes/` folder. Most web viewers accept the whole folder dragged in, or a ZIP of it. Desktop viewers such as [urdf-viz](https://github.com/openrr/urdf-viz) open the `.urdf` file directly.

**In Python** (PyBullet, yourdfpy, Isaac Sim's URDF importer, MuJoCo), load `three_arm_cell.urdf` from inside this folder so the relative mesh paths resolve. For example:

```python
import yourdfpy
robot = yourdfpy.URDF.load("three_arm_cell.urdf")
robot.show()
```

**In ROS 2**, copy this folder into your workspace's `src/` directory, then:

```bash
colcon build --packages-select three_arm_cell_description
source install/setup.bash
ros2 launch three_arm_cell_description display.launch.py
```

That starts `robot_state_publisher`, a joint slider window, and RViz. In RViz, set **Fixed Frame** to `world` and add a **RobotModel** display.

## Joints

The root link is `world`. Everything else hangs off it.

| Arm | Movable joints |
|---|---|
| Left SO-101 | `left_Rotation`, `left_Pitch`, `left_Elbow`, `left_Wrist_Pitch`, `left_Wrist_Roll` |
| Right SO-101 | `right_Rotation`, `right_Pitch`, `right_Elbow`, `right_Wrist_Pitch`, `right_Wrist_Roll` |
| DexArm | `bases` (base rotation), `B_Rot->In_1`, `Join->In_2`, `GripperMount->GripperAxis` (gripper spin), `GripperFingerA1->GripperFingerA2` (gripper open/close) |

Each SO-101's `Wrist_Roll` is limited to −254° to 0° (−4.433136 to 0 rad), so the custom gripper can't roll into the wrist. Past 0° it hits the wrist, from about +0.4° to +105°.

The DexArm's other joints are `mimic` joints. They follow the joints above to keep its parallel linkage and the gripper fingers in step, so you don't set them yourself.

**Starting pose.** The model was set up with both SO-101s at Pitch −90°, Elbow 90° and Wrist Pitch −90°, and the DexArm at `B_Rot->In_1` 0° and `Join->In_2` −34.4° (its lower limit). All other joints start at 0:

| Joint | Value (rad) |
|---|---|
| `left_Pitch`, `right_Pitch` | −1.5708 |
| `left_Elbow`, `right_Elbow` | 1.5708 |
| `left_Wrist_Pitch`, `right_Wrist_Pitch` | −1.5708 |
| `B_Rot->In_1` | 0 |
| `Join->In_2` | −0.6 |

A URDF can't store a starting pose, so set these in your joint state publisher if you want to start there.

## Known limitations

- **The SO-101s rest on the base.** Both SO-101 bases touch `baseforreal`, which has a collision mesh, so those two pairs are always in contact. `config/three_arm_cell.srdf` marks them as always in contact so MoveIt ignores them; merge it into your MoveIt configuration. Every other part, including the rest of each SO-101, still collides with the base.
- **Tight fit at the DexArm's base.** `B_Fix` sits about 0.23 mm above `baseforreal`, so it doesn't touch it. A planner that uses a safety margin larger than that will treat the gap as contact.
- **The SO-101 grippers reach inside the tube.** `tube_1` has a collision mesh. In the starting pose both SO-101 grippers sit inside it (about 8–10 mm), which is intended. `config/three_arm_cell.srdf` allows those two pairs; every other part, including the DexArm's gripper, is still kept out of the tube.
- **DexArm meshes are offset from their joints.** Every DexArm mesh is shifted 2 mm along its own x axis relative to the original model, so its rotating parts pivot slightly off their mesh centers.
- **Approximate masses.** The custom gripper uses the stock SO-101 gripper's mass and inertia. The camera mounts, laser camera and base mount have no inertial data.
- **The DexArm URDF is unofficial.** The original author marked it as a work in progress. Its suction-cup module is included but commented out in the `.urdf`; uncomment it to switch end effectors.

## Credits and licenses

- **SO-101:** designed by TheRobotStudio ([SO-ARM100 project](https://github.com/TheRobotStudio/SO-ARM100), Apache License 2.0). The URDF and meshes come from the ROS 2 package by MuammerBay ([SO-ARM_ROS2_URDF](https://github.com/MuammerBay/SO-ARM_ROS2_URDF)). See `licenses/SO-ARM100_Apache-2.0.txt`.
- **Rotrics DexArm URDF:** by Yonatan Bisk ([URDF-of-DexArm](https://github.com/Rotrics-Dev/URDF-of-DexArm)), MIT License. See `licenses/DexArm-URDF_MIT.txt`.
- **Custom parts** (`baseforreal`, `gripper`, `camera_mount`, `lasercamera`) and the cell layout belong to this project's author.
