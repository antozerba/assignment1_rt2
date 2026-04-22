# Assignment1_rt2

ROS2 package for controlling a robot toward a user-specified target, based on **Action Server/Client**, **TF2**, and **velocity publishing**.

---

## Project Structure

```
assignment1_rt2/
├── action/
│   └── Target.action          # Action interface definition
├── src/
│   ├── ui_node.cpp            # User interface node (action client)
│   └── controller.cpp         # Controller node (action server)
├── CMakeLists.txt
└── package.xml
```

---

## Components

### 1. Action Interface — `action/Target.action`

Defines the communication protocol between client and server:

```
float32[] target_pose      # Goal:     target position [x, y, theta]
---
float32[] final_pose       # Result:   final pose reached by the robot
---
float32[] partial_pose     # Feedback: current pose during execution
```

- **Goal**: the client sends the desired position as an array `[x, y, theta]`
- **Result**: once finished, the server returns the robot's final pose
- **Feedback**: during execution, the server periodically sends the current pose

---

### 2. UI Node — `src/ui_node.cpp` → executable `target_interface`

**Action client** node that handles user interaction and goal sending.

**Methods:**

| Method | Description |
|---|---|
| `get_input()` | Reads target coordinates `x y theta` from stdin |
| `make_target()` | Broadcasts the static TF2 transform `base_link → target` |
| `send_goal()` | Sends the goal to the action server and registers callbacks |
| `goal_response_callback()` | Logs whether the goal was accepted or rejected |
| `feedback_callback()` | Logs the partial pose received during execution |
| `result_callback()` | Logs the final pose and handles abort/cancel cases |

**Execution flow:**

```
Node startup
    │
    ▼
get_input()          ← reads x, y, theta from stdin
    │
    ▼
make_target()        ← broadcasts static TF: base_link → target
    │
    ▼
Timer (500ms)        ← waits for the system to be ready
    │
    ▼
send_goal()          ← sends goal to the action server
    │
    ├── feedback_callback()   ← periodic pose updates
    └── result_callback()     ← final result
```

---

### 3. Controller Node — `src/controller.cpp` → component `controller_component`

**Action server** node loaded as a ROS2 component. Handles execution of the movement toward the target.

**Infrastructure:**

| Element | Type | Purpose |
|---|---|---|
| `action_server` | `rclcpp_action::Server<Target>` | Receives and manages goals |
| `tf_buffer` + `tf_listener` | TF2 | Reads the robot's current transform |
| `vel_pub` | `Publisher<Twist>` on `/cmd_vel` | Publishes velocity commands |

**Server callbacks:**

| Callback | Behavior |
|---|---|
| `handle_goal()` | Always accepts the goal and logs the received coordinates |
| `handle_cancel()` | Always accepts cancellation requests |
| `handle_accepted()` | Spawns a detached thread that runs `execute()` |
| `execute()` | *(to be implemented)* Navigation logic toward the target |

> **Note:** The `execute()` method contains the main control logic, which is not yet implemented. This is where the navigation algorithm should go (e.g. proportional control on position/orientation by reading TF2 and publishing to `/cmd_vel`).

---

## Dependencies

Declared in `package.xml`:

| Package | Usage |
|---|---|
| `rclcpp` | Core ROS2 C++ API |
| `rclcpp_action` | Action server/client framework |
| `rclcpp_components` | Loading the controller as a component |
| `tf2_ros` | Broadcasting and listening to TF transforms |
| `geometry_msgs` | `Twist` (velocity) and `TransformStamped` messages |
| `rosidl_default_generators` | Generation of the `.action` interface |

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select assignment1_rt2
source install/setup.bash
```

---

## Running

Launch the controller as a component inside a container:

```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager assignment1_rt2 target_controller::TargetController
```

Launch the UI node (in a second terminal):

```bash
ros2 run assignment1_rt2 target_interface
```

Enter the coordinates when prompted:

```
Enter target position (x y theta): 2.0 1.5 1.57
```

---

## Architectural Notes

- The controller is registered as a **component** (`RCLCPP_COMPONENTS_REGISTER_NODE`) to be loaded dynamically, rather than as a plain executable. This makes it compatible with ROS2 lifecycle management.
- The `target` TF frame is published as **static** relative to `base_link`, so it remains fixed for the entire duration of the goal.
- Goal execution runs in a **detached thread** (`std::thread::detach`) to avoid blocking the main executor during navigation.