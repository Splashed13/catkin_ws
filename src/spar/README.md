# spar
A mid-level navigation software to implement flight actions with PX4 and ROS

For instructions on compiling, running, and available interfaces, see the `spar_node` [documentation here](spar_node/README.md).





---

## Explanation of ROS TF2 Python Scripts

The three Python scripts you've provided are part of a ROS (Robot Operating System) setup and are designed to handle transformations and communications about positions and orientations of different objects (like a UAV, camera, and a target) in a robot's environment. Here’s a detailed breakdown of each file's role and their interaction:

### 1. `tf2_broadcaster_frames.py`

**Responsibility:**
- This script is tasked with broadcasting the static and dynamic transformations between different coordinate frames.
- Specifically, it sets up and broadcasts the position and orientation of a camera relative to a UAV (`uav_name` to `camera_name`).

**How It Works:**
- It creates and sends a static transform for the camera, positioned slightly below the UAV and pointing downwards.
- The commented-out section suggests it was also intended to listen to a UAV's pose and dynamically update its position in a global frame, though this functionality is currently deactivated.

```python
def send_tf_camera():
    # [code to set up and send transform]
```

### 2. `tf2_broadcaster_target.py`

**Responsibility:**
- Broadcasts the dynamic position of a target relative to the camera frame and publishes a timestamp indicating when the target was "found."
  
**How It Works:**
- Calculates and sends a transform representing the target's position relative to the camera, assumed to be detected via unspecified computer vision algorithms.
- Publishes a `Time` message marking when the target was found, used by other system components for synchronization or event triggering.

```python
def send_tf_target():
    # [code to broadcast transform and publish time]
```

### 3. `tf2_listener.py`

**Responsibility:**
- Listens for transformations and the "target found" timestamp, using this data to determine the target's position relative to a global frame (`map`).

**How It Works:**
- Subscribes to the timestamp published by `tf2_broadcaster_target.py` and retrieves the transformation from the camera to the target at that exact moment.
- Converts this local frame data to global coordinates, potentially triggering further actions such as sending a robot to the target location.

```python
def callback_target_found(msg_in):
    # [code to lookup transform and handle target position]
```

### Interaction Among Scripts:

- **Broadcasting Transforms:** `tf2_broadcaster_frames.py` and `tf2_broadcaster_target.py` manage the broadcasting of transformations for different system parts, specifically the UAV's camera and detected targets.
- **Listening and Acting:** `tf2_listener.py` listens to these transformations and acts upon them by converting local frame data (target's position relative to the camera) into global coordinates for navigation or further processing.
- **Event Triggering:** The publication of the "target found" timestamp acts as a synchronization point or trigger, used by `tf2_listener.py` to look up the relevant transformation and process it accordingly.

Together, these scripts enable a system where a UAV can detect a target, communicate its position, and allow other system parts to use this information for further actions, leveraging ROS and its TF2 library.

---
