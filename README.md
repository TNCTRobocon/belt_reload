# belt_reload

ROS 2 Action Server for moving a belt by a configured incremental distance and resetting its position origin through CAN.

## Actions

Both actions use the existing `sequence_interfaces/action/Sequence` type. No changes to `sequence_interfaces` are required.

### `/reload/execute`

Send `operation: 2` (`OPERATION_PROCEED`).

The node captures the latest fresh position feedback and calculates:

```text
target_position_mm = current_position_mm + step_mm
```

It sends the absolute target position using CAN ID `0x280`, DLC 4, float32 millimetres, little-endian. The same target is retransmitted every `control_period_ms` until feedback remains within the configured tolerance for `stable_time_sec`.

### `/reload/reset_origin`

Send `operation: 4` (`OPERATION_HOME`).

The node repeatedly sends CAN ID `0x281`, DLC 1, data `0x01`, until a new position feedback sample reports a value within the configured tolerance around 0 mm for `stable_time_sec`.

The board is expected to reset both its measured position and its target position to 0 mm when it receives this command.

## CAN interface

- Receive: `/can_rx`, standard CAN ID `0x180`, DLC 4, float32 mm, little-endian.
- Send position target: `/can_tx`, standard CAN ID `0x280`, DLC 4, float32 mm, little-endian.
- Send origin reset: `/can_tx`, standard CAN ID `0x281`, DLC 1, uint8 value `1`.

The package publishes and subscribes directly to `can_msgs/msg/Frame`. The existing SocketCAN bridge is expected to provide `/can_rx` and consume `/can_tx`.

## RELOAD permission

The node subscribes to /shoot_belt/operation_state. It publishes a latched
std_msgs/msg/Bool on /reload/ready. RELOAD is enabled only after a
successful RUNNING (1) -> READY (2) shooting transition. Startup READY,
RESETTING -> READY, ERROR, CANCELLED, and RESET_FAILED do not grant permission.
The permission is consumed when /reload/execute is accepted and is not restored
after success, cancellation, or failure. A new successful shot is required.
The execute Action Server independently checks that shoot_belt is READY, so
a goal is rejected while the shooting Action is active.

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `step_mm` | 100.0 | Increment from the latest measured position for each execute action |
| `position_tolerance_mm` | 3.0 | Position tolerance for arrival |
| `stable_time_sec` | 0.1 | Required continuous time within tolerance |
| `control_period_ms` | 20 | Feedback monitoring and CAN command retransmission period |
| `feedback_timeout_ms` | 500 | Feedback timeout |
| `motion_timeout_sec` | 15.0 | Maximum duration of either action |

## Build and launch

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select belt_reload --symlink-install
source install/setup.bash
ros2 launch belt_reload belt_reload.launch.py
```

## Send actions

Move the belt by the configured increment:

```bash
ros2 action send_goal /reload/execute sequence_interfaces/action/Sequence "{operation: 2}" --feedback
```

Reset the board-side position and target to zero:

```bash
ros2 action send_goal /reload/reset_origin sequence_interfaces/action/Sequence "{operation: 4}" --feedback
```

## Cancellation and failures

When an action is canceled, the node sends the latest measured position once as an absolute target on CAN ID `0x280`, provided the feedback is fresh. For `reset_origin`, a new feedback sample received after the reset request is also required before sending this hold target.

On feedback timeout or motion timeout, the action is aborted without sending an additional CAN command. Motor-side safety behavior is expected to be implemented by the board firmware.
