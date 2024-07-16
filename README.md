# Anveshak
# ROS Node: arm_drive

This ROS node reads joystick commands from the topic `joy_arm`, processes them, and publishes processed data to the topic `stm_write`.


## Usage

1. Clone this repository into your ROS workspace (`catkin_ws/src`).
2. Build your workspace: `cd catkin_ws && catkin_make`
3. Source the setup file: `source devel/setup.bash`
4. Launch the node: `roslaunch <package_name> arm_drive.launch`

## Parameters

- `joy_arm` (sensor_msgs/Joy): Input topic for joystick commands.
- `stm_write` (std_msgs/Int32MultiArray): Output topic for processed commands.

## Nodes

### `arm_drive`

#### Subscribed Topics

- `joy_arm` (sensor_msgs/Joy): Joystick input commands.

#### Published Topics

- `stm_write` (std_msgs/Int32MultiArray): Processed joystick commands.

## Node Details

### Variables Used

- `self.outbuff`: List containing processed joystick data before publishing.
- `axes`: List of scaled joystick axes values.
- `buttons`: List of calculated button states from joystick message.
- `msg`: Instance of `std_msgs.Int32MultiArray` used for message creation.

### Detailed Explanation

The `arm_drive` node initializes a ROS publisher and subscriber. It processes incoming joystick commands (`sensor_msgs/Joy`) in the `joyCallback` method, computes motor control values based on joystick axes and buttons, and publishes the results to `stm_write` using the `createMsg` method.


