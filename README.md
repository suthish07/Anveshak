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

- `self.outbuff`: Temporary List containing processed joystick data before publishing .
- `axes`: List of scaled joystick axes values.
- `buttons`: List of calculated button states from joystick message.
- `msg`: Instance of `std_msgs.Int32MultiArray` used for message creation. Later the data, layout are added and published to the topic 'stm_write'.

### Functions and Methods

- **`__init__`**: Initializes the node, setting up the publisher (`stm_write`) and subscriber (`joy_arm`).
- **`joyCallback`**: Callback function that processes incoming joystick commands (`sensor_msgs/Joy`), computes motor control values and button states, and updates the output buffer (`self.outbuff`).
- **`createMsg`**: Constructs a ROS message (`std_msgs.Int32MultiArray`) using the processed data from `self.outbuff`, configuring its layout and metadata.
- **`run`**: Main execution loop that publishes the constructed ROS message (`std_msgs.Int32MultiArray`) at a specified rate (`50 Hz`).



