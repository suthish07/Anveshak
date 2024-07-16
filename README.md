# Anveshak
# ROS Node: arm_drive

This ROS node reads joystick commands from the topic `joy_arm`, processes them, and publishes processed data to the topic `stm_write`.

## Requirements

- ROS Kinetic (or any compatible version)
- Python 3
- Dependencies:
  - `rospy`
  - `std_msgs`
  - `sensor_msgs`

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

### Example

```python
class Node:
    def __init__(self):
        self.outbuff = [0] * 6  # Output buffer for joystick data
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        rospy.init_node('arm_drive')
        rospy.Subscriber('joy_arm', sensor_msgs.Joy, self.joyCallback)

    def joyCallback(self, msg):
        axes = [int(msg.axes[i] * 0xFF) for i in range(5)]  # Scaled joystick axes
        buttons = [(msg.buttons[1] - msg.buttons[3]) * 255]  # Calculated button states
        buttons.append((msg.buttons[0] - msg.buttons[4]) * 255)
        
        self.outbuff[0] = -axes[1]
        self.outbuff[1] = -axes[0]
        self.outbuff[2] = -buttons[1] + buttons[0]
        self.outbuff[3] = axes[2]
        self.outbuff[4] = -axes[3]
        self.outbuff[5] = -buttons[1] - buttons[0]
        
        print(self.outbuff)

    def createMsg(self, buff):
        msg = std_msgs.Int32MultiArray()
        msg.data = buff[:]
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        
        msg.layout.dim = [std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        
        return msg

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = self.createMsg(self.outbuff)
            self.pub.publish(msg)
