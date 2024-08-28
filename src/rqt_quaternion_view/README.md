# rqt_quaternion_view
A display for showing quaternion information.

![Screenshot](/resource/screenshot.png)

## Usage
The mode of operation can be configured in the _rqt_ gear tool. Euler angles are calculated in aerospace ZYX order.

#### Manual Mode
Allows the user to input values directly into either the quaternion or euler angle fields. Pressing `Enter` during editting a field will instruct the application to normalize and update all text fields.

#### Subscriber Mode
The user can set a topic and field (must be of type geometry_msgs/Quaternion), which will then be displayed in the visualisation. The `Update Rate` field controls how often the application is instructed to refresh, with additional messages dropped.
