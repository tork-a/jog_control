# JogFrame.msg

Message for jogging frames (ex. tool or hand) position and orientation.

```
Header header
string group_name
string link_name
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
bool avoid_collisions
```

# JogJoint.msg

Message for jogging joint angles.

```
Header header
string[] name
float64[] displacement
```
