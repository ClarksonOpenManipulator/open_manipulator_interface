To start low level control of the open manipulator run the following command:

```roslaunch open_manipulator_controllers position_controller.launch```

After launching the following node, issue basic position commands to the manipulator through the topic /position_contoller/position_command with the following format:

```rostopic pub /position_controller/position_command sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: [id_1, id_2, id_3, id_4, id_5, id_6, id_7]
position: [0, 0, 0, 0, 0, 0, -1.57]
velocity: [0, 0, 0, 0, 0, 0, 0]
effort: [0, 0, 0, 0, 0, 0, 0]" 
```

Where id_1 is the first joint, sequentially to id_7 which is the gripper.
NOTE: There is a limited safe range of the gripper (id_7) which is between -1.00 and -3.14. Any values outside this range may cause damage to the gripper 


Before running any of the above code, the USB latency between the computer and the manipulator must be lowered. Run the following 
terminal commands to resolve this:
```$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
```


Then verify that the latency has been set to 1 with the following command:

```$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer```
