# ken_ghost
ROS2 packages for robot kendo arm



## Environment setup

- Add to dialout group
```
sudo adduser <user name> dialout
```

- [latency_timer setting](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting)


- Set Dynamixel return delay time to 0.

## Memo
- ros2 contorl currently supports JointStateController and Joint TrajectroyController.