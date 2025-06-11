# ROS Tutorial: Writing a ROS Node with Python

## Objective
To familiarise with how to write your own ROS node and how to start it in the terminal 

## Procedure

1. Make a scripts foler

```bash
cd catkin_ws/src/my_robot_controller/
mkdir scripts
```

2. Create a python file

```bash
cd scripts/
touch my_first_node.py
```

3. Make the python file excutable

```bash
chmod +x my_first_node.py
```
4. Under catkin_ws/src, open VScode Editor

```bash
cd ..
cd catkin_ws/src/
code .
```

  If VScode is not installed, run

```bash
sudo snap install code --clasic
```

5. In my_first_node.py, initialise the node with name "test_node"

```python
#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
  rospy.init_node("test_node")
```

6. Add these lines of code
```python

  rospy.loginfo("Hello from test node") 
  rospy.logwarn("This is a warning")
  rospy.logerr("This is an error")

```

  Code should look as shown below

```python
#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
  rospy.init_node("test_node")
  rospy.loginfo("Hello from test node") 
  rospy.logwarn("This is a warning")
  rospy.logerr("This is an error")

  rospy.sleep(1.0)# in seconds
```
7. In a new terminal, start a ROS master
```bash
roscore
```

8. Run the ROS node
```bash
rosrun my_robot_controller my_first_node.py
```
The results in the terminal should be shown as below.
![Image](https://github.com/user-attachments/assets/dade2039-51ae-4144-97d2-85145ba7262f)


9. Now add these lines code. These code will print out "Hello" in terminal infinitely every 0.1 seconds (10 hz) until the node is terminated. This will be useful in a scenario where robotic sensor data has to be printed in the terminal continuously.
```python

rate = rospy.Rate(10) # rate is in hertz
while not rospy.is_shutdown(): # will loop until the node is terminated
  rospy.loginfo("Hello")
  rate.sleep()

```
  Code should look as shown below

```python
#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
  rospy.init_node("test_node")
  rospy.loginfo("Hello from test node") 
  rospy.logwarn("This is a warning")
  rospy.logerr("This is an error")

  rate = rospy.Rate(10) # rate is in hertz
  while not rospy.is_shutdown(): # will loop until the node is terminated
    rospy.loginfo("Hello")
    rate.sleep()
```
10. Run the ROS node
```bash
rosrun my_robot_controller my_first_node.py
```
The results in the terminal should be shown as below.
![Image](https://github.com/user-attachments/assets/c864ec31-ec5f-4981-a268-1198018b0b62)
