# ROS Tutorial: Create a workspace for catkin

## Objective
To create a workspace for catkin which packages can be built on

## Procedure

1. Make a catkin_ws directory

```bash
mkdir catkin_ws
```

2. Make a src directory under catkin_ws

```bash
cd catkin_ws/
mkdir src
```

3. Initialise catkin workspace

```bash
catkin_make
```

> [!WARNING]
> Make sure to initialise catkin workspace under catkin_ws and not under catkin_ws/src.

4. Source the Workspace Environment
```bash
cd
source ~/catkin_ws/devel/setup.bash
```

5. To make this permanent

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
