## Assignment of the Research Track 2 course 
 GINNE VIKAS REDDY (matricola 5061211)

The nodes FSM and position server are written in ros2 , By using ros1_bridge they can be interfaces with the ros nodes and with the simulation in gazebo.

# Excuting the package(individual)

create a workspace in root repositories


```
mkdir "ros1"
```

```
cd "ros1"
```

```
mkdir src
```

Build the package

```
catkin_make
```

clone the repository 

```
git clone -b ros2 https://github.com/vikasreddy636/rt2_assignment1.git

```

create a new workspace for thr ros2 part


```
mkdir "mkdir ros2"
```

```
cd "ros2"
```

```
mkdir src
```

Build the package

```
colcon build
```

move the package  rt2_assignment1_ros2 to ros2 workspace.


In ros2 src clone the ros bridge

```

git clone https://github.com/ros2/ros1_bridge.git
```

open a new terminal

```
source ros.sh
```

```
cd ros1
```

```
catkin_make
```

open another terminal

```
source ros2.sh
```

```
cd ros2
```

```
colcon build --symlink-install --packages-skip ros1_bridge
```

```
 colcon build --packages-select ros1_bridge --cmake-force-configure

```

to start the simulation

```
source bridge.sh
```