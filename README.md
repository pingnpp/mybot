# mybot
test mybot with command:

        $roslaunch bot_gazebo simulation.launch

---

## pkg bot_description

build from urdf (.xacro)
- geometry (.STL) , inertia from solidwork
  - **chassis.xacro** 
  - **wheel.xacro**
  - **caster.xacro**
- use plugin from [gazebo plugin](http://gazebosim.org/tutorials?tut=ros_gzplugins)
  - **lidar.xacro** 
  - **cam.xacro**

---

## pkg teleop

have 2 python node in this pkg
1. **key_input.py** receive command from keyboard

velocity from this node

<img src = "/images/IMG_0316.JPG" alt = "velocity" title = "from key_input" height = "400" />

2. **analyze_vel.py** build velocity trajectory with coefficient equation

velocity from this node

<img src = "/images/IMG_0317.JPG" alt = "velocity" title = "from analyze_vel" height = "400" />

---

## pkg bot_gazebo

- build world from gazebo tool (.world)
- simulation.launch
  - spawn bot_description to world
  - run teleop to control mybot
  - load .world to empty_world
