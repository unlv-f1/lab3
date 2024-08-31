# Lab 2: Automatic Emergency Braking

## 1. Learning Goals

- Using the `LaserScan` message in ROS 2
- Instantaneous Time to Collision (iTTC)
- Safety critical systems

## 2. Overview

The goal of this lab is to develop a safety node for the race cars
that will stop the car from collision when travelling at higher
velocities. We will implement Instantaneous Time to Collision (iTTC)
using the `LaserScan` message in the simulator.

For different commonly used ROS 2 messages, they are kept mostly the
same as in ROS 1. You can use `ros2 interface show <msg_name>` to see
the definition of messages. Note for messages that are not installed
by default by the distro we use in our container, you'll have to first
install it for this to work.

This assignment will begin your group efforts. Every group member must
implement their own version of automatic emergency braking as a ROS
node. For a group of three students alice, bob, and charley, their
nodes would be:

    ${HOME}/sim_ws/src/
	   |
	   +-- alice_safety_node/
	   |
	   +-- bob_safety_node/
	   |
	   +-- charley_safety_node/
	   
The launch file must accept a command line parameter to select which
safety node to run. The remaining instructions use `safety_node` as a
placeholder for the individual implemenatations.


#### The `LaserScan` Message

[LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)
message contains several fields that will be useful to us. You can see
detailed descriptions of what each field contains in the API. The one
we'll be using the most is the `ranges` field. This is an array that
contains all range measurements from the LiDAR radially
ordered. You'll need to subscribe to the `/scan` topic and calculate
iTTC with the LaserScan messages.

#### The `Odometry` Message

Both the simulator node and the car itself publish
[Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
messages. Within its several fields, the message includes the cars
position, orientation, and velocity. You'll need to explore this
message type in this lab.

#### The `AckermannDriveStamped` Message

You've already used
[AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)
in the previous lab. It will be the message type that we'll use
throughout the course to send driving commands to the simulator and
the car. In the simulator, you can stop the car by sending an
`AckermannDriveStamped` message with the `speed` field set to 0.0.

## 3. The TTC Calculation

Time to Collision (TTC) is the time it would take for the car to
collide with an obstacle if it maintained its current heading and
velocity. We approximate the time to collision using Instantaneous
Time to Collision (iTTC), which is the ratio of instantaneous range to
range rate calculated from current range measurements and velocity
measurements of the vehicle.

As discussed in the lecture, we can calculate the iTTC as:

$$ iTTC=\frac{r}{\lbrace- \dot{r}\rbrace_{+}} $$

where $r$ is the instantaneous range measurements, and $\dot{r}$ is
the current range rate for that measurement.
And the operator $\lbrace \rbrace_{+}$ is defined as $\lbrace
x\rbrace_{+} = \text{max}( x, 0 )$.
The instantaneous range $r$ to an obstacle is easily obtained by using
the current measurements from the `LaserScan` message. Since the LiDAR
effectively measures the distance from the sensor to some obstacle.
The range rate $\dot{r}$ is the expected rate of change along each
scan beam. A positive range rate means the range measurement is
expanding, and a negative one means the range measurement is
shrinking.
Thus, it can be calculated in two different ways.
First, it can be calculated by mapping the vehicle's current
longitudinal velocity onto each scan beam's angle by using $v_x
\cos{\theta_{i}}$. Be careful with assigning the range rate a positive
or a negative value.
The angles could also be determined by the information in `LaserScan`
messages. The range rate could also be interpreted as how much the
range measurement will change if the vehicle keeps the current
velocity and the obstacle remains stationary.
Second, you can take the difference between the previous range
measurement and the current one, divide it by how much time has passed
in between (timestamps are available in message headers), and
calculate the range rate that way.
Note the negation in the calculation this is to correctly interpret
whether the range measurement should be decreasing or increasing. For
a vehicle travelling forward towards an obstacle, the corresponding
range rate for the beam right in front of the vehicle should be
negative since the range measurement should be shrinking. Vice versa,
the range rate corresponding to the vehicle travelling away from an
obstacle should be positive since the range measurement should be
increasing. The operator is in place so the iTTC calculation will be
meaningful. When the range rate is positive, the operator will make
sure iTTC for that angle goes to infinity.

After your calculations, you should end up with an array of iTTCs that
correspond to each angle. When a time to collision drops below a
certain threshold, it means a collision is imminent.

## 4. Automatic Emergency Braking with iTTC

For this lab, you will make a Safety Node that should halt the car
before it collides with obstacles. To do this, you will make a ROS 2
node that subscribes to the `LaserScan` and `Odometry` messages. It
should analyze the `LaserScan` data and, if necessary, publish an
`AckermannDriveStamped` with the `speed` field set to 0.0 m/s to
brake. After you've calculated the array of iTTCs, you should decide
how to proceed with this information. You'll have to decide how to
threshold, and how to best remove false positives (braking when
collision isn't imminent). Don't forget to deal with `inf`s or `nan`s
in your arrays.

To test your node, you can launch the sim container with `kb_teleop`
set to `True` in `sim.yaml`. Then in another `bash` session inside the
sim container, launch the `teleop_twist_keyboard` node from
`teleop_twist_keyboard` package for keyboard teleop. It should already
be installed as a dependency of the simulator. After running the
simulation, the keyboard teleop, and your safety node, use the reset
tool for the simulation and drive the vehicle towards a wall.

Note the following topic names for your publishers and subscribers:

- `LaserScan`: /scan
- `Odometry`: /ego_racecar/odom, specifically, the longitudinal velocity of the vehicle can be found in `twist.twist.linear.x`
- `AckermannDriveStamped`: /drive

## 5. Package

You can implement this node in either C++ or Python. Clone this repository
and make necessary changes to personalize the "**safety_node**" in the 
directory created in the previous lab.

```bash
# On the localhost
gordon@f1sim:~$ mkdir git
gordon@f1sim:~$ cd git/
gordon@f1sim:~/git$ git clone https://github.com/ctessler/f1tenth_lab2_template
```

Copy the skeleton into the same directory containing the gym workspace.
```bash
# On the localhost
gordon@f1sim:~$ cd sim_ws/src/
gordon@f1sim:~sim_ws/src/$ cp -r ~/git/f1tenth_lab2_template/safety_node .
gordon@f1sim:~sim_ws/src/$ ls
f1tenth_gym_ros safety_node
```
### 5.1 Personalizing your Node by Name 
You cannot have more than one node of the same name built and sourced to run at one time in ROS.
You will need to change the name of the node to fit the following convention: 
**student's_name**_*safety_node*. 

You will need to edit or change the following within the copied package:
1. package.xml
2. CMakeLists.txt 
3. setup.py
4. setup.cfg
5. safety_node/ to **student's_name**_*safety_node*
6. Both the cpp file and python file to **student's_name**_*safety_node*.py/cpp

An example package will be uploaded to this repository that will show changes required 
for an example node **bree_safety_node**.

Develop your solution directly in the simulation container, with your
package in `/sim_ws/src` alongside the simulation package. To do so,
edit `docker-compose.yml` in the `f1tenth_gym_ros` container.

```yaml
# sim_ws/src/f1tenth_gym_ros/docker-compose.yml
# Do for each node you create in this course
<<< SNIP >>>
    build: ./
    volumes:
      - .:/sim_ws/src/f1tenth_gym_ros                             # this should have been modified already
      - <abspath>/sim_ws/src/YOUR_NAME_safety_node:/sim_ws/src/YOUR_NAME_safety_node  #Newly added
    environment:
<<< SNIP >>>
```

If the following error is encountered, the safety_node has not been properly included in the container.
Retry adding it as a mount point and restarting the composition.
```bash
#In Container
root@445b65e71801:/sim_ws# colcon build --packages-select safety_node
[0.362s] WARNING:colcon.colcon_core.package_selection:ignoring unknown package ’safety_node’ in --packages-select
```
Note that if you're using Windows, make sure your files have Unix
style line endings. You can use `dos2unix` or have correct settings in
your text editor.

## 6. Physical Vehicle Changes
Given the maximum speed of 60 miles per hour on the vehicle, the 
maximum permissible TTC value is 1.5 seconds.

Within the simulator, the odometry topic is published to **/ego_racecar/odom**, 
on the vehicle the topic is published to **/odom**. Before deploying to 
the vehicle, subscriptions must be updated.

You must be able to change the parameter on the command line between the two for 
your implementation. You should be able to run the following 
command in your launch file:

```bash
ros2 launch safety_node_launch.py mode:=sim student:=bree ttc:=2.0
```

Note that the mode changes your odometry publisher between **/ego_racecar/odom**, for simulator, 
and **/odom**, for vehicle.

## 7. Deliverables

**Deliverable 1**: After you're finished, update the entire skeleton
package directory with your `safety_node` package. When all group
members have completed their implementations, select one group member
as the leader and create a new branch for the group's submission. 
The submission will contain your group's launch file, a **`SUBMISSION.md`** file 
with instructions to run all individual nodes, and the nodes themselves. 
The group leader will need to make the TA a collaborator on this "group" 
submission.



**Deliverable 2**: Students will present their implementations 
in class to the TA using the designated group leader's computer 
for the simulator portion.

- **Simulator**: Drive the car with keyboard teleop along the hallways
of Levine (the default map), showing it doesn't brake when traveling straight in the
hallway. You need to show that your safety node doesn't generate false
positives. i.e. The car doesn't suddenly stop while traveling down
the hallway. Then show the car driving towards a wall and braking
correctly. After a successful demonstration, students will be able to run 
their nodes on vehicle. 
- **Vehicle**: A successful vehicle demonstration does not 
collide with any wall during manual control. The TTC will be set to 3 seconds.


## 8. Grading Rubric 
### Individual
- Compilation: **20** Points
- Student Simulator Demo: **30** Points
- Student Vehicle Demo: **30** Points

### Group
- Launch File Implemented Correctly: **20** Points

### Total: 100pts