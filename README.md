# LIMO-Velo [Alpha]
:red_circle: [16 February 2022] :red_circle: The project is on ``alpha`` stage, so be sure to **open Issues and Discussions** and give all the **feedback** you can!
Better to have 20 people giving lots of feedback than 1000 not saying anything.

Contact me at ``andreu.huguet@estudiantat.upc.edu`` for questions or ideas.

## A real-time, direct and tightly-coupled LiDAR-Inertial SLAM that works (surprisingly) well under high velocities - even with spinning LiDARs.
Designed for easy modifying via modular and easy to understand code. Relying upon [HKU-Mars](https://github.com/hku-mars)'s [IKFoM](https://github.com/hku-mars/IKFoM) and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) open-source libraries. Based also on their [FAST_LIO2](https://github.com/hku-mars/FAST_LIO).

### Tested and made for racing at Formula Student Driverless
Tested on and made for Barcelona's own "[Xaloc](https://www.youtube.com/watch?v=ly_ax8w-T7E&feature=emb_logo)". Common working speeds are 20m/s in straights and 100deg/s in the turns.

<p align="center">
  <img src="./config/docs/img/Localization.gif" alt="Perfomance of the algorithm" /><br />
  Visualization of the algorithm with <code>delta = 0.01</code> (100Hz)
</p>

### Centimeter-level accuracy is kept under racing speeds
Only algorithm that can deliver centimeter-level resolution on real-time. See the part of my thesis where I explain the algorithm and its results: [LIMOVelo + Results](https://github.com/Huguet57/LIMO-Velo/blob/main/config/docs/Thesis%20-%20LIMOVelo%20%2B%20Results.pdf).

<p align="center">
  <img src="./config/docs/img/cones-comparison.png" alt="Map comparison - Cones" /><br />
  Comparison of cones under racing speeds running all algorithms in real-time, except for LIO-SAM (-r 0.5). It failed otherwise.
</p>

### Designed to be easily understood even by beginners
Developing an algorithm for a team requires the algorithm to be easy enough to understand being passed through generations.

<p align="center">
  <img src="./config/docs/img/pipeline.png" alt="Map comparison - Cones" /><br />
  LIMO-Velo's pipeline. Here are seen the different modules (blue), data (orange) and libraries (dark green).
</p>

# Dependencies
- [Ubuntu](https://ubuntu.com) (tested on 20.04)
- [ROS](http://wiki.ros.org/ROS/Installation) (tested on Noetic)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PCL](http://www.pointclouds.org/downloads/linux.html) (tested on 1.8)

# Support for all types of LiDAR
- Velodyne
- Ouster
- Hesai
- Livox

# Using LIMO-Velo

## 0. Cloning the repository
When cloning the repository, we also need to clone the [IKFoM](https://github.com/hku-mars/IKFoM) and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) submodules. Hence we will use the ``--recurse-submodules`` tag.

``git clone --recurse-submodules https://github.com/Huguet57/LIMO-Velo.git``

## 1. Compiling the code
We either can do ``catkin_make`` or ``catkin build`` to compile the code. By default it will compile it optimized already

## 2. Running LIMO-Velo
To run LIMO-Velo, we can run the launch file ``roslaunch limovelo test.launch`` if we want a visualization or ``roslaunch limovelo run.launch`` if we want it without.

### 2.1 Debugging LIMO-Velo
An additional launch file ``roslaunch limovelo debug.launch`` is added that uses [Valgrind](https://valgrind.org/) as a analysing tool to check for leaks and offers detailed anaylsis of program crashes.

## 3. Changing parameters
To adapt LIMO-Velo to our own hardware infrastructure, a [YAML](https://yaml.org/) file ``config/params.yaml`` is available and we need to change it to our own topic names and sensor specs.

Relevant parameters are:
- ``real_time`` if you want to get real time experience.
- ``mapping_offline`` is on an pre-alpha stage and it does not work 100% as it should of.
- ``heuristic`` which you can choose how you want the initialization of the pointcloud sizes (sizes =: deltas, in seconds).

## 4. Modifying the LiDAR driver to get true real-time performance
*TODO* - This section is intended to explain how to modify the LiDAR driver to increase its frequency by publishing parts of the pointcloud instead of waiting for all of it.

# Sample datasets
Try ``xaloc.launch`` with Xaloc's own rosbags. High velocity in the straights (~15m/s) and tight turns (~80deg/s).

Find them here: [https://drive.google.com/drive/folders/1uex3OhSXszxFhOLbmLHyYZTLkc01otxx?usp=sharing](https://drive.google.com/drive/folders/1uex3OhSXszxFhOLbmLHyYZTLkc01otxx?usp=sharing)

See Issue [#10](https://github.com/Huguet57/LIMO-Velo/issues/10) to see other sample datasets. Don't hesitate to ask there for more data on specific scenarios/cases.

# References
- [IKFoM](https://github.com/hku-mars/IKFoM): Iterated Kalman Filters on Manifolds
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree): Incremental KD-Tree for Robotic Applications
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO): Fast and Direct LIO SLAM

# TODO list
### Urgent fixes
- [ ] Rethink ``mapping_offline`` (see Discussions)
- [ ] Adding Livox as a LiDAR type
- [ ] Rewrite the most confusing parts according to [qpc001's feedback](https://github.com/qpc001/LIMO-Velo/commit/a45b6489cbbcefc68515565eeaeaed267c976da8). (thank you!)
- [ ] [Livox] Investigate why Fast-LIO outperforms it in 2020-09-16-quick-shack.bag (given by the Fast-LIO authors). 

### Design choices
- [ ] Renew Buffer private structure. Interesting answer in StackOverflow: [https://stackoverflow.com/a/67236232](https://stackoverflow.com/a/67236232)
- [ ] Simplify the upsampling in the Compensator. (called it.)

### Fixes to investigate
- [ ] Interpolation and smoothing of states when mapping offline
- [ ] Erase unused (potentially dangerous) points in the map
- [ ] Check if need to add point in map
- [ ] Try to add a module for removing dynamic objects such as people or vehicles
- [ ] Use UKF instead of EKF
- [ ] Add vision buffer and ability to paint the map's points
- [ ] Initialize IMU measurements
