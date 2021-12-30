# LIMO-Velo (based on Fast-LIO2)
## A real-time, direct and LM-detached LIO SLAM.
Designed for easy modifying via modular and easy to understand code. Relying upon [HKU-Mars](https://github.com/hku-mars)'s [IKFoM](https://github.com/hku-mars/IKFoM) and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) open-source libraries. 

![Perfomance of the algorithm](./config/docs/img/Localization.gif)

## TODO list
- [ ] Interpolation and smoothing of states when mapping offline
- [ ] Check quality vs. other SOTA algorithms
- [ ] Add degeneracy detection and correction
- [ ] Try to add a module for removing dynamic objects such as people or vehicles

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

## 4. Modifying the LiDAR driver to get true real-time performance
*TODO* - This section is intended to explain how to modify the LiDAR driver to increase its frequency by publishing parts of the pointcloud instead of waiting for all of it.

## References
- [IKFoM](https://github.com/hku-mars/IKFoM): Iterated Kalman Filters on Manifolds
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree): Incremental KD-Tree for Robotic Applications
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO): Fast and Direct LIO SLAM
