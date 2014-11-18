These are the ROS packages to go with [my report on Laser Scanners for Real-Time Odometry in Forests](https://github.com/daxtens/laser-scanner-forests-report).

You also need the polar scan matcher from [my `scan_tools` fork](https://github.com/daxtens/scan_tools).

There are readmes scattered throughout the repository which should help you navigate the individual packages. 

A good place to start is:

 * copy the tennis court bag from `laser_hover/demo/flying` to `laser_hover/demo/mounted.bag`.
 * cd to `tree_fix/demo`
 * `roslaunch forcelocalise.launch`
