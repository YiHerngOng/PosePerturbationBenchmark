# Benchmarking robotic grippers and grasping strategies at non-ideal poses
We introduce a novel robotic manipulation benchmark to evaluate the robustness of robot gripper and its grasping strategies by starting the hand pose relatively random to the object.
This is a [ROS](https://www.ros.org/) package that requires [MoveIT](https://ros-planning.github.io/moveit_tutorials/) and [Kinova ROS](https://github.com/Kinovarobotics/kinova-ros) installed.
This benchmark runs autonomously, without human intervention, to determine the limits of the gripper based on our structured hand pose perturbations. 
It has been tested on Kinova Jaco 7 DOF with KG-3 gripper. Detailed instruction of installing essential package will be updated. 

Go */scripts* folder
```
cd pose_perturbation_benchmark/scripts
```
```
python kinova_path_planning_new.py 
```


<!-- Protocol of pose perturbation / hallucination benchmark
1. generate pose variation from ppB_Benchmark.py
2. read file 
3. pick an axis from xyz
4. check if extremes are tested
5. Test extremes and conduct binary search
6. Repeat 3 - 5 until all translational axes are found
7. Compute translational variation
8. randomly pick an axis from rpw
9. check if extremes are tested
10. Test extremes and conduct binary search
11. Repeat 8 - 10 until all translational axes are found
12. Compute orientation variation
13. Randomly pick a pose and start grasping -->