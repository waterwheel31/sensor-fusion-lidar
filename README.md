# Lidar-Data Obstacle Detection Excercise

This is an excersice of Lidar data obstacle detection, as a project of Udacity Sensor Fusion Nanodegree. 




## What this is doing? 
- Read streaming point cloud data (output of Lidar sensor) 
- Filter data to decrease data size
    - Down sampling, using Vowxel grid to reduce the data size per region
    - Cropping, to focus on important region
- Cluster the point clouds into segements 
    - Segment into "obstacles" and "ground", using RANSAC clustering algorithm
    - Then, further cluster "obstacles" into different objects, using Euclidean clustering algorithm with KD-Tree
    - Note:  The performance largely depends on clustering parameters, and they have not been optimized yet


## How to Run? 

- Install followings 
    - cMake (for compiling)
    - PCL (Point Cloud Library; www.pointclouds.org)
- Type following commands on terminal
    - mkdir build && cd build
    - cmake .. 
    - make 
    - ./environment 

    
## Environment 
- Tested on Ubuntu 18.04 

