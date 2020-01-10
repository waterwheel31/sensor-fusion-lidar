# Lidar Data Handling Excercise

This is an excersice of Lidar data obstacle detection, as a project of Udacity Sensor Fusion Nanodegree. 




## what this is doing? 
- simulate point cloud data (output of Lidar sensor) from a sequence of 3d data
- cluster the point clouds into segements 
    - first, segment into plane (ground) and obstacles, using RANSAC clustering algorithm
    - then, further cluster obstacles into different objects, using KD-Tree clustering algorithm (a high speed clustering algorithm)



### How to Run? 

- Install following 
    - cMake
    - PCL 
- Following code on terminal
    - mkdir build && cd build
    - cmake .. 
    - make 
    - ./environment 

    
### Environment 
- Tested on Ubuntu 18.04 

