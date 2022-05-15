

![header line](Aspose.Words.123998f6-1d2d-4ce4-97be-f6df68e43d09.001.png)

**Robotics/ROS**
Obstacle detection

Accio Robotics

May 14, 2022

![horizontal line](Aspose.Words.123998f6-1d2d-4ce4-97be-f6df68e43d09.002.png)
# **Task**
We need to develop an obstacle detection module using 2D Lidar scans. The environment contains one or more moving objects. When objects are within the range of 2 meters, a ros message is published as an output containing: the number of obstacles, the distances to the obstacles and the sizes of obstacles.
## **Approach**
The approach to the solution is via a custom extended object tracking algorithm, known as DBSCAN. Steps: 

1. Detect the laser points in the area
1. Constructing the clusters of the laser points based on some filtering parameters
1. Filter out clusters that are outside the range of 2 meters
1. Apply the necessary mathematical operations to find: angle subtended at source, size of the object and average distance.

Main filtering parameters:

1. Maximum distance between two samples for one to be considered as in the neighborhood
1. Minimum number of samples in a neighborhood for a point to be considered


## **Assumptions**
1. Threshold Distance between 2 consecutive 'Non-infinity' points to consider them different clusters is taken as 0.5m.
1. Minimum number of points in a neighborhood for a point to be considered 8.
1. Any object or point within a proximity of 2m from the laser source, fulfilling the above 2 conditions, will be considered as the resulting obstacle.
1. There was a confusion regarding the determination of size of object in terms of start angle and end angle(as these two were not defined in the PDF). So size is determined in two ways:![](Aspose.Words.123998f6-1d2d-4ce4-97be-f6df68e43d09.003.png)
   1. Distance between two extreme ends of the object
   1. Angle subtended from the source to the objects
1. Whenever any point of a cluster comes within the 2m proximity of the laser, the whole cluster will be considered an obstacle.
1. A wall or a static object is within the proximity of 2m from the laser. As it fulfills all the conditions of the cluster, its data is also published on the topic.
## **Difficulties faced**
- There was a situation in which the point at the start of laser sweep and end of the sweep can be of the same cluster. Because of this exception, there was a major bug in the code which took me a few tries to debug that.
## **Result**
- My obstacle detection package can detect obstacles and determine the number of objects, the average distance, and size of the object with a maximum error of ±5% and average error of ±2% and publishes the required data on a topic “/fin\_data” with a custom message type “Fin.msg”.

![](Aspose.Words.123998f6-1d2d-4ce4-97be-f6df68e43d09.004.png)

## **P.S.: [sklearn.cluster](https://scikit-learn.org/stable/modules/classes.html#module-sklearn.cluster)**.DBSCAN is a function from sklearn package that can alternatively be used to find the clusters of objects and can output all the required deliverables.
##
![footer line](Aspose.Words.123998f6-1d2d-4ce4-97be-f6df68e43d09.001.png)

