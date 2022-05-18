# obstacle_cluster_detection

I developed an obstacle detection module using 2D Lidar scans. The environment contains one or more moving objects. When objects are within the range of 2 meters, a ros message is published as an output containing: the number of obstacles, the distances to the obstacles and the sizes of obstacles.
## **Approach**
The approach to the solution is via a custom extended object tracking algorithm, known as DBSCAN. Steps: 

1. Detect the laser points in the area
2. Constructing the clusters of the laser points based on some filtering parameters
3. Filter out clusters that are outside the range of 2 meters
4. Apply the necessary mathematical operations to find: angle subtended at source, size of the object and average distance.

#### Main filtering parameters:

1. Maximum distance between two samples for one to be considered as in the neighborhood
2. Minimum number of samples in a neighborhood for a point to be considered


## **Assumptions**
1. Threshold Distance between 2 consecutive 'Non-infinity' points to consider them different clusters is taken as 0.5m.
2. Minimum number of points in a neighborhood for a point to be considered 8.
3. Any object or point within a proximity of 2m from the laser source, fulfilling the above 2 conditions, will be considered as the resulting obstacle.
4. There was a confusion regarding the determination of size of object in terms of start angle and end angle(as these two were not defined in the PDF). So size is determined in two ways:

   **a.** Distance between two extreme ends of the object
   
   **b.** Angle subtended from the source to the objects
   
   ![](https://github.com/snktshrma/obstacle_cluster_detection/blob/main/images/3.png)
   
5. Whenever any point of a cluster comes within the 2m proximity of the laser, the whole cluster will be considered an obstacle.
6. A wall or a static object is within the proximity of 2m from the laser. As it fulfills all the conditions of the cluster, its data is also published on the topic.

## **Result**
- My obstacle detection package can detect obstacles and determine the number of objects, the average distance, and size of the object with a maximum error of ±5% and average error of ±2% and publishes the required data on a topic “/fin\_data” with a custom message type “Fin.msg”.

![](https://github.com/snktshrma/obstacle_cluster_detection/blob/main/images/4.png)

