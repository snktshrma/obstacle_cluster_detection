#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <bits/stdc++.h>
#include "cluster_counter/Fin.h"

using namespace std;

#define PI 3.14159265


ros::Publisher pub;


const float thresh_dist = 0.5;  //Threshold Distance between 2 consecutive 'Non-infinity' points to consider them different clusters


// Callback function - subscribes to laser scan

double findAngle(double M1, double M2)
{
    // Store the tan value  of the angle
    double angle = abs((M2 - M1)
                       / (1 + M1 * M2));
 
    // Calculate tan inverse of the angle
    double ret = atan(angle);
 
    // Convert the angle from
    // radian to degree
    double val = (ret * 180) / PI;
 
    // Print the result
    return val;
}




double aveX(int sum, vector<int> numOfPoints, vector<long double> storeMinDist, int j)
{
       
      if(storeMinDist[sum] == 100000){
        sum = sum + 1;
        aveX(sum,numOfPoints,storeMinDist, j);
      }
      else{
        double first = storeMinDist[sum];
        return first;
      }

      

    }




double aveY(int sum, vector<int> numOfPoints, vector<long double> storeMinDist, int j)
{
       
      if(storeMinDist[sum + numOfPoints[j] - 1] == 100000){
        sum = sum - 1;
        aveY(sum,numOfPoints,storeMinDist, j);
      }
      else{
        double second = storeMinDist[sum + numOfPoints[j] - 1];
        return second;
      }

      

    }


double findSlopeX1(int sum, vector<int> numOfPoints, vector<double> sign_distX, vector<double> sign_distY, int j)
{
       
      if(sign_distX[sum] == 100000){
        sum = sum + 1;
        findSlopeX1(sum,numOfPoints,sign_distX, sign_distY, j);
      }
      else{
        double finX1 = sign_distX[sum];
        return finX1;
      }

      

    }


double findSlopeX2(int sum, vector<int> numOfPoints, vector<double> sign_distX, vector<double> sign_distY, int j)
{
      
      if(sign_distX[sum+numOfPoints[j] - 1] == 100000){
        sum = sum - 1;
        findSlopeX2(sum,numOfPoints,sign_distX, sign_distY, j);
      }
      else{
        double finX2 = sign_distX[sum+numOfPoints[j] - 1];
        return finX2;
      }


}


double findSlopeY1(int sum, vector<int> numOfPoints, vector<double> sign_distX, vector<double> sign_distY, int j)
{
      
      if(sign_distY[sum] == 100000){
        sum = sum + 1;
        findSlopeY1(sum,numOfPoints,sign_distX, sign_distY, j);
      }
      else{
        double finY1 = sign_distY[sum];
        return finY1;
      }

    }

double findSlopeY2(int sum, vector<int> numOfPoints, vector<double> sign_distX, vector<double> sign_distY, int j)
{
      
      if(sign_distY[sum+numOfPoints[j] - 1] == 100000){
        sum = sum - 1;
        findSlopeY2(sum,numOfPoints,sign_distX, sign_distY, j);
      }
      else{
        double finY2 = sign_distY[sum+numOfPoints[j] - 1];
        return finY2;
      }

    }


void cluster_callback(const sensor_msgs::LaserScan::ConstPtr& lsrscan_msg){

  cluster_counter::Fin msg;

  int numOfClusters = 0;    //Number of Clusters Counter
  vector<int> numOfPoints;  //Number of Points for Each Cluster
  vector<long double> distPoints;   //Distace of points for each Cluster
  vector<long double> storeMinDist; //Store min distance for each Cluster
  vector<double> sign_distX;
  vector<double> sign_distY;



  int length_range = 720;   //Length of Ranges array from Laser Scan;
  ROS_INFO("Length of Array = %d",length_range);

  bool prev_inf;            //To check if previous scan point value was infinity or not

  // For First-last Concatenation (Explained Later)
  float x_first=0.0,x_last=0.0;
  float y_first=0.0,y_last=0.0;
  //

  // For x,y coordinates of each point
  float x_prev=0.0,x_curr=0.0;
  float y_prev=0.0,y_curr=0.0;
  float theta;
  float distance;
 

  // To check for and store the first point before loop is started
  if(isfinite(lsrscan_msg->ranges[0])){
    numOfClusters++;
    numOfPoints.push_back(1);
    


    prev_inf = false;
    theta = lsrscan_msg->angle_min;

    x_prev = lsrscan_msg->ranges[0]*cos(theta);
    y_prev = lsrscan_msg->ranges[0]*sin(theta);


    if(lsrscan_msg->ranges[0] > 0.8){
    distPoints.push_back(lsrscan_msg->ranges[0]);
    sign_distX.push_back(x_prev);
    sign_distY.push_back(y_prev);
    }
    else{
      distPoints.push_back(100000);
      sign_distX.push_back(100000);
      sign_distY.push_back(100000);
    }
    
    

    // For First-last Concatenation
    x_first = x_prev;
    y_first = y_prev;
    //

  }
  else{prev_inf = true;}


//////// Loop to check each point and find where new clusters start
  for(int i=1;i<length_range;i++){

     //If the point is a Finite value only then coordinates and cluster check would be done
     if(isfinite(lsrscan_msg->ranges[i])){
	theta = lsrscan_msg->angle_min + (i*lsrscan_msg->angle_increment);

	x_curr = lsrscan_msg->ranges[i]*cos(theta);
	y_curr = lsrscan_msg->ranges[i]*sin(theta);

  
	
	// For First-last Concatenation
	if(numOfClusters==0){
	  x_first = x_curr;
	  y_first = y_curr;
	}
	//


	if(prev_inf==false){
	    distance = sqrt(pow((x_curr-x_prev),2)+pow((y_curr-y_prev),2));
	    if(distance>thresh_dist){
		numOfClusters++;
		numOfPoints.push_back(1);
    if(lsrscan_msg->ranges[i] > 0.8){
      sign_distX.push_back(x_curr);
  sign_distY.push_back(y_curr);
    distPoints.push_back(lsrscan_msg->ranges[i]);
  }
  else{
    sign_distX.push_back(100000);
      sign_distY.push_back(100000);
    distPoints.push_back(100000);
  }


	    }
	    else{
        numOfPoints[numOfClusters-1]++;
        if(lsrscan_msg->ranges[i] > 0.8){
          sign_distX.push_back(x_curr);
  sign_distY.push_back(y_curr);
    distPoints.push_back(lsrscan_msg->ranges[i]);
  }
  else{
    sign_distX.push_back(100000);
      sign_distY.push_back(100000);
    distPoints.push_back(100000);
  }

      }
      
	}
	else{
	    numOfClusters++;
	    numOfPoints.push_back(1);
      if(lsrscan_msg->ranges[i] > 0.8){
        sign_distX.push_back(x_curr);
  sign_distY.push_back(y_curr);
    distPoints.push_back(lsrscan_msg->ranges[i]);
  }
  else{
    sign_distX.push_back(100000);
      sign_distY.push_back(100000);
    distPoints.push_back(100000);
  }
	}
	x_prev = x_curr;
	y_prev = y_curr;
	prev_inf = false;
    }

    //Else the point is infinity, update prev_inf to true
    else{prev_inf=true;}
  }
//////// Loop End -------------------



//////// First-last Concatenation
/*
The sweep is 360 degrees. This means that the point at the start
of the sweep and the end of the sweep CAN be of the same cluster.

The loop above will treat them as different clusters.

Below function checks if first and last points are within the threshold distance.
If yes, they are treated as the same cluster and no. of points for both are added. 
*/

  x_last = x_prev;
  y_last = y_prev;

  distance = sqrt(pow((x_first-x_last),2)+pow((y_first-y_last),2));


  if(distance<=thresh_dist){

  auto lastNum = numOfPoints.back();
  auto start = distPoints.end() - lastNum + 1;
  auto end = distPoints.end();

  auto startX = sign_distX.end() - lastNum + 1;
  auto endX = sign_distX.end();

  auto startY = sign_distY.end() - lastNum + 1;
  auto endY = sign_distY.end();



  vector<double> res(numOfPoints.back());
  vector<double> resX(numOfPoints.back());
  vector<double> resY(numOfPoints.back());

  copy(start, end, res.begin());
  copy(startX, endX, resX.begin());
  copy(startY, endY, resY.begin());

  distPoints.erase(start - 1, end);
  sign_distX.erase(startX - 1, endX);
  sign_distY.erase(startY - 1, endY);

  storeMinDist.insert(storeMinDist.begin(), res.begin(), res.end());
  storeMinDist.insert(storeMinDist.end(), distPoints.begin(), distPoints.end());
  res.clear();

  sign_distX.insert(sign_distX.begin(), resX.begin(), resX.end());
  resX.clear();

  sign_distY.insert(sign_distY.begin(), resY.begin(), resY.end());
  resY.clear();


   //  cout << numOfPoints.back() << " Num prev" << endl;
   //  cout << numOfPoints.front() << " Num first" << endl;
     numOfClusters--;
     numOfPoints.front()=numOfPoints.front()+numOfPoints.back();
     numOfPoints.pop_back();
  }
  else{
    storeMinDist = distPoints;
  }
////////


  //Print values of cluster and points
  ROS_INFO("No. of Clusters : %d", numOfClusters);
  ROS_INFO("No. of Points in ");
  int sum = 0;

  vector<double> aveD;
  vector<double> minD;
  vector<double> angle;
  vector<double> size;
  int num = 0;


  for(int j=0;j<numOfPoints.size();j++){

    replace(storeMinDist.begin(), storeMinDist.end(), 0, 1000000);
    replace(sign_distX.begin(), sign_distX.end(), 0, 1000000);
    replace(sign_distY.begin(), sign_distY.end(), 0, 1000000);

    auto beg = storeMinDist.begin() + sum;
    auto end = beg + numOfPoints[j] - 1 ;


    //replace(storeMinDist.begin(), storeMinDist.end(), 0, 1000000);

    //cout <<sign_distX.size() << "\t" << storeMinDist.size() << endl;




    long double minEle =  *min_element(beg, end);

    double meanX = aveX(sum, numOfPoints, storeMinDist, j);
    double meanY = aveY(sum, numOfPoints, storeMinDist, j);

    double x1 = findSlopeX1(sum, numOfPoints, sign_distX, sign_distY, j);
    double x2 = findSlopeX2(sum, numOfPoints, sign_distX, sign_distY, j);
    double y1 = findSlopeY1(sum, numOfPoints, sign_distX, sign_distY, j);
    double y2 = findSlopeY2(sum, numOfPoints, sign_distX, sign_distY, j);

    //for(int i = 0; i < storeMinDist.size(); i++) {
     // cout << sign_distX[i] << "\t";
   // }


    double m1 = y1/x1;
    double m2 = y2/x2;

    double mean = (meanX + meanY)/2;

    double sizeD = distance = sqrt(pow((x1-x2),2)+pow((y1-y2),2));

    double  angleD = findAngle(m1,m2);

   // findAngle(m1,m2);






    if(minEle < 2 and numOfPoints[j] > 7) {

      //ROS_INFO("Angle %f", angleD);
      //ROS_INFO("Mean %f", mean);
      ROS_INFO("size %f", sizeD);

    ROS_INFO("Cluster %d : %d points", j+1, numOfPoints[j]);
    //ROS_INFO("Cluster %d : %Lf Min Distance", j+1, minEle);

    aveD.push_back(mean);
    minD.push_back(minEle);
    angle.push_back(angleD);
    size.push_back(sizeD);
    //cout << storeMinDist.size() << endl;

   // for(int i = 0; i < storeMinDist.size(); i++) {
     // cout << storeMinDist[i] << "\t";
    //}
    num = num+1;
     
    
  }
  sum = sum + numOfPoints[j];

  }

  msg.num = num;
  msg.aveD = aveD;
  msg.minD = minD;
  msg.angle = angle;
  msg.size = size;
  ROS_INFO("---------------");

  pub.publish(msg);

  storeMinDist.clear();
  distPoints.clear();
  
  numOfPoints.clear();
  //
}










int main(int argc, char** argv){

  ros::init(argc, argv, "cluster_counter");
  ros::NodeHandle n;

  pub = n.advertise<cluster_counter::Fin>("fin_data", 10);
  // Subscriber to the laser scan on /scan topic
  ros::Subscriber cluster_sub = n.subscribe("/scan",1,cluster_callback);

  ros::spin();
  return 0;
}
