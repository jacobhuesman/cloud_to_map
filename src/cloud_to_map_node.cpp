#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>

//ROS specific includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

PointCloud::ConstPtr mostRecentPC;
PointCloud::ConstPtr currentPC;
bool mostRecent = false;

void callback(const PointCloud::ConstPtr& msg){
	mostRecentPC = msg;
	mostRecent = true;
}

// ----------------------------------------------------------------
// -----Calculate surface normals with a search radius of 0.05-----
// ----------------------------------------------------------------
void calcSurfaceNormals(PointCloud::ConstPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.05);
  ne.compute(*normals);
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "cloud_to_map_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("cloud_pcd", 1, callback);
	ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid> ("grid", 1);
	//Define loop Rate in parameter
  ros::Rate loop_rate(10);

	while(ros::ok()){
		ros::spinOnce();
		if(mostRecent == NULL){
			std::cout << "Waiting for PointCloud..." << "\n";
	    loop_rate.sleep();
		} else if(mostRecent == false){
			loop_rate.sleep();
			std::cout << "Waiting for PointCloud..." << "\n";
		} else {
		  // Record start time
		  uint32_t sec = ros::Time::now().sec;
		  uint32_t nsec = ros::Time::now().nsec;

			currentPC = mostRecentPC;
			mostRecent = false;

			//Calculate Surface Normals
		  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		  calcSurfaceNormals(currentPC, cloud_normals);

		  // Figure out size of matrix needed to store data.
		  float xMax = 0, yMax = 0, xMin = 0, yMin = 0;
		  for(size_t i = 0; i < currentPC->size(); i++){
		  	float x = currentPC->points[i].x;
		  	float y = currentPC->points[i].y;
		  	if(xMax < x){
		  		xMax = x;
		  	} if(xMin > x) {
		  		xMin = x;
		  	} if(yMax < y){
		  		yMax = y;
		  	} if(yMin > y){
		  		yMin = y;
		  	}
		  }
		  std::cout << "Matrix dimension: xMax: "<< xMax << ", yMax: " << yMax << ", xMin: " << xMin << ", yMin: " << yMin << "\n";
		  //Determine resolution of costmap (m/cell).
		  float cellResolution = .02;
		  float xSize = xMax - xMin;
		  float ySize = yMax - yMin;
		  int xCells = (int)(xSize / cellResolution);
		  int yCells = (int)(ySize / cellResolution);
		  std::cout << "Resolution info: xSize: " << xSize << ", ySize: " << ySize << ", xCells: " << xCells << ", yCells: " << yCells << "\n";
		  //Record Origin
		  float originX = xMin;
		  float originY = yMin;
		  float originZ = 0.0;
		  std::vector<int> map(yCells*xCells);
		  //Cut off height (m)
		  //Allowable deviation from the normal of the ground plane (radians). pi /4
		  float deviation = 1.2;
		  // Loop through cloud populating map with cost values
		  size_t x = currentPC->size();
		  for(size_t i = 0; i < x; i++){
		  	float x = currentPC->points[i].x;
				float y = currentPC->points[i].y;
				double z = cloud_normals->points[i].normal_z;

				//Compute phi
				float phi = acos(fabs(z));
				//Check cut off height
				int xCell, yCell;
				if(z == z){// Get rid of nan
					//Calculate cell
					xCell = (int)((x - xMin)/cellResolution);
					yCell = (int)((y - yMin)/cellResolution);

					//Check deviation
					if(phi > deviation){
						map[yCell*xCells+xCell]++;
					} else {
						map[yCell*xCells+xCell]--;
					}
				}
		  }

		  //Generate OccupancyGrid that matches ROS standard.

		  std::vector<signed char> ocGrid(yCells*xCells);
		  for(int i=0; i<yCells*xCells; i++){
				if(map[i] < 0){
					ocGrid[i] = 0;
				} else if(map[i] > 0){
					ocGrid[i] = 100;
				} else if (map[i] == 0){
					ocGrid[i] = 0; //Should be -1
				}
		  }

		  //Create OccupancyGrid
		  nav_msgs::OccupancyGridPtr grid (new nav_msgs::OccupancyGrid);
		  grid->header.seq = 1;
		  grid->header.stamp.sec = ros::Time::now().sec;
		  grid->header.stamp.nsec = ros::Time::now().nsec;
		  grid->header.frame_id = "map";
		  grid->info.map_load_time = ros::Time::now();
		  grid->info.resolution = cellResolution;
		  grid->info.width = xCells;
		  grid->info.height = yCells;
		  grid->info.origin.position.x = originX;
		  grid->info.origin.position.y = originY;
		  grid->info.origin.position.z = originZ;
		  grid->info.origin.orientation.w = 1;
		  grid->info.origin.orientation.x = 0;
		  grid->info.origin.orientation.y = 0;
		  grid->info.origin.orientation.z = 0;
		  //Data has to be stored in a vector
		  grid->data = ocGrid;

		  //Record end time
		  uint32_t esec =ros::Time::now().sec;
		  uint32_t ensec = ros::Time::now().nsec;

		  std::cout << "Seconds: " << esec - sec << "\nNSeconds: " << ensec - nsec << "\n";
	    pub.publish(grid);
		}
	}
}
