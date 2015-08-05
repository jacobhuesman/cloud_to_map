#include <iostream>
#include <fstream>
#include <stdint.h>

#include <ros/ros.h>
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

#include <math.h>

ros::Publisher pub;

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_to_map_node");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<nav_msgs::OccupancyGrid> ("grid", 1);

  // --------------------------
  // -----Load point cloud-----
  // --------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Loading point cloud.\n";

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/ros/ros_ws/cloud_to_map/include/cloud_to_map/trial.pcd", *point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file trial.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << point_cloud_ptr->width * point_cloud_ptr->height
            << " data points."
            << std::endl;


  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);

  // ----------------------
  // -----Save Normals-----
  // ----------------------
  pcl::io::savePCDFileASCII ("normals.pcd", *cloud_normals1);

  // ------------------------------
  // -----Save Cloud & Normals-----
  // ------------------------------
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);

  pcl::copyPointCloud(*point_cloud_ptr, *cloud_with_normals_ptr);
  pcl::copyPointCloud(*cloud_normals1, *cloud_with_normals_ptr);

  // -----------------------
  // -----Algorithm 1.0-----
  // -----------------------
  // Figure out size of matrix needed to store data.
  float xMax = 0, yMax = 0, xMin = 0, yMin = 0;
  for(size_t i = 0; i < cloud_with_normals_ptr->size(); i++){
  	float x = cloud_with_normals_ptr->points[i].x;
  	float y = cloud_with_normals_ptr->points[i].y;
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
  float cellResolution = .1;
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
  float deviation = 0.78539816339;
  // Loop through cloud populating map with cost values
  std::ofstream myfile;
  myfile.open("/home/ros/ros_ws/cloud_to_map/include/cloud_to_map/log.txt");

  std::cout << "cloud populating\n";
  size_t x = cloud_with_normals_ptr->size();
  for(size_t i = 0; i < x; i++){
  	float x = cloud_with_normals_ptr->points[i].x;
		float y = cloud_with_normals_ptr->points[i].y;
		float z = cloud_with_normals_ptr->points[i].normal_z;
		//Compute phi
		float phi = acos(abs(z));
		//Check cut off height
		int xCell, yCell;
		if(true){//z < cutHeight){
			//Calculate cell
			xCell = (int)((x - xMin)/cellResolution);
			yCell = (int)((y - yMin)/cellResolution);
			myfile << i << ": x: " << x << ", y: " << y << ", xCell: " << xCell << ", yCell: " << yCell << ", maploc: " << yCell*yCells+xCell << "\n";
			//Check deviation
			if(phi > deviation){
				map[yCell*yCells+xCell]++;
			} else {
				map[yCell*yCells+xCell]--;
			}
		}
  }
  myfile.close();

  std::cout << "OccupancyGrid generation\n";
  //Generate OccupancyGrid that matches ROS standard.

  std::vector<signed char> ocGrid(yCells*xCells);
  for(int i=0; i<yCells*xCells; i++){
		if(map[i] < 0){
			ocGrid[i] = 0;
		} else if(map[i] > 0){
			ocGrid[i] = 100;
		} else if (map[i] == 0){
			ocGrid[i] = -1;
		}
  }

  std::cout << "done\n";
  pcl::io::savePCDFileASCII("cloud_with_normals_ptr.pcd", *cloud_with_normals_ptr);

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

  ros::Rate loop_rate(10);
  while(ros::ok()){
    pub.publish(grid);
    loop_rate.sleep();
  }

  // Spin
  ros::spin ();
}
