#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

/* Define the two point cloud types used in this code */
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

/* Create Point Clouds */
PointCloud::ConstPtr newPC;
PointCloud::ConstPtr currentPC;
bool newPointCloud = false;

// ------------------------------------------------------
// -----Update current PointCloud if msg is received-----
// ------------------------------------------------------
void callback(const PointCloud::ConstPtr& msg) {
  newPC = msg;
  newPointCloud = true;
}

// ----------------------------------------------------------------
// -----Calculate surface normals with a search radius of 0.05-----
// ----------------------------------------------------------------
void calcSurfaceNormals(PointCloud::ConstPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);
  ne.compute(*normals);
}

// ---------------------------------------
// -----Initialize Occupancy Grid Msg-----
// ---------------------------------------
void initGrid(nav_msgs::OccupancyGridPtr grid) {
  grid->header.seq = 1;
  grid->header.frame_id = "map";
  grid->info.origin.position.z = 0;
  grid->info.origin.orientation.w = 1;
  grid->info.origin.orientation.x = 0;
  grid->info.origin.orientation.y = 0;
  grid->info.origin.orientation.z = 0;
}

// -----------------------------------
// -----Update Occupancy Grid Msg-----
// -----------------------------------
void updateGrid(nav_msgs::OccupancyGridPtr grid, float cellRes, int xCells, int yCells,
    float originX, float originY, std::vector<signed char> *ocGrid) {
  grid->header.seq++;
  grid->header.stamp.sec = ros::Time::now().sec;
  grid->header.stamp.nsec = ros::Time::now().nsec;
  grid->info.map_load_time = ros::Time::now();
  grid->info.resolution = cellRes;
  grid->info.width = xCells;
  grid->info.height = yCells;
  grid->info.origin.position.x = originX;
  grid->info.origin.position.y = originY;
  grid->data = *ocGrid;
}

// ------------------------------------------
// -----Calculate size of Occupancy Grid-----
// ------------------------------------------
void calcSize(float *xMax, float *yMax, float *xMin, float *yMin) {
  for (size_t i = 0; i < currentPC->size(); i++) {
    float x = currentPC->points[i].x;
    float y = currentPC->points[i].y;
    if (*xMax < x) {
      *xMax = x;
    }
    if (*xMin > x) {
      *xMin = x;
    }
    if (*yMax < y) {
      *yMax = y;
    }
    if (*yMin > y) {
      *yMin = y;
    }
  }
}

// ---------------------------------------
// -----Populate map with cost values-----
// ---------------------------------------
void populateMap(NormalCloud::Ptr cloud_normals, std::vector<int> &map, float xMin, float yMin,
    float cellResolution, int xCells, int yCells) {
  float deviation = 1.2; //TODO parameterize

  for (size_t i = 0; i < currentPC->size(); i++) {
    float x = currentPC->points[i].x;
    float y = currentPC->points[i].y;
    double z = cloud_normals->points[i].normal_z;

    float phi = acos(fabs(z));
    int xCell, yCell;

    if (z == z) { //TODO Get rid of nan and implement cutoff height
      xCell = (int) ((x - xMin) / cellResolution);
      yCell = (int) ((y - yMin) / cellResolution);

      if (phi > deviation) {
        map[yCell * xCells + xCell]++;
      } else {
        map[yCell * xCells + xCell]--;
      }
    }
  }
}

// ---------------------------------
// -----Generate Occupancy Grid-----
// ---------------------------------
void genOccupancyGrid(std::vector<signed char> &ocGrid, std::vector<int> &countGrid, int size){
  for (int i = 0; i < size; i++) {
    if (countGrid[i] < 0) {
      ocGrid[i] = 0;
    } else if (countGrid[i] > 0) {
      ocGrid[i] = 100;
    } else if (countGrid[i] == 0) {
      ocGrid[i] = 0; // TODO Should be -1
    }
  }
}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
  /* Initialize ROS */
  ros::init(argc, argv, "cloud_to_map_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("cloud_pcd", 1, callback);
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("grid", 1);
  ros::Rate loop_rate(10); // TODO create parameter

  /* Initialize Grid */
  nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
  initGrid(grid);

  while (ros::ok()) {
    ros::spinOnce();
    if (newPointCloud == false) {
      std::cout << "Waiting for PointCloud..." << "\n";
      loop_rate.sleep();
    } else {
      /* Record start time */
      uint32_t sec = ros::Time::now().sec;
      uint32_t nsec = ros::Time::now().nsec;

      /* Update current pointcloud */
      currentPC = newPC;
      newPointCloud = false;

      /* Calculate Surface Normals */
      NormalCloud::Ptr cloud_normals(new NormalCloud);
      calcSurfaceNormals(currentPC, cloud_normals);

      /* Figure out size of matrix needed to store data. */
      float xMax = 0, yMax = 0, xMin = 0, yMin = 0;
      calcSize(&xMax, &yMax, &xMin, &yMin);

      /* Determine resolution of grid (m/cell) */
      float cellResolution = .02; //TODO Parameterize
      int xCells = (int) ((xMax - xMin) / cellResolution);
      int yCells = (int) ((yMax - yMin) / cellResolution);

      /* Populate Map */
      std::vector<int> countGrid(yCells * xCells);
      populateMap(cloud_normals, countGrid, xMin, yMin, cellResolution, xCells, yCells);

      /* Generate OccupancyGrid Data Vector */
      std::vector<signed char> ocGrid(yCells * xCells);
      genOccupancyGrid(ocGrid, countGrid, xCells*yCells);

      /* Update OccupancyGrid Object */
      updateGrid(grid, cellResolution, xCells, yCells, xMin, yMin, &ocGrid);

      /* Record end time */
      uint32_t esec = ros::Time::now().sec;
      uint32_t ensec = ros::Time::now().nsec;
      std::cout << "Seconds: " << esec - sec << "\nNSeconds: " << ensec - nsec << "\n";

      /* Publish Occupancy Grid */
      pub.publish(grid);

      /* Wait */
      loop_rate.sleep();
    }
  }
}
