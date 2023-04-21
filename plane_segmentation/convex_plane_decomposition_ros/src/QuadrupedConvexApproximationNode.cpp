#include <mutex>

#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/PointStamped.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

#include "elevation_map_msgs/ConvexAppro.h"

const std::string frameId = "odom";
std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr1;

auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) { return 0.0; }; //! 这个之后需要加上kinematic penalty

int numberOfVertices = 8;  //! 这个参数确实该给大，应该是一个上限。太小的话会形成狭长的多边形
double growthFactor = 1.05;

ros::Publisher positionPublisher_LF;
ros::Publisher projectionPublisher_LF;
ros::Publisher convexTerrainPublisher_LF;
ros::Publisher positionPublisher_RF;
ros::Publisher projectionPublisher_RF;
ros::Publisher convexTerrainPublisher_RF;
ros::Publisher positionPublisher_LH;
ros::Publisher projectionPublisher_LH;
ros::Publisher convexTerrainPublisher_LH;
ros::Publisher positionPublisher_RH;
ros::Publisher projectionPublisher_RH;
ros::Publisher convexTerrainPublisher_RH;

ros::Subscriber terrainSubscriber;

ros::ServiceServer ConvexApproServer;

geometry_msgs::PointStamped toMarker(const Eigen::Vector3d& position, const std_msgs::Header& header) {
  geometry_msgs::PointStamped sphere;
  sphere.header = header;
  sphere.point.x = position.x();
  sphere.point.y = position.y();
  sphere.point.z = position.z();
  return sphere;
}

void callback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) 
{
  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
      new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg)));

  // std::lock_guard<std::mutex> lock(terrainMutex); 不加🔓看看会咋样
  planarTerrainPtr1.swap(newTerrain);
}

bool FootConvexAprroximation(elevation_map_msgs::ConvexAppro::Request &request, elevation_map_msgs::ConvexAppro::Response &response)
{
  // std::cout << "Service called" << std::endl;
  uint8_t LegId = request.Footid;
  Eigen::Matrix<double,3,1> query_base{request.nominalFoothold.x, request.nominalFoothold.y, request.nominalFoothold.z};  // base 坐标系下的foothold

  //! query不再是positionInWorld了
  const auto projection = getBestPlanarRegionAtPositionInWorld(query_base, planarTerrainPtr1->planarRegions, penaltyFunction);  // base坐标系下的map和base坐标系下的elevation_map

  // CgalPolygon2d = CGAL::Polygon_2<K>;
  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
      projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numberOfVertices, growthFactor);

  std_msgs::Header header;
  header.stamp.fromNSec(planarTerrainPtr1->gridMap.getTimestamp());
  header.frame_id = frameId;  //! "odom"

  auto convexRegionMsg =  //geometry_msgs::PolygonStamped
      convex_plane_decomposition::to3dRosPolygon(convexRegion, projection.regionPtr->transformPlaneToWorld, header);
      // convexRegionMsg.polygon.points[]
      
  if(LegId==0)  //LF
  {
    convexTerrainPublisher_LF.publish(convexRegionMsg);
    positionPublisher_LF.publish(toMarker(query_base, header));
    projectionPublisher_LF.publish(toMarker(projection.positionInWorld, header));
  }
  else if(LegId==1)   //RF
  {
    convexTerrainPublisher_RF.publish(convexRegionMsg);
    positionPublisher_RF.publish(toMarker(query_base, header));
    projectionPublisher_RF.publish(toMarker(projection.positionInWorld, header));
  }
  else if(LegId==2)   // LH
  {
    convexTerrainPublisher_LH.publish(convexRegionMsg);
    positionPublisher_LH.publish(toMarker(query_base, header));
    projectionPublisher_LH.publish(toMarker(projection.positionInWorld, header));
  }
  else if(LegId==3)   // RH
  {
    convexTerrainPublisher_RH.publish(convexRegionMsg);
    positionPublisher_RH.publish(toMarker(query_base, header));
    projectionPublisher_RH.publish(toMarker(projection.positionInWorld, header));
  }

  //! 直接在这里把polugon转换为系数了
  double height = 0;

  for(int i=0; i<numberOfVertices-1; i++)
  {
    double A = (convexRegionMsg.polygon.points[i+1].y - convexRegionMsg.polygon.points[i].y)/(convexRegionMsg.polygon.points[i+1].x - convexRegionMsg.polygon.points[i].x);
    double Coeff_y = -1.0;
    double b = (convexRegionMsg.polygon.points[i].y * convexRegionMsg.polygon.points[i+1].x - convexRegionMsg.polygon.points[i+1].y * convexRegionMsg.polygon.points[i].x) / (convexRegionMsg.polygon.points[i+1].x - convexRegionMsg.polygon.points[i].x);
    height += convexRegionMsg.polygon.points[i].z;
    // std::cout << convexRegionMsg.polygon.points[i].z << std::endl;


  // Eigen::Vector3d positionInWorld{0.0, 0.0, 0.0}
    if(A * projection.positionInWorld(0) - projection.positionInWorld(1) + b >= 0)
    {
      response.CoeffA[i] = A;
      response.Coeffb[i] = b;
      response.Coeff_y[i] = -1;
    }
    else
    {
      response.CoeffA[i] = -A;
      response.Coeffb[i] = -b;
      response.Coeff_y[i] = 1;
    }
  }

  double A = (convexRegionMsg.polygon.points[numberOfVertices-1].y - convexRegionMsg.polygon.points[0].y)/(convexRegionMsg.polygon.points[numberOfVertices-1].x - convexRegionMsg.polygon.points[0].x);
  double Coeff_y = -1.0;
  double b = (convexRegionMsg.polygon.points[0].y * convexRegionMsg.polygon.points[numberOfVertices-1].x - convexRegionMsg.polygon.points[numberOfVertices-1].y * convexRegionMsg.polygon.points[0].x) / (convexRegionMsg.polygon.points[numberOfVertices-1].x - convexRegionMsg.polygon.points[0].x);
  height += convexRegionMsg.polygon.points[numberOfVertices-1].z;
  
  if(A * projection.positionInWorld(0) - projection.positionInWorld(1) + b >= 0)
  {
    response.CoeffA[numberOfVertices-1] = A;
    response.Coeffb[numberOfVertices-1] = b;
    response.Coeff_y[numberOfVertices-1] = -1;
  }
  else
  {
    response.CoeffA[numberOfVertices-1] = -A;
    response.Coeffb[numberOfVertices-1] = -b;
    response.Coeff_y[numberOfVertices-1] = 1;
  }

  height /= numberOfVertices;
  // response.average_Height = height + 0.3;  
  response.average_Height = height; 
  if(LegId == 0)  //LF
  {
    std::cout << "LF Polygon Vertex[0]: " << std::endl;
    std::cout << "x: " << convexRegionMsg.polygon.points[0].x << std::endl << "y:" << convexRegionMsg.polygon.points[0].y << std::endl << "z: " << convexRegionMsg.polygon.points[0].z << std::endl;
  }
  else if(LegId == 1)  //RF
  {
    std::cout << "RF Polygon Vertex[0]: " << std::endl;
    std::cout << "x: " << convexRegionMsg.polygon.points[0].x << std::endl << "y:" << convexRegionMsg.polygon.points[0].y << std::endl << "z: " << convexRegionMsg.polygon.points[0].z << std::endl;
  }
  else if(LegId == 2)  //LH
  {
    std::cout << "LH Polygon Vertex[0]: " << std::endl;
    std::cout << "x: " << convexRegionMsg.polygon.points[0].x << std::endl << "y:" << convexRegionMsg.polygon.points[0].y << std::endl << "z: " << convexRegionMsg.polygon.points[0].z << std::endl;
  }
  else if(LegId == 3)  //RH
  {
    std::cout << "RH Polygon Vertex[0]: " << std::endl;
    std::cout << "x: " << convexRegionMsg.polygon.points[0].x << std::endl << "y:" << convexRegionMsg.polygon.points[0].y << std::endl << "z: " << convexRegionMsg.polygon.points[0].z << std::endl;
  }

  return 1;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadruped_convexAprroximation_server_node");
  ros::NodeHandle nodeHandle("~");

  ros::Duration(6).sleep();

  positionPublisher_LF = nodeHandle.advertise<geometry_msgs::PointStamped>("queryPosition_LF", 1);
  projectionPublisher_LF = nodeHandle.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_LF", 1);
  convexTerrainPublisher_LF = nodeHandle.advertise<geometry_msgs::PolygonStamped>("convex_terrain_LF", 1);

  positionPublisher_RF = nodeHandle.advertise<geometry_msgs::PointStamped>("queryPosition_RF", 1);
  projectionPublisher_RF = nodeHandle.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_RF", 1);
  convexTerrainPublisher_RF = nodeHandle.advertise<geometry_msgs::PolygonStamped>("convex_terrain_RF", 1);

  positionPublisher_LH = nodeHandle.advertise<geometry_msgs::PointStamped>("queryPosition_LH", 1);
  projectionPublisher_LH = nodeHandle.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_LH", 1);
  convexTerrainPublisher_LH = nodeHandle.advertise<geometry_msgs::PolygonStamped>("convex_terrain_LH", 1);

  positionPublisher_RH = nodeHandle.advertise<geometry_msgs::PointStamped>("queryPosition_RH", 1);
  projectionPublisher_RH = nodeHandle.advertise<geometry_msgs::PointStamped>("projectedQueryPosition_RH", 1);
  convexTerrainPublisher_RH = nodeHandle.advertise<geometry_msgs::PolygonStamped>("convex_terrain_RH", 1);

  terrainSubscriber = nodeHandle.subscribe("/convex_plane_decomposition_ros/planar_terrain", 1, &callback);

  ros::Duration(2).sleep();

  ConvexApproServer = nodeHandle.advertiseService("/ConvexAppro", FootConvexAprroximation);
  
  ROS_INFO("Ready2ConvexApproximat");
  
  ros::spin();

  return 0;
}



