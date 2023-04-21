//
// Created by rgrandia on 24.06.20.
//

#include <mutex>

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

const std::string frameId = "odom";
std::mutex terrainMutex;
std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr;

void callback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
      new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg)));

  std::lock_guard<std::mutex> lock(terrainMutex);
  planarTerrainPtr.swap(newTerrain);
}

geometry_msgs::PointStamped toMarker(const Eigen::Vector3d& position, const std_msgs::Header& header) {
  geometry_msgs::PointStamped sphere;
  sphere.header = header;
  sphere.point.x = position.x();
  sphere.point.y = position.y();
  sphere.point.z = position.z();
  return sphere;
}

float randomFloat(float a, float b) {
  float random = ((float)rand()) / (float)RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "convex_approximation_demo_node");
  ros::NodeHandle nodeHandle("~");

  // Publishers for visualization
  auto positionPublisher = nodeHandle.advertise<geometry_msgs::PointStamped>("queryPosition", 1);
  auto projectionPublisher = nodeHandle.advertise<geometry_msgs::PointStamped>("projectedQueryPosition", 1);
  auto convexTerrainPublisher = nodeHandle.advertise<geometry_msgs::PolygonStamped>("convex_terrain", 1);
 
  //! get segemented_terrain from Convex Plane Decomposition
  auto terrainSubscriber = nodeHandle.subscribe("/convex_plane_decomposition_ros/planar_terrain", 1, &callback);

  // Node loop
  ros::Rate rate(ros::Duration(1.0));
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(terrainMutex);
      if (planarTerrainPtr) 
      {
        const auto& map = planarTerrainPtr->gridMap;

        // Find edges.
        double maxX = map.getPosition().x() + map.getLength().x() * 0.5;
        double minX = map.getPosition().x() - map.getLength().x() * 0.5;
        double maxY = map.getPosition().y() + map.getLength().y() * 0.5;
        double minY = map.getPosition().y() - map.getLength().y() * 0.5;

        //! 要把下一个落足的位置发布出去
        Eigen::Vector3d query{randomFloat(minX, maxX), randomFloat(minY, maxY), randomFloat(0.0, 1.0)};


        auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) { return 0.0; };

        //! 将原本随机取的点投影到最近的面上
        const auto projection = getBestPlanarRegionAtPositionInWorld(query, planarTerrainPtr->planarRegions, penaltyFunction);

        //! 生成多边形的参数
        int numberOfVertices = 16;  //! 这个参数确实该给大，应该是一个上限。太小的话会形成狭长的多边形
        double growthFactor = 1.05;

        const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
            projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numberOfVertices, growthFactor);

        std_msgs::Header header;
        header.stamp.fromNSec(planarTerrainPtr->gridMap.getTimestamp());
        header.frame_id = frameId;

        //! 这个就是我需要的信息
        auto convexRegionMsg =
            convex_plane_decomposition::to3dRosPolygon(convexRegion, projection.regionPtr->transformPlaneToWorld, header);

        convexTerrainPublisher.publish(convexRegionMsg);
        positionPublisher.publish(toMarker(query, header));
        projectionPublisher.publish(toMarker(projection.positionInWorld, header));
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}