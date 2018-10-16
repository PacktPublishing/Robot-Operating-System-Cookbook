cmake_minimum_required(VERSION 2.8.3)
project(geographic_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  uuid_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  BoundingBox.msg
  GeographicMapChanges.msg
  GeographicMap.msg
  GeoPath.msg
  GeoPoint.msg
  GeoPointStamped.msg
  GeoPose.msg
  GeoPoseStamped.msg
  KeyValue.msg
  MapFeature.msg
  RouteNetwork.msg
  RoutePath.msg
  RouteSegment.msg
  WayPoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetGeographicMap.srv
  GetGeoPath.srv
  GetRoutePlan.srv
  UpdateGeographicMap.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  uuid_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs uuid_msgs std_msgs
)
