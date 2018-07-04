#include <pluginlib/class_list_macros.h>
#include <pluginlib_tutorials/polygon_base.h>
#include <pluginlib_tutorials/polygon_plugins.h>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
