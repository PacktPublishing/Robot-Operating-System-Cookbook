# Geographic map for a specified region.

Header header            # stamp specifies time
                         # frame_id (normally /map)

uuid_msgs/UniqueID id    # identifier for this map
BoundingBox  bounds      # 2D bounding box containing map

WayPoint[]   points      # way-points
MapFeature[] features    # map features
KeyValue[]   props       # map properties
