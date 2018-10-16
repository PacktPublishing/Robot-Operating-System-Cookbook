# Path through a route network.
#
# A path is a sequence of RouteSegment edges.  This information is
# extracted from a RouteNetwork graph.  A RoutePath lists the route
# segments needed to reach some chosen goal.

Header header

uuid_msgs/UniqueID   network    # Route network containing this path
uuid_msgs/UniqueID[] segments   # Sequence of RouteSegment IDs
KeyValue[]           props      # Key/value properties
