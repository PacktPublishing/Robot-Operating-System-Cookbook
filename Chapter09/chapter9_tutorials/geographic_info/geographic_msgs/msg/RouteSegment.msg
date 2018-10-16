# Route network segment.
#
# This is one directed edge of a RouteNetwork graph. It represents a
# known path from one way point to another.  If the path is two-way,
# there will be another RouteSegment with "start" and "end" reversed.

uuid_msgs/UniqueID id           # Unique identifier for this segment

uuid_msgs/UniqueID start        # beginning way point of segment
uuid_msgs/UniqueID end          # ending way point of segment

KeyValue[] props                # segment properties
