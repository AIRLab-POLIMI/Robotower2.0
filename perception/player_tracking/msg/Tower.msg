# Position of tower as estimated from laser
geometry_msgs/Point position        # leg postion (cluster centroid)
geometry_msgs/Point32[] points      # cluster (leg scan points)
int32[] point_indexes # indexes of cluster points in the LaserScan range array