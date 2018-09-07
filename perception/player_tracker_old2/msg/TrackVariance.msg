# TrackVariance message
std_msgs/Header header

# real data
geometry_msgs/Point centroid
geometry_msgs/Point32[] points    # cluster (leg scan points)
float32 variance
