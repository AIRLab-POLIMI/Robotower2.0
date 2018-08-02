#include <player_tracker/spatial_temporal.h>
#include <exception>

spatial_temporal::Extractor::Extractor(ros::NodeHandle nh, std::string scanTopic, float windowDuration, std::string logFilename)
    : nh_(nh), scanTopic_(scanTopic), windowDuration_(windowDuration), onRunningWindow_(false), window_counter_(0), win_stamp_counter_(0){
    
    ros::NodeHandle nh_private("~");
    logFile_.open(logFilename);
    baseFrame_ = "odom";
    plane_normal_.col(0) << 0, 0, 1;
    initRosComunication();
}

spatial_temporal::Extractor::Extractor(ros::NodeHandle nh, std::string scanTopic, float windowDuration)
    : nh_(nh), scanTopic_(scanTopic), windowDuration_(windowDuration), onRunningWindow_(false), window_counter_(0), win_stamp_counter_(0){
    
    ros::NodeHandle nh_private("~");
    baseFrame_ = "odom";
    plane_normal_.col(0) << 0, 0, 1;
    initRosComunication();
}

void spatial_temporal::Extractor::initRosComunication()
{
    laserScanSubscriber_    = nh_.subscribe("/scan", 1, &Extractor::laserCallback, this);
    legArraySubscriber_     = nh_.subscribe("detected_leg_clusters", 1, &Extractor::legArrayCallback, this);

    assembledCloudPublisher_= nh_.advertise<sensor_msgs::PointCloud>("assembled_cloud", 10);
    laserToCloudPublisher_  = nh_.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);
    cloudClusterPublisher_  = nh_.advertise<sensor_msgs::PointCloud2>("clusters", 10);
    cloudClusterTextPublisher_ = nh_.advertise<visualization_msgs::Marker>("cluster_text", 10);
    legCloudPublisher_      = nh_.advertise<sensor_msgs::PointCloud>("leg_cloud", 10);
    markerPublisher_        = nh_.advertise<visualization_msgs::Marker>("eigenvector_marker", 20);

    ros::service::waitForService("assemble_scans");
    assembleScanServiceClient_ = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
}

spatial_temporal::Extractor::~Extractor(){
    if (logFile_.is_open()){
        logFile_.close();
    }

    for(int i=0; i< current_clusters_.size(); i++){
        delete current_clusters_[i];
    }

    current_clusters_.clear();
}

sensor_msgs::PointCloud spatial_temporal::
Extractor::laserMsgAsPointCloud(const sensor_msgs::LaserScan::ConstPtr &scanMsg){
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud(baseFrame_, *scanMsg, cloud, tfListener_);
    return cloud;
}

void spatial_temporal::Extractor::publishAssembledCloud(ros::Time from, ros::Time to){
    sensor_msgs::PointCloud cloud;

    if (assembledCloud(from, to, cloud)){
        assembledCloudPublisher_.publish(cloud);
    }else{
        ROS_ERROR("Failed to publish merged cloud!");
    }
}

void spatial_temporal::Extractor::publishAssembledCloud(sensor_msgs::PointCloud cloud){
    assembledCloudPublisher_.publish(cloud);
}

void spatial_temporal::Extractor::publishEigenvectorMarker(std_msgs::Header header, Eigen::VectorXd eigenvect, 
                                                           int marker_id, geometry_msgs::Point offset, float marker_lifetime){
    // Publish marker to rviz
    visualization_msgs::Marker m;
    m.header.stamp = header.stamp;
    m.header.frame_id = header.frame_id;
    m.ns = "Eigen";
    m.id = marker_id;
    m.type = m.ARROW;
    m.points.push_back(offset);

    if (eigenvect(2) > 0)
        eigenvect = eigenvect * -1;

    eigenvect = eigenvect * 0.25;
    geometry_msgs::Point eigen_point;
    eigen_point.x = eigenvect(0) + offset.x;
    eigen_point.y = eigenvect(1) + offset.y;
    eigen_point.z = eigenvect(2) + offset.z;
    m.points.push_back(eigen_point);
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.color.a = 1;
    m.color.r = 1;
    m.color.g = 0;
    m.color.b = 0;
    m.lifetime = ros::Duration(marker_lifetime);

    #pragma omp critical
    markerPublisher_.publish(m);
}

void spatial_temporal::Extractor::publishVariance(MinimalPointCloud &minimalCloud)
{
    player_tracker::TrackVariance msg;
    msg.header.stamp = ros::Time::now();
    msg.variance = minimalCloud.variance;
    msg.centroid.x = minimalCloud.centroid.x();
    msg.centroid.y = minimalCloud.centroid.y();
    msg.centroid.z = minimalCloud.centroid.z();
    msg.points = minimalCloud.points.points;
    //variance_pub_.publish(msg);
}

int spatial_temporal::Extractor::assembledCloud(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud){
    assembleScanServiceMsg_.request.begin = from;
    assembleScanServiceMsg_.request.end   = to;

    ROS_DEBUG("Assembled time diff: %f", (assembleScanServiceMsg_.request.end.toSec() - startTime_.toSec()));

    if (assembleScanServiceClient_.call(assembleScanServiceMsg_)){
        ROS_DEBUG("Got cloud with %lu points\n", assembleScanServiceMsg_.response.cloud.points.size());
        cloud = assembleScanServiceMsg_.response.cloud;
        return 1;
    }else{
        ROS_ERROR("Service call to assemble_scans failed\n");
        return 0;
    }
}

void spatial_temporal::Extractor::setTemporalDimension(sensor_msgs::PointCloud &cloud, double time){
    #pragma omp parallel for
    for (int i=0; i < cloud.points.size(); i++){
        cloud.points[i].z = time;
    }
}

void spatial_temporal::Extractor::saveToLogFile(std::string text){
        if (logFile_.is_open()){
        logFile_ << text.c_str() << std::endl << std::flush;
    }
}

void spatial_temporal::Extractor::saveCloudDataToLog(sensor_msgs::PointCloud &cloud){
    if (logFile_.is_open()){
        for (int i=0; i < cloud.points.size(); i++){
            logFile_ << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << std::endl;
        }
    }
}

void spatial_temporal::Extractor::legArrayCallback(const player_tracker::LegArray::ConstPtr &legArrayMsg){
    #pragma omp parallel for
    for (int i=0; i < legArrayMsg->legs.size(); i++){
        sensor_msgs::PointCloud cloud;
        cloud.header = legArrayMsg->header;
        cloud.points = legArrayMsg->legs[i].points;

        try{
            tfListener_.waitForTransform("base_link", "odom", ros::Time(0.0), ros::Duration(0.1));
            tfListener_.transformPointCloud("odom", cloud, cloud);
            setTemporalDimension(cloud, (ros::Time::now().toSec() - startTime_.toSec()));
            #pragma omp critical
            legCloudPublisher_.publish(cloud);
        }catch (std::exception& ex){
            ROS_ERROR("%s",ex.what());
        }
    }
}

geometry_msgs::Point spatial_temporal::Extractor::computeClusterMean(sensor_msgs::PointCloud2 &input){
    sensor_msgs::PointCloud cloud;
    if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
        geometry_msgs::Point sum;
        float factor = (float) 1/cloud.points.size();
        for (int i=0; i < cloud.points.size(); i++){
            sum.x += cloud.points[i].x * factor;
            sum.y += cloud.points[i].y * factor;
            sum.z += cloud.points[i].z * factor;
        }
        return sum;
    }else{
        ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
    }
}

Eigen::MatrixXd spatial_temporal::Extractor::projectToEigenvector(Eigen::MatrixXd &dataset, Eigen::VectorXd &eigenvect){
    return eigenvect.transpose() * dataset;
}

Eigen::MatrixXd spatial_temporal::Extractor::cloudPointAsMatrix(sensor_msgs::PointCloud2 &input) {
    sensor_msgs::PointCloud cloud;
    if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
        Eigen::MatrixXd output(3, cloud.points.size());
        #pragma omp parallel for
        for (int i=0; i < cloud.points.size(); i++){
            output.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
        }
        return output;
    }else{
        ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
    }
}

Eigen::MatrixXd spatial_temporal::Extractor::cloudPointAsMatrix(sensor_msgs::PointCloud &cloud) {
    Eigen::MatrixXd output(3, cloud.points.size());
    #pragma omp parallel for
    for (int i=0; i < cloud.points.size(); i++){
        output.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
    }
    return output;
}


Eigen::MatrixXd spatial_temporal::Extractor::computeClusterVariance(Eigen::MatrixXd &input) {
    Eigen::MatrixXd centered = input.colwise() - input.rowwise().mean();
    Eigen::MatrixXd cov = (centered * centered.transpose()) / double(input.cols() - 1);
    return cov;
}


Eigen::MatrixXd spatial_temporal::Extractor::computeClusterVariance(sensor_msgs::PointCloud2 &input){
    sensor_msgs::PointCloud cloud;
    if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
        Eigen::MatrixXd point_matrix = cloudPointAsMatrix(input);
        return computeClusterVariance(point_matrix);
    }else{
        ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
    }

}

Cloud2List spatial_temporal::Extractor::getClusters(sensor_msgs::PointCloud2 &input){

    Cloud2List clusters;

    pcl::PCLPointCloud2 to_convert;

    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl_conversions::toPCL(input, to_convert);

    pcl::PointCloud<pcl::PointXYZ>::Ptr converted(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(to_convert,*converted);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(converted);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.50); // 50cm
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(converted);
    ec.extract(cluster_indices);

    int count(0);

    #pragma omp parallel for
    for (int i = 0; i < cluster_indices.size(); i++) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); pit++) {
            cloud_cluster->points.push_back(converted->points[*pit]); //*
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        // Convert the pointcloud to be used in ROS
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_cluster, *output);
        output->header.frame_id = input.header.frame_id;

        // store cluster
        #pragma omp critical
        clusters.push_back(*output);
        count++;
    }

    ROS_DEBUG("Number of clusters: %d", count);
    return clusters;
}

Eigen::VectorXd spatial_temporal::Extractor::getClusterSmallestEigenvector(Eigen::MatrixXd cov){
    Eigen::VectorXd eigenvals = cov.eigenvalues().real();
    Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
    Eigen::MatrixXd eigenvecs = es.eigenvectors().real();
    Eigen::MatrixXd::Index minIndex;
    double minVal = eigenvals.col(0).minCoeff(&minIndex);
    return eigenvecs.col(minIndex);
}


Eigen::VectorXd spatial_temporal::Extractor::getClusterSmallestEigenvector(sensor_msgs::PointCloud2 &cluster){
    // get cluster variance
    Eigen::MatrixXd cov = computeClusterVariance(cluster);
    Eigen::VectorXd eigenvals = cov.eigenvalues().real();
    Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
    Eigen::MatrixXd eigenvecs = es.eigenvectors().real();
    Eigen::MatrixXf::Index minIndex;
    double minVal = eigenvals.col(0).minCoeff(&minIndex);
    return eigenvecs.col(minIndex);
}

Eigen::MatrixXd::Index spatial_temporal::Extractor::findSteepestEigenvector(Eigen::MatrixXd eigenvectors){
    Eigen::MatrixXd A = eigenvectors.topRows(2);
    Eigen::VectorXd B = A.colwise().squaredNorm();
    Eigen::MatrixXd::Index minIndex;
    B.minCoeff(&minIndex);
    return minIndex;
}


void spatial_temporal::Extractor::saveProjectionToFile(){
     Cloud2List clusters = getClustersInWindow();
    
    if (clusters.empty()) {
        ROS_DEBUG("ON computeSpatialTemporalFeatures: No clusters found. Skipping...");
        return;
    }

    Eigen::VectorXd direction = computeSteepestDirection(clusters);

    // Save to file
    for(int i=0; i < clusters.size(); i++){
        Eigen::MatrixXd asMatrix = cloudPointAsMatrix(clusters[i]);       //Contains to_merge_cloud as matrix.
        Eigen::MatrixXd projected = projectToEigenvector(asMatrix, direction);
        // ROS_INFO_STREAM(asMatrix.cols() << " <?> " << projected.cols());
        if (projected.cols() != clusters[i].width * clusters[i].height){
            ROS_ERROR("NOT EQUAL");
        }
        if (logFile_.is_open()){
            #pragma omp critical
            for (int i=0; i < projected.cols(); i++){
                logFile_ <<  projected(i) << ",";
            }
        }
    }
}


std::vector<std::pair <int,float>> spatial_temporal::Extractor::getSimilarity(sensor_msgs::PointCloud2 &cluster){
    sensor_msgs::PointCloud cluster_converted;
    
    if(!sensor_msgs::convertPointCloud2ToPointCloud(cluster, cluster_converted)){
        ROS_ERROR("Error when converting clusters for comparison.");
    }

    std::vector<std::pair <int,float>> jaccardSim;

    for(int i=0; i < current_clusters_.size(); i++){
        sensor_msgs::PointCloud c;

        sensor_msgs::PointCloud2* ptcloud = current_clusters_[i];

        if(!sensor_msgs::convertPointCloud2ToPointCloud(*ptcloud, c)){
            ROS_ERROR("Error when converting clusters for comparison.");
        }
        
        ROS_DEBUG("New cluster #points: %lu", cluster_converted.points.size());
        ROS_DEBUG("On bag #points: %lu", c.points.size());

        float result = computeJaccardSimilarity(c, cluster_converted);
        ROS_DEBUG("Jaccard: %f",result);
        jaccardSim.push_back(std::make_pair(i, result));
    }

    return jaccardSim;
}

float spatial_temporal::Extractor::computeJaccardSimilarity(sensor_msgs::PointCloud &cloudIn1, sensor_msgs::PointCloud &cloudIn2){
    // Sort lists
    // std::sort(cloudIn1.points.begin(), cloudIn1.points.end(), spatial_temporal::compPoints);
    // std::sort(cloudIn2.points.begin(), cloudIn2.points.end(), spatial_temporal::compPoints);


    // std::vector<geometry_msgs::Point32> union_set;
    // std::vector<geometry_msgs::Point32> intersection_set;

    // std::set_union(cloudIn1.points.begin(), cloudIn1.points.end(),
    //                cloudIn2.points.begin(), cloudIn2.points.end(),                  
    //                std::back_inserter(union_set),
    //                spatial_temporal::compPoints);
    
    // std::set_intersection(cloudIn1.points.begin(), cloudIn1.points.end(),
    //                       cloudIn2.points.begin(), cloudIn2.points.end(),                  
    //                       std::back_inserter(intersection_set),
    //                       spatial_temporal::compPoints);

    // float inter_size = intersection_set.size();
    // float union_size = union_set.size();
    
    int inter_size = 0;
    for(int i=0; i < cloudIn1.points.size(); i++){
        for(int j=0; j < cloudIn2.points.size(); j++){
            if (fabs(cloudIn1.points[i].x - cloudIn2.points[j].x) < 0.001 &&
                fabs(cloudIn1.points[i].y - cloudIn2.points[j].y) < 0.001 &&
                fabs(cloudIn1.points[i].z - cloudIn2.points[j].z) < 0.001){
                inter_size++;
            }
        }
    }

    int union_size = (cloudIn1.points.size() + cloudIn2.points.size()) - inter_size;


    ROS_DEBUG("Intersection size: %f", inter_size);
    ROS_DEBUG("Union size: %f", union_size);

    return ((float) inter_size) / ((float)union_size);         // compute jaccard
}


void spatial_temporal::Extractor::publishClusterTextMarker(int marker_id, geometry_msgs::Point position){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cluster";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.text = std::to_string(marker_id);

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;

    marker.color.r = 1.0f;
    marker.color.g = 0.85f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.1);

    cloudClusterTextPublisher_.publish(marker);
}


void spatial_temporal::Extractor::computeSpatialTemporalFeatures(){

    Cloud2List clusters = getClustersInWindow();
    
    if (clusters.empty()) {
        ROS_DEBUG("ON computeSpatialTemporalFeatures: No clusters found. Skipping...");
        return;
    }

    if (current_clusters_.empty()){
        for (int i=0; i < clusters.size(); i++){
            current_clusters_.push_back(new sensor_msgs::PointCloud2(clusters[i]));
        }
    }

    Eigen::MatrixXd eigenvects(3, clusters.size());      // Holds all eigenvectors;

    // std::stringstream stream;
    // stream << "Number of clusters: " << clusters.size() << "\tCurrent clusters: " << current_clusters_.size();
    // saveToLogFile(stream.str());
    
    std::vector<float> substitutions(current_clusters_.size(),-500);    
    for(int i=0; i < clusters.size(); i++){
        ROS_WARN("Processing cluster %d \t Number of Clusters in list: %lu", i, current_clusters_.size());
        Eigen::VectorXd eigenvect = getClusterSmallestEigenvector(clusters[i]);
        eigenvects.col(i) << eigenvect(0), eigenvect(1), eigenvect(2);
        geometry_msgs::Point mean = computeClusterMean(clusters[i]);
        
        std::vector<std::pair <int,float>> similarities = getSimilarity(clusters[i]);

        if (similarities.empty()){
            ROS_DEBUG("Empty. Size of clusters: %lu", current_clusters_.size()); //cannot be empty
        }

        std::pair <int,float> maxSim = *std::max_element(similarities.begin(), similarities.end(), compPair);
        float angle = getAngleWithNormal(eigenvect);

        if (maxSim.second != 0){
            current_clusters_[maxSim.first] = new sensor_msgs::PointCloud2(clusters[i]);
            substitutions[maxSim.first] = angle;
        }else{
            current_clusters_.push_back(new sensor_msgs::PointCloud2(clusters[i]));
            ROS_WARN("Similarity is zero for cluster %d. Adding new cluster", i);
            substitutions.push_back(angle);
        }

        for (int j = 0; j < similarities.size(); j++){
            ROS_WARN("Idx: %d\t Jaccard: %f", similarities[j].first, similarities[j].second);
        }

        ROS_WARN("-------");

         // publish markers
        cloudClusterPublisher_.publish(clusters[i]);

        publishEigenvectorMarker(clusters[i].header, eigenvect, i, mean, 5);

        publishClusterTextMarker(i, mean);
    }

    std::stringstream stream;
    for (int i = 0; i < substitutions.size(); i++) {
        if (substitutions[i] == -500) continue;
        stream << i << "," << win_stamp_counter_ << "," << substitutions[i] << std::endl;
    }
    
    saveToLogFile(stream.str());
    win_stamp_counter_++;
}

float spatial_temporal::Extractor::getAngleWithNormal(Eigen::VectorXd &eigenvect){
    // compute dot product between eigenvect and plane_normal

    if (eigenvect(2) > 0)
        eigenvect = eigenvect * -1;

    double dot = eigenvect.dot(plane_normal_);
    double cosine = dot / (eigenvect.norm() * plane_normal_.norm());
    float angle = acos(cosine) * 180.0 / PI;
    return angle;
}

Cloud2List spatial_temporal::Extractor::getClustersInWindow() {
    sensor_msgs::PointCloud to_merge_cloud;
    sensor_msgs::PointCloud2 as_pointcloud_2;

    int mergedClouds = assembledCloud(startTime_, ros::Time::now(), to_merge_cloud);

    if (!mergedClouds) {
        return Cloud2List();
    }

    int convertedClouds = sensor_msgs::convertPointCloudToPointCloud2(to_merge_cloud, as_pointcloud_2);
    if (!convertedClouds) {
        return Cloud2List();
    }

    Cloud2List clusters = getClusters(as_pointcloud_2);
    return clusters;
}

Eigen::MatrixXd spatial_temporal::Extractor::computeMinEigenvectors(Cloud2List &clusters){
    Eigen::MatrixXd eigenvects(3, clusters.size());      // Holds all eigenvectors;
    #pragma omp parallel for
    for(int i=0; i < clusters.size(); i++){
        ROS_DEBUG("Processing cluster %d", i);
        Eigen::VectorXd eigenvect = getClusterSmallestEigenvector(clusters[i]);
        eigenvects.col(i) << eigenvect(0), eigenvect(1), eigenvect(2);
    }

    return eigenvects;
}

Eigen::VectorXd spatial_temporal::Extractor::computeSteepestDirection(Cloud2List &clusters) {
    Eigen::MatrixXd eigenvects(3, clusters.size());      // Holds all eigenvectors;
    #pragma omp parallel for
    for(int i=0; i < clusters.size(); i++){
        ROS_DEBUG("Processing cluster %d", i);
        Eigen::VectorXd eigenvect = getClusterSmallestEigenvector(clusters[i]);
        eigenvects.col(i) << eigenvect(0), eigenvect(1), eigenvect(2);

        #pragma omp critical
        cloudClusterPublisher_.publish(clusters[i]);
        geometry_msgs::Point mean = computeClusterMean(clusters[i]);
        publishEigenvectorMarker(clusters[i].header, eigenvect, i, mean, 0.1);
    }


    Eigen::MatrixXd::Index steepest_eigenvect = findSteepestEigenvector(eigenvects);
    return eigenvects.col(steepest_eigenvect);
}

void spatial_temporal::Extractor::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg){

    // check whether we can transform data
    try{
        if (!tfListener_.waitForTransform(scanMsg->header.frame_id,
            baseFrame_, scanMsg->header.stamp + ros::Duration().fromSec(scanMsg->ranges.size() *
            scanMsg->time_increment), ros::Duration(1.0))) {
            return;
        }
    }catch (std::exception& ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    sensor_msgs::PointCloud cloud;

    if (!onRunningWindow_) {

        ROS_DEBUG("Starting NEW windows...");
        onRunningWindow_ = true;
        startTime_ = scanMsg->header.stamp;

    } else if ((scanMsg->header.stamp.toSec() - startTime_.toSec()) > windowDuration_.toSec()) {
        ROS_DEBUG("End of windows...");
        onRunningWindow_ = false;
        saveToLogFile("--");
        clearWindowCloudList();
        clearCurrentClusterList();
        window_counter_++;
        win_stamp_counter_ = 0;
        return;
    }

    cloud = laserMsgAsPointCloud(scanMsg);
    appendWindowCloudList(cloud);

    // set time dimension
    setTemporalDimension(cloud, (scanMsg->header.stamp.toSec() - startTime_.toSec()));

    // publish point cloud
    laserToCloudPublisher_.publish(cloud);

    // Compute running window features
    computeSpatialTemporalFeatures();

}

PointCloudList& spatial_temporal::Extractor::getWindowCloudList(){
    return cloudsInWindow;
}

void spatial_temporal::Extractor::clearWindowCloudList(){
    cloudsInWindow.clear();
}

void spatial_temporal::Extractor::clearCurrentClusterList(){
    ROS_WARN("Cleaning current_clusters_ list.");
    for(int i=0; i< current_clusters_.size(); i++){
        delete current_clusters_[i];
    }
    current_clusters_.clear();
}

void spatial_temporal::Extractor::appendWindowCloudList(sensor_msgs::PointCloud &cloud){
    cloudsInWindow.push_back(cloud);
}
