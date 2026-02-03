#include <kinects_human_tracking/closest_pt_tracking.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

/**
   Subscribe to a pointCloud and track the closest
   point to the robot's end-effector
 */
visualization_msgs::MarkerArray closest_pt;
visualization_msgs::MarkerArray center_pt;
ros::Publisher center_pt_pub;
ros::Publisher vel_accel_pub_;
ros::Publisher pos_vel_pub_;
nav_msgs::Odometry pub_data2;

Eigen::Vector3f last_obs;
ros::Time last_time ;
pcl::PointXYZ Global_centroid;

void marker_pub_odom(int id, float x, float y, float z,tf::StampedTransform velodyne2odom_reg){
  
  // cout<<"pub maker"<<endl;
  tf::Transform odom2velodyne;
  odom2velodyne.setOrigin(tf::Vector3( velodyne2odom_reg.getOrigin().x(),  velodyne2odom_reg.getOrigin().y(), velodyne2odom_reg.getOrigin().z()));
  tf::Quaternion q(velodyne2odom_reg.getRotation().x(),
                   velodyne2odom_reg.getRotation().y(),
                   velodyne2odom_reg.getRotation().z(),
                   velodyne2odom_reg.getRotation().w());
  odom2velodyne.setRotation(q);

  tf::Transform velodyne2obj;
  velodyne2obj.setOrigin(tf::Vector3(x,y,z));
  tf::Quaternion qx(0,0,0,1);
  velodyne2obj.setRotation(qx);

  tf::Transform odom2obj;
  odom2obj = odom2velodyne * velodyne2obj;
  // cout << "odom2obj.getOrigin().x(): " << odom2obj.getOrigin().x() << endl;
  // cout << "odom2obj.getOrigin().y(): " << odom2obj.getOrigin().y()<< endl;

  visualization_msgs::Marker m;
  m.header.frame_id = "odom";
  m.header.stamp=ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.id = id;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = odom2obj.getOrigin().x();
  Global_centroid.x = m.pose.position.x;
  m.pose.position.y = odom2obj.getOrigin().y();
  Global_centroid.y = m.pose.position.y;
  m.pose.position.z = odom2obj.getOrigin().z();
  Global_centroid.z = m.pose.position.z;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.r = 255;
  m.color.g = 255;
  m.color.b = 255;
  m.color.a = 1.0;
  center_pt.markers.clear();
  center_pt.markers.push_back(m);

  
}

void marker_pub(int id, float x, float y, float z){

  
  visualization_msgs::Marker m;
  m.header.frame_id = "velodyne";
  m.header.stamp=ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.id = id;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.r = 255;
  m.color.g = 255;
  m.color.b = 255;
  m.color.a = 1.0;
  center_pt.markers.push_back(m);

  
}

// const char* filename = "/home/tm5/LEO_Workspace/ocs2_ws_real/output/estvel.xls";
// FILE* output_file = fopen(filename, "w+");
ros::Time first_time,this_time,ros_first_time ;
bool first_frame = true;

int main(int argc, char** argv){
  ros::init(argc, argv, "closest_pt_tracking");
  ros::NodeHandle nh, nh_priv("~");
  
  ROS_INFO("Initializing tracking...");  
  
  tf_listener_ = new tf::TransformListener();
   
  // Get params topics and frames names
  string kinect_topic_name, clusters_topic_name, out_topic_name;
  XmlRpc::XmlRpcValue clipping_rules_bounds;
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  params_loaded *= nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  params_loaded *= nh_priv.getParam("out_topic_name",out_topic_name);
  params_loaded *= nh_priv.getParam("voxel_size",voxel_size_);
  params_loaded *= nh_priv.getParam("min_cluster_size",min_cluster_size_);
  params_loaded *= nh_priv.getParam("kinect_noise",kinect_noise_);
  params_loaded *= nh_priv.getParam("kinect_noise_z",kinect_noise_z_);
  params_loaded *= nh_priv.getParam("process_noise",process_noise_);
  params_loaded *= nh_priv.getParam("minimum_height",minimum_height_);
  params_loaded *= nh_priv.getParam("max_tracking_jump",max_tracking_jump_);
  params_loaded *= nh_priv.getParam("clipping_rules",clipping_rules_bounds);
  params_loaded *= nh_priv.getParam("clustering_tolerance",clustering_tolerance_);
  params_loaded *= nh_priv.getParam("downsampling",downsampling_);
  params_loaded *= nh_priv.getParam("end_eff_frame",enf_eff_frame_);
  
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    return -1;
  }
  
  if (clipping_rules_bounds.size()>0){
    if (clipping_rules_bounds.size()%3)
      ROS_ERROR("Problem in defining the clipping rules.\n Use the following format:\n [x, GT, 1.0, y, LT, 3.1, ...]");
    else{
      clipping_rules_.resize(clipping_rules_bounds.size()/3);
      ClippingRule new_rule;
      for(int i=0; i<clipping_rules_bounds.size()/3;i++){
	new_rule.axis = static_cast<string>(clipping_rules_bounds[i*3]);
	new_rule.op = static_cast<string>(clipping_rules_bounds[i*3+1]);
	new_rule.val = static_cast<double>(clipping_rules_bounds[i*3+2]);
	clipping_rules_.at(i) = new_rule;
      }
    }
    ROS_INFO("%d clipping rules loaded", static_cast<int>(clipping_rules_.size()));
  }
  
  // Initialize PointClouds
  kinects_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  cluster_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  kinects_pc_->reserve(10000);
  cluster_cloud_->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_clustered_pub_ = nh.advertise<PointCloudSM>(clusters_topic_name, 1);
  cluster_pc_pub_ = nh.advertise<PointCloudSM>(out_topic_name, 1);
  cloud_mini_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>(kinect_topic_name+"/min_pt",1);
  // cluster_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>(kinect_topic_name+"/tracking_state",1);
  // track_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>(kinect_topic_name+"/closest_pt_tracking",1);
  // dist_vect_pub_ = nh.advertise<geometry_msgs::Vector3>(kinect_topic_name+"/vector_closest_frame",1);
  // min_pub_ = nh.advertise<std_msgs::Float32>(kinect_topic_name+"/minimum_distance",1);
  // vel_pub_ = nh.advertise<geometry_msgs::Twist>(kinect_topic_name+"/closest_vel_tracking",1);
  ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>(kinect_topic_name, 1, callback);
  center_pt_pub = nh.advertise<visualization_msgs::MarkerArray>("center_pt",1);
  // vel_accel_pub_ = nh.advertise<std_msgs::Float64MultiArray>("vel_accel",1);
  pos_vel_pub_ = nh.advertise<nav_msgs::Odometry>("pos_vel",1);
  
  // Initialize Kalman filter
  Eigen::Matrix<float, 9, 1> x_k1;
  x_k1.fill(0.0);
  Eigen::Matrix<float, 9, 9> init_cov;
  init_cov.fill(0.0);
  kalman_.init(Eigen::Vector3f(kinect_noise_, kinect_noise_, kinect_noise_z_), Eigen::Vector3f(process_noise_ ,process_noise_,process_noise_), -1, x_k1, init_cov);
  
  // Initializing the tracking position at the origin
  last_pos_ = Eigen::Vector3f(0.0, 0.0, 0.0);
  pub_data2.header.frame_id = "odom";
  pub_data2.child_frame_id  = "obs";
  pub_data2.pose.pose.position.x = 1000;
  pub_data2.pose.pose.position.y = 1000;
  pub_data2.pose.pose.position.z = 1000;
  
  ROS_INFO("Tracking ready !");
  
  last_observ_time_ = ros::Time::now();
  
  char str_my[80];
  sprintf(str_my,"time\tvx\tvy\tvz\n");
  // fwrite(str_my, 1, strlen(str_my), output_file);
  
  ros_first_time = ros::Time::now();
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& kinect_pc_msg){
  
  
  
  // std::cout << "hi\n";
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*kinect_pc_msg, *kinects_pc_);
  
  // Clip pointcloud using the rules defined in params
  // pc_clipping(kinects_pc_, clipping_rules_ , kinects_pc_);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*kinects_pc_, *kinects_pc_, indices);
  
  // Downsampling the two pointClouds
  if(downsampling_)
    pc_downsampling(kinects_pc_, voxel_size_, kinects_pc_);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices = pc_clustering(kinects_pc_, min_cluster_size_, clustering_tolerance_ ,kinects_pc_);
  // printf("clustering size is : %ld\n",cluster_indices.size());
  // Gives each cluster a random color
  srand(time(NULL));
  // for(int i=0; i<cluster_indices.size();i++){
  //   uint8_t r(rand()%255), g(rand()%255), b(rand()%255);
  //   // printf("r is %d, g is %d, b is %d\n",r,g,b);
  //   for(int j=0; j<cluster_indices[i].indices.size();j++){
  //     kinects_pc_->points[cluster_indices[i].indices[j]].r = r;
  //     kinects_pc_->points[cluster_indices[i].indices[j]].g = g;
  //     kinects_pc_->points[cluster_indices[i].indices[j]].b = b;
  //   }
  // }
  
  // Publishing clusters
  pc_clustered_pub_.publish(*kinects_pc_);
  cout << "minimum_height_ : " << minimum_height_ << endl;
  if(minimum_height_ >0){
    // Getting heights for all clusters
    std::vector<double> cluster_heights; 
    vector<ClusterStats> stats = get_clusters_stats (kinects_pc_ , cluster_indices);
    for(int i=0; i<cluster_indices.size();i++){
      std::string tmp = boost::lexical_cast<std::string>(stats[i].max(2)-stats[i].min(2));
      double cluster_height = (double)atof(tmp.c_str());
      cluster_heights.push_back(cluster_height);
    }

    // Selecting only the clusters with the minimum_height   
    std::vector<pcl::PointIndices> tmp_cluster_indices;
    for(int i=0; i<cluster_indices.size(); i++){
      if (cluster_heights[i]>=minimum_height_){
	      // cout << "cluster_heights[i] : " << cluster_heights[i] << endl;
        tmp_cluster_indices.push_back(cluster_indices[i]);
      }
    }
    cluster_indices = tmp_cluster_indices;
  }
   
  // Get closest cluster to the robot
  printf("cluster_indices.size() %ld\n",cluster_indices.size());
  
  if (cluster_indices.size()>0){
    get_closest_cluster_to_frame(kinects_pc_, cluster_indices, tf_listener_, enf_eff_frame_, cluster_cloud_, last_min_dist_, last_cluster_pt_);
    

    // Publish minimum point
    // find nearest pointcloud
    cloud_mini_pt_pub_.publish<geometry_msgs::PointStamped>(last_cluster_pt_);
    

    // get centroid
    pcl::PointXYZ centroid;
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    //   float x = 0.0;
    //   float y = 0.0;
    //   float z = 0.0;
    //   float numPts = 0.0;
    //   for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
    //     x += cluster_cloud_->points[*pit].x;
    //     y += cluster_cloud_->points[*pit].y;
    //     z += cluster_cloud_->points[*pit].z;
    //     numPts++;
    //   }
      
    //   centroid.x = x / numPts;
    //   centroid.y = y / numPts;
    //   centroid.z = z / numPts;
    // }

    // ===================== find pointcloud centroid ======================
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float numPts = 0.0;
    // printf("cluster_cloud_->points.size() %d\n",cluster_cloud_->points.size());
    for (int pit=0;pit<cluster_cloud_->points.size();pit++) {
      x += cluster_cloud_->points[pit].x;
      y += cluster_cloud_->points[pit].y;
      z += cluster_cloud_->points[pit].z;
      numPts++;
    }
    
    centroid.x = x / numPts;
    centroid.y = y / numPts;
    centroid.z = z / numPts;
    
    tf::StampedTransform velodyne2odom_reg;
    try{
      // tf_listener->waitForTransform(pc_in->header.frame_id, frame_id, ros::Time(0.0), ros::Duration(1.0));
      // tf_listener->lookupTransform(pc_in->header.frame_id, frame_id, ros::Time(0.0), transform);
      tf_listener_->waitForTransform("/odom","/velodyne", ros::Time(0), ros::Duration(1.0));
      tf_listener_->lookupTransform("/odom", "/velodyne",ros::Time(0), velodyne2odom_reg);
    }
    catch (tf::TransformException ex){ 
      ROS_ERROR("%s",ex.what());
    }
    marker_pub_odom(0,centroid.x, centroid.y, centroid.z,velodyne2odom_reg);
    
    // marker_pub(0,centroid.x, centroid.y, centroid.z);
    center_pt_pub.publish(center_pt);
    // Publish cluster's' pointCloud
    cluster_pc_pub_.publish(*cluster_cloud_);

    double lowestZ = 0.0;
    pcl::PointXYZRGB minPt,maxPt;
    pcl::getMinMax3D (*cluster_cloud_, minPt, maxPt);
    lowestZ = minPt.z;
    
    
    // Get pose observation from the stats
    Eigen::VectorXf obs(6);
    // obs(0) = last_cluster_pt_.point.x;
    // obs(1) = last_cluster_pt_.point.y;   
    // obs(2) = last_cluster_pt_.point.z;   
    // obs(0) = centroid.x;
    // obs(1) = centroid.y;   
    // obs(2) = centroid.z; 

    obs(0) = center_pt.markers[0].pose.position.x;
    obs(1) = center_pt.markers[0].pose.position.y;   
    obs(2) = center_pt.markers[0].pose.position.z;

    
    
    // If the new observation is too far from the previous one, reinitialize
    if(max_tracking_jump_>0){
      if ( (last_pos_-obs).norm() > max_tracking_jump_){
        Eigen::Matrix<float, 9, 1>  x_k1;
        x_k1.fill(0.);
        x_k1(0,0) = obs(0);
        x_k1(1,0) = obs(1);
        x_k1(2,0) = obs(2);
        kalman_.init(Eigen::Vector3f(kinect_noise_, kinect_noise_, kinect_noise_z_), Eigen::Vector3f(process_noise_ ,process_noise_, process_noise_), -1, x_k1);
        ROS_INFO("New pose too far. Reinitializing tracking!");
      }
    }
  
    // Feed the Kalman filter with the observation and get back the estimated state
    this_time = ros::Time::now();
    float delta_t;
    if (last_observ_time_.sec == 0)
      delta_t = -1;
    else
      delta_t = (ros::Time::now() - last_observ_time_).toSec();
      
    
    Eigen::Matrix<float, 9, 1> est;
    
    if(first_frame){
      last_time = this_time;
      first_time = this_time;
      first_frame=!first_frame;
      obs(3) = 0;
      obs(4) = 0;
      obs(5) = 0;
    }else{
      obs(3) = (obs(0) - last_pos_(0))/ delta_t;
      obs(4) = (obs(1) - last_pos_(1))/ delta_t;
      obs(5) = (obs(2) - last_pos_(2))/ delta_t;
      last_time = this_time;
      // last_obs(0) = est(0);
      // last_obs(1) = est(1);
      // last_obs(2) = est(2);
    }
    last_observ_time_ = ros::Time::now();

    kalman_.estimate(obs, delta_t, est); 
    
    
    // Save new estimated pose
    last_pos_(0) = est(0);
    last_pos_(1) = est(1);    
    last_pos_(2) = est(2);    
    
    // Visualize pose and speed
    // visualize_state(est, cluster_state_pub_);  
    
    // Publish minimum distance and speed
    // std_msgs::Float32 float32_msg;
    // float32_msg.data = last_min_dist_;
    // min_pub_.publish(std_msgs::Float32(float32_msg));
    // geometry_msgs::Twist twist;
    // twist.linear.x = est(3);
    // twist.linear.y = est(4);
    // twist.linear.z = est(5);
    // vel_pub_.publish(twist);

    // std_msgs::Float64MultiArray pub_data;
    // pub_data.data.push_back(est(3)); // linear x
    // pub_data.data.push_back(est(6)); // accel x
    // pub_data.data.push_back(est(4)); // linear y
    // pub_data.data.push_back(est(7)); // accel y
    // cout << " pub linear x : " << est(3) << ", accel x : " << est(6) << endl << ", linear y : " << est(4) << ", accel y : " << est(7) << endl; 
    // vel_accel_pub_.publish(pub_data);

    
    pub_data2.header.frame_id = "odom";
    pub_data2.header.stamp = ros::Time::now();
    pub_data2.child_frame_id  = "obs";
    // pub_data2.pose.pose.position.x = est(0);
    // pub_data2.pose.pose.position.y = est(1);
    // pub_data2.pose.pose.position.z = est(2);
    
    tf::Transform odom2velodyne2;
    odom2velodyne2.setOrigin(tf::Vector3( velodyne2odom_reg.getOrigin().x(),  velodyne2odom_reg.getOrigin().y(), velodyne2odom_reg.getOrigin().z()));
    tf::Quaternion q(velodyne2odom_reg.getRotation().x(),
                    velodyne2odom_reg.getRotation().y(),
                    velodyne2odom_reg.getRotation().z(),
                    velodyne2odom_reg.getRotation().w());
    odom2velodyne2.setRotation(q);

    tf::Transform velodyne2obj;
    velodyne2obj.setOrigin(tf::Vector3(last_cluster_pt_.point.x,last_cluster_pt_.point.y,last_cluster_pt_.point.z));
    tf::Quaternion qx(0,0,0,1);
    velodyne2obj.setRotation(qx);

    tf::Transform odom2obj;
    odom2obj = odom2velodyne2 * velodyne2obj;

    pub_data2.pose.pose.position.x = odom2obj.getOrigin().x();
    pub_data2.pose.pose.position.y = odom2obj.getOrigin().y();
    pub_data2.pose.pose.position.z = odom2obj.getOrigin().z();

    // pub_data2.pose.pose.position.x = last_cluster_pt_.point.x;
    // pub_data2.pose.pose.position.y = last_cluster_pt_.point.y;
    // pub_data2.pose.pose.position.z = last_cluster_pt_.point.z;

    pub_data2.pose.pose.orientation.w = 1;

    pub_data2.twist.twist.linear.x = est(3);
    pub_data2.twist.twist.linear.y = est(4);
    pub_data2.twist.twist.linear.z = est(5);
    pub_data2.pose.covariance[0] = Global_centroid.x;
    pub_data2.pose.covariance[1] = Global_centroid.y;
    pub_data2.pose.covariance[2] = Global_centroid.z;

    // boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_or_create, "OBSshm", boost::interprocess::read_write);
    // shdmem.truncate(sizeof(nav_msgs::Odometry));
    // boost::interprocess::mapped_region region(shdmem, boost::interprocess::read_write);
    // // std::cout << std::hex << region.get_address() << std::endl;
    // // std::cout << std::dec << region.get_size() << std::endl;
    // nav_msgs::Odometry* ObsData = static_cast<nav_msgs::Odometry*>(region.get_address());
    // ObsData->pose.pose.position.x = pub_data2.pose.pose.position.x;
    // ObsData->pose.pose.position.y = pub_data2.pose.pose.position.y;
    // ObsData->pose.pose.position.z = pub_data2.pose.pose.position.z;
    // ObsData->twist.twist.linear.x=pub_data2.twist.twist.linear.x;
    // ObsData->twist.twist.linear.y=pub_data2.twist.twist.linear.y;
    // ObsData->twist.twist.linear.z=pub_data2.twist.twist.linear.z;

    pos_vel_pub_.publish(pub_data2);

    char str[50];
    float time_use = (this_time - ros_first_time).toSec();
    sprintf(str,"%.2f\t%f\t%f\t%f\n",time_use,est(3),est(4),est(5));
    std::cout <<  "x : " << est(0) << std::endl <<  "y : " << est(1) << std::endl << " z : " << est(2) << std::endl <<  
                 "vx : " << est(3) << std::endl << "vy : " << est(4) << std::endl << "vz : " << est(5) << std::endl ;
    // fwrite(str, 1, strlen(str), output_file);
    
    
    
    // Publish vector between point and end-effector
    // tf::StampedTransform end_eff_transform;
    // try{
    //   tf_listener_->waitForTransform(kinects_pc_->header.frame_id, enf_eff_frame_, ros::Time(0.0), ros::Duration(1.0));
    //   tf_listener_->lookupTransform(kinects_pc_->header.frame_id, enf_eff_frame_, ros::Time(0.0), end_eff_transform);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   return;
    // }
    
    // geometry_msgs::Vector3 dist_vect;
    // dist_vect.x = end_eff_transform.getOrigin().getX() - est(0);
    // dist_vect.y = end_eff_transform.getOrigin().getY() - est(1);
    // dist_vect.z = end_eff_transform.getOrigin().getZ() - est(2);
    // dist_vect_pub_.publish(dist_vect);
    
    // geometry_msgs::PointStamped closest_pt;
    // closest_pt.header.frame_id = kinects_pc_->header.frame_id;
    // closest_pt.point.x = est(0);
    // closest_pt.point.y = est(1);
    // closest_pt.point.z = est(2);
    // track_pt_pub_.publish(closest_pt);
    
  }
  else{
    
    // nav_msgs::Odometry pub_data2;
    // pub_data2.header.frame_id = "odom";
    pub_data2.header.stamp = ros::Time::now();
    pub_data2.child_frame_id  = "obs";
    pub_data2.pose.pose.position.x = 1000;
    pub_data2.pose.pose.position.y = 1000;
    pub_data2.pose.pose.position.z = 1000;

    pub_data2.pose.pose.orientation.w = 1;

    pub_data2.twist.twist.linear.x = 0;
    pub_data2.twist.twist.linear.y = 0;
    pub_data2.twist.twist.linear.z = 0;

    pos_vel_pub_.publish(pub_data2);
  }
}

void visualize_state (Eigen::Matrix<float, 9, 1> state, ros::Publisher state_pub){
  
  visualization_msgs::MarkerArray markers_arr;
  visualization_msgs::Marker velx_marker, vely_marker, velz_marker, vel_marker;
  markers_arr.markers.clear();
  
  tf::Vector3 axis_vector, right_vector;
  tf::Vector3 x_axis(1,0,0);
  tf::Quaternion quat;
  
  // Marker for velocity on x
  velx_marker.header.frame_id = kinects_pc_->header.frame_id;
  velx_marker.header.stamp = ros::Time::now();
  velx_marker.id = 200;
  velx_marker.ns = "velx";
  velx_marker.type = visualization_msgs::Marker::ARROW;
  velx_marker.action = visualization_msgs::Marker::ADD;
  velx_marker.pose.position.x = state(0);
  velx_marker.pose.position.y = state(1);
  velx_marker.pose.position.z = state(2);
  velx_marker.scale.y = 0.03;
  velx_marker.scale.z = 0.03;
  velx_marker.scale.x = abs(state(3));
  velx_marker.color.r = 1.0f;
  velx_marker.color.g = 0.0f;
  velx_marker.color.b = 0.0f;
  velx_marker.color.a = 1.0f;
  velx_marker.lifetime = ros::Duration();

  if (state(3)<=0){    
    quat.setEuler(M_PI, 0, 0);
    velx_marker.pose.orientation.x = quat.getX(); 
    velx_marker.pose.orientation.y = quat.getY();
    velx_marker.pose.orientation.z = quat.getZ();
    velx_marker.pose.orientation.w = quat.getW();
    
  }
  markers_arr.markers.push_back(velx_marker);
  
  // Marker for velocity on y
  vely_marker = velx_marker;
  vely_marker.id = 201;
  vely_marker.ns = "vely";
  vely_marker.scale.x = abs(state(4));
  vely_marker.color.r = 0.0f;
  vely_marker.color.g = 1.0f;
  vely_marker.color.b = 0.0f;
  vely_marker.lifetime = ros::Duration();
  
  axis_vector = tf::Vector3(0,state(4),0);
  right_vector = axis_vector.cross(x_axis);
  right_vector.normalize();
  quat = tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(x_axis)));
  quat.normalize();
  vely_marker.pose.orientation.x = quat.getX(); 
  vely_marker.pose.orientation.y = quat.getY();
  vely_marker.pose.orientation.z = quat.getZ();
  vely_marker.pose.orientation.w = quat.getW();
  
  markers_arr.markers.push_back(vely_marker);
  
  // Marker for velocity on z
  velz_marker = vely_marker;
  velz_marker.id = 202;
  velz_marker.ns = "velz";
  velz_marker.scale.x = abs(state(5));
  velz_marker.color.r = 0.0f;
  velz_marker.color.g = 0.0f;
  velz_marker.color.b = 1.0f;
  velz_marker.lifetime = ros::Duration();
  
  axis_vector = tf::Vector3(0,0,state(5));
  right_vector = axis_vector.cross(x_axis);
  right_vector.normalize();
  quat = tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(x_axis)));
  quat.normalize();
  velz_marker.pose.orientation.x = quat.getX(); 
  velz_marker.pose.orientation.y = quat.getY();
  velz_marker.pose.orientation.z = quat.getZ();
  velz_marker.pose.orientation.w = quat.getW();
  
  markers_arr.markers.push_back(velz_marker);
  
  // Marker for global velocity
  vel_marker = velz_marker;
  vel_marker.id = 204;
  vel_marker.ns = "vel";
  vel_marker.scale.x = sqrt(pow(state(3),2)+pow(state(4),2)+pow(state(5),2));
  vel_marker.color.r = 1.0f;
  vel_marker.color.g = 0.0f;
  vel_marker.color.b = 1.0f;
  vel_marker.lifetime = ros::Duration();
  
  axis_vector = tf::Vector3(state(3),state(4),state(5));
  right_vector = axis_vector.cross(x_axis);
  right_vector.normalize();
  quat = tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(x_axis)));
  quat.normalize();
  vel_marker.pose.orientation.x = quat.getX(); 
  vel_marker.pose.orientation.y = quat.getY();
  vel_marker.pose.orientation.z = quat.getZ();
  vel_marker.pose.orientation.w = quat.getW();
  markers_arr.markers.push_back(vel_marker);
  
  
  state_pub.publish(markers_arr);
  
}