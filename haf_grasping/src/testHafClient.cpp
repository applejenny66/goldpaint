//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>
//from geometry_msgs.msg import Pose, Quaternion
#include <visualization_msgs/Marker.h>

//actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//action message
#include <haf_grasping/CalcGraspPointsServerAction.h>

//service message
#include <pmc_msgs/GraspPoseEst_direct.h>

//pcd
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//tf transform
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <list>
#include <math.h>
#include <assert.h>

class hafGraspClient
{
	public:
		// topics and services
		//ros::Subscriber camera_img_sub;
		ros::Subscriber camera_pc_sub;
		ros::Publisher vis_pub;
		ros::Publisher pc_pub;
		ros::ServiceServer srv_set_bbox_and_pc_sub;
		sensor_msgs::Image camera_img;
		sensor_msgs::PointCloud2 camera_pc;

		// callback
		void get_camera_img_cb(sensor_msgs::Image);
		void get_camera_pc_cb(sensor_msgs::PointCloud2);
		bool set_bbox_and_pc_sub(pmc_msgs::GraspPoseEst_direct::Request &req, pmc_msgs::GraspPoseEst_direct::Response &res);

		// parameters
		std::string base_frame;
		std::string arm_frame;
		std::string topic_img;
		std::string topic_pc;
		int gripper_opening_width;
		double bbox_factor;
		double search_area;
		geometry_msgs::Vector3 approach_vector;

		// frame transformation
		tf::TransformListener* transform_listener;
		geometry_msgs::Point point_transform(geometry_msgs::Point, std::string, std::string);

		hafGraspClient(ros::NodeHandle nh)
		{
			//get parameter
			std::vector<float> gripper_approach_vector_tmp;
			if (nh.getParam("/gripper_approach_vector", gripper_approach_vector_tmp))
			{
				this->approach_vector.x = gripper_approach_vector_tmp[0];
				this->approach_vector.y = gripper_approach_vector_tmp[1];
				this->approach_vector.z = gripper_approach_vector_tmp[2];
			}
			std::string base_frame_name;
			std::string arm_frame_name;
			std::string topic_img_name;
			std::string topic_pc_name;
			nh.param("/base_frame", this->base_frame, base_frame_name);
			nh.param("/arm_frame", this->arm_frame, arm_frame_name);
			nh.param("/topic_img", this->topic_img, topic_img_name);
			nh.param("/topic_pc", this->topic_pc, topic_pc_name);
			nh.param("/gripper_width", this->gripper_opening_width, 1);
			nh.param("/bbox_factor", this->bbox_factor, 1.0);
			nh.param("/search_area", this->search_area, 1.0);
			ROS_INFO("topics: %s %s ", this->topic_img.c_str(), this->topic_pc.c_str());
			this->transform_listener = new tf::TransformListener();

			//topics and services
			//this->camera_img_sub = nh.subscribe(this->topic_img, 1, &hafGraspClient::get_camera_img_cb, this);
			this->camera_pc_sub = nh.subscribe(this->topic_pc, 1, &hafGraspClient::get_camera_pc_cb, this);
			this->vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0 , this);
			this->srv_set_bbox_and_pc_sub = nh.advertiseService("grasping_pose_estimation", &hafGraspClient::set_bbox_and_pc_sub, this);
			this->pc_pub = nh.advertise<sensor_msgs::PointCloud2>("final_pc", 1);
		}

};

void hafGraspClient::get_camera_img_cb(sensor_msgs::Image img_in)
{
	this->camera_img = img_in;
}

void hafGraspClient::get_camera_pc_cb(sensor_msgs::PointCloud2 pc_in)
{
		//ROS_INFO("Get pc");
		this->camera_pc = pc_in;
}

geometry_msgs::Point hafGraspClient::point_transform(geometry_msgs::Point src, std::string src_frame, std::string dst_frame)
{
		tf::Vector3 src_tf = tf::Vector3(src.x, src.y, src.z);
		tf::Stamped<tf::Point> src_stamped_tf(src_tf, ros::Time(0), src_frame);
		tf::Stamped<tf::Point> dst_stamped_tf;
		this->transform_listener->transformPoint(dst_frame, src_stamped_tf, dst_stamped_tf);
		geometry_msgs::Vector3Stamped dst_stamped;
		tf::vector3StampedTFToMsg(dst_stamped_tf, dst_stamped);
		geometry_msgs::Point dst;
		dst.x = dst_stamped.vector.x;
		dst.y = dst_stamped.vector.y;
		dst.z = dst_stamped.vector.z;
		return dst;
	/*
	tf::Vector3 pc_bbox_center_tf = tf::Vector3(pc_bbox_center.x, pc_bbox_center.y, pc_bbox_center.z);
	tf::Stamped<tf::Point> pc_bbox_center_camera(pc_bbox_center_tf, ros::Time(0), this->base_frame);
	tf::Stamped<tf::Point> pc_bbox_center_arm;
	this->transform_listener->transformPoint(this->arm_frame, pc_bbox_center_camera, pc_bbox_center_arm);
	geometry_msgs::Vector3Stamped pc_bbox_center_trans;
	tf::vector3StampedTFToMsg(pc_bbox_center_arm, pc_bbox_center_trans);
	*/

}

bool hafGraspClient::set_bbox_and_pc_sub(pmc_msgs::GraspPoseEst_direct::Request &req,
										 pmc_msgs::GraspPoseEst_direct::Response &res)
{
	actionlib::SimpleActionClient<haf_grasping::CalcGraspPointsServerAction> ac("/calc_grasppoints_svm_action_server", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server Ready.");

	//wait pc
	ros::Rate r(1);
	while (this->camera_pc.width == 0)
	{
		ROS_INFO("Waiting for point cloud...");
		ros::spinOnce();
		r.sleep();
	}
	pcl::PointCloud<pcl::PointXYZ> cloud_src;
	pcl::fromROSMsg(this->camera_pc , cloud_src);
	int pc_width = this->camera_pc.width;	//assert img.size  == pc.size
	int pc_height = this->camera_pc.height;
	//ROS_INFO("image size: w=%d, h=%d", camera_img.width, camera_img.height);
	ROS_INFO("Point cloud:\tw=%d, h=%d", pc_width, pc_height);
  res.grasp_pose.position.x = 0.0;
  res.grasp_pose.position.y = 0.0;
  res.grasp_pose.position.z = 0.0;

	// negative approach vector may cause error
	// flip the whole pc
	for (int i = 0; i < cloud_src.size(); ++i)
	{
		cloud_src.points[i].z *= -1;
	}

	// check bbox
	if (req.bbox_corner1.y < 0 || req.bbox_corner1.y > pc_width ||
		req.bbox_corner2.y < 0 || req.bbox_corner2.y > pc_width ||
		req.bbox_corner1.x < 0 || req.bbox_corner1.x > pc_height ||
		req.bbox_corner2.x < 0 || req.bbox_corner2.x > pc_height)
	{
		ROS_INFO("Bbox out of range.");
		return true;
	}

	// avoid nan points
	haf_grasping::CalcGraspPointsServerGoal goal;
	ROS_INFO("Input:\t\tx=%d, y=%d, x=%d, y=%d", (int)req.bbox_corner1.y, (int)req.bbox_corner1.x, (int)req.bbox_corner2.y, (int)req.bbox_corner2.x);
	int bboxXMin = std::min((int)req.bbox_corner1.x, (int)req.bbox_corner2.x);
	int bboxYMin = std::min((int)req.bbox_corner1.y, (int)req.bbox_corner2.y);
	int bboxXMax = std::max((int)req.bbox_corner1.x, (int)req.bbox_corner2.x);
	int bboxYMax = std::max((int)req.bbox_corner1.y, (int)req.bbox_corner2.y);
	double angle = atan((bboxXMin - bboxXMax)/(bboxYMin - bboxYMax));

	int rgb_bbox_corner1_idx = (int)bboxXMin * pc_width + (int)bboxYMin;
	std::list<int> MinCands;
#if 0

	if (isnan(cloud_src.points[rgb_bbox_corner1_idx].x))
	{
		ROS_WARN("Non valid point found.");
		return true;
	}
#endif
#if 1
	while (isnan(cloud_src.points[rgb_bbox_corner1_idx].x))
	//while (1)
	{
		//ROS_WARN("Non valid point found.");
		//return true;
		ROS_INFO("DFS part");
		int cand = 0;
		int currX = rgb_bbox_corner1_idx / pc_width;
		int currY = rgb_bbox_corner1_idx - currX * pc_width;
		if (currX > 1)
		{
			cand = (int)(currX-1) * pc_width + (int)currY;
			//ROS_INFO("cand:\t%d", cand);
			if (find(MinCands.begin(), MinCands.end(), cand) == MinCands.end())
			{
				//ROS_INFO("Insert:\tx=%d, y=%d, idx=%d",  (int)(currX-1), (int)currY, cand);
				MinCands.push_back(cand);
			}
		}
		if (currY > 1)
		{
			cand = (int)currX * pc_width + (int)(currY-1);
			//ROS_INFO("cand:\t%d", cand);
			if (find(MinCands.begin(), MinCands.end(), cand) == MinCands.end())
			{
				//ROS_INFO("Insert:\tx=%d, y=%d, idx=%d",  (int)(currX), (int)currY-1, cand);
				MinCands.push_back(cand);
			}
		}
		if (currX > 1 && currY > 1)
		{
			cand = (int)(currX-1) * pc_width + (int)(currY-1);
			//ROS_INFO("cand:\t%d", cand);
			if (find(MinCands.begin(), MinCands.end(), cand) == MinCands.end())
			{
				//ROS_INFO("Insert:\tx=%d, y=%d, idx=%d",  (int)(currX-1), (int)currY-1, cand);
				MinCands.push_back(cand);
			}
		}
		if (MinCands.empty())
		{
			ROS_WARN("Non valid point found.");
			return true;
		}
		std::list<int>::iterator it = MinCands.begin();
		rgb_bbox_corner1_idx = *it;
		//ROS_INFO("Remove:\trgb_bbox_corner1_idx=%d", rgb_bbox_corner1_idx);
		MinCands.erase(it);
		//for (std::list<int>::iterator itt = MinCands.begin(); itt != MinCands.end(); itt++)
		//{
		//	cout << *itt << " ";
		//}
		//cout << endl;
		//r.sleep();
	}
#endif
	int rgb_bbox_corner2_idx = (int)bboxXMax * pc_width + (int)bboxYMax;
	std::list<int> MaxCands;
#if 0
	if (isnan(cloud_src.points[rgb_bbox_corner2_idx].x))
	{
		ROS_WARN("Non valid point found.");
		return true;
	}
#endif
#if 1
	while (isnan(cloud_src.points[rgb_bbox_corner2_idx].x))
	{
		ROS_INFO("DFS part");
		int cand = 0;
		int currX = rgb_bbox_corner1_idx / pc_width;
		int currY = rgb_bbox_corner1_idx - currX * pc_width;
		if (currX < pc_height-1)
		{
			cand = (int)(currX+1) * pc_width + (int)currY;
			if (find(MaxCands.begin(), MaxCands.end(), cand) == MaxCands.end())
			{
				MaxCands.push_back(cand);
			}
		}
		if (currY < pc_width-1)
		{
			cand = (int)currX * pc_width + (int)(currY+1);
			if (find(MaxCands.begin(), MaxCands.end(), cand) == MaxCands.end())
			{
				MaxCands.push_back(cand);
			}
		}
		if (currX < pc_height-1 && currY < pc_width-1)
		{
			cand = (int)(currX+1) * pc_width + (int)(currY+1);
			if (find(MaxCands.begin(), MaxCands.end(), cand) == MaxCands.end())
			{
				MaxCands.push_back(cand);
			}
		}
		if (MaxCands.empty())
		{
			ROS_WARN("Non valid point found.");
			return true;
		}
		std::list<int>::iterator it = MaxCands.begin();
		rgb_bbox_corner1_idx = *it;
		MaxCands.erase(it);
	}
#endif

	// markers
	visualization_msgs::Marker marker1;
	marker1.header.frame_id = this->base_frame;
	marker1.ns = "origin";
	marker1.type = visualization_msgs::Marker::SPHERE;
	marker1.pose.position.y = -cloud_src.points[0].x;
	marker1.pose.position.z = -cloud_src.points[0].y;
	marker1.pose.position.x = cloud_src.points[0].z;
	marker1.scale.x = 0.01;
	marker1.scale.y = 0.01;
	marker1.scale.z = 0.01;
	marker1.color.a = 1.0;
	marker1.color.r = 1.0;
	marker1.color.g = 0.0;
	marker1.color.b = 0.0;
	this->vis_pub.publish(marker1);
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = this->base_frame;
	marker2.ns = "corner1";
	marker2.type = visualization_msgs::Marker::SPHERE;
	marker2.pose.position.y = -cloud_src.points[rgb_bbox_corner1_idx].x;
	marker2.pose.position.z = -cloud_src.points[rgb_bbox_corner1_idx].y;
	marker2.pose.position.x = cloud_src.points[rgb_bbox_corner1_idx].z;
	marker2.scale.x = 0.03;
	marker2.scale.y = 0.03;
	marker2.scale.z = 0.03;
	marker2.color.a = 1.0;
	marker2.color.r = 0.0;
	marker2.color.g = 1.0;
	marker2.color.b = 0.0;
	this->vis_pub.publish(marker2);
	visualization_msgs::Marker marker3;
	marker3.header.frame_id = this->base_frame;
	marker3.ns = "corner2";
	marker3.type = visualization_msgs::Marker::SPHERE;
	marker3.pose.position.y = -cloud_src.points[rgb_bbox_corner2_idx].x;
	marker3.pose.position.z = -cloud_src.points[rgb_bbox_corner2_idx].y;
	marker3.pose.position.x = cloud_src.points[rgb_bbox_corner2_idx].z;
	marker3.scale.x = 0.03;
	marker3.scale.y = 0.03;
	marker3.scale.z = 0.03;
	marker3.color.a = 1.0;
	marker3.color.r = 0.0;
	marker3.color.g = 1.0;
	marker3.color.b = 0.0;
	this->vis_pub.publish(marker3);

	// position
	/*
	ROS_INFO("idx=%d, idx=%d, idx=%d", rgb_bbox_corner1_idx, rgb_bbox_corner2_idx, rgb_bbox_center_idx);
	ROS_INFO("%d x=%f, y=%f, z=%f", rgb_bbox_corner1_idx, cloud_src.points[rgb_bbox_corner1_idx].x, cloud_src.points[rgb_bbox_corner1_idx].y, cloud_src.points[rgb_bbox_corner1_idx].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", rgb_bbox_corner2_idx, cloud_src.points[rgb_bbox_corner2_idx].x, cloud_src.points[rgb_bbox_corner2_idx].y, cloud_src.points[rgb_bbox_corner2_idx].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", rgb_bbox_center_idx, cloud_src.points[rgb_bbox_center_idx].x, cloud_src.points[rgb_bbox_center_idx].y, cloud_src.points[rgb_bbox_center_idx].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", 0, cloud_src.points[0].x, cloud_src.points[0].y, cloud_src.points[0].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", 1, cloud_src.points[1].x, cloud_src.points[1].y, cloud_src.points[1].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", 2, cloud_src.points[2].x, cloud_src.points[2].y, cloud_src.points[2].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", 3, cloud_src.points[3].x, cloud_src.points[3].y, cloud_src.points[3].z);
	ROS_INFO("%d x=%f, y=%f, z=%f", pc_width*pc_height-1, cloud_src.points[pc_width*pc_height-1].x, cloud_src.points[pc_width*pc_height-1].y, cloud_src.points[pc_width*pc_height-1].z);
	*/
	geometry_msgs::Point pc_bbox_center;
	geometry_msgs::Point pc_bbox_center_trans;
	int rgb_bbox_center_idx = (int)((bboxXMin+bboxXMax)/2) * pc_width + (int)((bboxYMin+bboxYMax)/2);
  if (!isnan(cloud_src.points[rgb_bbox_center_idx].x))
	{
		pc_bbox_center.y = -cloud_src.points[rgb_bbox_center_idx].x;
		pc_bbox_center.z = -cloud_src.points[rgb_bbox_center_idx].y;
		pc_bbox_center.x = cloud_src.points[rgb_bbox_center_idx].z;
	}
  else
	{
		pc_bbox_center.y = -(cloud_src.points[rgb_bbox_corner1_idx].x+cloud_src.points[rgb_bbox_corner2_idx].x)/2;
		pc_bbox_center.z = -(cloud_src.points[rgb_bbox_corner1_idx].y+cloud_src.points[rgb_bbox_corner2_idx].y)/2;
		pc_bbox_center.x = (cloud_src.points[rgb_bbox_corner1_idx].z+cloud_src.points[rgb_bbox_corner2_idx].z)/2;
	}
	//ROS_INFO("idx=%d, idx=%d", rgb_bbox_corner1_idx, rgb_bbox_corner2_idx);
	ROS_INFO("Corner1:\tx=%f, y=%f, z=%f", cloud_src.points[rgb_bbox_corner1_idx].z, cloud_src.points[rgb_bbox_corner1_idx].x, cloud_src.points[rgb_bbox_corner1_idx].y);
	ROS_INFO("Corner2:\tx=%f, y=%f, z=%f", cloud_src.points[rgb_bbox_corner2_idx].z, cloud_src.points[rgb_bbox_corner2_idx].x, cloud_src.points[rgb_bbox_corner2_idx].y);
	ROS_INFO("Center:\t\tx=%f, y=%f, z=%f", pc_bbox_center.x, pc_bbox_center.y, pc_bbox_center.z);
	goal.graspinput.grasp_area_center = pc_bbox_center;

	// area
	int pc_bbox_width = floor(fabs(cloud_src.points[rgb_bbox_corner2_idx].x - cloud_src.points[rgb_bbox_corner1_idx].x) * 100)*this->search_area;
	int pc_bbox_height = floor(fabs(cloud_src.points[rgb_bbox_corner2_idx].y - cloud_src.points[rgb_bbox_corner1_idx].y) * 100)*this->search_area;
	//ROS_INFO("Area:\t\tw=%d, h=%d", pc_bbox_width, pc_bbox_height);
	goal.graspinput.grasp_area_length_x = pc_bbox_width;
	goal.graspinput.grasp_area_length_y = pc_bbox_height;

	// fliter the tablepc_bbox_center
	// filter the bbox
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud_bbox;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_src_ptr = cloud_src.makeShared();
#if 0
  //visualization
	pcl::visualization::CloudViewer viewersrc("src");
	viewersrc.showCloud(cloud_src_ptr);
	while(!viewersrc.wasStopped()){}
#endif
	float x_upper_ex = std::max(cloud_src.points[rgb_bbox_corner1_idx].x, cloud_src.points[rgb_bbox_corner2_idx].x)+fabs(cloud_src.points[rgb_bbox_corner2_idx].x - cloud_src.points[rgb_bbox_corner1_idx].x)*this->bbox_factor;
	float x_lower_ex = std::min(cloud_src.points[rgb_bbox_corner1_idx].x, cloud_src.points[rgb_bbox_corner2_idx].x)-fabs(cloud_src.points[rgb_bbox_corner2_idx].x - cloud_src.points[rgb_bbox_corner1_idx].x)*this->bbox_factor;
	float y_upper_ex = std::max(cloud_src.points[rgb_bbox_corner1_idx].y, cloud_src.points[rgb_bbox_corner2_idx].y)+fabs(cloud_src.points[rgb_bbox_corner2_idx].y - cloud_src.points[rgb_bbox_corner1_idx].y)*this->bbox_factor;
	float y_lower_ex = std::min(cloud_src.points[rgb_bbox_corner1_idx].y, cloud_src.points[rgb_bbox_corner2_idx].y)-fabs(cloud_src.points[rgb_bbox_corner2_idx].y - cloud_src.points[rgb_bbox_corner1_idx].y)*this->bbox_factor;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_src_ptr);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(x_lower_ex, x_upper_ex);
	//pass.setFilterLimits(-10, 10);
	pass.filter(*cloud_bbox_ptr);
	pass.setInputCloud(cloud_bbox_ptr);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y_lower_ex, y_upper_ex);
	//pass.setFilterLimits(-10, 10);
	pass.filter(*cloud_bbox_ptr);
	cloud_bbox = *cloud_bbox_ptr;
	ROS_INFO("point cloud:\t%zu", cloud_bbox_ptr->size());

#if 0
  //visualization
	pcl::visualization::CloudViewer viewerbbox("bbox");
	viewerbbox.showCloud(cloud_bbox_ptr);
	while(!viewerbbox.wasStopped()){}
#endif

#if 0
	float min = 999.9;
	float mean = 0.0;
	int validPoints = 0;
	for (int idx = 0; idx < cloud_bbox.size(); ++idx)
	{
		//std::cout << j <<std::endl;
		//std::cout << clusters[i] <<std::endl;
		//std::cout << clusters[i].indices[j] <<std::endl;
		//std::cout << cloud_filtered.points[clusters[i].indices[j]].z <<std::endl;
		//ROS_INFO("# of clusters %zu", clusters[i].indices.size());
		if (!isnan(cloud_bbox.points[idx].z))
		{
			validPoints++;
			mean += cloud_bbox.points[idx].z;
			min = std::min(min, cloud_bbox.points[idx].z);
		}
	}
	mean /= validPoints;
	ROS_INFO("mean: %f, min: %f", mean, min);
#endif

	// RANSAC
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setDistanceThreshold(mean-min);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud_bbox_ptr);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		ROS_INFO("Could not estimate a planar model for the given dataset.");
		return true;
	}
	ROS_INFO("background:\t%zu", inliers->indices.size());

	// extract object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_bbox_ptr);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered_ptr);

#if 0
  //visualization
	pcl::visualization::CloudViewer viewer("Filtered");
	viewer.showCloud(cloud_filtered_ptr);
	while(!viewer.wasStopped()){}
#endif

	cloud_filtered = *cloud_filtered_ptr;
	pcl::toROSMsg(cloud_filtered, this->camera_pc);
	const sensor_msgs::PointCloud2ConstPtr input_pc_const_ptr = boost::make_shared<sensor_msgs::PointCloud2>(this->camera_pc);
	goal.graspinput.input_pc = *input_pc_const_ptr;
	goal.graspinput.goal_frame_id = this->base_frame;


	for (int i = 0; i < cloud_filtered.size(); ++i)
	{
		cloud_filtered.points[i].z *= -1;
	}
	pcl::toROSMsg(cloud_filtered, this->camera_pc);
	this->pc_pub.publish(this->camera_pc);

	goal.graspinput.max_calculation_time = ros::Duration(3);
	goal.graspinput.show_only_best_grasp = true;
	goal.graspinput.approach_vector = this->approach_vector;
	//ROS_INFO("Approach:\tx=%f, y=%f, z=%f", this->approach_vector.x, this->approach_vector.y, this->approach_vector.z);
	goal.graspinput.gripper_opening_width = this->gripper_opening_width;

	ac.sendGoal(goal);
	bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		boost::shared_ptr<const haf_grasping::CalcGraspPointsServerResult_<std::allocator<void> > > result = ac.getResult();

		//bbox check
		float x_upper = std::max(cloud_src.points[rgb_bbox_corner1_idx].x, cloud_src.points[rgb_bbox_corner2_idx].x);
		float x_lower = std::min(cloud_src.points[rgb_bbox_corner1_idx].x, cloud_src.points[rgb_bbox_corner2_idx].x);
		float y_upper = std::max(cloud_src.points[rgb_bbox_corner1_idx].y, cloud_src.points[rgb_bbox_corner2_idx].y);
		float y_lower = std::min(cloud_src.points[rgb_bbox_corner1_idx].y, cloud_src.points[rgb_bbox_corner2_idx].y);
	  ROS_INFO("Result:\t\tx=%f, y=%f, z=%f", -(*(result)).graspOutput.averagedGraspPoint.x, (*(result)).graspOutput.averagedGraspPoint.y, (*(result)).graspOutput.averagedGraspPoint.z);
		ROS_INFO("Bounds:\t\tyl=%f, yu=%f, zl=%f, zu=%f", x_upper, x_lower, y_upper, y_lower);
		if (-(*(result)).graspOutput.averagedGraspPoint.y < x_lower || -(*(result)).graspOutput.averagedGraspPoint.y > x_upper ||
			  -(*(result)).graspOutput.averagedGraspPoint.z < y_lower || -(*(result)).graspOutput.averagedGraspPoint.z > y_upper)
		{
			ROS_INFO("Result out of bound.");
			return true;
		}

		geometry_msgs::Point result_point;
		geometry_msgs::Point result_point_trans;
		//result_point.x = -(*(result)).graspOutput.averagedGraspPoint.x;
		result_point.x = -pc_bbox_center.x;
		//result_point.y = (*(result)).graspOutput.averagedGraspPoint.y;
		result_point.y = pc_bbox_center.y;
		//result_point.z = (*(result)).graspOutput.averagedGraspPoint.z;
		result_point.z = pc_bbox_center.z;

		result_point_trans = this->point_transform(result_point, this->base_frame, this->arm_frame);
		ROS_INFO("Result(trans):\tx=%f, y=%f, z=%f", result_point_trans.x, result_point_trans.y, result_point_trans.z);
		visualization_msgs::Marker marker4;
		marker4.header.frame_id = this->arm_frame;
		marker4.ns = "trans";
		marker4.type = visualization_msgs::Marker::CUBE;
		marker4.pose.position = result_point_trans;
		//marker4.pose.position.z = -result_point_trans.z;
		marker4.scale.x = 0.03;
		marker4.scale.y = 0.03;
		marker4.scale.z = 0.03;
		marker4.color.a = 1.0;
		marker4.color.r = 240.0;
		marker4.color.g = 40.0;
		marker4.color.b = 120.0;
		this->vis_pub.publish(marker4);

		res.grasp_pose.position = result_point_trans;

		//tf2::Quaternion quat;
		//geometry_msgs::Quaternion quat_msg;
		//quat.setRPY(0.00, 3.14, (*(result)).graspOutput.roll+1.57);
		//quat_msg = tf2::toMsg(quat);
		tf::Quaternion quat_tf = tf::createQuaternionFromRPY(0.00, 3.14, (*(result)).graspOutput.roll+1.57);
		//tf::Quaternion quat_tf = tf::createQuaternionFromRPY(0.00, 3.14, angle+1.57);
		geometry_msgs::Quaternion quat;
		quaternionTFToMsg(quat_tf, quat);

		res.grasp_pose.orientation = quat;
		//res.grasp_pose.orientation.x = approach_vector.x;
		//res.grasp_pose.orientation.y = approach_vector.y;
		//res.grasp_pose.orientation.z = approach_vector.z;
		//res.grasp_pose.orientation.w = (*(result)).graspOutput.roll;
		//res.grasp_pose.orientation.w = 0;
		ROS_INFO("Action finished: %s", state.toString().c_str());
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return true;
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "haf_test_client");
	ros::NodeHandle nh;

	hafGraspClient grasp_client(nh);

	ROS_INFO("Waiting for GraspPoseEst_direct service.");
	ros::spin();
}
