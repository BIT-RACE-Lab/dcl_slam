#include "distributedMapping.h"

#include <cmath>
#include <limits>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: handle message callback 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::loopInfoHandler(
	const dcl_slam::loop_infoConstPtr& msg,
	int& id)
{
	// A geometrically verified observation can be produced by either robot.
	// Route every observation to the lower-ID robot so the consistency counter
	// is not split across two processes.
	if(msg->header.frame_id == "inter_robot_transform_candidate")
	{
		if(msg->robot0 != id_)
		{
			return;
		}

		dcl_slam::loop_info confirmed_loop;
		Pose3 confirmed_pose_between;
		const Pose3 pose_between = transformToGtsamPose(msg->pose_between);
		if(confirmInterRobotTransform(
			*msg, pose_between, confirmed_loop, confirmed_pose_between))
		{
			confirmed_loop.pose_between = gtsamPoseToTransform(confirmed_pose_between);
			addConfirmedInterRobotLoop(confirmed_loop, true);
		}
		return;
	}

	// Situation 1: need to add pointcloud for loop closure verification
	if((int)msg->noise == 999)
	{
		if(msg->robot0 != id_)
		{
			return;
		}

		// copy message
		dcl_slam::loop_info loop_msg;
		loop_msg.robot0 = msg->robot0;
		loop_msg.robot1 = msg->robot1;
		loop_msg.index0 = msg->index0;
		loop_msg.index1 = msg->index1;
		loop_msg.init_yaw = msg->init_yaw;
		loop_msg.noise = 888.0; // this loop need verification

		CHECK_LT(loop_msg.index0,keyposes_cloud_6d->size());
		CHECK_LT(loop_msg.index0,robots[id_].keyframe_cloud_array.size());

		// Send the same accumulated submap that underlies the startup descriptor.
		// Sending one MID-360 scan here would discard most of the angular coverage
		// gained during descriptor accumulation.
		pcl::PointCloud<PointPose3D>::Ptr cloudTemp(new pcl::PointCloud<PointPose3D>());
		buildLocalRegistrationSubmap(cloudTemp, loop_msg.index0);
		downsample_filter_for_inter_loop2.setInputCloud(cloudTemp);
		downsample_filter_for_inter_loop2.filter(*cloudTemp);
		pcl::toROSMsg(*cloudTemp, loop_msg.scan_cloud);
		// relative pose
		loop_msg.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_msg.index0]));

		// publish to others for verification
		robots[id_].pub_loop_info.publish(loop_msg);
	}
	// Situation 2: need to verify loop closure in this robot
	else if((int)msg->noise == 888)
	{
		if(msg->robot1 != id_)
		{
			return;
		}

		LOG(INFO) << "[loopInfoHandler(" << id << ")]" << " check loop "
			<< msg->robot0 << "-" << msg->index0 << " " << msg->robot1 << "-" << msg->index1 << "." << endl;

		{
			std::lock_guard<std::mutex> lock(loop_candidate_mutex_);
			loop_closures_candidates.push_back(*msg);
		}
	}
	// Situation 3: add verified loop closure
	else
	{
		LOG(INFO) << "[loopInfoHandler(" << id << ")] add loop "
			<< msg->robot0 << "-" << msg->index0 << " " << msg->robot1 << "-" << msg->index1 << "." << endl;
		addConfirmedInterRobotLoop(*msg, false);
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: loop closure
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::performRSIntraLoopClosure()
{
	if(copy_keyposes_cloud_3d->size() <= intra_robot_loop_ptr || intra_robot_loop_closure_enable_)
	{
		return;
	}

	// find intra loop closure with radius search
	auto matching_result = detectLoopClosureDistance(intra_robot_loop_ptr);
	int loop_key0 = intra_robot_loop_ptr;
	int loop_key1 = matching_result;
	intra_robot_loop_ptr++;

	if(matching_result < 0) // no loop found
	{
		return;
	}

	LOG(INFO) << "[IntraLoopRS<" << id_ << ">] [" << loop_key0 << "] and [" << loop_key1 << "]." << endl;

	calculateTransformation(loop_key0, loop_key1);
}

int distributedMapping::detectLoopClosureDistance(
	const int& cur_ptr)
{
	int loop_key0 = cur_ptr;
	int loop_key1 = -1;

	// find the closest history key frame
	vector<int> indices;
	vector<float> distances;
	kdtree_history_keyposes->setInputCloud(copy_keyposes_cloud_3d);
	kdtree_history_keyposes->radiusSearch(copy_keyposes_cloud_3d->points[cur_ptr],
		search_radius_, indices, distances, 0);
	
	for (int i = 0; i < (int)indices.size(); ++i)
	{
		int index = indices[i];
		if(loop_key0 > exclude_recent_frame_num_ + index)
		{
			loop_key1 = index;
			break;
		}
	}

	if(loop_key1 == -1 || loop_key0 == loop_key1)
	{
		return -1;
	}

	return loop_key1;
}

void distributedMapping::performIntraLoopClosure()
{
	if(!intra_robot_loop_closure_enable_)
	{
		return;
	}

	std::pair<int, float> matching_result;
	int loop_key0;
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		if(keyframe_descriptor->getSize(id_) <= intra_robot_loop_ptr)
		{
			return;
		}
		matching_result = keyframe_descriptor->detectIntraLoopClosureID(intra_robot_loop_ptr);
		loop_key0 = intra_robot_loop_ptr;
		intra_robot_loop_ptr++;
	}
	int loop_key1 = matching_result.first;

	if(matching_result.first < 0) // no loop found
	{
		return;
	}

	LOG(INFO) << "[IntraLoop<" << id_ << ">] [" << loop_key0 << "] and [" << loop_key1 << "]." << endl;

	calculateTransformation(loop_key0, loop_key1);
}

void distributedMapping::calculateTransformation(
	const int& loop_key0,
	const int& loop_key1)
{
	CHECK_LT(loop_key0, copy_keyposes_cloud_6d->size());

	// get initial pose
	Pose3 loop_pose0 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key0]);
	Pose3 loop_pose1 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key1]);

	// extract cloud
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(scan_cloud, loop_key0, 0);
	downsample_filter_for_intra_loop.setInputCloud(scan_cloud);
	downsample_filter_for_intra_loop.filter(*scan_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(map_cloud, loop_key1, history_keyframe_search_num_);
	downsample_filter_for_intra_loop.setInputCloud(map_cloud);
	downsample_filter_for_intra_loop.filter(*map_cloud_ds);

	// fail safe check for cloud
	if(scan_cloud->size() < 300 || map_cloud->size() < 1000)
	{
		ROS_WARN("keyFrameCloud too little points 1");
		return;
	}

	// Publish the target immediately; the source is published after fine ICP so
	// RViz shows the transform that is actually being validated.
	if(pub_map_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud_ds, map_cloud_msg);
		map_cloud_msg.header.stamp = ros::Time::now();
		map_cloud_msg.header.frame_id = world_frame_;
		pub_map_of_scan2map.publish(map_cloud_msg);
	}
	
	// icp settings
	static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
	icp.setMaxCorrespondenceDistance(2*search_radius_);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);
	// icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold_);

	// align clouds
	icp.setInputSource(scan_cloud_ds);
	icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr unused_result(new pcl::PointCloud<PointPose3D>());
	icp.align(*unused_result);

	// check if pass ICP fitness score
	float fitness_score = icp.getFitnessScore();
	if(icp.hasConverged() == false || fitness_score > fitness_score_threshold_)
	{
		ROS_DEBUG("\033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
			id_, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
		LOG(INFO) << "[IntraLoop<" << id_ << ">] ICP failed ("
			<< fitness_score << " > " << fitness_score_threshold_ << "). Reject." << endl;
		return;
	}
	ROS_DEBUG("\033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP passed (%.2f < %.2f). Add.\033[0m",
		id_, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
	LOG(INFO) << "[IntraLoop<" << id_ << ">] ICP passed ("
		<< fitness_score << " < " << fitness_score_threshold_ << "). Add." << endl;

	// get pose transformation
	float x, y, z, roll, pitch, yaw;
	Eigen::Affine3f icp_final_tf;
	icp_final_tf = icp.getFinalTransformation();
	pcl::getTranslationAndEulerAngles(icp_final_tf, x, y, z, roll, pitch, yaw);
	Eigen::Affine3f origin_tf = gtsamPoseToAffine3f(loop_pose0);
	Eigen::Affine3f correct_tf = icp_final_tf * origin_tf;
	pcl::getTranslationAndEulerAngles(correct_tf, x, y, z, roll, pitch, yaw);
	Pose3 pose_from = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
	Pose3 pose_to = loop_pose1;
	Pose3 pose_between = pose_from.between(pose_to);
	LOG(INFO) << "[IntraLoop<" << id_ << ">] pose_between: " << pose_between.translation().x() << " "
		<< pose_between.translation().y() << " " << pose_between.translation().z() << "." << endl;
	
	// add loop factor
	Vector vector6(6);
	vector6 << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score, fitness_score;
	noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(vector6);
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		Symbol('a'+id_, loop_key0), Symbol('a'+id_, loop_key1), pose_between, loop_noise));
	isam2_graph.add(factor);
	local_pose_graph->add(factor);
	local_pose_graph_no_filtering->add(factor);
	sent_start_optimization_flag = true; // enable distributed mapping
	intra_robot_loop_close_flag = true;

	// save loop factor in local map (for PCM)
	auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
	Matrix covariance = loop_noise->covariance();
	robot_local_map.addTransform(*new_factor, covariance);

	auto it = loop_indexs.find(loop_key0);
	if(it == loop_indexs.end() || (it != loop_indexs.end() && it->second != loop_key1))
	{
		loop_indexs[loop_key0] = loop_key1;
	}
}

void distributedMapping::loopFindNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = copy_keyposes_cloud_6d->size();
	CHECK_LE(pose_num, robots[id_].keyframe_cloud_array.size());
	for(int i = -search_num; i <= search_num; ++i)
	{
		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num)
		{
			continue;
		}
		*near_keyframes += *transformPointCloud(
			robots[id_].keyframe_cloud_array[key_near], &copy_keyposes_cloud_6d->points[key_near]);
	}

	if(near_keyframes->empty())
	{
		return;
	}
}

void distributedMapping::performInterLoopClosure()
{
	if(!inter_robot_loop_closure_enable_)
	{
		return;
	}

	std::pair<int, float> matching_result;
	int loop_robot0 = -1;
	int loop_key0 = -1;
	int loop_robot1 = -1;
	int loop_key1 = -1;
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		if(keyframe_descriptor->getSize() <= inter_robot_loop_ptr)
		{
			return;
		}

		matching_result = keyframe_descriptor->detectInterLoopClosureID(inter_robot_loop_ptr);
		loop_robot0 = keyframe_descriptor->getIndex(inter_robot_loop_ptr).first;
		loop_key0 = keyframe_descriptor->getIndex(inter_robot_loop_ptr).second;
		inter_robot_loop_ptr++;

		if(matching_result.first >= 0)
		{
			loop_robot1 = keyframe_descriptor->getIndex(matching_result.first).first;
			loop_key1 = keyframe_descriptor->getIndex(matching_result.first).second;
		}
	}

	if(matching_result.first < 0) // no loop found
	{
		return;
	}

	float init_yaw = matching_result.second;

	// Both robots receive both startup descriptors.  Only the lower-ID robot
	// initiates this pair.  Either descriptor may have arrived first, so the
	// query itself can belong to either robot.
	const int initiator_robot = std::min(loop_robot0, loop_robot1);
	if(id_ != initiator_robot)
	{
		return;
	}
	const pair<int, int> robot_pair(
		std::min(loop_robot0, loop_robot1), std::max(loop_robot0, loop_robot1));
	if(loop_key0 < startup_keyframe_count_ && loop_key1 < startup_keyframe_count_)
	{
		std::lock_guard<std::mutex> lock(inter_robot_factor_mutex_);
		if(initialized_inter_robot_pairs_.find(robot_pair) !=
			initialized_inter_robot_pairs_.end())
		{
			ROS_DEBUG("Startup alignment for robots %d-%d is already locked",
				robot_pair.first, robot_pair.second);
			return;
		}
	}
	Key proposed_key0 = Symbol('a'+loop_robot0, loop_key0).key();
	Key proposed_key1 = Symbol('a'+loop_robot1, loop_key1).key();
	if(proposed_key1 < proposed_key0)
	{
		std::swap(proposed_key0, proposed_key1);
	}
	if(!proposed_inter_robot_loops_.insert(
		std::make_pair(proposed_key0, proposed_key1)).second)
	{
		return;
	}

	ROS_INFO("Inter-robot descriptor matched: [%d][%d] <-> [%d][%d], yaw-bin=%.0f",
		loop_robot0, loop_key0, loop_robot1, loop_key1, init_yaw);

	LOG(INFO) << "[InterLoop<" << id_ << ">] found between ["
		<< loop_robot0 << "]-[" << loop_key0 << "][" << inter_robot_loop_ptr-1 << "] and ["
		<< loop_robot1 << "]-[" << loop_key1 << "][" << matching_result.first << "]." << endl;

	dcl_slam::loop_info inter_loop_candidate;
	inter_loop_candidate.robot0 = loop_robot0;
	inter_loop_candidate.robot1 = loop_robot1;
	inter_loop_candidate.index0 = loop_key0;
	inter_loop_candidate.index1 = loop_key1;
	inter_loop_candidate.init_yaw = init_yaw;
	if(loop_robot0 != id_) // send to other for filling pointcloud
	{
		inter_loop_candidate.noise = 999.0;
	}
	else // fill filtered pointcloud
	{
		CHECK_LT(loop_key0, keyposes_cloud_6d->size());
		CHECK_LT(loop_key0, robots[loop_robot0].keyframe_cloud_array.size());
		
		inter_loop_candidate.noise = 888.0;
		pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
		pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
		buildLocalRegistrationSubmap(scan_cloud, loop_key0);
		downsample_filter_for_inter_loop3.setInputCloud(scan_cloud);
		downsample_filter_for_inter_loop3.filter(*scan_cloud_ds);
		pcl::toROSMsg(*scan_cloud_ds, inter_loop_candidate.scan_cloud);
		inter_loop_candidate.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_key0]));
	}
	robots[id_].pub_loop_info.publish(inter_loop_candidate);
}

void distributedMapping::performExternLoopClosure()
{
	if(!inter_robot_loop_closure_enable_)
	{
		return;
	}

	// extract loop for verification
	dcl_slam::loop_info inter_loop;
	{
		std::lock_guard<std::mutex> lock(loop_candidate_mutex_);
		if(loop_closures_candidates.empty())
		{
			return;
		}
		inter_loop = loop_closures_candidates.front();
		loop_closures_candidates.pop_front();
	}

	auto loop_symbol0 = Symbol('a'+inter_loop.robot0, inter_loop.index0);
	auto loop_symbol1 = Symbol('a'+inter_loop.robot1, inter_loop.index1);
	{
		std::lock_guard<std::mutex> lock(inter_robot_factor_mutex_);
		const pair<int, int> robot_pair(
			std::min(inter_loop.robot0, inter_loop.robot1),
			std::max(inter_loop.robot0, inter_loop.robot1));
		if(inter_loop.index0 < startup_keyframe_count_ &&
			inter_loop.index1 < startup_keyframe_count_ &&
			initialized_inter_robot_pairs_.find(robot_pair) !=
				initialized_inter_robot_pairs_.end())
		{
			return;
		}
		// check the loop closure if added before
		auto find_key_indexes0 = loop_indexes.find(loop_symbol0);
		auto find_key_indexes1 = loop_indexes.find(loop_symbol1);
		if ((find_key_indexes0 != loop_indexes.end() && find_key_indexes0->second == loop_symbol1) ||
			(find_key_indexes1 != loop_indexes.end() && find_key_indexes1->second == loop_symbol0))
		{
			ROS_DEBUG("\033[1;33m[LoopClosure] Loop has added. Skip.\033[0m");
			return;
		}
	}

	// The descriptor can arrive just before the corresponding local keyframe
	// becomes visible to this thread. Retry that short race until the complete
	// accumulated MID-360 block is available locally.
	if (!initial_values->exists(loop_symbol1) ||
		inter_loop.index1 >= static_cast<int>(robots[id_].keyframe_cloud_array.size()))
	{
		std::lock_guard<std::mutex> lock(loop_candidate_mutex_);
		loop_closures_candidates.push_back(inter_loop);
		return;
	}

	// logging
	LOG(INFO) << "[performExternLoopClosure<" << id_ << ">] Loop: "
		<< inter_loop.robot0 << " " << inter_loop.index0 << " "
		<< inter_loop.robot1 << " " << inter_loop.index1 << endl;
	
	// get initial pose
	CHECK_LT(inter_loop.index1, initial_values->size());
	double initial_yaw_;
	if (descriptor_type_num_ == DescriptorType::LidarIris)
	{
		initial_yaw_ = inter_loop.init_yaw*2*M_PI/static_cast<double>(iris_column_);
	}
	else
	{
		initial_yaw_ = inter_loop.init_yaw*M_PI/180.0;
	}
	if(initial_yaw_ > M_PI)
		initial_yaw_ -= 2*M_PI;
	
	const Pose3 initial_loop_pose0 = initial_values->at<Pose3>(loop_symbol1);
	Pose3 loop_pose0 = initial_loop_pose0;
	auto loop_pose1 = initial_values->at<Pose3>(loop_symbol1);

	// extract cloud
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_local_ds(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	pcl::fromROSMsg(inter_loop.scan_cloud, *scan_cloud_local_ds);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());
	if(startup_multi_frame_enable_ && inter_loop.index1 < startup_keyframe_count_)
	{
		buildGlobalRegistrationSubmap(map_cloud, inter_loop.index1);
	}
	else
	{
		loopFindGlobalNearKeyframes(map_cloud, inter_loop.index1, history_keyframe_search_num_);
	}
	downsample_filter_for_inter_loop.setInputCloud(map_cloud); // downsample near keyframes
	downsample_filter_for_inter_loop.filter(*map_cloud_ds);

	// safe check for cloud
	if (scan_cloud_local_ds->size() < 300 || map_cloud_ds->size() < 300)
	{
		ROS_WARN("Inter-robot startup alignment rejected: too few points (scan=%zu, map=%zu)",
			scan_cloud_local_ds->size(), map_cloud_ds->size());
		return;
	}
	if (!scan_cloud_local_ds->is_dense || !map_cloud_ds->is_dense)
	{
		ROS_WARN("keyFrameCloud is not dense");
		return;
	}

	// publish target cloud
	if(pub_map_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud_ds, map_cloud_msg);
		map_cloud_msg.header.stamp = ros::Time::now();
		map_cloud_msg.header.frame_id = world_frame_;
		pub_map_of_scan2map.publish(map_cloud_msg);
	}

	/*** yaw hypotheses followed by coarse-to-fine registration ***/
	// Iris provides a useful yaw basin, but a few degrees of bias on a sparse
	// MID-360 pattern can trap a single point-to-point ICP run. Test a small set
	// around that estimate and rank coarse solutions by fitness and overlap.
	auto calculate_overlap = [&](const pcl::PointCloud<PointPose3D>::Ptr& source,
		const double max_distance)
	{
		pcl::Correspondences forward;
		pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> forward_estimation;
		forward_estimation.setInputCloud(source);
		forward_estimation.setInputTarget(map_cloud_ds);
		forward_estimation.determineCorrespondences(forward, max_distance);

		pcl::Correspondences reverse;
		pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> reverse_estimation;
		reverse_estimation.setInputCloud(map_cloud_ds);
		reverse_estimation.setInputTarget(source);
		reverse_estimation.determineCorrespondences(reverse, max_distance);

		const double forward_ratio = static_cast<double>(forward.size()) /
			static_cast<double>(source->size());
		const double reverse_ratio = static_cast<double>(reverse.size()) /
			static_cast<double>(map_cloud_ds->size());
		return std::min(forward_ratio, reverse_ratio);
	};

	std::vector<double> yaw_offsets_deg(1, 0.0);
	for(double offset = inter_robot_yaw_search_step_deg_;
		offset <= inter_robot_yaw_search_range_deg_ + 1e-6;
		offset += inter_robot_yaw_search_step_deg_)
	{
		yaw_offsets_deg.push_back(offset);
		yaw_offsets_deg.push_back(-offset);
	}

	bool coarse_solution_found = false;
	double best_selection_score = std::numeric_limits<double>::infinity();
	double best_coarse_fitness = std::numeric_limits<double>::infinity();
	double best_coarse_overlap = 0.0;
	double selected_yaw_offset_deg = 0.0;
	Eigen::Matrix4f best_coarse_transform = Eigen::Matrix4f::Identity();
	pcl::PointCloud<PointPose3D>::Ptr best_scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	for(const double yaw_offset_deg : yaw_offsets_deg)
	{
		const double yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;
		const Pose3 hypothesis_pose0(
			Rot3::RzRyRx(
				initial_loop_pose0.rotation().roll(),
				initial_loop_pose0.rotation().pitch(),
				initial_loop_pose0.rotation().yaw() + initial_yaw_ + yaw_offset_rad),
			initial_loop_pose0.translation());
		pcl::PointCloud<PointPose3D>::Ptr hypothesis_cloud =
			transformPointCloud(*scan_cloud_local_ds, hypothesis_pose0);

		pcl::IterativeClosestPoint<PointPose3D, PointPose3D> coarse_icp;
		coarse_icp.setMaxCorrespondenceDistance(inter_robot_max_correspondence_distance_);
		coarse_icp.setMaximumIterations(60);
		coarse_icp.setTransformationEpsilon(1e-6);
		coarse_icp.setEuclideanFitnessEpsilon(1e-6);
		coarse_icp.setInputSource(hypothesis_cloud);
		coarse_icp.setInputTarget(map_cloud_ds);
		pcl::PointCloud<PointPose3D>::Ptr coarse_cloud(new pcl::PointCloud<PointPose3D>());
		coarse_icp.align(*coarse_cloud);
		if(!coarse_icp.hasConverged())
		{
			continue;
		}

		const double coarse_fitness = coarse_icp.getFitnessScore(
			inter_robot_max_correspondence_distance_);
		const double coarse_overlap = calculate_overlap(
			coarse_cloud, inter_robot_overlap_distance_);
		const double selection_score = coarse_fitness /
			std::max(0.05, coarse_overlap);
		ROS_INFO("Inter-robot yaw hypothesis: iris=%.1fdeg offset=%+.1fdeg "
			"fitness=%.4f overlap=%.3f score=%.4f",
			initial_yaw_ * 180.0 / M_PI, yaw_offset_deg,
			coarse_fitness, coarse_overlap, selection_score);
		if(std::isfinite(selection_score) && selection_score < best_selection_score)
		{
			coarse_solution_found = true;
			best_selection_score = selection_score;
			best_coarse_fitness = coarse_fitness;
			best_coarse_overlap = coarse_overlap;
			selected_yaw_offset_deg = yaw_offset_deg;
			best_coarse_transform = coarse_icp.getFinalTransformation();
			best_scan_cloud_ds = hypothesis_cloud;
			loop_pose0 = hypothesis_pose0;
		}
	}
	if(!coarse_solution_found)
	{
		ROS_WARN("No inter-robot yaw hypothesis converged: [%d][%d] <-> [%d][%d]",
			inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1);
		return;
	}
	scan_cloud_ds = best_scan_cloud_ds;
	ROS_INFO("Inter-robot yaw hypothesis selected: iris=%.1fdeg offset=%+.1fdeg "
		"coarse_fitness=%.4f overlap=%.3f",
		initial_yaw_ * 180.0 / M_PI, selected_yaw_offset_deg,
		best_coarse_fitness, best_coarse_overlap);

	// GICP uses local surface covariance and is substantially more informative
	// for yaw than point-to-point ICP in planar indoor scenes.
	pcl::GeneralizedIterativeClosestPoint<PointPose3D, PointPose3D> fine_icp;
	fine_icp.setMaxCorrespondenceDistance(inter_robot_fine_correspondence_distance_);
	fine_icp.setMaximumIterations(80);
	fine_icp.setTransformationEpsilon(1e-7);
	fine_icp.setEuclideanFitnessEpsilon(1e-7);
	fine_icp.setInputSource(scan_cloud_ds);
	fine_icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr correct_scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	fine_icp.align(*correct_scan_cloud_ds, best_coarse_transform);
	inter_loop.noise = fine_icp.getFitnessScore(
		inter_robot_fine_correspondence_distance_);
	if(!fine_icp.hasConverged() || !std::isfinite(inter_loop.noise))
	{
		ROS_WARN("Inter-robot fine ICP did not converge: [%d][%d] <-> [%d][%d]",
			inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1);
		return;
	}
	const Pose3 refined_pose0 =
		Pose3(fine_icp.getFinalTransformation().cast<double>()) * loop_pose0;
	ROS_INFO("Inter-robot yaw refined: iris=%.1fdeg selected_offset=%+.1fdeg "
		"source_yaw=%.2fdeg fitness=%.4f",
		initial_yaw_ * 180.0 / M_PI, selected_yaw_offset_deg,
		refined_pose0.rotation().yaw() * 180.0 / M_PI, inter_loop.noise);
	if(pub_scan_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 scan_cloud_msg;
		pcl::toROSMsg(*correct_scan_cloud_ds, scan_cloud_msg);
		scan_cloud_msg.header.stamp = ros::Time::now();
		scan_cloud_msg.header.frame_id = world_frame_;
		pub_scan_of_scan2map.publish(scan_cloud_msg);
	}

	/*** verification using RANSAC ***/
	// Use a finite distance for both directions.  PCL's default unbounded
	// correspondence query can report a deceptively good one-sided match.
	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> correspondence_estimation;
	correspondence_estimation.setInputCloud(correct_scan_cloud_ds);
	correspondence_estimation.setInputTarget(map_cloud_ds);
	correspondence_estimation.determineCorrespondences(
		*correspondences, inter_robot_overlap_distance_);
	if(correspondences->empty())
	{
		ROS_WARN("Inter-robot startup alignment rejected: no ICP correspondences");
		return;
	}
	boost::shared_ptr<pcl::Correspondences> reverse_correspondences(new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> reverse_estimation;
	reverse_estimation.setInputCloud(map_cloud_ds);
	reverse_estimation.setInputTarget(correct_scan_cloud_ds);
	reverse_estimation.determineCorrespondences(
		*reverse_correspondences, inter_robot_overlap_distance_);
	const double forward_overlap = static_cast<double>(correspondences->size()) /
		static_cast<double>(correct_scan_cloud_ds->size());
	const double reverse_overlap = static_cast<double>(reverse_correspondences->size()) /
		static_cast<double>(map_cloud_ds->size());
	const double bidirectional_overlap = std::min(forward_overlap, reverse_overlap);
	if(bidirectional_overlap < inter_robot_min_overlap_ratio_)
	{
		ROS_WARN("Inter-robot loop rejected by overlap: [%d][%d] <-> [%d][%d], "
			"forward=%.3f reverse=%.3f required=%.3f",
			inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			forward_overlap, reverse_overlap, inter_robot_min_overlap_ratio_);
		return;
	}

	// RANSAC matching to find inlier
	pcl::Correspondences new_correspondences;
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointPose3D> correspondence_ransac;
	correspondence_ransac.setInputSource(correct_scan_cloud_ds);
	correspondence_ransac.setInputTarget(map_cloud_ds);
	correspondence_ransac.setMaximumIterations(ransac_maximum_iteration_);
	correspondence_ransac.setInlierThreshold(ransac_outlier_reject_threshold_);
	correspondence_ransac.setInputCorrespondences(correspondences);
	correspondence_ransac.getCorrespondences(new_correspondences);

	// check if pass RANSAC outlier threshold
	if(new_correspondences.size() < ransac_threshold_*correspondences->size())
	{
		ROS_WARN("Inter-robot loop rejected by RANSAC: [%d][%d] <-> [%d][%d], inlier=%.3f threshold=%.3f",
			inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			new_correspondences.size()*1.0/correspondences->size(), ransac_threshold_);
		ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] RANSAC failed (%.2f < %.2f). Reject.\033[0m",
			id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			new_correspondences.size()*1.0/correspondences->size()*1.0, ransac_threshold_);
		LOG(INFO) << "[InterLoop<" << id_ << ">] RANSAC failed ("
			<< new_correspondences.size()*1.0/correspondences->size()*1.0 << " < " 
			<< ransac_threshold_ << "). Reject." << endl;
		return;
	}
	// check if pass ICP fitness score
	if(inter_loop.noise > fitness_score_threshold_)
	{
		ROS_WARN("Inter-robot loop rejected by ICP: [%d][%d] <-> [%d][%d], converged=%d fitness=%.4f threshold=%.4f",
			inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			fine_icp.hasConverged(), inter_loop.noise, fitness_score_threshold_);
		ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
			id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			inter_loop.noise, fitness_score_threshold_);
		LOG(INFO) << "[InterLoop<" << id_ << ">] ICP failed ("
			<< inter_loop.noise << " > " << fitness_score_threshold_ << "). Reject." << endl;
		return;
	}
	ROS_INFO("Inter-robot registration accepted geometrically: [%d][%d] <-> [%d][%d], "
		"fitness=%.4f overlap=[%.3f %.3f]",
		inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
		inter_loop.noise, forward_overlap, reverse_overlap);
	ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] inlier (%.2f > %.2f) fitness (%.2f < %.2f). Add.\033[0m",
		id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
		new_correspondences.size()*1.0/correspondences->size()*1.0, ransac_threshold_,
		inter_loop.noise, fitness_score_threshold_);
	LOG(INFO) << "[InterLoop<" << id_ << ">] inlier ("
		<< new_correspondences.size()*1.0/correspondences->size()*1.0
		<< " > " << ransac_threshold_ << ") fitness (" << inter_loop.noise << " < "
		<< fitness_score_threshold_ << ") overlap (" << bidirectional_overlap << " > "
		<< inter_robot_min_overlap_ratio_ << "). Add." << endl;

	// get pose transformation
	auto icp_final_tf = Pose3(fine_icp.getFinalTransformation().cast<double>());
	auto pose_from = icp_final_tf * loop_pose0;
    auto pose_to = loop_pose1;

	inter_loop.pose1 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[inter_loop.index1]));
	if(inter_loop.robot0 > inter_loop.robot1) // the first robot always set to the lower id
	{
		swap(pose_from, pose_to);
		swap(inter_loop.robot0, inter_loop.robot1);
		swap(inter_loop.index0, inter_loop.index1);
		swap(inter_loop.pose0, inter_loop.pose1);
		loop_symbol0 = Symbol('a'+inter_loop.robot0, inter_loop.index0);
		loop_symbol1 = Symbol('a'+inter_loop.robot1, inter_loop.index1);
	}
	Pose3 pose_between = pose_from.between(pose_to);
	inter_loop.pose_between = gtsamPoseToTransform(pose_between);
	LOG(INFO) << "[InterLoop<" << id_ << ">] pose_between: " << pose_between.translation().x()
		<< " " << pose_between.translation().y() << " " << pose_between.translation().z() << endl;

	// Descriptor, ICP and RANSAC validate one observation.  Broadcast that
	// compact result so every observation is counted by the lower-ID robot,
	// regardless of which robot happened to execute ICP.
	inter_loop.header.frame_id = "inter_robot_transform_candidate";
	inter_loop.scan_cloud.data.clear();
	inter_loop.scan_cloud.width = 0;
	inter_loop.scan_cloud.height = 0;
	inter_loop.scan_cloud.row_step = 0;
	robots[id_].pub_loop_info.publish(inter_loop);
	if(id_ != inter_loop.robot0)
	{
		return;
	}

	// Do not connect the two pose graphs until several distinct keyframe pairs
	// independently imply the same odom-to-odom transform.
	dcl_slam::loop_info confirmed_loop;
	Pose3 confirmed_pose_between;
	if(!confirmInterRobotTransform(
		inter_loop, pose_between, confirmed_loop, confirmed_pose_between))
	{
		return;
	}
	inter_loop = confirmed_loop;
	pose_between = confirmed_pose_between;
	inter_loop.pose_between = gtsamPoseToTransform(pose_between);
	addConfirmedInterRobotLoop(inter_loop, true);
}

bool distributedMapping::confirmInterRobotTransform(
	const dcl_slam::loop_info& inter_loop,
	const Pose3& pose_between,
	dcl_slam::loop_info& confirmed_loop,
	Pose3& confirmed_pose_between)
{
	std::lock_guard<std::mutex> lock(inter_robot_transform_mutex_);
	CHECK_LT(inter_loop.robot0, inter_loop.robot1);

	// T_odom0_odom1 = T_odom0_body0 * T_body0_body1 * T_body1_odom1.
	// Comparing this normalized transform makes estimates from different
	// keyframe pairs directly comparable.
	const Pose3 pose0 = transformToGtsamPose(inter_loop.pose0);
	const Pose3 pose1 = transformToGtsamPose(inter_loop.pose1);
	const Pose3 odom_alignment = pose0 * pose_between * pose1.inverse();
	const pair<int, int> robot_pair(inter_loop.robot0, inter_loop.robot1);
	auto& candidates = inter_robot_transform_candidates_[robot_pair];

	InterRobotTransformCandidate candidate;
	candidate.loop = inter_loop;
	// The cloud is no longer needed after ICP; retaining several ROS point-cloud
	// payloads would waste memory on the onboard computer.
	candidate.loop.scan_cloud.data.clear();
	candidate.loop.scan_cloud.width = 0;
	candidate.loop.scan_cloud.height = 0;
	candidate.loop.scan_cloud.row_step = 0;
	candidate.pose_between = pose_between;
	candidate.odom_alignment = odom_alignment;
	candidates.push_back(candidate);

	// Bound rejected/outlier history.  This is deliberately larger than the
	// required consensus so a valid cluster can survive a few bad estimates.
	const size_t max_history = static_cast<size_t>(
		std::max(12, inter_robot_consistency_count_ * 4));
	if(candidates.size() > max_history)
	{
		candidates.erase(candidates.begin(),
			candidates.begin() + (candidates.size() - max_history));
	}

	size_t best_anchor = 0;
	size_t best_support = 0;
	for(size_t i = 0; i < candidates.size(); ++i)
	{
		size_t support = 0;
		for(size_t j = 0; j < candidates.size(); ++j)
		{
			const Pose3 delta = candidates[i].odom_alignment.between(
				candidates[j].odom_alignment);
			const double translation_error = delta.translation().norm();
			const double rotation_error = Rot3::Logmap(delta.rotation()).norm();
			if(translation_error <= inter_robot_consistency_translation_threshold_ &&
				rotation_error <= inter_robot_consistency_rotation_threshold_)
			{
				++support;
			}
		}
		if(support > best_support ||
			(support == best_support &&
			 candidates[i].loop.noise < candidates[best_anchor].loop.noise))
		{
			best_anchor = i;
			best_support = support;
		}
	}

	const Point3 alignment_translation = odom_alignment.translation();
	if(best_support < static_cast<size_t>(inter_robot_consistency_count_))
	{
		ROS_INFO("Inter-robot transform pending: robots=%d-%d evidence=%zu/%d history=%zu "
			"alignment_xyz=[%.3f %.3f %.3f]",
			inter_loop.robot0, inter_loop.robot1, best_support,
			inter_robot_consistency_count_, candidates.size(),
			alignment_translation.x(), alignment_translation.y(), alignment_translation.z());
		return false;
	}

	confirmed_loop = candidates[best_anchor].loop;
	const Pose3& anchor_alignment = candidates[best_anchor].odom_alignment;
	Vector6 mean_tangent = Vector6::Zero();
	double worst_fitness = 0.0;
	size_t fused_count = 0;
	for(size_t j = 0; j < candidates.size(); ++j)
	{
		const Pose3 delta = anchor_alignment.between(candidates[j].odom_alignment);
		if(delta.translation().norm() <= inter_robot_consistency_translation_threshold_ &&
			Rot3::Logmap(delta.rotation()).norm() <= inter_robot_consistency_rotation_threshold_)
		{
			mean_tangent += Pose3::Logmap(delta);
			worst_fitness = std::max(worst_fitness,
				static_cast<double>(candidates[j].loop.noise));
			++fused_count;
		}
	}
	mean_tangent /= static_cast<double>(fused_count);
	const Pose3 confirmed_alignment = anchor_alignment * Pose3::Expmap(mean_tangent);
	const Pose3 confirmed_pose0 = transformToGtsamPose(confirmed_loop.pose0);
	const Pose3 confirmed_pose1 = transformToGtsamPose(confirmed_loop.pose1);
	confirmed_pose_between = confirmed_pose0.inverse() *
		confirmed_alignment * confirmed_pose1;
	// Use the worst score in the consensus cluster as a conservative factor
	// variance instead of over-trusting the best individual registration.
	confirmed_loop.noise = static_cast<float>(worst_fitness);
	const Point3 confirmed_translation = confirmed_alignment.translation();
	double max_translation_delta = 0.0;
	double max_rotation_delta = 0.0;
	for(size_t j = 0; j < candidates.size(); ++j)
	{
		const Pose3 delta = confirmed_alignment.between(candidates[j].odom_alignment);
		const double translation_error = delta.translation().norm();
		const double rotation_error = Rot3::Logmap(delta.rotation()).norm();
		if(translation_error <= inter_robot_consistency_translation_threshold_ &&
			rotation_error <= inter_robot_consistency_rotation_threshold_)
		{
			max_translation_delta = std::max(max_translation_delta, translation_error);
			max_rotation_delta = std::max(max_rotation_delta, rotation_error);
		}
	}
	ROS_INFO("Inter-robot transform confirmed: robots=%d-%d evidence=%zu/%d "
		"alignment_xyz=[%.3f %.3f %.3f] alignment_yaw=%.2fdeg "
		"max_delta=[%.3fm %.3frad] fitness=%.4f",
		confirmed_loop.robot0, confirmed_loop.robot1, fused_count,
		inter_robot_consistency_count_, confirmed_translation.x(),
		confirmed_translation.y(), confirmed_translation.z(),
		confirmed_alignment.rotation().yaw() * 180.0 / M_PI,
		max_translation_delta, max_rotation_delta, confirmed_loop.noise);

	// Future factors must earn a fresh batch of confirmations instead of reusing
	// old evidence indefinitely.
	candidates.clear();
	return true;
}

void distributedMapping::addConfirmedInterRobotLoop(
	const dcl_slam::loop_info& inter_loop,
	const bool publish_to_team)
{
	std::lock_guard<std::mutex> lock(inter_robot_factor_mutex_);
	const Symbol loop_symbol0('a'+inter_loop.robot0, inter_loop.index0);
	const Symbol loop_symbol1('a'+inter_loop.robot1, inter_loop.index1);
	const auto existing = loop_indexes.find(loop_symbol0);
	if(existing != loop_indexes.end() && existing->second == loop_symbol1)
	{
		return;
	}
	const bool is_startup_alignment =
		inter_loop.index0 < startup_keyframe_count_ &&
		inter_loop.index1 < startup_keyframe_count_;
	const pair<int, int> robot_pair(
		std::min(inter_loop.robot0, inter_loop.robot1),
		std::max(inter_loop.robot0, inter_loop.robot1));
	if(is_startup_alignment &&
		initialized_inter_robot_pairs_.find(robot_pair) !=
			initialized_inter_robot_pairs_.end())
	{
		ROS_DEBUG("Startup alignment for robots %d-%d is already committed",
			robot_pair.first, robot_pair.second);
		return;
	}
	if(is_startup_alignment)
	{
		initialized_inter_robot_pairs_.insert(robot_pair);
	}

	Vector factor_variances(6);
	if(is_startup_alignment)
	{
		// GTSAM Pose3 tangent order is rotation then translation. ICP fitness is
		// measured in m^2, so reusing it as rad^2 made a valid yaw constraint much
		// weaker than intended. The three-observation gate justifies explicit,
		// independently tunable startup variances.
		factor_variances <<
			inter_robot_rotation_variance_, inter_robot_rotation_variance_,
			inter_robot_rotation_variance_, inter_robot_translation_variance_,
			inter_robot_translation_variance_, inter_robot_translation_variance_;
	}
	else
	{
		factor_variances << inter_loop.noise, inter_loop.noise, inter_loop.noise,
			inter_loop.noise, inter_loop.noise, inter_loop.noise;
	}
	noiseModel::Diagonal::shared_ptr loop_noise =
		noiseModel::Diagonal::Variances(factor_variances);
	const Pose3 pose_between = transformToGtsamPose(inter_loop.pose_between);
	if(is_startup_alignment)
	{
		latchStartupWorldToOdom(inter_loop, pose_between);
	}
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		loop_symbol0, loop_symbol1, pose_between, loop_noise));

	adjacency_matrix(inter_loop.robot1, inter_loop.robot0) += 1;
	adjacency_matrix(inter_loop.robot0, inter_loop.robot1) += 1;
	if(inter_loop.robot0 == id_ || inter_loop.robot1 == id_)
	{
		local_pose_graph->add(factor);
		local_pose_graph_no_filtering->add(factor);
		sent_start_optimization_flag = true;

		graph_utils::PoseWithCovariance pose;
		pose.covariance_matrix = loop_noise->covariance();
		pose.pose = transformToGtsamPose(inter_loop.pose1);
		updatePoseEstimateFromNeighbor(inter_loop.robot1, loop_symbol1.key(), pose);
		pose.pose = transformToGtsamPose(inter_loop.pose0);
		updatePoseEstimateFromNeighbor(inter_loop.robot0, loop_symbol0.key(), pose);

		auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
		const Matrix covariance_matrix = loop_noise->covariance();
		// Inter-robot constraints belong to the PCM measurement base class;
		// RobotLocalMap::addTransform expects a same-trajectory odometry edge.
		robot_local_map.robot_measurements::RobotMeasurements::addTransform(
			*new_factor, covariance_matrix);
	}

	loop_indexes.emplace(make_pair(loop_symbol0, loop_symbol1));
	loop_indexes.emplace(make_pair(loop_symbol1, loop_symbol0));
	ROS_INFO("Inter-robot loop accepted: [%d][%d] <-> [%d][%d], fitness=%.4f "
		"rotation_sigma=%.2fdeg translation_sigma=%.3fm",
		inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
		inter_loop.noise,
		std::sqrt(is_startup_alignment ? inter_robot_rotation_variance_ : inter_loop.noise)
			* 180.0 / M_PI,
		std::sqrt(is_startup_alignment ? inter_robot_translation_variance_ : inter_loop.noise));

	if(publish_to_team)
	{
		dcl_slam::loop_info accepted_loop = inter_loop;
		accepted_loop.header.frame_id.clear();
		robots[id_].pub_loop_info.publish(accepted_loop);
	}
}

void distributedMapping::buildLocalRegistrationSubmap(
	pcl::PointCloud<PointPose3D>::Ptr& submap,
	const int& key)
{
	submap->clear();
	if(key < 0 || key >= static_cast<int>(keyposes_cloud_6d->size()) ||
		key >= static_cast<int>(robots[id_].keyframe_cloud_array.size()))
	{
		ROS_WARN("Cannot build local registration submap: robot=%d key=%d poses=%zu clouds=%zu",
			id_, key, keyposes_cloud_6d->size(), robots[id_].keyframe_cloud_array.size());
		return;
	}

	int first_key = key;
	if(startup_multi_frame_enable_ && key < startup_keyframe_count_)
	{
		first_key = key - (key % startup_accumulation_frames_);
	}

	const Pose3 reference_pose = pclPointTogtsamPose3(keyposes_cloud_6d->points[key]);
	for(int frame = first_key; frame <= key; ++frame)
	{
		if(frame >= static_cast<int>(robots[id_].keyframe_cloud_array.size()))
		{
			break;
		}
		const Pose3 frame_pose = pclPointTogtsamPose3(keyposes_cloud_6d->points[frame]);
		const Pose3 frame_to_reference = reference_pose.between(frame_pose);
		*submap += *transformPointCloud(
			robots[id_].keyframe_cloud_array[frame], frame_to_reference);
	}
}

void distributedMapping::buildGlobalRegistrationSubmap(
	pcl::PointCloud<PointPose3D>::Ptr& submap,
	const int& key)
{
	submap->clear();
	if(key < 0 || key >= static_cast<int>(robots[id_].keyframe_cloud_array.size()))
	{
		ROS_WARN("Cannot build global registration submap: robot=%d key=%d clouds=%zu",
			id_, key, robots[id_].keyframe_cloud_array.size());
		return;
	}

	int first_key = key;
	if(startup_multi_frame_enable_ && key < startup_keyframe_count_)
	{
		first_key = key - (key % startup_accumulation_frames_);
	}

	for(int frame = first_key; frame <= key; ++frame)
	{
		const Symbol frame_symbol('a' + id_, frame);
		if(!initial_values->exists(frame_symbol) ||
			frame >= static_cast<int>(robots[id_].keyframe_cloud_array.size()))
		{
			break;
		}
		*submap += *transformPointCloud(
			robots[id_].keyframe_cloud_array[frame],
			initial_values->at<Pose3>(frame_symbol));
	}
}

void distributedMapping::loopFindGlobalNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = initial_values->size();
	CHECK_LE(pose_num, robots[id_].keyframe_cloud_array.size());
	int add_num = 0;
	for(int i = -search_num; i <= search_num*2; ++i)
	{
		if(add_num >= search_num*2)
		{
			break;
		}

		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num)
		{
			continue;
		}
		
		*near_keyframes += *transformPointCloud(robots[id_].keyframe_cloud_array[key_near],
			initial_values->at<Pose3>(Symbol('a'+id_, key_near)));
		add_num++;
	}

	if(near_keyframes->empty())
	{
		return;
	}
}

void distributedMapping::updatePoseEstimateFromNeighbor(
	const int& rid,
	const Key& key,
	const graph_utils::PoseWithCovariance& pose)
{
	graph_utils::TrajectoryPose trajectory_pose;
	trajectory_pose.id = key;
	trajectory_pose.pose = pose;
	// find trajectory
	if(pose_estimates_from_neighbors.find(rid) != pose_estimates_from_neighbors.end())
	{
		// update pose
		if(pose_estimates_from_neighbors.at(rid).trajectory_poses.find(key) != 
			pose_estimates_from_neighbors.at(rid).trajectory_poses.end())
		{
			pose_estimates_from_neighbors.at(rid).trajectory_poses.at(key) = trajectory_pose;
		}
		// new pose
		else
		{
			pose_estimates_from_neighbors.at(rid).trajectory_poses.insert(make_pair(key, trajectory_pose));
			if(key < pose_estimates_from_neighbors.at(rid).start_id)
			{
				pose_estimates_from_neighbors.at(rid).start_id = key;
			}
			if(key > pose_estimates_from_neighbors.at(rid).end_id)
			{
				pose_estimates_from_neighbors.at(rid).end_id = key;
			}
		}
	}
	// insert new trajectory
	else
	{
		graph_utils::Trajectory new_trajectory;
		new_trajectory.trajectory_poses.insert(make_pair(key, trajectory_pose));
		new_trajectory.start_id = key;
		new_trajectory.end_id = key;
		pose_estimates_from_neighbors.insert(make_pair(rid, new_trajectory));
	}
}

void distributedMapping::loopClosureThread()
{
	// Terminate the thread if loop closure are not needed
	if(!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_)
	{
		return;
	}

	ros::Rate rate(1.0/loop_closure_process_interval_);

	while(ros::ok())
	{
		rate.sleep();
		processPendingDescriptors();

		performRSIntraLoopClosure(); // find intra-loop with radius search

		performIntraLoopClosure(); // find intra-loop with descriptor

		performInterLoopClosure(); // find inter-loop with descriptor

		performExternLoopClosure(); // verify all inter-loop here
	}
}
