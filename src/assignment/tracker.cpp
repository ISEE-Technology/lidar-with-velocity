///////////////////////////////////////////////////////////////////////////////

#include "tracker/tracker.h"

int kfTracker::kf_count = 0;

bool plane_judge(
	float a, float b, float c
)
{
	float coeff = 10.0;
	std::vector<float> vec;
	vec.push_back(a);
	vec.push_back(b);
	vec.push_back(c);

	std::sort(vec.begin(), vec.end());

	if (vec[2] > coeff * vec[0] && vec[1] > coeff * vec[0])
	{
		return true;
	}
	else
	{
		return false;
	}

}

class localOptimizer
{
public:
    localOptimizer(
		const Eigen::Vector3d & center,
		const pcl::PointCloud<pcl::PointXYZI> & points,
		const Eigen::Vector3d & dir)
    {
		points_ = points;
		dir_ = dir;
		center_ = center;
    }

template<typename T>
    bool operator() (
        const T* const v_motion, 
        T* residual 
    ) const
    {
		Eigen::Matrix<T,3,1> base(-(T)dir_[1], (T)dir_[0], (T)0.0);
		T mean_x = (T)0.0;
		T mean_y = (T)0.0;
		for (size_t i = 0; i < points_.size(); i++)
		{
			T cur_time = (T)points_.points[i].intensity;
			T cur_x = (T)points_.points[i].x - cur_time * v_motion[0] - center_[0];
			T cur_y = (T)points_.points[i].y - cur_time * v_motion[1] - center_[1];
			T cur_z = (T)points_.points[i].z - cur_time * v_motion[2] - center_[2];
			T weight1 = 
			(dir_[0] * cur_x + 
            dir_[1] * cur_y + 
            dir_[2] * cur_z);
			T weight2 = 
			(base[0] * cur_x + 
            base[1] * cur_y + 
            base[2] * cur_z);
			T base1_x = weight1 * dir_[0];
			T base1_y = weight1 * dir_[1];
			T base1_z = weight1 * dir_[2];
			T base2_x = weight2 * base[0];
			T base2_y = weight2 * base[1];
			T base2_z = weight2 * base[2];
			T dst1 = ceres::sqrt(base1_x * base1_x + base1_y * base1_y + base1_z * base1_z);
			T dst2 = ceres::sqrt(base2_x * base2_x + base2_y * base2_y + base2_z * base2_z);
			mean_x += dst1;
			mean_y += dst2;
		}
		mean_x /= (T)points_.size();
		mean_y /= (T)points_.size();
		T accum_x = (T)0.0;
		T accum_y = (T)0.0;
		for (size_t i = 0; i < points_.size(); i++)
		{
			T cur_time = (T)points_.points[i].intensity;
			T cur_x = (T)points_.points[i].x - cur_time * v_motion[0] - center_[0];
			T cur_y = (T)points_.points[i].y - cur_time * v_motion[1] - center_[1];
			T cur_z = (T)points_.points[i].z - cur_time * v_motion[2] - center_[2];
			T weight1 = 
			(dir_[0] * cur_x + 
            dir_[1] * cur_y + 
            dir_[2] * cur_z);
			T weight2 = 
			(base[0] * cur_x + 
            base[1] * cur_y + 
            base[2] * cur_z);
			T base1_x = weight1 * dir_[0];
			T base1_y = weight1 * dir_[1];
			T base1_z = weight1 * dir_[2];
			T base2_x = weight2 * base[0];
			T base2_y = weight2 * base[1];
			T base2_z = weight2 * base[2];
			T dst1 = ceres::sqrt(base1_x * base1_x + base1_y * base1_y + base1_z * base1_z);
			T dst2 = ceres::sqrt(base2_x * base2_x + base2_y * base2_y + base2_z * base2_z);
			accum_x += (dst1 - mean_x) * (dst1 - mean_x);
			accum_y += (dst2 - mean_y) * (dst2 - mean_y);
		}

		accum_x /= (T)(points_.size() - 1);
		accum_y /= (T)(points_.size() - 1);
		accum_x = ceres::sqrt(accum_x);
		accum_y = ceres::sqrt(accum_y);
		residual[0] = (accum_x + accum_y) * (T)points_.size();
		residual[1] = (T)0;

        return true;
    }

    static ceres::CostFunction* Create(
		const Eigen::Vector3d & center,
		const pcl::PointCloud<pcl::PointXYZI> & points,
		const Eigen::Vector3d & dir)
    {
        return (
            new ceres::AutoDiffCostFunction<localOptimizer,2 ,3>(
				new localOptimizer(center, points, dir)
			)
        );
    }

private:
	Eigen::Vector3d center_;
    pcl::PointCloud<pcl::PointXYZI> points_;
	Eigen::Vector3d dir_;
};

class globalOptimizer
{
public:
    globalOptimizer(
		const Eigen::Vector3d & center,
		const pcl::PointCloud<pcl::PointXYZI> & points,
		const Eigen::Vector3d & dir)
    {
		center_ = center;
		points_ = points;
		dir_ = dir;
    }

template<typename T>
    bool operator() (
        const T* const v_motion, 
        T* residual 
    ) const
    {
		Eigen::Matrix<T,3,1> base(-(T)dir_[1], (T)dir_[0], (T)0.0);
		T mean_x = (T)0.0;
		T mean_y = (T)0.0;
		for (size_t i = 0; i < points_.size(); i++)
		{
			T cur_time = (T)points_.points[i].intensity;
			T cur_x = (T)points_.points[i].x - cur_time * v_motion[0] - center_[0];
			T cur_y = (T)points_.points[i].y - cur_time * v_motion[1] - center_[1];
			T cur_z = (T)points_.points[i].z - cur_time * v_motion[2] - center_[2];
			T weight1 = 
			(dir_[0] * cur_x + 
            dir_[1] * cur_y + 
            dir_[2] * cur_z);
			T weight2 = 
			(base[0] * cur_x + 
            base[1] * cur_y + 
            base[2] * cur_z);
			T base1_x = weight1 * dir_[0];
			T base1_y = weight1 * dir_[1];
			T base1_z = weight1 * dir_[2];
			T base2_x = weight2 * base[0];
			T base2_y = weight2 * base[1];
			T base2_z = weight2 * base[2];
			T dst1 = ceres::sqrt(base1_x * base1_x + base1_y * base1_y + base1_z * base1_z);
			T dst2 = ceres::sqrt(base2_x * base2_x + base2_y * base2_y + base2_z * base2_z);
			mean_x += dst1;
			mean_y += dst2;
		}
		mean_x /= (T)points_.size();
		mean_y /= (T)points_.size();
		T accum_x = (T)0.0;
		T accum_y = (T)0.0;
		for (size_t i = 0; i < points_.size(); i++)
		{
			T cur_time = (T)points_.points[i].intensity;
			T cur_x = (T)points_.points[i].x - cur_time * v_motion[0] - center_[0];
			T cur_y = (T)points_.points[i].y - cur_time * v_motion[1] - center_[1];
			T cur_z = (T)points_.points[i].z - cur_time * v_motion[2] - center_[2];
			T weight1 = 
			(dir_[0] * cur_x + 
            dir_[1] * cur_y + 
            dir_[2] * cur_z);
			T weight2 = 
			(base[0] * cur_x + 
            base[1] * cur_y + 
            base[2] * cur_z);
			T base1_x = weight1 * dir_[0];
			T base1_y = weight1 * dir_[1];
			T base1_z = weight1 * dir_[2];
			T base2_x = weight2 * base[0];
			T base2_y = weight2 * base[1];
			T base2_z = weight2 * base[2];
			T dst1 = ceres::sqrt(base1_x * base1_x + base1_y * base1_y + base1_z * base1_z);
			T dst2 = ceres::sqrt(base2_x * base2_x + base2_y * base2_y + base2_z * base2_z);
			accum_x += (dst1 - mean_x) * (dst1 - mean_x);
			accum_y += (dst2 - mean_y) * (dst2 - mean_y);
		}

		accum_x /= (T)(points_.size() - 1);
		accum_y /= (T)(points_.size() - 1);
		accum_x = ceres::sqrt(accum_x);
		accum_y = ceres::sqrt(accum_y);
		residual[0] = (T)0;
		residual[1] = (accum_x + accum_y) * (T)points_.size();

        return true;
    }

    static ceres::CostFunction* Create(
		const Eigen::Vector3d & center,
		const pcl::PointCloud<pcl::PointXYZI> & points,
		const Eigen::Vector3d & dir)
    {
        return (
            new ceres::AutoDiffCostFunction<globalOptimizer,2 ,3>(
				new globalOptimizer(center, points, dir)
			)
        );
    }

private:
	Eigen::Vector3d center_;
    pcl::PointCloud<pcl::PointXYZI> points_;
	Eigen::Vector3d dir_;
};

class POINT_COST
{
public:
    POINT_COST(
		pcl::PointXYZI point,
		Eigen::Vector3d target_centroid,
		Eigen::Vector3d obj_direction,
		Eigen::Vector3d axis_weight,
		double ptsize
    )
    {
		point_ = point;
		target_centroid_ = target_centroid;
		obj_direction_ = obj_direction;
		axis_weight_ = axis_weight;
		ptsize_ = ptsize;
    }

template<typename T>
    bool operator() (
        const T* const v_motion, 
        T* residual 
    ) const
    {
		Eigen::Matrix<T,3,1> v_motion_T;
		v_motion_T[0] = (T)obj_direction_[0] * v_motion[0];
		v_motion_T[1] = (T)obj_direction_[1] * v_motion[0];
		v_motion_T[2] = (T)obj_direction_[2] * v_motion[0];

		T undist_x = (T)point_.x - v_motion_T[0] * (T)point_.intensity;
		T undist_y = (T)point_.y - v_motion_T[1] * (T)point_.intensity;
		T undist_z = (T)point_.z - v_motion_T[2] * (T)point_.intensity;

		Eigen::Matrix<T,3,1> vec2(
            undist_x - (T)target_centroid_[0],
            undist_y - (T)target_centroid_[1],
            undist_z - (T)target_centroid_[2]
        );

        Eigen::Matrix<T,3,1> base2((T)0.0,(T)0.0,(T)1.0);
        Eigen::Matrix<T,3,1> base3;
        base3 = {
            base2[1] * obj_direction_[2] - obj_direction_[1] * base2[2],
            - base2[0] * obj_direction_[2] + obj_direction_[0] * base2[2],
            base2[0] * obj_direction_[1] - obj_direction_[0] * base2[1]
        };

        T dst1 = ceres::abs(
            obj_direction_[0] * vec2[0] + 
            obj_direction_[1] * vec2[1] + 
            obj_direction_[2] * vec2[2])  /
            ceres::sqrt(
                obj_direction_[0] * obj_direction_[0] + 
                obj_direction_[1] * obj_direction_[1] + 
                obj_direction_[2] * obj_direction_[2]);

        T dst2 = ceres::abs(
            base2[0] * vec2[0] + 
            base2[1] * vec2[1] + 
            base2[2] * vec2[2])  /
            ceres::sqrt(
                base2[0] * base2[0] + 
                base2[1] * base2[1] + 
                base2[2] * base2[2]);

        T dst3 = ceres::abs(
            base3[0] * vec2[0] + 
            base3[1] * vec2[1] + 
            base3[2] * vec2[2])  /
            ceres::sqrt(
                base3[0] * base3[0] + 
                base3[1] * base3[1] + 
                base3[2] * base3[2]);

		residual[0] = axis_weight_[0] * (dst1 * dst1) / ptsize_;
		residual[1] = axis_weight_[1] * (dst2 * dst2) / ptsize_;
		residual[2] = axis_weight_[2] * (dst3 * dst3) / ptsize_;
        return true;
    }

    static ceres::CostFunction* Create(
		pcl::PointXYZI point,
		Eigen::Vector3d target_centroid,
		Eigen::Vector3d obj_direction,
		Eigen::Vector3d axis_weight,
		double ptsize
    )
    {
        return (
            new ceres::AutoDiffCostFunction<POINT_COST,3,1>(
				new POINT_COST(point,target_centroid,obj_direction,axis_weight,ptsize)
			)
        );
    }

private:
    pcl::PointXYZI point_;
    Eigen::Vector3d target_centroid_;
	Eigen::Vector3d obj_direction_;
    Eigen::Vector3d axis_weight_;
    double ptsize_;
};

// ======================== kfTracker ========================
alignedDet kfTracker::predict()
{
	cv::Mat p = kf_.predict();
	m_age += 1;

	if (m_time_since_update > 0)
	{
		m_hit_streak = 0;
	}
	m_time_since_update += 1;

	return detection_cur_;
}

void kfTracker::init_kf(
	alignedDet detection_in
)
{
	int stateNum = 10;
	int measureNum = 10;

	kf_ = cv::KalmanFilter(stateNum, measureNum, 0);

	measurement_ = cv::Mat::zeros(measureNum, 1, CV_32F);

	kf_.transitionMatrix = (Mat_<float>(stateNum, stateNum) <<
		1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0, 0.1, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0.1,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1
	);

	setIdentity(kf_.measurementMatrix); 
	setIdentity(kf_.processNoiseCov, Scalar::all(0.1));  // Q
	setIdentity(kf_.measurementNoiseCov, Scalar::all(0.6));  // R
	setIdentity(kf_.errorCovPost, Scalar::all(1000)); // P

	std::vector<float> state_ = getState(detection_in);

	kf_.statePost.at<float>(0, 0) = state_[0];
	kf_.statePost.at<float>(1, 0) = state_[1];
	kf_.statePost.at<float>(2, 0) = state_[2];
	kf_.statePost.at<float>(3, 0) = state_[3];
	kf_.statePost.at<float>(4, 0) = state_[4];
	kf_.statePost.at<float>(5, 0) = state_[5];
	kf_.statePost.at<float>(6, 0) = state_[6];

	detection_cur_ = detection_in;

	rgb3[0] = (rand() % 255) + 0;
	rgb3[1] = (rand() % 255) + 0;
	rgb3[2] = (rand() % 255) + 0;
}

std::vector<float> kfTracker::getState(
	alignedDet detection_in
)
{
	std::vector<float> out_state;
	float centerx = 0.0;
	float centery = 0.0;
	float centerz = 0.0;
	for (size_t pt_idx = 0; pt_idx < detection_in.vertex3d_.rows(); pt_idx++)
	{
		centerx += detection_in.vertex3d_(pt_idx, 0);
		centery += detection_in.vertex3d_(pt_idx, 1);
		centerz += detection_in.vertex3d_(pt_idx, 2);
	}
	centerx /= detection_in.vertex3d_.rows();
	centery /= detection_in.vertex3d_.rows();
	centerz /= detection_in.vertex3d_.rows();
	out_state.push_back(centerx);
	out_state.push_back(centery);
	out_state.push_back(centerz);

	float yaw = std::acos(
		centery / std::sqrt(centerx * centerx + centery * centery)
	);
	out_state.push_back(yaw);

	float long_ = std::abs(
		2.0 * (detection_in.vertex3d_(0, 0) - centerx)
	);
	float width_ = std::abs(
		2.0 * (detection_in.vertex3d_(0, 1) - centery)
	);
	float depth_ = std::abs(
		2.0 * (detection_in.vertex3d_(0, 2) - centerz)
	);
	out_state.push_back(long_);
	out_state.push_back(width_);
	out_state.push_back(depth_);

	return out_state;
}

void kfTracker::update(
	alignedDet detection_in,
	const Eigen::Vector3d & vel_in,
	const Eigen::Matrix3d & vel_cov_
)
{
	m_time_since_update = 0;
	measurement_history_.clear();
	m_hits += 1;
	m_hit_streak += 1;
	if (std::isnan(vel_in(0)) || std::isnan(vel_in(1)) || std::isnan(vel_in(2)))
	{
		measurement_.at<float>(7, 0) = kf_.statePost.at<float>(7, 0);
		measurement_.at<float>(8, 0) = kf_.statePost.at<float>(8, 0);
		measurement_.at<float>(9, 0) = kf_.statePost.at<float>(9, 0);
	}
	

	std::vector<float> measuredata = getState(detection_in);
	measurement_.at<float>(0, 0) = measuredata[0];
	measurement_.at<float>(1, 0) = measuredata[1];
	measurement_.at<float>(2, 0) = measuredata[2];
	measurement_.at<float>(3, 0) = measuredata[3];
	measurement_.at<float>(4, 0) = measuredata[4];
	measurement_.at<float>(5, 0) = measuredata[5];
	measurement_.at<float>(6, 0) = measuredata[6];
	measurement_.at<float>(7, 0) = vel_in(0);
	measurement_.at<float>(8, 0) = vel_in(1);
	measurement_.at<float>(9, 0) = vel_in(2);

	cv::Mat vel_measurement_cov;
	cv::eigen2cv(vel_cov_, vel_measurement_cov);
	kf_.measurementNoiseCov(cv::Rect(7, 7, 3, 3)) = vel_measurement_cov;

	kf_.correct(measurement_);

	detection_last_ = detection_cur_;
	detection_cur_ = detection_in;


}

void kfTracker::get_kf_vel(
	Eigen::Vector3d & vel_
)
{
	vel_(0) = kf_.statePost.at<float>(7, 0);
	vel_(1) = kf_.statePost.at<float>(8, 0);
	vel_(2) = kf_.statePost.at<float>(9, 0);
}

void kfTracker::update_estimated_vel()
{
	Eigen::Vector3d kf_vel(
		kf_.statePost.at<float>(7, 0),
		kf_.statePost.at<float>(8, 0),
		kf_.statePost.at<float>(9, 0)
	);
	estimated_vel_ = kf_vel;
	estimated_vel_cov_(0,0) = kf_.errorCovPost.at<float>(7, 7);
	estimated_vel_cov_(0,1) = kf_.errorCovPost.at<float>(7, 8);
	estimated_vel_cov_(0,2) = kf_.errorCovPost.at<float>(7, 9);
	estimated_vel_cov_(1,0) = kf_.errorCovPost.at<float>(8, 7);
	estimated_vel_cov_(1,1) = kf_.errorCovPost.at<float>(8, 8);
	estimated_vel_cov_(1,2) = kf_.errorCovPost.at<float>(8, 9);
	estimated_vel_cov_(2,0) = kf_.errorCovPost.at<float>(9, 7);
	estimated_vel_cov_(2,1) = kf_.errorCovPost.at<float>(9, 8);
	estimated_vel_cov_(2,2) = kf_.errorCovPost.at<float>(9, 9);

}

fusion_tracker::fusion_tracker()
{
	total_frames_ = 0;
	frame_count_ = 0;
	trkNum_ = 0;
	detNum_ = 0;
	iouThreshold_ = 0.01;
}

fusion_tracker::~fusion_tracker()
{
}

void fusion_tracker::tracking(
	const std::vector<alignedDet> & detections_in,
	const cv::Mat & img_in,
	const Config & config_,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
	visualization_msgs::MarkerArray & obj_vel_txt_markerarray,
	VisHandel * vis_ros_
)
{
	Timer tracker_timer("tracking time");
	total_frames_++;
	frame_count_++;
	if (trackers_.size() == 0)
	{
		for (size_t obj_idx = 0; obj_idx < detections_in.size(); obj_idx++)
		{
			kfTracker trk(detections_in[obj_idx]);
			trackers_.push_back(trk);
		}
		if(frame_count_ = 1)
		{
			last_detection_ = detections_in;
			last_img_ = img_in;
			return;
		}
		else
		{
			std::cout << "first tracking frame" << std::endl;
		}
	}
	else
	{
		std::cout << "last tracker num = " << trackers_.size() << std::endl;
	}
	
	predictedBoxes_.clear();

	for (auto it = trackers_.begin(); it != trackers_.end();)
	{
		alignedDet predict_det = it->predict();

		if (predict_det.confidence3d_ > 0.0)
		{
			predictedBoxes_.push_back(predict_det);
			it++;
		}
		else
		{
			it = trackers_.erase(it);
			cerr << "Box invalid at frame: " << frame_count_ << endl;
		}
	}

	trkNum_ = predictedBoxes_.size();
	detNum_ = detections_in.size();

	iouMatrix_.clear();
	iouMatrix_.resize(trkNum_, vector<double>(detNum_, 1));

	for (unsigned int i = 0; i < trkNum_; i++)
	{
		for (unsigned int j = 0; j < detNum_; j++)
		{
			iouMatrix_[i][j] = 1 - GetIOU(predictedBoxes_[i], detections_in[j]);
		}
	}

	HungarianAlgorithm HungAlgo;
	HungariaAssignment_.clear();
	HungAlgo.Solve(iouMatrix_, HungariaAssignment_);

	unmatchedTrajectories_.clear();
	unmatchedDetections_.clear();
	allItems_.clear();
	matchedItems_.clear();

	if (detNum_ > trkNum_) 
	{
		for (unsigned int n = 0; n < detNum_; n++)
			allItems_.insert(n);

		for (unsigned int i = 0; i < trkNum_; ++i)
			matchedItems_.insert(HungariaAssignment_[i]);

		set_difference(
			allItems_.begin(), 
			allItems_.end(),
			matchedItems_.begin(), 
			matchedItems_.end(),
			insert_iterator<set<int>>(unmatchedDetections_, unmatchedDetections_.begin())
		);
	}
	else if (detNum_ < trkNum_) // there are unmatched trajectory/predictions
	{
		for (unsigned int i = 0; i < trkNum_; ++i)
			if (HungariaAssignment_[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
				unmatchedTrajectories_.insert(i);
	}
	else;

	matchedPairs_.clear();
	for (unsigned int i = 0; i < trkNum_; ++i)
	{
		if (HungariaAssignment_[i] == -1) // pass over invalid values
			continue;
		if (1 - iouMatrix_[i][HungariaAssignment_[i]] < iouThreshold_)
		{
			unmatchedTrajectories_.insert(i);
			unmatchedDetections_.insert(HungariaAssignment_[i]);
		}
		else
		{
			matchedPairs_.push_back(cv::Point(i, HungariaAssignment_[i]));
		}
	}

	vector<alignedDet> match_trackers;
	vector<alignedDet> match_detections;
	for (unsigned int i = 0; i < matchedPairs_.size(); i++)
	{
		int detIdx, trkIdx;
		trkIdx = matchedPairs_[i].x;
		detIdx = matchedPairs_[i].y;
		match_trackers.push_back(trackers_[trkIdx].detection_cur_);
		match_detections.push_back(detections_in[detIdx]);
	}

	for (auto umd : unmatchedDetections_)
	{
		kfTracker new_tracker(detections_in[umd]);
		trackers_.push_back(new_tracker);
	}
	tracker_timer.rlog("tracking cost time");

	std::vector<Eigen::Vector2d> obj_means;
	std::vector<Eigen::Matrix2d> obj_covariances;
	optical_estimator(
		last_img_, 
		img_in, 
		match_trackers, 
		match_detections, 
		config_,
		obj_means,
		obj_covariances
	);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr undistorted_obj_clouds(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_obj_clouds(new pcl::PointCloud<pcl::PointXYZRGB>);
	visualization_msgs::Marker obj_vel_arrow;
	obj_vel_arrow.header.frame_id = "livox";
	obj_vel_arrow.ns = "obj_vel_arrow";
    obj_vel_arrow.lifetime = ros::Duration(0);
	obj_vel_arrow.type = visualization_msgs::Marker::ARROW;
	obj_vel_arrow.action = visualization_msgs::Marker::ADD;
	obj_vel_arrow.scale.x = 0.05;
	obj_vel_arrow.scale.y = 0.08;
	obj_vel_arrow.scale.z = 0.1;
	obj_vel_arrow.color.a = 1.0;
	obj_vel_arrow.color.g = 1.0;

	if (obj_vel_txt_markerarray.markers.size() > 0)
	{
		for (size_t obj_idx = 0; obj_idx < obj_vel_txt_markerarray.markers.size(); obj_idx++)
		{
			obj_vel_txt_markerarray.markers[obj_idx].color.a = 0.0;
		}
		vis_ros_->obj_vel_txt_publisher(obj_vel_txt_markerarray);
		obj_vel_txt_markerarray.markers.clear();
	}
	

	for (size_t obj_idx = 0; obj_idx < match_trackers.size(); obj_idx++)
	{
		int detIdx, trkIdx;
		trkIdx = matchedPairs_[obj_idx].x;
		detIdx = matchedPairs_[obj_idx].y;
		Eigen::Vector3f track_center_pcl = match_trackers[obj_idx].vertex3d_.colwise().mean().cast<float>();
		Eigen::Vector3f detect_center_pcl = match_detections[obj_idx].vertex3d_.colwise().mean().cast<float>();
		pcl::PointXYZ track_center;
		track_center.x = track_center_pcl[0];
		track_center.y = track_center_pcl[1];
		track_center.z = track_center_pcl[2];
		pcl::PointXYZ detect_center;
		detect_center.x = detect_center_pcl[0];
		detect_center.y = detect_center_pcl[1];
		detect_center.z = detect_center_pcl[2];
		viewer->addSphere(
			detect_center, 
			0.1, 
			255, 0, 0, 
			"sphere_center" + std::to_string(rand()), 
			0
		);
		viewer->addLine(
			track_center, 
			detect_center, 
			0, 255, 0, 
			"tracking line" + std::to_string(rand())
		);
		Eigen::Vector3d fused_vel;
		Eigen::Matrix3d fused_vel_cov;
		Eigen::Vector3d points_vel;
		points_estimator(
			viewer,
			trackers_[trkIdx].detection_last_,
			match_trackers[obj_idx], 
			match_detections[obj_idx],
			points_vel,
			obj_means[obj_idx],
			obj_covariances[obj_idx],
			&trackers_[trkIdx],
			fused_vel,
			fused_vel_cov,
			config_
		);
		trackers_[trkIdx].update(
			detections_in[detIdx],
			fused_vel,
			fused_vel_cov
		);
		Eigen::Vector3d out_vel;
		trackers_[trkIdx].get_kf_vel(out_vel);
		trackers_[trkIdx].update_estimated_vel();

		Eigen::Vector3d vel = out_vel;
		cloud_undistortion(
			trackers_[trkIdx],
			detections_in[detIdx],
			out_vel,
			undistorted_obj_clouds,
			raw_obj_clouds,
			obj_vel_arrow,
			obj_vel_txt_markerarray
		);
	}

	sensor_msgs::PointCloud2::Ptr undistorted_obj_cloud_msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*undistorted_obj_clouds, *undistorted_obj_cloud_msg);
	vis_ros_->undistorted_obj_cloud_publisher(*undistorted_obj_cloud_msg);
	vis_ros_->obj_vel_arrow_publisher(obj_vel_arrow);
	vis_ros_->obj_vel_txt_publisher(obj_vel_txt_markerarray);
	sensor_msgs::PointCloud2::Ptr raw_obj_cloud_msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*raw_obj_clouds, *raw_obj_cloud_msg);
	vis_ros_->raw_obj_cloud_publisher(*raw_obj_cloud_msg);

	last_detection_ = detections_in;
	last_img_ = img_in;
}

void fusion_tracker::alignedDet2rotaterect(
	const alignedDet & detection_in,
	cv::RotatedRect & out_rect
)
{
    cv::Point2f det_center;
    for (size_t pt_idx = 0; pt_idx < detection_in.vertex3d_.rows(); pt_idx++)
    {
        det_center.x += detection_in.vertex3d_(pt_idx, 0);
        det_center.y += detection_in.vertex3d_(pt_idx, 1);
    }
    det_center.x /= detection_in.vertex3d_.rows();
    det_center.y /= detection_in.vertex3d_.rows();

    float det_width = sqrt(
        pow(detection_in.vertex3d_(0,0) - detection_in.vertex3d_(1,0), 2) + 
        pow(detection_in.vertex3d_(0,1) - detection_in.vertex3d_(1,1), 2)
    );
    float det_height = sqrt(
        pow(detection_in.vertex3d_(0,0) - detection_in.vertex3d_(3,0), 2) + 
        pow(detection_in.vertex3d_(0,1) - detection_in.vertex3d_(3,1), 2)
    );
    cv::Size2f det_size(det_width, det_height);

    Eigen::Vector2f base_dir(-1.0, 0.0);
    Eigen::Vector2f angle_dir(
        detection_in.vertex3d_(1,0) - detection_in.vertex3d_(0,0),
        detection_in.vertex3d_(1,1) - detection_in.vertex3d_(0,1)
    );
    float det_angle = base_dir.dot(angle_dir) / (base_dir.norm() * angle_dir.norm());
    det_angle = acos(det_angle) * 180.0 / M_PI;

    cv::RotatedRect rect(
        det_center, 
        det_size,
        det_angle
    );

    out_rect = rect;
}

double fusion_tracker::GetIOU(
	const alignedDet & bb_test, 
	const alignedDet & bb_gt
)
{
	/* a 2d projection iou method */
    cv::RotatedRect rect1;
	alignedDet2rotaterect(bb_test, rect1);
    cv::RotatedRect rect2;
	alignedDet2rotaterect(bb_gt, rect2);
    float areaRect1 = rect1.size.width * rect1.size.height;
    float areaRect2 = rect2.size.width * rect2.size.height;
    vector<cv::Point2f> vertices;

    int intersectionType = cv::rotatedRectangleIntersection(rect1, rect2, vertices);
    if (vertices.size()==0)
        return 0.0;
    else{
        vector<cv::Point2f> order_pts;

        cv::convexHull(cv::Mat(vertices), order_pts, true);
        double area = cv::contourArea(order_pts);
        float inner = (float) (area / (areaRect1 + areaRect2 - area + 0.0001));

        return inner;
    }
}

void fusion_tracker::optical_estimator(
	const cv::Mat & prev_,
	const cv::Mat & cur_,
	const std::vector<alignedDet> & prev_detection,
	const std::vector<alignedDet> & cur_detection,
	const Config & config_,
	std::vector<Eigen::Vector2d> & obj_means,
	std::vector<Eigen::Matrix2d> & obj_covariances
)
{
	Timer lk_timer("optical flow time");

	int obj_num = prev_detection.size();
	Mat mask = Mat::zeros(
		prev_.size(), 
		prev_.type()
	);
	cv::Mat prev_gray, cur_gray;
	vector<cv::Point2f> cur_pts;
	vector<cv::Point2f> prev_pts;
	vector<cv::Point2f> tracked_cur_pts;
	vector<cv::Point2f> tracked_prev_pts;
	if (prev_pts_.size() == 0)
	{
		cv::cvtColor(
			prev_,
			prev_gray,
			cv::COLOR_RGB2GRAY
		);
		cv::goodFeaturesToTrack(
			prev_gray,
			prev_pts, 
			config_.maxCorners_,
			config_.qualityLevel_, 
			config_.minDistance_,
			Mat(), 
			config_.blockSize_, 
			false, config_.Harris_k_value_
		);
		for (size_t pt_idx = 0; pt_idx < prev_pts.size(); pt_idx++)
		{
			for (size_t obj_idx = 0; obj_idx < obj_num; obj_idx++)
			{
				if (
					prev_pts[pt_idx].x >= prev_detection[obj_idx].vertex2d_.x
					&& prev_pts[pt_idx].x <= prev_detection[obj_idx].vertex2d_.x + prev_detection[obj_idx].vertex2d_.width
					&& prev_pts[pt_idx].y >= prev_detection[obj_idx].vertex2d_.y
					&& prev_pts[pt_idx].y <= prev_detection[obj_idx].vertex2d_.y + prev_detection[obj_idx].vertex2d_.height
				)
				{
					tracked_prev_pts.push_back(prev_pts[pt_idx]);
					break;
				}
			}
		}
	}
	else
	{
		tracked_prev_pts = prev_pts_;
		prev_gray = prev_gray_;
	}
	

	cv::cvtColor(
		cur_,
		cur_gray,
		cv::COLOR_RGB2GRAY
	);

	cv::goodFeaturesToTrack(
		cur_gray,
		cur_pts, 
		config_.maxCorners_, 
		config_.qualityLevel_, 
		config_.minDistance_,
		Mat(), 
		config_.blockSize_, 
		false, config_.Harris_k_value_
	);
	for (size_t pt_idx = 0; pt_idx < cur_pts.size(); pt_idx++)
	{
		for (size_t obj_idx = 0; obj_idx < cur_detection.size(); obj_idx++)
		{
			if (
				cur_pts[pt_idx].x >= cur_detection[obj_idx].vertex2d_.x
				&& cur_pts[pt_idx].x <= cur_detection[obj_idx].vertex2d_.x + cur_detection[obj_idx].vertex2d_.width
				&& cur_pts[pt_idx].y >= cur_detection[obj_idx].vertex2d_.y
				&& cur_pts[pt_idx].y <= cur_detection[obj_idx].vertex2d_.y + cur_detection[obj_idx].vertex2d_.height
			)
			{
				tracked_cur_pts.push_back(cur_pts[pt_idx]);
				break;
			}
		}
	}

	vector<uchar> status;
	vector<float> err;
	Timer calcOpticalFlowPyrLK_timer("calcOpticalFlowPyrLK time");
	cv::calcOpticalFlowPyrLK(
		cur_gray, 
		prev_gray, 
		tracked_cur_pts, 
		tracked_prev_pts, 
		status, 
		err, 
		cv::Size(20, 20), 3
	);
	calcOpticalFlowPyrLK_timer.rlog("calcOpticalFlowPyrLK cost");
	vector<Point2f> matchedpoints1;
	vector<Point2f> matchedpoints2;
	vector<Point2f> obj_features_prev[obj_num];
	vector<Point2f> obj_features_cur[obj_num];
	for(uint i = 0; i < tracked_cur_pts.size(); i++)
	{
		if(status[i] == 1) 
		{
			for (size_t obj_idx = 0; obj_idx < obj_num; obj_idx++)
			{
				if (
					tracked_prev_pts[i].x >= prev_detection[obj_idx].vertex2d_.x
					&& tracked_prev_pts[i].x <= prev_detection[obj_idx].vertex2d_.x + prev_detection[obj_idx].vertex2d_.width
					&& tracked_prev_pts[i].y >= prev_detection[obj_idx].vertex2d_.y
					&& tracked_prev_pts[i].y <= prev_detection[obj_idx].vertex2d_.y + prev_detection[obj_idx].vertex2d_.height
				)
				{
					obj_features_prev[obj_idx].push_back(tracked_prev_pts[i]);
					obj_features_cur[obj_idx].push_back(tracked_cur_pts[i]);
					break;
				}
			}
		}
	}

	lk_timer.rlog("ransac timer cost : ");

	Timer ransac_timer("optical flow time");
	std::vector<uchar> ransac_status[obj_num];
	std::pair<std::vector<Point2f>, std::vector<Point2f>>  pixels_vel[obj_num];
	for (size_t obj_idx = 0; obj_idx < obj_num; obj_idx++)
	{
		if (obj_features_prev[obj_idx].size() == 0)
		{
			continue;
		}
		cv::Mat matrix_fundamental = 
			cv::findFundamentalMat(
				obj_features_prev[obj_idx], 
				obj_features_cur[obj_idx], 
				ransac_status[obj_idx], 
				CV_FM_RANSAC
			);

		int rand_r = (rand() % 255) + 0;
		int rand_g = (rand() % 255) + 0;
		int rand_b = (rand() % 255) + 0;
		for (size_t pix_idx = 0; pix_idx < ransac_status[obj_idx].size(); pix_idx++)
		{
			if (ransac_status[obj_idx][pix_idx] != 0)
			{
				line(
					mask,
					obj_features_prev[obj_idx][pix_idx], 
					obj_features_cur[obj_idx][pix_idx], 
					cv::Scalar(rand_r, rand_g, rand_b), 2
				);
				pixels_vel[obj_idx].first.push_back(obj_features_prev[obj_idx][pix_idx]);
				pixels_vel[obj_idx].second.push_back(obj_features_cur[obj_idx][pix_idx]);
			}
		}
	}
	ransac_timer.rlog("ransac cost");

	std::vector<Eigen::Vector2d> means;
	std::vector<Eigen::Matrix2d> covariances;
	means.resize(obj_num);
	covariances.resize(obj_num);
	for (size_t obj_idx = 0; obj_idx < obj_num; obj_idx++)
	{
		Eigen::Vector2d mean_;
		mean_.setZero();
		Eigen::Matrix2d covariance_;
		covariance_.setZero();
		if (pixels_vel[obj_idx].first.size())
		{
			for (size_t pix_idx = 0; pix_idx < pixels_vel[obj_idx].first.size(); pix_idx++)
			{
				mean_[0] += (pixels_vel[obj_idx].second[pix_idx].x - pixels_vel[obj_idx].first[pix_idx].x);
				mean_[1] += (pixels_vel[obj_idx].second[pix_idx].y - pixels_vel[obj_idx].first[pix_idx].y);
			}
			mean_ /= (float)pixels_vel[obj_idx].first.size();
			for (size_t pix_idx = 0; pix_idx < pixels_vel[obj_idx].first.size(); pix_idx++)
			{
				covariance_(0, 0) += pow((pixels_vel[obj_idx].second[pix_idx].x - pixels_vel[obj_idx].first[pix_idx].x - mean_[0]), 2) / pixels_vel[obj_idx].first.size();
				covariance_(1, 1) += pow((pixels_vel[obj_idx].second[pix_idx].y - pixels_vel[obj_idx].first[pix_idx].y - mean_[1]), 2) / pixels_vel[obj_idx].first.size();
			}
			covariances[obj_idx] = covariance_;
			means[obj_idx] = mean_;
		}
	}
	obj_means = means;
	obj_covariances = covariances;

	Mat img;
	cv::add(prev_, mask, img);

	prev_pts_ = tracked_cur_pts;
	prev_gray_ = cur_gray;
}

void fusion_tracker::points_estimator(
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
	const alignedDet & pp_detection,
	const alignedDet & prev_detection,
	const alignedDet & cur_detection,
	Eigen::Vector3d & optimal_vel,
	const Eigen::Vector2d & pix_vel,
	const Eigen::Matrix2d & pix_vel_cov,
	kfTracker * tracker,
	Eigen::Vector3d & fused_vel,
	Eigen::Matrix3d & fused_vel_cov,
	const Config & config_
)
{
	if (
		prev_detection.cloud_.size() == 0 || 
		cur_detection.cloud_.size() < 20
	)
	{
		fused_vel = tracker->estimated_vel_;
		fused_vel_cov = tracker->estimated_vel_cov_;
		return;
	}

	Eigen::Vector3d target_centroid = cur_detection.vertex3d_.colwise().mean();

	pcl::PointXYZ arrow_start, arrow_end;
	arrow_start.x = target_centroid[0];
	arrow_start.y = target_centroid[1];
	arrow_start.z = target_centroid[2];
	Eigen::Vector3d cube_side_1, cube_side_2;
	cube_side_1[0] = cur_detection.vertex3d_(0, 0) - cur_detection.vertex3d_(3, 0);
	cube_side_1[1] = cur_detection.vertex3d_(0, 1) - cur_detection.vertex3d_(3, 1);
	cube_side_1[2] = cur_detection.vertex3d_(0, 2) - cur_detection.vertex3d_(3, 2);
	cube_side_2[0] = cur_detection.vertex3d_(0, 0) - cur_detection.vertex3d_(1, 0);
	cube_side_2[1] = cur_detection.vertex3d_(0, 1) - cur_detection.vertex3d_(1, 1);
	cube_side_2[2] = cur_detection.vertex3d_(0, 2) - cur_detection.vertex3d_(1, 2);
	Eigen::Vector3d cube_side_start, cube_side_end;
	if (cube_side_1.norm() > cube_side_2.norm())
	{
		arrow_end.x = arrow_start.x + cube_side_1[0];
		arrow_end.y = arrow_start.y + cube_side_1[1];
		arrow_end.z = arrow_start.z + cube_side_1[2];
		cube_side_start[0] = arrow_end.x;
		cube_side_start[1] = arrow_end.y;
		cube_side_start[2] = arrow_end.z;
		cube_side_end[0] = arrow_start.x - cube_side_1[0];
		cube_side_end[1] = arrow_start.y - cube_side_1[1];
		cube_side_end[2] = arrow_start.z - cube_side_1[2];
	}
	else
	{
		arrow_end.x = arrow_start.x + cube_side_2[0];
		arrow_end.y = arrow_start.y + cube_side_2[1];
		arrow_end.z = arrow_start.z + cube_side_2[2];
		cube_side_start[0] = arrow_end.x;
		cube_side_start[1] = arrow_end.y;
		cube_side_start[2] = arrow_end.z;
		cube_side_end[0] = arrow_start.x - cube_side_1[0];
		cube_side_end[1] = arrow_start.y - cube_side_1[1];
		cube_side_end[2] = arrow_start.z - cube_side_1[2];
	}
	Eigen::Vector3d optimal_direction(
		arrow_end.x - arrow_start.x,
		arrow_end.y - arrow_start.y,
		arrow_end.z - arrow_start.z
	);
	optimal_direction.normalize();
	arrow_end.x = arrow_start.x + optimal_direction[0];
	arrow_end.y = arrow_start.y + optimal_direction[1];
	arrow_end.z = arrow_start.z + optimal_direction[2];

	Eigen::Vector3d estimated_v_init = optimal_direction;
	ceres::Problem vel_problem;
	ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

	pcl::octree::OctreePointCloudPointVector<pcl::PointXYZI> octree(1.0);
	octree.setInputCloud(cur_detection.cloud_.makeShared());
	octree.addPointsFromInputCloud();
	std::vector<std::vector<int> > voxel_buffer;
	for (auto it = octree.leaf_begin(), it_end = octree.leaf_end(); it != it_end; ++it)
	{
		pcl::octree::OctreeContainerPointIndices & leaf_container = 
			it.getLeafContainer();
		std::vector<int> points_indices;
		points_indices = leaf_container.getPointIndicesVector();
		voxel_buffer.push_back(points_indices);
	}
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> voxel_colored_cloud;
	std::vector<pcl::PointCloud<pcl::PointXYZI> > voxel_cloud;
	for(int vox_idx = 0;vox_idx < voxel_buffer.size();++vox_idx)
	{
		int rand_r = (rand() % 155) + 100;
		int rand_g = (rand() % 155) + 100;
		int rand_b = (rand() % 155) + 100;
		pcl::PointCloud<pcl::PointXYZI> cloud_tmp;
		pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
		for (size_t pt_idx = 0; pt_idx < voxel_buffer[vox_idx].size(); pt_idx++)
		{
			pcl::PointXYZRGB pt_tmp;
			pcl::PointXYZI pti_tmp = cur_detection.cloud_.points[voxel_buffer[vox_idx][pt_idx]];
			pt_tmp.x = cur_detection.cloud_.points[voxel_buffer[vox_idx][pt_idx]].x;
			pt_tmp.y = cur_detection.cloud_.points[voxel_buffer[vox_idx][pt_idx]].y;
			pt_tmp.z = cur_detection.cloud_.points[voxel_buffer[vox_idx][pt_idx]].z;
			pt_tmp.r = rand_r;
			pt_tmp.g = rand_g;
			pt_tmp.b = rand_b;
			// pt_tmp.r = 255;
			// pt_tmp.g = 0;
			// pt_tmp.b = 0;
			colored_cloud.push_back(pt_tmp);
			cloud_tmp.push_back(pti_tmp);
		}
		voxel_colored_cloud.push_back(colored_cloud);
		voxel_cloud.push_back(cloud_tmp);
	}
	
	for(int vox_idx = 0;vox_idx < voxel_cloud.size();vox_idx++)
	{
		if (voxel_cloud[vox_idx].size() < 5)
		{
			continue;
		}

		std::string name = std::to_string(rand());
		viewer->addPointCloud<pcl::PointXYZRGB>(
			voxel_colored_cloud[vox_idx].makeShared(),
			name
		);
		viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);

		ceres::CostFunction* cost_function;

		cost_function = localOptimizer::Create(
			target_centroid,
			voxel_cloud[vox_idx],
			optimal_direction
		);

		vel_problem.AddResidualBlock(
			cost_function,
			loss_function,
			estimated_v_init.data()
		);
	}

	ceres::CostFunction* cost_function;
	cost_function = globalOptimizer::Create(
		target_centroid,
		cur_detection.cloud_,
		optimal_direction
	);

	vel_problem.AddResidualBlock(
		cost_function,
		loss_function,
		estimated_v_init.data()
	);

	ceres::Solver::Options options;
	options.max_num_iterations = 50;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_line_search_step_size_iterations = 10;
	options.logging_type = ceres::SILENT;

	ceres::Solver::Summary summary;
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	ceres::Solve(options, &vel_problem, &summary);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);


	Eigen::Matrix<double, 3, 3> cov_vel_optimized;

	ceres::Covariance::Options cov_options;
	cov_options.algorithm_type = ceres::DENSE_SVD;
	cov_options.null_space_rank = -1;
	cov_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
	ceres::Covariance covariance(cov_options);
	std::vector<std::pair<const double*, const double*> > covariance_blocks; 
	covariance_blocks.push_back(std::make_pair(estimated_v_init.data(), estimated_v_init.data())); 
	CHECK(covariance.Compute(covariance_blocks, &vel_problem));
	covariance.GetCovarianceBlock(
		estimated_v_init.data(),
		estimated_v_init.data(),
		cov_vel_optimized.data()
	);


	Eigen::Vector3d points_vel = estimated_v_init;
	Eigen::Matrix3d points_vel_cov = cov_vel_optimized;
	Eigen::Vector3d fusion_vel;
	Eigen::Matrix3d fusion_vel_cov;
	vel_fusion(
		cur_detection,
		prev_detection,
		points_vel,
		points_vel_cov,
		pix_vel,
		pix_vel_cov,
		fusion_vel,
		fusion_vel_cov,
		config_
	);

	fused_vel = fusion_vel;
	fused_vel_cov = fusion_vel_cov;
}

void fusion_tracker::vel_fusion(
	const alignedDet cur_detection,
	const alignedDet prev_detection,
	const Eigen::Vector3d points_vel,
	const Eigen::Matrix3d points_vel_cov,
	const Eigen::Vector2d pix_vel,
	const Eigen::Matrix2d pix_vel_cov,
	Eigen::Vector3d & fusion_vel,
	Eigen::Matrix3d & fusion_vel_cov,
	const Config & config_
)
{
	Eigen::Vector3d cur_centroid = cur_detection.vertex3d_.colwise().mean().cast<double>();
	Eigen::Vector3d prev_centroid = prev_detection.vertex3d_.colwise().mean().cast<double>();

	Eigen::Vector4d cur_centroid_global(
		cur_centroid[0],
		cur_centroid[1],
		cur_centroid[2],
		1.0
	);
	Eigen::Vector4d prev_centroid_global(
		prev_centroid[0],
		prev_centroid[1],
		prev_centroid[2],
		1.0
	);
	cur_centroid_global = cur_detection.global_pose_.inverse() * cur_centroid_global;
	prev_centroid_global = prev_detection.global_pose_.inverse() * prev_centroid_global;

	Eigen::Vector3d pix_vel_3d;
	double width_u, height_v;
	width_u = ((-prev_centroid[1]) * config_.camera_intrinsic_(0, 0)) / prev_centroid[0] + config_.camera_intrinsic_(0,2);
	height_v = ((-prev_centroid[2]) * config_.camera_intrinsic_(1, 1)) / prev_centroid[0] + config_.camera_intrinsic_(1,2);
	width_u += pix_vel[0];
	height_v += pix_vel[1];
	Eigen::Vector4d pix_moved;
	pix_moved[0] = cur_centroid[0];
	pix_moved[1] = -(width_u - config_.camera_intrinsic_(0, 2)) * pix_moved[0] / config_.camera_intrinsic_(0, 0);
	pix_moved[2] = -(height_v - config_.camera_intrinsic_(1, 2)) * pix_moved[0] / config_.camera_intrinsic_(1, 1);
	pix_moved[3] = 1.0;
	pix_moved = cur_detection.global_pose_.inverse() * pix_moved;
	pix_vel_3d = (pix_moved - prev_centroid_global).segment(0, 3) * 10.0;

	Eigen::Matrix3d uv2xyz_jacob, pix_cov_3d;
	pix_cov_3d.setIdentity();
	uv2xyz_jacob.setIdentity();
	uv2xyz_jacob(0, 0) = 1.0;
	uv2xyz_jacob(1, 1) = cur_centroid[0] * (1.0 / config_.camera_intrinsic_(0,0));
	uv2xyz_jacob(2, 2) = cur_centroid[0] * (1.0 / config_.camera_intrinsic_(1,1));
	pix_cov_3d(0,0) = 1.0;
	pix_cov_3d(1,1) = pix_vel_cov(0,0);
	pix_cov_3d(2,2) = pix_vel_cov(1,1);
	pix_cov_3d = 100.0 * cur_centroid[0] * cur_centroid[0] * uv2xyz_jacob * pix_cov_3d * uv2xyz_jacob;

	Eigen::Vector3d base_radial(
		cur_centroid[0],
		cur_centroid[1],
		cur_centroid[2]
	);
	base_radial.normalize();
	Eigen::Matrix3d base_radial_cov;
	base_radial_cov.setIdentity();

	Eigen::Vector3d vel_point_radial,vel_point_tangential;
	vel_point_radial = (points_vel.dot(base_radial) / base_radial.norm()) * base_radial;
	vel_point_tangential = points_vel - vel_point_radial;
	Eigen::Vector3d vel_pix_radial, vel_pix_tangential;
	vel_pix_radial = (pix_vel_3d.dot(base_radial) / base_radial.norm()) * base_radial;
	vel_pix_tangential = pix_vel_3d - vel_pix_radial;

	Eigen::Vector3d fused_vel_radial, fused_vel_tangential;
	fused_vel_radial = vel_point_radial;
	Eigen::Matrix3d gain_A;
	gain_A = points_vel_cov *(points_vel_cov + pix_cov_3d).inverse();
	fused_vel_tangential = vel_point_tangential + gain_A * (vel_pix_tangential - vel_point_tangential);

	fusion_vel = fused_vel_radial + fused_vel_tangential;
	fusion_vel_cov = points_vel_cov + gain_A * (pix_cov_3d + points_vel_cov) * gain_A.transpose();
}

void cloud_undistortion(
	const kfTracker & tracker,
    const alignedDet detection_in,
    const Eigen::Vector3d vel_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds_buffer,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_clouds_buffer,
	visualization_msgs::Marker & arrow_buffer,
	visualization_msgs::MarkerArray & txt_buffer
)
{
	if(std::isnan(vel_[0]) || std::isnan(vel_[1]) || std::isnan(vel_[2]))
	{
		return;
	}
	Eigen::Vector3d target_centroid = detection_in.vertex3d_.colwise().mean();
	int rand_r = tracker.rgb3[0];
	int rand_g = tracker.rgb3[1];
	int rand_b = tracker.rgb3[2];

	pcl::PointCloud<pcl::PointXYZRGB> cur_cloud_rgb;
	pcl::copyPointCloud(detection_in.cloud_, cur_cloud_rgb);
	for (size_t pt_idx = 0; pt_idx < cur_cloud_rgb.size(); pt_idx++)
	{
		cur_cloud_rgb.points[pt_idx].r = rand_r;
		cur_cloud_rgb.points[pt_idx].g = rand_g;
		cur_cloud_rgb.points[pt_idx].b = rand_b;
	}

	Eigen::Vector3d undistort_vel = vel_;
	pcl::PointCloud<pcl::PointXYZRGB> prev_cloud_rgb;
	pcl::PointCloud<pcl::PointXYZRGB> raw_cloud_rgb;
	for (size_t pt_idx = 0; pt_idx < detection_in.cloud_.size(); pt_idx++)
	{
		pcl::PointXYZRGB pt_temp;
		pcl::PointXYZRGB raw_pt_temp;
		raw_pt_temp.x = detection_in.cloud_.points[pt_idx].x;
		raw_pt_temp.y = detection_in.cloud_.points[pt_idx].y;
		raw_pt_temp.z = detection_in.cloud_.points[pt_idx].z;
		pt_temp.x = 
			detection_in.cloud_.points[pt_idx].x - undistort_vel[0] * detection_in.cloud_.points[pt_idx].intensity;
		pt_temp.y = 
			detection_in.cloud_.points[pt_idx].y - undistort_vel[1] * detection_in.cloud_.points[pt_idx].intensity;
		pt_temp.z = 
			detection_in.cloud_.points[pt_idx].z - undistort_vel[2] * detection_in.cloud_.points[pt_idx].intensity;
		pt_temp.r = rand_r;
		pt_temp.g = rand_g;
		pt_temp.b = rand_b;
		raw_pt_temp.r = rand_r;
		raw_pt_temp.g = rand_g;
		raw_pt_temp.b = rand_b;
		prev_cloud_rgb.push_back(pt_temp);
		raw_cloud_rgb.push_back(raw_pt_temp);
	}
	(*clouds_buffer) += prev_cloud_rgb;
	(*raw_clouds_buffer) += raw_cloud_rgb;

	pcl::PointXYZ arrow_start, optimal_vel_arrow;
	arrow_start.x = target_centroid[0];
	arrow_start.y = target_centroid[1];
	arrow_start.z = target_centroid[2];
	optimal_vel_arrow.x = arrow_start.x + undistort_vel[0];
	optimal_vel_arrow.y = arrow_start.y + undistort_vel[1];
	optimal_vel_arrow.z = arrow_start.z + undistort_vel[2];

	geometry_msgs::Point point1, point2;
	point1.x = target_centroid[0];
	point1.y = target_centroid[1];
	point1.z = target_centroid[2];
	undistort_vel = undistort_vel.normalized() * 3.0;
	point2.x = target_centroid[0] + undistort_vel[0];
	point2.y = target_centroid[1] + undistort_vel[1];
	point2.z = target_centroid[2] + undistort_vel[2];

	visualization_msgs::Marker obj_vel_txt;
	obj_vel_txt.header.frame_id = "livox";
	obj_vel_txt.ns = "obj_vel_txt";
	obj_vel_txt.id = rand() % 1000;
	obj_vel_txt.lifetime = ros::Duration(0);
	obj_vel_txt.action = visualization_msgs::Marker::ADD;
	obj_vel_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	obj_vel_txt.scale.z = 1.0;
	obj_vel_txt.color.a = 1.0;
	obj_vel_txt.color.r = 1.0;
	obj_vel_txt.color.g = 1.0;
	obj_vel_txt.color.b = 1.0;
	geometry_msgs::Pose txt_pose;
	txt_pose.position.x = target_centroid[0];
	txt_pose.position.y = target_centroid[1];
	txt_pose.position.z = target_centroid[2] + 3.0;
	std::ostringstream vel_txt;
	vel_txt << vel_.norm() * 3.6;
	obj_vel_txt.text = vel_txt.str().substr(0,2) + "km/h";
	obj_vel_txt.pose = txt_pose;
	txt_buffer.markers.push_back(obj_vel_txt);
}