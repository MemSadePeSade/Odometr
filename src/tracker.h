#pragma once
#include<vector>
#include<numeric>

#include"opencv2/opencv.hpp"

#include"draw.h"

namespace cameraparam {
	struct CameraParam {
		CameraParam() : pp(358.9874749825216f, 201.7120939366421f), intrisic_mat(1, 0, 0, 0, 1, 0, 0, 0, 1),
			dist_coeff(0, 0, 0, 0) {}
		cv::Matx<double, 1, 4> dist_coeff;
		//cv::Matx<double, 1, 5> dist_coeff;
		cv::Matx33d intrisic_mat;
		double focal_length = 681.609f;
		cv::Point2d pp;
	};

	void MakeIntrisicMatFromVector(CameraParam& camera_param,
		const std::vector<double>& parametrs) {
		camera_param.intrisic_mat(0, 0) = parametrs[0]; //fx
		camera_param.intrisic_mat(0, 2) = parametrs[2]; //cx
		camera_param.intrisic_mat(1, 1) = parametrs[1]; //fy
		camera_param.intrisic_mat(1, 2) = parametrs[3]; //cy
		camera_param.focal_length = 681.609f;//cv::norm(cv::Point2d(parametrs[0], parametrs[1]));
		camera_param.pp.x = parametrs[2];
		camera_param.pp.y = parametrs[3];
	}

	int LoadCameraParam(const std::string& filename, CameraParam& camera_param) {
		cv::FileStorage fs;
		fs.open(filename, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			std::cerr << "Failed to open " << filename << std::endl;
			return 1;
		}

		cv::FileNode n = fs["cam0"];
		cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
		for (; it != it_end; ++it)
		{
			std::cout << (*it).name() << std::endl;
			if ((*it).name() == "distortion_coeffs") {
				std::vector<double> data;
				(*it) >> data;
				camera_param.dist_coeff = cv::Mat(1, 4, CV_64F, data.data());
				for (auto& elem : data)
					std::cout << elem << std::endl;
			}
			if ((*it).name() == "intrinsics") {
				std::vector<double> data;
				(*it) >> data;
				MakeIntrisicMatFromVector(camera_param, data);
				for (auto& elem : data)
					std::cout << elem << std::endl;
			}
			if ((*it).name() == "resolution") {
				std::vector<int> data;
				(*it) >> data;
				for (auto& elem : data)
					std::cout << elem << std::endl;
			}
		}
		fs.release();
		return 0;
	}
	int LoadOpenCVCameraParam(const std::string& filename, CameraParam& camera_param){
		cv::FileStorage fs;
		fs.open(filename, cv::FileStorage::READ);
		std::cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
		std::cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
		cv::Mat intrisic;
		cv::Mat distcoeff;
		fs["camera_matrix"] >> intrisic;
		fs["distortion_coefficients"] >> distcoeff;
		camera_param.intrisic_mat = intrisic;
		camera_param.dist_coeff = distcoeff;
		
		camera_param.focal_length = camera_param.intrisic_mat(0, 0);
		camera_param.pp = cv::Point2d(camera_param.intrisic_mat(0, 2), camera_param.intrisic_mat(1, 2));
		std::cout << "\nintrinsic matrix:" << camera_param.intrisic_mat;
		std::cout << "\ndistortion coefficients: " << camera_param.dist_coeff << std::endl;
		return 0;
	}
}

namespace {
	// Checks if a matrix is a valid rotation matrix.
	bool isRotationMatrix(const cv::Matx33d& R) {
		cv::Matx33d Rt;
		cv::transpose(R, Rt);
		cv::Matx33d shouldBeIdentity = Rt * R;
		cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
		return  cv::norm(I, shouldBeIdentity) < 1e-6;
	}

	// Calculates rotation matrix to euler angles
	// The result is the same as MATLAB except the order
	// of the euler angles ( x and z are swapped ).
	cv::Vec3d rotationMatrixToEulerAngles(const cv::Matx33d &R) {
		assert(isRotationMatrix(R));
		double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
		bool singular = sy < 1e-6; // If
		double x, y, z;
		if (!singular) {
			x = atan2(R(2, 1), R(2, 2));
			y = atan2(-R(2, 0), sy);
			z = atan2(R(1, 0), R(0, 0));
		}
		else {
			x = atan2(-R(1, 2), R(1, 1));
			y = atan2(-R(2, 0), sy);
			z = 0;
		}
		return cv::Vec3d(x, y, z);
	}
}

namespace preprocess {
	struct PreProcess {
		PreProcess(const cameraparam::CameraParam& camera_param_init) {
			camera_param = camera_param_init;
		}
		cv::Mat operator() (const cv::Mat& src) const {
			cv::Mat src_gray;
			cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
			cv::Mat dst;
			cv::undistort(src_gray, dst, camera_param.intrisic_mat, camera_param.dist_coeff);
			return dst;
		}
		cameraparam::CameraParam camera_param;
	};
}

namespace detector {
	struct GFTTParams {
		const int maxCorners = 300;
		const double qualityLevel = 0.01;
		const double minDistance = 20.0;
		const int blockSize = 3;
		const bool useHarrisDetector = false;
		const double k = 0.04;
	};
	struct GFTTDetector : GFTTParams {
		std::vector<cv::Point2f> operator()(const cv::Mat& img) {
			std::vector<cv::Point2f> points;
			cv::Mat mask;
			cv::goodFeaturesToTrack(img, points, maxCorners, qualityLevel, minDistance,
				mask, blockSize, useHarrisDetector, k);
			return points;
		}
	};
}

namespace tracker {
	struct TrackerParams {
		cv::Size winSize = cv::Size(21, 21);
		int iters = 30;
		int maxLevel = 3;
		cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
	};

	class Tracker : TrackerParams {
	public:
		Tracker() {};
		std::vector<cv::Point2f> ToTrackandCorrectIndex(const cv::Mat& img) {// Track points and  delete points
			std::vector<cv::Point2f> points;
			std::vector<uchar> status;
			std::vector<float> err;
			calcOpticalFlowPyrLK(m_state.img, img, m_state.points, points,
				status, err, winSize, 3, termcrit, 0, 0.001);
			int indexCorrection = 0;
			for (size_t i = 0; i < status.size(); i++) {
				cv::Point2d pt = points.at(i - indexCorrection);
				if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
					m_state.points.erase(m_state.points.begin() + (i - indexCorrection));
					points.erase(points.begin() + (i - indexCorrection));
					indexCorrection++;
				}
			}
			return points;
		}
		int  NumPoints() { return m_state.points.size(); };
		const std::vector<cv::Point2f> GetPoints() { return m_state.points; };
		void UpdateStateTrackerImage(const cv::Mat& img) { m_state.img = img.clone(); };
		void UpdateStateTrackerPoints(const std::vector<cv::Point2f>& points) { m_state.points = points; };
	private:
		struct StateTracker {
			cv::Mat img;
			std::vector<cv::Point2f> points;
		};
		StateTracker m_state;
	};
}

namespace odometr {
    #define MIN_NUM_FEAT 2000
    #define KeyFrThresh  0.0
	enum class FeatureType {
		BRISK, ORB, MSER, FAST,
		AGAST, GFTT, KAZE, SURF,
		SIFT
	};
	struct OdometryData {
		OdometryData() : t(0, 0, 0), R(1, 0, 0, 0, 1, 0, 0, 0, 1) {};
		cv::Matx31d t;
		cv::Matx33d R;
	};
	struct OdometryDataAll {
		OdometryDataAll() : t(0, 0, 0), R1(1, 0, 0, 0, 1, 0, 0, 0, 1),R2(1, 0, 0, 0, 1, 0, 0, 0, 1) {};
		cv::Matx31d t;
		cv::Matx33d R1;
		cv::Vec3d euler1;
		cv::Matx33d R2;
		cv::Vec3d euler2;
	};
	
	class Odometr {
	public:	
		Odometr(cameraparam::CameraParam& camera_param, const cv::Mat& img) :
		m_preprocessor(camera_param), m_pt_kfr(0,0){
			const auto&& dst = m_preprocessor(img);
			const auto&& pts_dst = m_detector(dst);
			m_tracker.UpdateStateTrackerImage(dst);
			m_tracker.UpdateStateTrackerPoints(pts_dst);
			m_pt_kfr = std::accumulate(pts_dst.begin(), pts_dst.end(),
				                       cv::Point2f(0, 0), std::plus<cv::Point2f>());
			m_pt_kfr *= (1.0 / pts_dst.size());
		}
		void CalculateRandT(const cv::Mat& E, cv::Matx33d& R, cv::Matx31d& t,
			const std::vector<cv::Point2f>& currFeatures,
			const std::vector<cv::Point2f>& prevFeatures) {
			cv::Mat R1, R2, T;
			decomposeEssentialMat(E, R1, R2, T);
			cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
			cv::Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
			P1(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; P1.col(3) = T * 1.0;
			P2(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; P2.col(3) = T * 1.0;
			P3(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; P3.col(3) = -T * 1.0;
			P4(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; P4.col(3) = -T * 1.0;

			std::vector<cv::Mat> allTriangulations(4);
			cv::Mat Q1, Q2, Q3, Q4;
		
			triangulatePoints(P0, P1, currFeatures, prevFeatures, Q1);
			Q1.row(0) /= Q1.row(3);
			Q1.row(1) /= Q1.row(3);
			Q1.row(2) /= Q1.row(3);
			Q1.row(3) /= Q1.row(3);

			triangulatePoints(P0, P2, currFeatures, prevFeatures, Q2);
			Q2.row(0) /= Q2.row(3);
			Q2.row(1) /= Q2.row(3);
			Q2.row(2) /= Q2.row(3);
			Q2.row(3) /= Q2.row(3);

			triangulatePoints(P0, P3, currFeatures, prevFeatures, Q3);
			Q3.row(0) /= Q3.row(3);
			Q3.row(1) /= Q3.row(3);
			Q3.row(2) /= Q3.row(3);
			Q3.row(3) /= Q3.row(3);

			triangulatePoints(P0, P4, currFeatures, prevFeatures, Q4);
			Q4.row(0) /= Q4.row(3);
			Q4.row(1) /= Q4.row(3);
			Q4.row(2) /= Q4.row(3);
			Q4.row(3) /= Q4.row(3);
			int count1 = 0, count2 = 0, count3 = 0, count4 = 0;
			int rows = Q1.rows;
			int cols = Q1.cols;
			cv::Size s = Q1.size();
			rows = s.height;
			cols = s.width;

			auto euler_angles1 = rotationMatrixToEulerAngles(R1);
			auto euler_angles2 = rotationMatrixToEulerAngles(R2);

			auto dist1 = abs(euler_angles1(0)) + abs(euler_angles1(1)) + abs(euler_angles1(2));
			auto dist2 = abs(euler_angles2(0)) + abs(euler_angles2(1)) + abs(euler_angles2(2));
			bool flag = true;
			if (dist1 > dist2)
				flag = false;
			
			for (int i = 0; i < cols; ++i)
			{
				if ((Q1.at<double>(2, i) >= Q2.at<double>(2, i)) &&
					(Q1.at<double>(2, i) >= Q3.at<double>(2, i)) &&
					(Q1.at<double>(2, i) >= Q4.at<double>(2, i)) && flag)
				{
					++count1;
				}
				if ((Q2.at<double>(2, i) >= Q1.at<double>(2, i)) &&
					(Q2.at<double>(2, i) >= Q3.at<double>(2, i)) &&
					(Q2.at<double>(2, i) >= Q4.at<double>(2, i)) && !flag)
				{
					++count2;
				}
				if ((Q3.at<double>(2, i) >= Q1.at<double>(2, i)) &&
					(Q3.at<double>(2, i) >= Q4.at<double>(2, i)) &&
					(Q3.at<double>(2, i) >= Q2.at<double>(2, i)) && flag)
				{
					++count3;
				}
				if ((Q4.at<double>(2, i) >= Q1.at<double>(2, i)) &&
					(Q4.at<double>(2, i) >= Q3.at<double>(2, i)) &&
					(Q4.at<double>(2, i) >= Q2.at<double>(2, i)) && !flag)
				{
					++count4;
				}
			}

			if ((count1 >= count2) &&
				(count1 >= count3) &&
				(count1 >= count4))
			{
				R = R1;
				t = T;
			}
			else if ((count2 >= count1) &&
				(count2 >= count3) &&
				(count2 >= count4))
			{
				R = R2;
				t = T;
			}
			else if ((count3 >= count2) &&
				(count3 >= count1)
				&& (count3 >= count4))
			{
				R = R1;
				t = T; t = -t;
			}
			else if ((count4 >= count1) &&
				(count4 >= count2) &&
				(count4 >= count3))
			{
				R = R2;
				t = T; t = -t;
			}
		}
		void RecoverPose(const std::vector<cv::Point2f>& pts_curr,
			             const std::vector<cv::Point2f>& pts_prev){
			const auto& focal_length = m_preprocessor.camera_param.focal_length;
			const auto& pp = m_preprocessor.camera_param.pp;
			auto& t = m_odometry_data.t;
			auto& R = m_odometry_data.R;
			cv::Mat E, mask;
			E = cv::findEssentialMat(pts_curr, pts_prev, focal_length,
				m_preprocessor.camera_param.pp, cv::RANSAC, 0.999, 1.0, mask);
			//cv::recoverPose(E, pts_curr, pts_prev, R, t, focal_length, pp, mask);
			// init R,t
			if (m_counter == 1)
				cv::recoverPose(E, pts_curr, pts_prev, R, t, focal_length, pp, mask);
			else { //calculate  R,t
				CalculateRandT(E, R, t, pts_curr, pts_prev);
				/*
				cv::Matx33d R1, R2;
				cv::Matx31d T;
				cv::decomposeEssentialMat(E, R1, R2, T);
				m_odometry_all.R1 = R1; m_odometry_all.R2 = R2; m_odometry_all.t = T;
				auto euler_angles1 = rotationMatrixToEulerAngles(R1);
				auto euler_angles2 = rotationMatrixToEulerAngles(R2);
				m_odometry_all.euler1 = euler_angles1;
				m_odometry_all.euler2 = euler_angles2;
				
				auto dist1 = abs(euler_angles1(0)) + abs(euler_angles1(1)) + abs(euler_angles1(2));
				auto dist2 = abs(euler_angles2(0)) + abs(euler_angles2(1)) + abs(euler_angles2(2));
				if (dist1 < dist2)
					cv::recoverPose(E, pts_curr, pts_prev, R, t, focal_length, pp, mask);
				else { R = R2; t = T; }*/
			}
		}
		
		void Do(const cv::Mat& img) {
			m_bad_flag = false;
			const auto&& img_curr = m_preprocessor(img);
			// matcher
			auto&& pts_curr = m_tracker.ToTrackandCorrectIndex(img_curr);
			const auto&& pts_prev = m_tracker.GetPoints();
			/// add keyframe 
			cv::Point2f pt_kfr_curr = std::accumulate(pts_curr.begin(), pts_curr.end(),
				cv::Point2f(0, 0), std::plus<cv::Point2f>());
			pt_kfr_curr *= (1.0 / pts_curr.size());
			double norm_movement = cv::norm(m_pt_kfr - pt_kfr_curr);
			if (norm_movement < KeyFrThresh) {
				//std::cout << "work" << std::endl;
				m_bad_flag = true;
				return;
			}
			draw::DrawOpticalFlow(pts_prev, pts_curr,img);
			RecoverPose(pts_curr, pts_prev);
			UpdateGlobalOdometryData();
			// Update state  tracker
			m_tracker.UpdateStateTrackerImage(img_curr);
			if (m_tracker.NumPoints() < MIN_NUM_FEAT)
				pts_curr = m_detector(img_curr);
			m_tracker.UpdateStateTrackerPoints(pts_curr);
			
			m_pt_kfr = pt_kfr_curr;
			++m_counter;
		}
		OdometryData GetOdometryData() { return m_odometry_data; }
		OdometryData GetGlobalOdometryData() { return m_odometry_global_data; }
		OdometryDataAll GetOdometryDataAll() { return m_odometry_all; }
		bool GetFlag() { return m_bad_flag; }
		void UpdateGlobalOdometryData(){
			const auto& t = m_odometry_data.t;
			const auto& R = m_odometry_data.R;
			auto& t_f = m_odometry_global_data.t;
			auto& R_f = m_odometry_global_data.R;
			if (m_counter == 1) {
				t_f = t;
				R_f = R;
				return;
			}
			/*else {
				double scale = 4;//0.35;
				t_f = t_f + scale * (R_f*t);
				R_f = R * R_f;
			}*/
			double scale = 4;//0.35;
			/*if (  (scale > 0.1) && (t(2) > t(0)) && (t(2) > t(1))  ) {
				t_f = t_f + scale * (R_f*t);
				R_f = R * R_f;
			}*/
			t_f = t_f + scale * (R_f*t);
			R_f = R * R_f;
		}
	private:
		int m_counter = 1;
		cv::Point2f m_pt_kfr;
		bool m_bad_flag = false;
		OdometryData m_odometry_data;
		OdometryData m_odometry_global_data;
		OdometryDataAll m_odometry_all;
		preprocess::PreProcess m_preprocessor;
		detector::GFTTDetector m_detector;
		tracker::Tracker m_tracker;
	};
}
