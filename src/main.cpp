#include<string>

#include <boost/filesystem/operations.hpp>
#include "opencv2/core/utility.hpp"

#include "tracker.h"
#include "draw.h"

namespace {
#define MAX_FRAME 384
#define MIN_NUM_FEAT 2000
//const char *img_path = "C:\\Users\\vponomarev\\Desktop\\01\\img_datasets\\e1i90v1a30_undistorted\\frame%06d.jpg";
const char *img_path = "C:\\Users\\vponomarev\\Desktop\\01\\img_datasets\\e1i90v1a30\\frame%06d.jpg";
const char* keys =
		"{help h usage ?  |    | print help message }"
		"{@calib   |        | specify calib file }";
} //unnamed namespace

int main(int argc, const char* argv[]) {
	cv::CommandLineParser cmd(argc, argv, keys);
	if (cmd.has("help") || !cmd.check()){
		cmd.printMessage();
		cmd.printErrors();
		return 0;
	}
	std::string calib_filename = cmd.get<std::string>("@calib"); calib_filename = "z.yaml";
	cameraparam::CameraParam camera_param;
	if (!boost::filesystem::exists(calib_filename))
		std::cout << "No calibrated file" << std::endl;
	else
		LoadCameraParam(calib_filename, camera_param);

	char filename[200];
	int  counter = 0;
	sprintf(filename, img_path, counter);

	cv::Mat img = cv::imread(filename);
	
	odometr::Odometr odometr(camera_param,img);
	
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
	cv::Matx31d t_f(0, 0, 0);
	cv::Matx33d R_f(1, 0, 0, 0, 1, 0, 0, 0, 1);

	std::clock_t start;
	double duration;
	start = std::clock();
	// cycle through the directory
	for (counter = 1; counter < 384; ++counter) {
		sprintf(filename, img_path, counter);
		img = cv::imread(filename);
		odometr.Do(img);
		odometr::OdometryData  odometry_data = odometr.GetOdometryData();
		
		cv::Matx31d& t = odometry_data.t;
		cv::Matx33d& R = odometry_data.R;
		if (counter == 1) {
			t_f = t;
			R_f = R;
			continue;
		}
		double scale = 4.0;
		if ((scale > 0.1) && (t(2) > t(0)) && (t(2) > t(1))) {
			t_f = t_f + scale * (R_f*t);
			R_f = R * R_f;
		}

		DrawTrajectory(t_f, traj);
		if (cv::waitKey(1) == 27)
			break;
	}
	cv::destroyAllWindows();
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "printf: " << duration << '\n';
}
