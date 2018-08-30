#include<string>

#include <boost/filesystem/operations.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/viz/vizcore.hpp>

#include "tracker.h"
#include "draw.h"
#include "reader.h"

namespace {
#define MAX_FRAME 384
#define MIN_NUM_FEAT 2000
//const char *img_path = "C:\\Users\\vponomarev\\Desktop\\01\\img_datasets\\e1i90v1a30_undistorted\\frame%06d.jpg";
const char *img_path = "C:\\Users\\vponomarev\\Desktop\\01\\img_datasets\\e1i90v1a30\\frame%06d.jpg";
//const char *img_path = "/home/ubuntu/Ponomarev/01/img_datasets/e1i90v1a30/frame%06d.jpg";
const char* keys =
		"{help h usage ?  |    | print help message }"
		"{@calib   |        | specify calib file }";
} //unnamed namespace

using namespace boost::filesystem; 
int main(int argc, const char* argv[]) {
	cv::CommandLineParser cmd(argc, argv, keys);
	if (cmd.has("help") || !cmd.check()){
		cmd.printMessage();
		cmd.printErrors();
		return 0;
	}
	std::string calib_filename = cmd.get<std::string>("@calib"); 
	calib_filename = "z.yaml";
	//calib_filename = "/home/ubuntu/Ponomarev/Odometr/z.yaml";
	cameraparam::CameraParam camera_param;
	if (!boost::filesystem::exists(calib_filename))
		std::cout << "No calibrated file" << std::endl;
	else
		LoadCameraParam(calib_filename, camera_param);
	ReaderFromPrintf reader(img_path);
	
	cv::Mat img;
	reader.Read(img);
	odometr::Odometr odometr(camera_param,img);
	
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
	cv::Matx31d t_f(0, 0, 0);
	cv::Matx33d R_f(1, 0, 0, 0, 1, 0, 0, 0, 1);

	std::clock_t start;
	double duration;
	start = std::clock();
	
	/// Create a window
	cv::viz::Viz3d mywindow("Viz Demo");
	mywindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
	
	// cycle through the directory
	std::vector<cv::Affine3f> path;
	while(reader.Read(img)) {
		odometr.Do(img);
		odometr::OdometryData  odometry_data = odometr.GetGlobalOdometryData();
		cv::Matx31d& t = odometry_data.t;
		cv::Matx33d& R = odometry_data.R;
		
		if (odometr.GetFlag())
			continue;
		
		DrawTrajectory(t, traj);
		if (cv::waitKey(1) == 27)
			break;
		cv::Vec3d tt(t(0), t(1), t(2));
		cv::Affine3d cam_pose = cv::Affine3d(R_f, tt);;
		path.push_back(cam_pose);
		cv::viz::WTrajectory  path_w(path,3);
		
		mywindow.showWidget("PATH", path_w);
		if (mywindow.wasStopped())
			break;
		mywindow.spinOnce(1, true);
	}
	cv::destroyAllWindows();
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "printf: " << duration << '\n';
}
