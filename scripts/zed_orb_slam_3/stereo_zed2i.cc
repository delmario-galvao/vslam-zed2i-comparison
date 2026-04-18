/**
 * stereo_zed2i.cc
 * ORB-SLAM3 + ZED 2i (Stereo only, no IMU)
 */

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sl/Camera.hpp>
#include <System.h>

using namespace std;

bool b_continue_session = true;

void exit_loop_handler(int)
{
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

static std::string MakeTimestampString()
{
    std::time_t now = std::time(nullptr);
    std::tm tm_now = *std::localtime(&now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

static bool EnsureDir(const std::string& dir_path)
{
    try {
        std::filesystem::create_directories(dir_path);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create directory: " << dir_path
                  << " | " << e.what() << std::endl;
        return false;
    }
}

static std::string FormatFrameId(uint64_t id)
{
    std::ostringstream oss;
    oss << std::setw(10) << std::setfill('0') << id;
    return oss.str();
}

static bool SaveStereoFrame(const std::string& left_dir,
                            const std::string& right_dir,
                            std::ofstream& ts_file,
                            uint64_t frame_id,
                            double timestamp,
                            const cv::Mat& imLeft,
                            const cv::Mat& imRight)
{
    const std::string frame_name = FormatFrameId(frame_id) + ".png";
    const std::string left_path = left_dir + "/" + frame_name;
    const std::string right_path = right_dir + "/" + frame_name;

    if (!cv::imwrite(left_path, imLeft)) {
        std::cerr << "Failed to save left image: " << left_path << std::endl;
        return false;
    }

    if (!cv::imwrite(right_path, imRight)) {
        std::cerr << "Failed to save right image: " << right_path << std::endl;
        return false;
    }

    ts_file << std::fixed << std::setprecision(9)
            << timestamp << " " << frame_name << "\n";

    return true;
}

static void WriteRunInfo(const std::string& info_path,
                         const sl::CameraInformation& cam_info,
                         const sl::CalibrationParameters& calib)
{
    std::ofstream f(info_path);
    if (!f.is_open()) {
        std::cerr << "Failed to write run info: " << info_path << std::endl;
        return;
    }

    f << "camera_model=ZED2i\n";
    f << "resolution_width=" << cam_info.camera_configuration.resolution.width << "\n";
    f << "resolution_height=" << cam_info.camera_configuration.resolution.height << "\n";
    f << "fps=" << cam_info.camera_configuration.fps << "\n";
    f << std::fixed << std::setprecision(6);
    f << "fx=" << calib.left_cam.fx << "\n";
    f << "fy=" << calib.left_cam.fy << "\n";
    f << "cx=" << calib.left_cam.cx << "\n";
    f << "cy=" << calib.left_cam.cy << "\n";
    f << "baseline_m=" << calib.getCameraBaseline() << "\n";
    f.close();
}

static bool WriteRuntimeZedYaml(const std::string& yaml_path,
                                const sl::CameraInformation& cam_info,
                                const sl::CalibrationParameters& calib,
                                const std::string& atlas_base_path)
{
    std::ofstream f(yaml_path);
    if (!f.is_open()) {
        std::cerr << "Failed to write YAML file: " << yaml_path << std::endl;
        return false;
    }

    const auto& left = calib.left_cam;
    const double baseline_m = calib.getCameraBaseline();
    const int width = cam_info.camera_configuration.resolution.width;
    const int height = cam_info.camera_configuration.resolution.height;
    const int fps = cam_info.camera_configuration.fps;

    f << "%YAML:1.0\n\n";
    f << "File.version: \"1.0\"\n";
    f << "Camera.type: \"Rectified\"\n\n";

    f << std::fixed << std::setprecision(6);
    f << "Camera1.fx: " << left.fx << "\n";
    f << "Camera1.fy: " << left.fy << "\n";
    f << "Camera1.cx: " << left.cx << "\n";
    f << "Camera1.cy: " << left.cy << "\n\n";

    f << "Camera2.fx: " << left.fx << "\n";
    f << "Camera2.fy: " << left.fy << "\n";
    f << "Camera2.cx: " << left.cx << "\n";
    f << "Camera2.cy: " << left.cy << "\n\n";

    f << "Stereo.b: " << baseline_m << "\n\n";

    f << "Camera.width: " << width << "\n";
    f << "Camera.height: " << height << "\n";
    f << "Camera.fps: " << fps << "\n";
    f << "Camera.RGB: 1\n\n";

    f << "Stereo.ThDepth: 40.0\n\n";

    f << "ORBextractor.nFeatures: 1250\n";
    f << "ORBextractor.scaleFactor: 1.2\n";
    f << "ORBextractor.nLevels: 8\n";
    f << "ORBextractor.iniThFAST: 20\n";
    f << "ORBextractor.minThFAST: 7\n\n";

    f << "Viewer.KeyFrameSize: 0.05\n";
    f << "Viewer.KeyFrameLineWidth: 1.0\n";
    f << "Viewer.GraphLineWidth: 0.9\n";
    f << "Viewer.PointSize: 2.0\n";
    f << "Viewer.CameraSize: 0.08\n";
    f << "Viewer.CameraLineWidth: 3.0\n";
    f << "Viewer.ViewpointX: 0.0\n";
    f << "Viewer.ViewpointY: -0.7\n";
    f << "Viewer.ViewpointZ: -3.5\n";
    f << "Viewer.ViewpointF: 500.0\n";
    f << "Viewer.imageViewScale: 1.0\n";

    f << "System.SaveAtlasToFile: \"" << atlas_base_path << "\"\n";

    f.close();
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./stereo_zed2i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    if (argc == 4) {
        file_name = string(argv[3]);
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 60;
    init_params.depth_mode = sl::DEPTH_MODE::NONE;
    init_params.coordinate_units = sl::UNIT::METER;

    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        cerr << "ZED open failed: " << sl::toString(err) << endl;
        return 1;
    }

    auto cam_info = zed.getCameraInformation();
    auto calib = cam_info.camera_configuration.calibration_parameters;

    std::string timestamp_str = MakeTimestampString();
    std::string run_dir = "runs/" + timestamp_str;
    std::string left_dir = run_dir + "/left";
    std::string right_dir = run_dir + "/right";

    if (!EnsureDir(left_dir) || !EnsureDir(right_dir)) {
        zed.close();
        return 1;
    }

    std::string runtime_yaml = run_dir + "/ZED2i_runtime.yaml";
    std::string traj_file = run_dir + "/trajectory_euroc.txt";
    std::string keyframes_file = run_dir + "/keyframes_euroc.txt";
    std::string run_info_file = run_dir + "/run_info.txt";
    std::string timestamps_file = run_dir + "/timestamps.txt";
    std::string atlas_base = run_dir + "/atlas";

    if (!WriteRuntimeZedYaml(runtime_yaml, cam_info, calib, atlas_base)) {
        zed.close();
        return 1;
    }

    WriteRunInfo(run_info_file, cam_info, calib);

    std::ofstream ts_file(timestamps_file);
    if (!ts_file.is_open()) {
        std::cerr << "Failed to open timestamps file: " << timestamps_file << std::endl;
        zed.close();
        return 1;
    }

    std::cout << "Run directory: " << run_dir << std::endl;
    std::cout << "Runtime YAML written to: " << runtime_yaml << std::endl;

    cout << "Left camera (rectified from SDK):\n";
    cout << " fx = " << calib.left_cam.fx << endl;
    cout << " fy = " << calib.left_cam.fy << endl;
    cout << " cx = " << calib.left_cam.cx << endl;
    cout << " cy = " << calib.left_cam.cy << endl;
    cout << " width = " << cam_info.camera_configuration.resolution.width << endl;
    cout << " height = " << cam_info.camera_configuration.resolution.height << endl;
    cout << "Baseline [m] = " << calib.getCameraBaseline() << endl;

    ORB_SLAM3::System SLAM(
        argv[1], runtime_yaml, ORB_SLAM3::System::STEREO, false, 0, file_name);

    float imageScale = SLAM.GetImageScale();

    sl::RuntimeParameters runtime_params;
    runtime_params.enable_depth = false;

    sl::Mat left_zed, right_zed;
    cv::Mat imLeft, imRight;

    uint64_t frame_id = 0;

    while (b_continue_session && !SLAM.isShutDown())
    {
        if (zed.grab(runtime_params) != sl::ERROR_CODE::SUCCESS) {
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }

        zed.retrieveImage(left_zed, sl::VIEW::LEFT);
        zed.retrieveImage(right_zed, sl::VIEW::RIGHT);

        cv::Mat left_bgra(
            left_zed.getHeight(),
            left_zed.getWidth(),
            CV_8UC4,
            left_zed.getPtr<sl::uchar1>(sl::MEM::CPU));

        cv::Mat right_bgra(
            right_zed.getHeight(),
            right_zed.getWidth(),
            CV_8UC4,
            right_zed.getPtr<sl::uchar1>(sl::MEM::CPU));

        cv::cvtColor(left_bgra, imLeft, cv::COLOR_BGRA2GRAY);
        cv::cvtColor(right_bgra, imRight, cv::COLOR_BGRA2GRAY);

        double timestamp =
            zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds() * 1e-9;

        if (imageScale != 1.f)
        {
            int width = static_cast<int>(imLeft.cols * imageScale);
            int height = static_cast<int>(imLeft.rows * imageScale);
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        if (!SaveStereoFrame(left_dir, right_dir, ts_file, frame_id, timestamp, imLeft, imRight)) {
            std::cerr << "Failed to save stereo frame " << frame_id << std::endl;
        }
        frame_id++;

        SLAM.TrackStereo(imLeft, imRight, timestamp);
    }

    cout << "Shutting down system..." << endl;
    
    SLAM.SaveTrajectoryEuRoC(traj_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(keyframes_file);
    SLAM.Shutdown();

    ts_file.close();
    zed.close();

    return 0;
}