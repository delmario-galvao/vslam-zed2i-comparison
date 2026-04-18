/**
 * stereo_dataset_zed.cc
 * ORB-SLAM3 offline runner for recorded ZED stereo runs
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "System.h"

using namespace std;

struct FrameEntry {
    double timestamp;
    std::string filename;
};

static bool LoadTimestamps(const std::string& path, std::vector<FrameEntry>& entries)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Failed to open timestamps file: " << path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        FrameEntry e;
        if (!(iss >> e.timestamp >> e.filename)) {
            std::cerr << "Malformed line in timestamps file: " << line << std::endl;
            return false;
        }
        entries.push_back(e);
    }

    return !entries.empty();
}

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 5) {
        cerr << endl
             << "Usage: ./stereo_dataset_zed path_to_vocabulary path_to_settings path_to_run_dir (trajectory_file_name)"
             << endl;
        return 1;
    }

    const std::string voc_file = argv[1];
    const std::string settings_file = argv[2];
    const std::string run_dir = argv[3];

    std::string sequence_name;
    if (argc == 5) {
        sequence_name = argv[4];
    }

    const std::string left_dir = run_dir + "/left";
    const std::string right_dir = run_dir + "/right";
    const std::string timestamps_file = run_dir + "/timestamps.txt";

    std::vector<FrameEntry> entries;
    if (!LoadTimestamps(timestamps_file, entries)) {
        std::cerr << "Could not load timestamps." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << entries.size() << " frames from " << run_dir << std::endl;

    ORB_SLAM3::System SLAM(
        voc_file,
        settings_file,
        ORB_SLAM3::System::STEREO,
        true,
        0,
        sequence_name
    );

    float imageScale = SLAM.GetImageScale();

    std::vector<float> track_times;
    track_times.reserve(entries.size());

    for (size_t i = 0; i < entries.size(); ++i)
    {
        const std::string left_path = left_dir + "/" + entries[i].filename;
        const std::string right_path = right_dir + "/" + entries[i].filename;

        cv::Mat imLeft = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
        cv::Mat imRight = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

        if (imLeft.empty()) {
            std::cerr << "Failed to load left image: " << left_path << std::endl;
            return 1;
        }

        if (imRight.empty()) {
            std::cerr << "Failed to load right image: " << right_path << std::endl;
            return 1;
        }

        if (imageScale != 1.f)
        {
            int width = static_cast<int>(imLeft.cols * imageScale);
            int height = static_cast<int>(imLeft.rows * imageScale);
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        auto t1 = std::chrono::steady_clock::now();

        SLAM.TrackStereo(imLeft, imRight, entries[i].timestamp);

        auto t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        track_times.push_back(static_cast<float>(ttrack));

        if (i + 1 < entries.size()) {
            double T = entries[i + 1].timestamp - entries[i].timestamp;
            if (ttrack < T) {
                usleep((T - ttrack) * 1e6);
            }
        }
    }

    std::cout << "End of sequence." << std::endl;

    const std::string traj_file = run_dir + "/trajectory_replayed.txt";
    const std::string kf_file = run_dir + "/keyframes_replayed.txt";

    SLAM.SaveTrajectoryEuRoC(traj_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    SLAM.Shutdown();

    return 0;
}
