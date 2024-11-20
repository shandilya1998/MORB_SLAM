/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/Tracking.h"
#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/LocalMapping.h"
#include "MORB_SLAM/LoopClosing.h"
#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/Settings.h"
#include "MORB_SLAM/Camera.hpp"
#include "MORB_SLAM/Packet.hpp"


namespace MORB_SLAM
{

class Viewer;
class ExternalMapViewer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
typedef std::shared_ptr<Tracking> Tracking_ptr;

class System {
 public:
    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

 public:
    
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const std::string &strVocFile, const std::string &strSettingsFile, const CameraType sensor);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    StereoPacket TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, double timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>());

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    RGBDPacket TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, double timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>());

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    MonoPacket TrackMonocular(const cv::Mat &im, double timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>());

    // Returns true if there have been a big map change (loop closure, global BA) since last call to this function
    bool MapChanged();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    virtual ~System();
	void Shutdown();

    TrackingState GetTrackingState();

    void ForceLost();

    friend Viewer;
    friend ExternalMapViewer;

    bool getHasMergedLocalMap();
    bool getIsDoneVIBA();

    std::shared_ptr<Settings> getSettings() const;

    bool getIsLoopClosed();
    void setIsLoopClosed(bool isLoopClosed);

    void RequestSystemReset();

    Sophus::SE3f GetInitialFramePose();
    bool HasInitialFramePose();

    void SaveAtlas(int type) const;

private:

    bool LoadAtlas(int type);

    std::string CalculateCheckSum(std::string filename, int type) const;

    // Input sensor
    CameraType mSensor;
    std::vector<Camera_ptr> cameras;

    // ORB vocabulary used for place recognition and feature matching.
    std::shared_ptr<ORBVocabulary> mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Atlas_ptr mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking_ptr mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    std::shared_ptr<LocalMapping> mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    std::shared_ptr<LoopClosing> mpLoopCloser;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread mptLocalMapping;
    std::thread mptLoopClosing;

    TrackingState mTrackingState;

    std::string mStrLoadAtlasFromFile;
    std::string mStrSaveAtlasToFile;

    std::string mStrVocabularyFilePath;

    std::shared_ptr<Settings> settings;

};
typedef std::shared_ptr<System> System_ptr;
typedef std::weak_ptr<System> System_wptr;

}// namespace ORB_SLAM
