/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Flag to activate the measurement of time in each process (track,localmap,
// place recognition).

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>
#include <memory>
#include <stdexcept>

#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/ImprovedTypes.hpp"

namespace MORB_SLAM {

class System;

class Settings {
 public:
  /*
   * Enum for the different camera types implemented
   */
  enum CameraModelType { PinHole = 0, Rectified = 1, KannalaBrandt = 2 };

  /* Constructor from file */
  Settings(const std::string& configFile, const CameraType& sensor);

  /* Ostream operator overloading to dump settings to the terminal */
  friend std::ostream& operator<<(std::ostream& output, const Settings& s);

  /* Getter methods */
  CameraModelType cameraModelType() const { return cameraModelType_; }
  std::shared_ptr<const GeometricCamera> camera1() const { return std::const_pointer_cast<const GeometricCamera>(calibration1_); }
  std::shared_ptr<const GeometricCamera> camera2() const { return std::const_pointer_cast<const GeometricCamera>(calibration2_); }

  cv::Mat camera1DistortionCoef() { return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F, vPinHoleDistorsion1_.data()); }
  cv::Mat camera2DistortionCoef() { return cv::Mat(vPinHoleDistorsion2_.size(), 1, CV_32F, vPinHoleDistorsion2_.data()); }

  const Sophus::SE3f &Tlr() const { return Tlr_; }
  float bf() const { return bf_; }
  float b() const { return b_; }
  float thDepth() const { return thDepth_; }

  bool needToUndistort() const { return bNeedToUndistort_; }

  const cv::Size &newImSize() const { return newImSize_; }
  float fps() const { return fps_; }
  bool needToResize() const { return bNeedToResize1_; }
  bool needToRectify() const { return bNeedToRectify_; }

  float noiseGyro() const { return noiseGyro_; }
  float noiseAcc() const { return noiseAcc_; }
  float gyroWalk() const { return gyroWalk_; }
  float accWalk() const { return accWalk_; }
  float accFrequency() const { return accFrequency_; }
  float gyroFrequency() const { return gyroFrequency_; }
  const Sophus::SE3f &Tbc() const { return Tbc_; }

  float depthMapFactor() const { return depthMapFactor_; }

  int nFeatures() const { return nFeatures_; }
  int nLevels() const { return nLevels_; }
  float initThFAST() const { return initThFAST_; }
  float minThFAST() const { return minThFAST_; }
  float scaleFactor() const { return scaleFactor_; }

  float keyFrameSize() const { return keyFrameSize_; }
  float keyFrameLineWidth() const { return keyFrameLineWidth_; }
  float graphLineWidth() const { return graphLineWidth_; }
  float pointSize() const { return pointSize_; }
  float cameraSize() const { return cameraSize_; }
  float cameraLineWidth() const { return cameraLineWidth_; }
  float viewPointX() const { return viewPointX_; }
  float viewPointY() const { return viewPointY_; }
  float viewPointZ() const { return viewPointZ_; }
  float viewPointF() const { return viewPointF_; }
  float imageViewerScale() const { return imageViewerScale_; }

  const std::string &atlasLoadFile() const { return sLoadFrom_; }
  const std::string &atlasSaveFile() const { return sSaveto_; }

  float thFarPoints() const { return thFarPoints_; }
  bool activeLoopClosing() const { return activeLoopClosing_; }
  bool fastIMUInit() const { return fastIMUInit_; }
  bool stationaryIMUInit() const { return stationaryIMUInit_; }
  bool newMapRelocalization() const { return newMapRelocalization_; }

  const cv::Mat &M1l() const { return M1l_; }
  const cv::Mat &M2l() const { return M2l_; }
  const cv::Mat &M1r() const { return M1r_; }
  const cv::Mat &M2r() const { return M2r_; }

 private:
  template <typename T>
  T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required = true) {
    cv::FileNode node = fSettings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return T();
      }

    } else {
      found = true;
      return (T)node;
    }
  }

  void readCamera1(cv::FileStorage& fSettings);
  void readCamera2(cv::FileStorage& fSettings);
  void readImageInfo(cv::FileStorage& fSettings);
  void readIMU(cv::FileStorage& fSettings);
  void readRGBD(cv::FileStorage& fSettings);
  void readORB(cv::FileStorage& fSettings);
  void readViewer(cv::FileStorage& fSettings);
  void readLoadAndSave(cv::FileStorage& fSettings);
  void readOtherParameters(cv::FileStorage& fSettings);

  void precomputeRectificationMaps();

  CameraType sensor_;
  CameraModelType cameraModelType_;

  /* Visual stuff */
  std::shared_ptr<GeometricCamera> calibration1_, calibration2_;  // Camera calibration
  std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

  cv::Size originalImSize_, newImSize_;
  float fps_;

  bool bNeedToUndistort_;
  bool bNeedToRectify_;
  bool bNeedToResize1_, bNeedToResize2_;

  Sophus::SE3f Tlr_;
  float thDepth_;
  float bf_, b_;

  /* Rectification stuff */
  cv::Mat M1l_, M2l_;
  cv::Mat M1r_, M2r_;

  /* Inertial stuff */
  float noiseGyro_, noiseAcc_;
  float gyroWalk_, accWalk_;
  float accFrequency_;
  float gyroFrequency_;
  Sophus::SE3f Tbc_;

  /* RGBD stuff */
  float depthMapFactor_;

  /* ORB stuff */
  int nFeatures_;
  float scaleFactor_;
  int nLevels_;
  int initThFAST_, minThFAST_;

  /* Viewer stuff */
  float keyFrameSize_;
  float keyFrameLineWidth_;
  float graphLineWidth_;
  float pointSize_;
  float cameraSize_;
  float cameraLineWidth_;
  float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
  float imageViewerScale_;

  /* Save & load maps */
  std::string sLoadFrom_, sSaveto_;

  /* Other stuff */
  float thFarPoints_;
  bool activeLoopClosing_;
  bool fastIMUInit_;
  bool stationaryIMUInit_;
  bool newMapRelocalization_;
};
}  // namespace MORB_SLAM

