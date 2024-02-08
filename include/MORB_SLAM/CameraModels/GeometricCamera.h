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

#include <Eigen/Geometry>
#include <boost/serialization/access.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sophus/se3.hpp>
#include <vector>
#include <memory>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/GeometricTools.h"
#include "MORB_SLAM/TwoViewReconstruction.h"

namespace MORB_SLAM {
class GeometricCamera {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& mnId;
    ar& mnType;
    ar& mvParameters;
  }

 public:
  GeometricCamera(): tvr{nullptr} {}
  GeometricCamera(const std::vector<float>& _vParameters) : mvParameters(_vParameters), tvr{nullptr} {}
  virtual ~GeometricCamera() {}

  virtual cv::Point2f project(const cv::Point3f& p3D) const = 0;
  virtual Eigen::Vector2d project(const Eigen::Vector3d& v3D) const = 0;
  virtual Eigen::Vector2f project(const Eigen::Vector3f& v3D) const = 0;
  virtual Eigen::Vector2f projectMat(const cv::Point3f& p3D) const = 0;

  virtual float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D) const = 0;

  virtual Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) const = 0;
  virtual cv::Point3f unproject(const cv::Point2f& p2D) const = 0;

  virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) const = 0;

  virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1,
                                       const std::vector<cv::KeyPoint>& vKeys2,
                                       const std::vector<int>& vMatches12,
                                       Sophus::SE3f& T21,
                                       std::vector<cv::Point3f>& vP3D,
                                       std::vector<bool>& vbTriangulated) const = 0;

  virtual cv::Mat toK() const = 0;
  virtual Eigen::Matrix3f toK_() const = 0;
  virtual bool IsEqual(const std::shared_ptr<GeometricCamera> &pCam) const = 0;
  virtual bool IsEqual(const std::shared_ptr<const GeometricCamera> &pCam) const = 0;

  virtual bool epipolarConstrain(const std::shared_ptr<const GeometricCamera> &otherCamera,
                                 const cv::KeyPoint& kp1,
                                 const cv::KeyPoint& kp2,
                                 const Eigen::Matrix3f& R12,
                                 const Eigen::Vector3f& t12,
                                 const float sigmaLevel, const float unc) const = 0;

  float getParameter(const int i) const { return mvParameters[i]; }
  void setParameter(const float p, const size_t i) { mvParameters[i] = p; }

  size_t size() const { return mvParameters.size(); }

  virtual bool matchAndtriangulate(const cv::KeyPoint& kp1,
                                   const cv::KeyPoint& kp2,
                                   GeometricCamera* pOther, Sophus::SE3f& Tcw1,
                                   Sophus::SE3f& Tcw2, const float sigmaLevel1,
                                   const float sigmaLevel2,
                                   Eigen::Vector3f& x3Dtriangulated) const = 0;

  unsigned int GetId() const { return mnId; }

  unsigned int GetType() const { return mnType; }

  const static unsigned int CAM_PINHOLE = 0;
  const static unsigned int CAM_FISHEYE = 1;

  static long unsigned int nNextId;

 protected:
  std::vector<float> mvParameters;

  unsigned int mnId;

  unsigned int mnType;

  mutable std::shared_ptr<TwoViewReconstruction> tvr;
};
}  // namespace MORB_SLAM