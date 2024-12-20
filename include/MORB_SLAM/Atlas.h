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

#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <mutex>
#include <set>
#include <map>
#include <vector>

#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/Map.h"
#include "MORB_SLAM/MapPoint.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"

namespace MORB_SLAM {

// Used to validate that the format of the saved atlas is correct. If any of the boost::serialize functions gets changed in code, or if any saved value changes datatype, increment the version number
const std::string SERIALIZED_ATLAS_FORMAT_VERSION = "v1.0";

class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

class Atlas {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar.template register_type<Pinhole>();
    ar.template register_type<KannalaBrandt8>();

    // Save/load a set structure, the set structure is broken in libboost 1.58
    // for ubuntu 16.04, a vector is serializated
    ar& mvpBackupMaps;
    ar& mvpCameras;
    // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
    ar& Map::nNextId;
    ar& Frame::nNextId;
    ar& KeyFrame::nNextId;
    ar& MapPoint::nNextId;
    ar& GeometricCamera::nNextId;
    ar& mnLastInitKFidMap;
  }

 public:
  

  Atlas();
  Atlas(int initKFid);  // When its initialization the first map is created
  ~Atlas();

  void CreateNewMap();
  void ChangeMap(std::shared_ptr<Map> pMap);

  // Method for change components in the current map
  void AddKeyFrame(std::shared_ptr<KeyFrame> pKF);
  void AddMapPoint(std::shared_ptr<MapPoint> pMP);

  std::shared_ptr<const GeometricCamera> AddCamera(const std::shared_ptr<const GeometricCamera> &pCam);
  std::vector<std::shared_ptr<const GeometricCamera>> GetAllCameras();

  /* All methods without Map pointer work on current map */
  void SetReferenceMapPoints(const std::vector<std::shared_ptr<MapPoint>>& vpMPs);
  void InformNewBigChange();
  int GetLastBigChangeIdx();

  long unsigned int MapPointsInMap();
  long unsigned KeyFramesInMap();

  // Method for get data in current map
  std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames();
  std::vector<std::shared_ptr<MapPoint>> GetAllMapPoints();

  std::vector<std::shared_ptr<Map>> GetAllMaps();

  int CountMaps();

  void clearMap();

  void clearAtlas();

  std::shared_ptr<Map> GetCurrentMap(bool waitForGoodMap = true);

  void SetMapBad(std::shared_ptr<Map> pMap);
  void RemoveBadMaps();

  void SetImuInitialized();
  bool isImuInitialized();

  // Function for guarantee the correction of serialization of this object
  void PreSave();
  void PostLoad();

  void SetKeyFrameDatabase(std::shared_ptr<KeyFrameDatabase> pKFDB);

  void SetORBVocabulary(std::shared_ptr<ORBVocabulary> pORBVoc);

  bool UseGravityDirectionFromLastMap() const { return mUseGravityDirectionFromLastMap; }
  void setUseGravityDirectionFromLastMap(bool is_true);

 protected:
  std::set<std::shared_ptr<Map>> mspMaps;
  std::set<std::shared_ptr<Map>> mspBadMaps;
  // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
  std::vector<std::shared_ptr<Map>> mvpBackupMaps;

  std::shared_ptr<Map> mpCurrentMap;

  std::vector<std::shared_ptr<const GeometricCamera>> mvpCameras;

  unsigned long int mnLastInitKFidMap;

  // Class references for the map reconstruction from the save file
  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
  std::shared_ptr<ORBVocabulary> mpORBVocabulary;

  // Mutex
  std::recursive_mutex mMutexAtlas;

  bool mUseGravityDirectionFromLastMap;

};  // class Atlas
typedef std::shared_ptr<Atlas> Atlas_ptr;
typedef std::weak_ptr<Atlas> Atlas_wptr;

}  // namespace MORB_SLAM
