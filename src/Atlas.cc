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

#include "MORB_SLAM/Atlas.h"

#include <iostream>
#include <mutex>
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"

namespace MORB_SLAM {

Atlas::Atlas() { mpCurrentMap = nullptr; }

Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid), mUseGravityDirectionFromLastMap(false) {
  mpCurrentMap = nullptr;
  CreateNewMap();
}

Atlas::~Atlas() {}

void Atlas::CreateNewMap() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  std::cout << "Creation of new Map with id: " << Map::nNextId << std::endl;
  if (mpCurrentMap) {
    //If it's not a new Atlas and there aren't 0 KFs in the current map, set the he map's init KF ID to the current map's maximum KF ID + 1
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;
    std::cout << "Stored Map with ID: " << mpCurrentMap->GetId() << std::endl;
  }
  std::cout << "Creation of new Map with last KF id: " << mnLastInitKFidMap << std::endl;

  mpCurrentMap = std::make_shared<Map>(mnLastInitKFidMap);
  mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(std::shared_ptr<Map> pMap) {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  std::cout << "Change to Map with id: " << pMap->GetId() << std::endl;
  mpCurrentMap = pMap;
}

void Atlas::AddKeyFrame(std::shared_ptr<KeyFrame> pKF) {
  std::shared_ptr<Map> pMapKF = pKF->GetMap();
  pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(std::shared_ptr<MapPoint> pMP) {
  std::shared_ptr<Map> pMapMP = pMP->GetMap();
  pMapMP->AddMapPoint(pMP);
}

std::shared_ptr<const GeometricCamera> Atlas::AddCamera(const std::shared_ptr<const GeometricCamera> &pCam) {
  // Check if the camera already exists
  bool bAlreadyInMap = false;
  int index_cam = -1;
  for (size_t i = 0; i < mvpCameras.size(); ++i) {
    std::shared_ptr<const GeometricCamera> pCam_i = mvpCameras[i];
    if (!pCam) std::cout << "Not pCam" << std::endl;
    if (!pCam_i) std::cout << "Not pCam_i" << std::endl;
    
    if (pCam->GetType() != pCam_i->GetType())
      continue;

    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      if (pCam_i->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
        break;
      }
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      if (pCam_i->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
        break;
      }
    }
  }

  if (bAlreadyInMap) {
    return mvpCameras[index_cam];
  } else {
    mvpCameras.push_back(pCam);
    return pCam;
  }
}

std::vector<std::shared_ptr<const GeometricCamera>> Atlas::GetAllCameras() { return mvpCameras; }

void Atlas::SetReferenceMapPoints(const std::vector<std::shared_ptr<MapPoint>>& vpMPs) {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

std::vector<std::shared_ptr<KeyFrame>> Atlas::GetAllKeyFrames() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

std::vector<std::shared_ptr<MapPoint>> Atlas::GetAllMapPoints() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

std::vector<std::shared_ptr<Map>> Atlas::GetAllMaps() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  struct compFunctor {
    inline bool operator()(std::shared_ptr<Map> elem1, std::shared_ptr<Map> elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::vector<std::shared_ptr<Map>> vMaps(mspMaps.begin(), mspMaps.end());
  sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

int Atlas::CountMaps() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

void Atlas::clearMap() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  mspMaps.clear();
  mpCurrentMap = nullptr;
  mnLastInitKFidMap = 0;
}

std::shared_ptr<Map> Atlas::GetCurrentMap(bool waitForGoodMap) {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  if (!mpCurrentMap) CreateNewMap();
  while (mpCurrentMap->IsBad()) {
    if (!waitForGoodMap) return nullptr;
  }

  return mpCurrentMap;
}

void Atlas::SetMapBad(std::shared_ptr<Map> pMap) {
  pMap->SetBad();
  mspBadMaps.insert(pMap);
  mspMaps.erase(pMap);
}

void Atlas::RemoveBadMaps() {
  mspBadMaps.clear();
}

void Atlas::SetImuInitialized() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
  std::unique_lock<std::recursive_mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave() {
  if (mpCurrentMap && !mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;  // The init KF ID is 1 greater than the current maximum

  struct compFunctor {
    inline bool operator()(std::shared_ptr<Map> elem1, std::shared_ptr<Map> elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };

  mvpBackupMaps.clear();
  for (std::shared_ptr<Map> pMi : mspMaps) {
    if (!pMi || pMi->IsBad())
      continue;

    if (pMi->GetAllKeyFrames().size() == 0) {
      // Empty map, erase before of save it.
      SetMapBad(pMi);
      continue;
    }
    mvpBackupMaps.push_back(pMi);
  }

  sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());
  std::set<std::shared_ptr<const GeometricCamera>> spCams(mvpCameras.begin(), mvpCameras.end());

  for (std::shared_ptr<Map> pMi : mvpBackupMaps) {
    pMi->PreSave(spCams, pMi);
  }
  RemoveBadMaps();
}

void Atlas::PostLoad() {
  std::map<unsigned int, std::shared_ptr<const GeometricCamera>> mpCams;
  for (std::shared_ptr<const GeometricCamera> pCam : mvpCameras) {
    mpCams[pCam->GetId()] = pCam;
  }

  mspMaps.clear();
  unsigned long int numKF = 0, numMP = 0;
  for (std::shared_ptr<Map> pMi : mvpBackupMaps) {
    mspMaps.insert(pMi);
    pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams, pMi);
    numKF += pMi->GetAllKeyFrames().size();
    numMP += pMi->GetAllMapPoints().size();
  }
  mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDatabase(std::shared_ptr<KeyFrameDatabase> pKFDB) {
  mpKeyFrameDB = pKFDB;
}

void Atlas::SetORBVocabulary(std::shared_ptr<ORBVocabulary> pORBVoc) { mpORBVocabulary = pORBVoc; }

void Atlas::setUseGravityDirectionFromLastMap(bool is_true) { mUseGravityDirectionFromLastMap = is_true; }

}  // namespace MORB_SLAM
