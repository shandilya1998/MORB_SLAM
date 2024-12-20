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

#include "MORB_SLAM/MapPoint.h"
#include "MORB_SLAM/KeyFrame.h"

#include <set>
#include <mutex>
#include <memory>
#include <boost/serialization/base_object.hpp>
#include <stdexcept>
#include <map>
#include <string>
#include <list>
#include <vector>


namespace MORB_SLAM
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

public:
    
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(std::shared_ptr<KeyFrame> pKF);
    void AddMapPoint(std::shared_ptr<MapPoint> pMP);
    void EraseMapPoint(std::shared_ptr<MapPoint> pMP);
    void EraseKeyFrame(std::shared_ptr<KeyFrame> pKF);
    void SetReferenceMapPoints(const std::vector<std::shared_ptr<MapPoint>> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames();
    std::vector<std::shared_ptr<MapPoint>> GetAllMapPoints();
    std::vector<std::shared_ptr<MapPoint>> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    long unsigned int GetMaxKFid();

    std::shared_ptr<KeyFrame> GetOriginKF();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialBA1();
    void SetInertialBA2();
    bool GetInertialBA1();
    bool GetInertialBA2();

    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    void PreSave(std::set<std::shared_ptr<const GeometricCamera>> &spCams, std::shared_ptr<Map> sharedMap);
    void PostLoad(std::shared_ptr<KeyFrameDatabase> pKFDB, std::shared_ptr<ORBVocabulary> pORBVoc, std::map<unsigned int, std::shared_ptr<const GeometricCamera>> &mpCams, std::shared_ptr<Map> sharedMap);

    std::vector<std::shared_ptr<KeyFrame>> mvpKeyFrameOrigins;
    std::vector<unsigned long int> mvBackupKeyFrameOriginsId;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

    long unsigned int mnId;

    std::set<std::shared_ptr<MapPoint>> mspMapPoints;
    std::set<std::shared_ptr<KeyFrame>> mspKeyFrames;

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    std::vector<std::shared_ptr<MapPoint>> mvpBackupMapPoints;
    std::vector<std::shared_ptr<KeyFrame>> mvpBackupKeyFrames;

    std::shared_ptr<KeyFrame> mpKFinitial;
    std::shared_ptr<KeyFrame> mpKFlowerID;

    long int mnBackupKFinitialID;
    long int mnBackupKFlowerID;

    std::vector<std::shared_ptr<MapPoint>> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    bool mbBad = false;

    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;

};

} //namespace MORB_SLAM
