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

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/LocalMapping.h"
#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/Tracking.h"
#include "MORB_SLAM/KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include <utility>
#include <map>
#include <string>
#include <set>
#include <vector>
#ifdef FactoryEngine
#include <apps/morb_g2o/g2o/types/types_seven_dof_expmap.h>
#else
#include "g2o/types/types_seven_dof_expmap.h"
#endif

namespace MORB_SLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;
typedef std::shared_ptr<Tracking> Tracking_ptr;


class LoopClosing
{
public:

    typedef std::pair<std::set<KeyFrame*>,int> ConsistentGroup;
    typedef std::map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

    bool hasMergedLocalMap;

public:

    bool loopClosed = false;

    LoopClosing(const Atlas_ptr &pAtlas, std::shared_ptr<KeyFrameDatabase> pDB, std::shared_ptr<ORBVocabulary> pVoc,const bool bFixScale, const bool bActiveLC, bool bInertial);

    void SetTracker(Tracking_ptr pTracker);

    void SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();
    void RequestResetActiveMap(std::shared_ptr<Map> pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(std::shared_ptr<Map> pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        std::unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        std::unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }

    void RequestFinish();

    bool isFinished();

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                std::set<MapPoint*> &spMatchedMPinOrigin, std::vector<MapPoint*> &vpMapPoints,
                                std::vector<MapPoint*> &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<MapPoint*> &vpMapPoints);
    void SearchAndFuse(const std::vector<KeyFrame*> &vConectedKFs, std::vector<MapPoint*> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void CheckObservations(std::set<KeyFrame*> &spKFsMap1, std::set<KeyFrame*> &spKFsMap2);

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    std::shared_ptr<Map> mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas_ptr mpAtlas;
    Tracking_ptr mpTracker;

    std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
    std::shared_ptr<ORBVocabulary> mpORBVocabulary;

    std::shared_ptr<LocalMapping> mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpLastCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    std::shared_ptr<Map> mpLastMap;

    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    KeyFrame* mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    KeyFrame* mpLoopMatchedKF;
    std::vector<MapPoint*> mvpLoopMPs;
    std::vector<MapPoint*> mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    KeyFrame* mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    KeyFrame* mpMergeMatchedKF;
    std::vector<MapPoint*> mvpMergeMPs;
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::jthread mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    int mnFullBAIdx;



    std::vector<double> vdPR_CurrentTime;
    std::vector<double> vdPR_MatchedTime;
    std::vector<int> vnPR_TypeRecogn;

    //DEBUG
    std::string mstrFolderSubTraj;
    int mnNumCorrection;
    int mnCorrectionGBA;


    // To (de)activate LC
    bool mbActiveLC = true;

    bool mbInertial;

#ifdef REGISTER_LOOP
    std::string mstrFolderLoop;
#endif
};

} //namespace ORB_SLAM

