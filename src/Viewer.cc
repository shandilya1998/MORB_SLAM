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

#include "MORB_SLAM/Viewer.h"

#include <pangolin/pangolin.h>

#include "MORB_SLAM/ImprovedTypes.hpp"
#include <chrono>
#include <ctime>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <string>
#include "MORB_SLAM/System.h"
#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {

Viewer::Viewer(const System_ptr &pSystem)
    : mpSystem(pSystem),
      mpAtlas(pSystem->mpAtlas),
      mpFrameDrawer(pSystem->mpAtlas),
      mpMapDrawer(pSystem->mpAtlas, *pSystem->getSettings()),
      mpTracker(pSystem->mpTracker),
      both(false),
      mbClosed(false) {
    newParameterLoader(*pSystem->getSettings());
    std::cout << "Creating Viewer thread" << std::endl;
    mptViewer = std::thread(&Viewer::Run, this);
}

Viewer::~Viewer(){
  close();
  if(mptViewer.joinable()) mptViewer.join();
}

void Viewer::newParameterLoader(const Settings &settings) {
  mImageViewerScale = 1.f;

  float fps = settings.fps();
  if (fps < 1) fps = 30;
  mT = 1e3 / fps;

  cv::Size imSize = settings.newImSize();
  mImageHeight = imSize.height;
  mImageWidth = imSize.width;

  mImageViewerScale = settings.imageViewerScale();
  mViewpointX = settings.viewPointX();
  mViewpointY = settings.viewPointY();
  mViewpointZ = settings.viewPointZ();
  mViewpointF = settings.viewPointF();

  if ((mpTracker->mSensor == CameraType::STEREO || mpTracker->mSensor == CameraType::IMU_STEREO || mpTracker->mSensor == CameraType::IMU_RGBD || mpTracker->mSensor == CameraType::RGBD) && settings.cameraModelType() == Settings::KannalaBrandt) {
    both = true;
    mpFrameDrawer.both = true;
  }
}

void Viewer::update(const Packet &pose){
  if(mpTracker->mState != TrackingState::NOT_INITIALIZED) {
    mpFrameDrawer.Update(mpTracker, pose);
    mpMapDrawer.SetCurrentCameraPose(mpTracker->mCurrentFrame.GetPose());
  }
}

static void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, float mCameraSize, float mCameraLineWidth) {
  const float &w = mCameraSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();

#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glLineWidth(mCameraLineWidth);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

static void GetCurrentOpenGLCameraMatrix(const Eigen::Matrix4f &Twc, pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw) {
  for (int i = 0; i < 4; i++) {
    M.m[4 * i] = Twc(0, i);
    M.m[4 * i + 1] = Twc(1, i);
    M.m[4 * i + 2] = Twc(2, i);
    M.m[4 * i + 3] = Twc(3, i);
  }

  MOw.SetIdentity();
  MOw.m[12] = Twc(0, 3);
  MOw.m[13] = Twc(1, 3);
  MOw.m[14] = Twc(2, 3);
}


void Viewer::Run() {

  pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer", 1024, 768);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
  pangolin::Var<bool> menuCamView("menu.Camera View", true, true);
  pangolin::Var<bool> menuTopView("menu.Top View", false, false);
  // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
  pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
  pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph", true, true);
  pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
  pangolin::Var<bool> menuReset("menu.Reset", false, false);
  pangolin::Var<bool> menuStop("menu.Stop", false, false);

  pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000), pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc, Twr;
  Twc.SetIdentity();
  pangolin::OpenGlMatrix Ow;  // Oriented with g in the z axis
  Ow.SetIdentity();
  cv::namedWindow("ORB-SLAM3: Current Frame");

  bool bFollow = true;
  bool bLocalizationMode = false;
  bool bCameraView = true;

  if (!mpTracker->mSensor.isInertial()) {
    menuShowGraph = true;
  }

  float trackedImageScale = 1.0;

  std::cout << "Starting the Viewer" << std::endl;
  while (isOpen()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GetCurrentOpenGLCameraMatrix(mpMapDrawer.getCameraPose(), Twc, Ow);

    if (menuFollowCamera && bFollow) {
      if (bCameraView)
        s_cam.Follow(Twc);
      else
        s_cam.Follow(Ow);
    } else if (menuFollowCamera && !bFollow) {
      if (bCameraView) {
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
        s_cam.Follow(Twc);
      } else {
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
        s_cam.Follow(Ow);
      }
      bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    if (menuCamView) {
      menuCamView = false;
      bCameraView = true;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc);
    }

    if (menuTopView && mpAtlas->isImuInitialized()) {
      menuTopView = false;
      bCameraView = false;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
      s_cam.Follow(Ow);
    }

    if (menuLocalizationMode && !bLocalizationMode) {
      mpTracker->ActivateLocalizationMode();
      bLocalizationMode = true;
    } else if (!menuLocalizationMode && bLocalizationMode) {
      mpTracker->DeactivateLocalizationMode();
      bLocalizationMode = false;
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    DrawCurrentCamera(Twc, mpMapDrawer.getCameraSize(), mpMapDrawer.getCameraLineWidth());
    if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
      mpMapDrawer.DrawKeyFrames(menuShowKeyFrames, menuShowGraph, menuShowInertialGraph, menuShowOptLba);
    
    if (menuShowPoints) mpMapDrawer.DrawMapPoints();

    pangolin::FinishFrame();

    cv::Mat toShow;
    cv::Mat im = mpFrameDrawer.DrawFrame(trackedImageScale);

    if (both) {
      cv::Mat imRight = mpFrameDrawer.DrawRightFrame(trackedImageScale);
      cv::hconcat(im, imRight, toShow);
    } else {
      toShow = im;
    }

    if (mImageViewerScale != 1.f) {
      int width = toShow.cols * mImageViewerScale;
      int height = toShow.rows * mImageViewerScale;
      cv::resize(toShow, toShow, cv::Size(width, height));
    }

    cv::imshow("ORB-SLAM3: Current Frame", toShow);
    cv::waitKey(mT);

    if (menuReset) {
      menuShowGraph = true;
      menuShowInertialGraph = true;
      menuShowKeyFrames = true;
      menuShowPoints = true;
      menuLocalizationMode = false;
      if (bLocalizationMode) mpTracker->DeactivateLocalizationMode();
      bLocalizationMode = false;
      bFollow = true;
      menuFollowCamera = true;
      mpTracker->RequestSystemReset();
      menuReset = false;
    }

    if (menuStop) {
      mpSystem->SaveAtlas(1);
      menuStop = false;
    }
  }

  close();
}

bool Viewer::isClosed() const {
  return mbClosed;
}

bool Viewer::isOpen() const {
  return !mbClosed;
}

void Viewer::close(){
  mbClosed = true;
}

}  // namespace MORB_SLAM
