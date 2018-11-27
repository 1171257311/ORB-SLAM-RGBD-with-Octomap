/**
* This file is part of ORB-SLAM2.
* 保存 octomap 相机参数独立成一个类进行管理==一些机器人位置等接口
*/


#ifndef SYSTEM_H
#define SYSTEM_H


#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "Camera.h"
//#include "CostMap/include/costmap_2d_HRG.h"//2d costmap


#define OCTOMAP_PATH "octomap.ot"
#define MAP_PATH     "Map.bin"


namespace ORB_SLAM2
{

class Viewer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
		
   // ====== 接口变换==============原来的==============
	
   // System(
   //        const string &strVocFile,
   //        const string &strSettingsFile,
   //        const eSensor sensor,
   //        const bool bUseViewer = true);	

    System(
           const string &strVocFile,
           const string &strSettingsFile,
           const eSensor sensor=MONOCULAR,
           Viewer* v=NULL,
           Map* m=NULL,
           ORBVocabulary* voc = NULL);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // Use this function in the monocular case.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

	Map* GetMap();
   //void setGlobalMap(costmap_2d::Costmap2D_HRG* globalMap);
   //void setLocalMap(costmap_2d::Costmap2D_HRG* localMap);
    // TODO: Save/Load functions
    bool SaveMap(const string &filename);
    // LoadMap(const string &filename);


	//	enum eTrackingState GetStatus();
	int GetStatus();
	void SetStatus(int status);
    bool GetRobotPos(cv::Mat &Rcw, cv::Mat  &Ow);


	Tracking* GetTracker() {return mpTracker;}
    //costmap_2d::Costmap2D_HRG *pGlobal_map;
    //costmap_2d::Costmap2D_HRG *pLocal_map;


private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.


    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;
    Tracking* mpTracker;
    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
