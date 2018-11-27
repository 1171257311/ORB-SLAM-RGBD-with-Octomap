/**
* This file is part of ORB-SLAM2.
* 原来的 viewr 拆开，留下的作为基类，具体的 可视化 平台 作为子类 胖果林显示子类
*/


#ifndef PANGOLIN_VIEWER_H
#define PANGOLIN_VIEWER_H

#include "Viewer.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "CostMap/include/costmap_2d_HRG.h"
#include "dataStruct.h"
#include "base_local_planner/include/dwa_planner_control.h"
//#include "global_planner/include/global_planner.h"
#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class PangolinViewer : public Viewer {
 public:
  PangolinViewer(const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    //void RequestFinish();

    //void RequestStop();

    //bool isFinished();

    //bool isStopped();

    //void Release();

	void UpdateFrame(Tracking *pTracker);
	void SetCurrentCameraPose(const cv::Mat &Tcw);
	void Register(System* pSystem);

    void RegisterGlobalCostMap(costmap_2d::Costmap2D_HRG* pCostMap)
    {
        mpGlobalMap = pCostMap;
    }
    void RegisterLocalCostMap(costmap_2d::Costmap2D_HRG* pCostMap)
    {
        mpLocalMap = pCostMap;
    }

    void RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs)
    {
        observation = mvObs;
    }

    void RegisterPlan(geometry_msg::Path plan)
    {
        globalPlan = plan;
    }

    void DrawObs()
    {
        glPointSize(2);
        glBegin(GL_POINTS);


        for(int i=0; i<observation.points.size(); i++)
        {

            glColor3f(observation.points[i].r/255.0, observation.points[i].g/255.0,observation.points[i].b/255.0);
            glVertex3f(observation.points[i].x,observation.points[i].y,observation.points[i].z);

        }

        glEnd();
    }

    void DrawPlan()
    {

             glPointSize(10.0f);
             glBegin(GL_POINTS);

             glVertex3f(0,0.6f,0);

             for(int i=0; i<globalPlan.poses.size(); i++)
             {
                  geometry_msg::Pose p =globalPlan.poses[i];
                  glColor3f(0.0,0.0,0.0); //gray
                  glVertex3f(p.position.x,  0.6f,p.position.y);

             }
            glEnd();

    }

	void RegisterMap(Map* map);
	void Finalize(void);

    bool GetCurrentCameraPos(cv::Mat &Rcw, cv::Mat  &Ow);

private:

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    costmap_2d::Costmap2D_HRG* mpGlobalMap;
    costmap_2d::Costmap2D_HRG* mpLocalMap;
    pcl::PointCloud<pcl::PointXYZRGB>observation;
    geometry_msg::Path globalPlan;
    base_local_planner::Trajectory localPlan;
    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

 

};

}


#endif // PANGOLIN_VIEWER_H
	

