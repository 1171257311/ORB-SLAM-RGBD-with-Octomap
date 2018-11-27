/**
* This file is part of ORB-SLAM2.
* 地图 绘制 加入显示 octo-map的部分
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>
#include<mutex>

// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(const string &strSettingPath);

    Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void Register(Map* pMap);

    bool GetCurrentCameraPos(cv::Mat &Rcw, cv::Mat  &Ow);
    //TODO
    void DrawGrid();
    void DrawOctoMap();
    void SaveOctoMap(const char*filename);
    void DrawObs(pcl::PointCloud<pcl::PointXYZRGB>observation);
protected:
     void GeneratePointCloud(KeyFrame* kf, pcl::PointCloud<pcl::PointXYZRGB>& ground, pcl::PointCloud<pcl::PointXYZRGB>& nonground);
     void InsertScan(octomap::point3d sensorOrigin, pcl::PointCloud<pcl::PointXYZRGB>& ground, pcl::PointCloud<pcl::PointXYZRGB>& nonground);
     bool isSpeckleNode(const octomap::OcTreeKey &nKey);
     void UpdateOctomap(vector<KeyFrame*> vKFs);
     void heightMapColor(double h, double& r, double &g, double& b);
     void LoadOctoMap();
private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    uint16_t     lastKeyframeSize =0;
    cv::Mat mCameraPose;
    std::mutex mMutexCamera;
 private:

    double fx, fy, cx, cy;

    octomap::ColorOcTree *m_octree;
    octomap::KeyRay m_keyRay; // temp storage for casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    bool m_useHeightMap;

    double m_colorFactor;
    double m_res;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    bool bIsLocalization;


    octomap::OcTreeKey m_paddedMinKey, m_paddedMaxKey;
    inline static void updateMinKey(const octomap::OcTreeKey&in, octomap::OcTreeKey& min)
    {
        for(unsigned int i=0; i<3; i++)
            min[i] = std::min(in[i], min[i]);
    }
    inline static void updateMaxKey(const octomap::OcTreeKey&in, octomap::OcTreeKey& max)
    {
        for(unsigned int i=0; i<3; i++)
            max[i] = std::max(in[i], max[i]);
    }



};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
