/**
* This file is part of ORB-SLAM2.
* 原来的 viewr 拆开，留下的作为基类，具体的 可视化 平台 作为子类 胖果林显示子类
*/


#ifndef VIEWER_H
#define VIEWER_H

#include "Tracking.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class System;

class Viewer {
 public:
  Viewer();

  virtual void Run();
  
  void RequestFinish();
  
  void RequestStop();
  
  bool isFinished();
  
  bool isStopped();
  
  void Release();
  
  virtual void UpdateFrame(Tracking *pTracker);
  virtual void SetCurrentCameraPose(const cv::Mat &Tcw);
  virtual void Register(System* pSystem);
  virtual void Finalize(void);

  protected:
	System* mpSystem;
    bool mbFinished;
    bool Stop();
	bool CheckFinish();
    void SetFinish();


  private:


    bool mbFinishRequested;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

