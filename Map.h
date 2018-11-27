/**
* This file is part of ORB-SLAM2.
* 地图管理器-添加地图保存、载入函数===
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
	// ============= 读取地图 还原关键帧时需要用到=====
class ORBextractor;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

	// ===============new====================
	bool Save(const string &filename);
	bool Load(const string &filename, ORBVocabulary &voc);
	// ======================================

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;

	// ====================new==============================
	void _WriteMapPoint(ofstream &f, MapPoint* mp);
	void _WriteKeyFrame(ofstream &f, KeyFrame* kf,  map<MapPoint*, unsigned long int>& idx_of_mp);
	MapPoint* _ReadMapPoint(ifstream &f);
	KeyFrame* _ReadKeyFrame(ifstream &f, ORBVocabulary &voc, std::vector<MapPoint*> amp, ORBextractor* ex);
	//=====================
	
};

} //namespace ORB_SLAM2

#endif // MAP_H
