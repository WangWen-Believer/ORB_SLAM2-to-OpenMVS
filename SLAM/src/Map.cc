/**
 * @file Map.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 地图的实现
 * @version 0.1
 * @date 2019-02-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

//构造函数,地图点中最大关键帧id归0
Map::Map():mnMaxKFid(0)
{
}

/*
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
//在地图中插入关键帧,同时更新关键帧的最大id
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

/*
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
//向地图中插入地图点
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

/**
 * @brief 从地图中删除地图点,但是其实这个地图点所占用的内存空间并没有被释放
 * 
 * @param[in] pMP 
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    //下面是作者加入的注释. 实际上只是从std::set中删除了地图点的指针, 原先地图点
    //占用的内存区域并没有得到释放
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    //是的,根据值来删除地图点
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/*
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * @param vpMPs Local MapPoints
 */
// 设置参考地图点用于绘图显示局部地图点（红色）
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

//REVIEW 这个好像没有用到
void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

//这个在原版的泡泡机器人注释的版本中是没有这个函数和上面的函数的
//REVIEW 目测也是当前在程序中没有被被用到过
int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

//获取地图中的所有关键帧
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

//获取地图中的所有地图点
vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

//获取地图点数目
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

//获取地图中的关键帧数目
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

//获取参考地图点
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

//获取地图中最大的关键帧id
long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

//清空地图中的数据
void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// 对关键帧相关数据进行保存
void Map::Save(const string &filename,const cv::MatSize image_size)
{
    std::cout << "SFM Saving to "<< filename << std::endl;
    ofstream f;
    f.open(filename.c_str());

    f << "MVS "<< image_size[1] << " "<< image_size[0] << endl;
    // 输出关键帧的数量
    cout << "The number of KeyFrames: " << mspKeyFrames.size() << endl;

    unsigned long int nKeyFrames = mspKeyFrames.size();
    f << nKeyFrames << endl;
    for(auto kf:mspKeyFrames)
        SaveKeyFrame(f,kf);

    // 输出空间三维点的数目
    cout << "The number of MapPoints: " << mspMapPoints.size();
    unsigned long int nMapPoints = mspMapPoints.size();
    f << nMapPoints << endl;

    for(auto mp:mspMapPoints)
        SaveMapPoint(f,mp);

    f.close();

}

//  保存地图点
void Map::SaveMapPoint(ofstream &f, MapPoint *mp)
{
    //保存当前MapPoint世界坐标值
    cv::Mat mpWorldPos = mp->GetWorldPos();
    f <<" " <<mpWorldPos.at<float>(0)<<" " << mpWorldPos.at<float>(1)<<" " << mpWorldPos.at<float>(2) << " ";
    f << (mp->nObs)/2<< " ";

    std::map<KeyFrame*,size_t> mapObservation = mp->GetObservations();
    for(auto mit = mapObservation.begin(); mit != mapObservation.end(); mit++)
    {
        int Frameid;
        Frameid = mit->first->mnId;
        auto keyid = find(KeyId.begin(),KeyId.end(),Frameid) - KeyId.begin();
        f << keyid << " ";
    }
    f << "\n";
}

//  保存关键帧
void Map::SaveKeyFrame(ofstream &f, KeyFrame *kf)
{
    KeyId.push_back(kf->mnId);
    // 保存当前关键帧的id
    f << KeyId.end() - KeyId.begin() - 1<< " ";
    // 关键帧内参
    f << kf->fx << " " << kf->fy << " " << kf->cx << " " << kf->cy << " ";
    // 保存当前关键帧的位姿
    cv::Mat Tcw = kf->GetPose();
    cout << "GetPose " << std::to_string(kf->mTimeStamp) <<"\nTcw\n" <<Tcw<< endl;
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cout << "Rcw\n" << Rcw << endl;
    // 通过四元数保存旋转矩阵
    std::vector<float> Quat = Converter::toQuaternion(Rcw);

    for(int i=0; i<4; i++)
    {
        f << Quat[(3+i)%4] << " ";// qw qx qy qz
    }
    //保存平移
    for(int i=0; i<3; i++)
    {
        f << Tcw.at<float>(i,3) << " ";
    }
    ostringstream sTimeStamp;
    sTimeStamp << std::to_string(kf->mTimeStamp);
    f << sTimeStamp.str();
    f << "\n";
}

} //namespace ORB_SLAM
