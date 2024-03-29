/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,    //里程计初始化
        OK=0,               //正常跟踪
        LOST                //里程计丢失
    };
    
    VOState     state_;     // current VO status
    Map::Ptr    map_;       // map with all frames and map points
    
    Frame::Ptr  ref_;       // reference key-frame 参考关键帧 key-frame
    Frame::Ptr  curr_;      // current frame 
    
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
    Mat                     descriptors_curr_;  // descriptor in current frame 
    
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher    FlannBasedMatcher:快速最近邻逼近搜索函数匹配算法  BFMatcher:尝试所有可能的匹配，从而使得它总能够找到最佳匹配。
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points        路标点 
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)  路标点对应像素点
   
    SE3 T_c_w_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier features in icp  ICP中内联特征点
    int num_lost_;           // number of lost times
    
    // parameters 
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    float match_ratio_;     // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    double  map_point_erase_ratio_; // remove map point ratio
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    //提取关键点
    void extractKeyPoints(); 
    //orb计算描述子   
    void computeDescriptors(); 
    //orb特征点匹配
    void featureMatching();
    //PnP位姿估计
    void poseEstimationPnP(); 
    //优化地图
    void optimizeMap();
    
    //向地图中添加关键帧
    void addKeyFrame();
    //向地图中添加路标点
    void addMapPoints();
    //检查位姿估计
    bool checkEstimatedPose(); 
    //检查关键帧
    bool checkKeyFrame();
    
    // ??
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
};
}

#endif // VISUALODOMETRY_H
