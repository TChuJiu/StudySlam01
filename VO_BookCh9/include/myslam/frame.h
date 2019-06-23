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

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam 
{
    
// forward declare 
class MapPoint;
class Frame
{   //Frame 帧： Ptr 指向该类对象(帧属性集合)的指针； id_ 该帧图像的id； 
    //time_stamp_ 该帧的时间戳； T_c_w 该帧下世界坐标系到相机坐标系下的转换矩阵T； Camera::Ptr 存储图像属性及坐标转换的指针（每帧）； 
    //Mat 存储图像 和 深度图的数据； is_key_frame_ 判断是否关键帧；
public:
    typedef std::shared_ptr<Frame> Ptr;         //shared_ptr智能指针  
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
    Mat                            color_, depth_; // color and depth image 
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame
    
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
