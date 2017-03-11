#include "key_frame.h"
#include <iostream>

namespace slam_sim
{
KeyFrame::KeyFrame(FrameId id, Mat3f intrinsic, Vec2i im_size)
    : id_(id), intrinsic_(intrinsic), im_size_(im_size)
{
}
KeyFrame::~KeyFrame()
{
    ass_map_points_.clear();
}
KeyFrame::KeyFrame(const KeyFrame & kf)
: id_(kf.GetId()), intrinsic_(kf.intrinsic_), im_size_(kf.im_size_), pose_(kf.GetPose())
{
    ass_map_points_.clear();
    ass_map_points_.insert(ass_map_points_.end(), kf.GetAssociateMapPoint().begin(), kf.GetAssociateMapPoint().end());
}
KeyFrame& KeyFrame::operator= (const KeyFrame & kf)
{
    id_ = kf.GetId();
    intrinsic_ = kf.intrinsic_;
    im_size_ = kf.im_size_;
    pose_ = kf.GetPose();
    ass_map_points_.clear();
    ass_map_points_.insert(ass_map_points_.end(), kf.GetAssociateMapPoint().begin(), kf.GetAssociateMapPoint().end());
    return *this;
}
void KeyFrame::SetPose(Mat4f pose) 
{
    pose_ = pose;
    pose_inv_ = pose_.inverse();
}
void KeyFrame::ProjectMapPoints(std::vector<MapPoint*> &mps)
{
    ass_map_points_.clear();
    for (auto it = mps.begin(); it != mps.end(); ++it)
    {
        if (PointInKeyFrame(*it))
        {
            ass_map_points_.push_back(*it);
        }
    }
}
bool KeyFrame::PointInKeyFrame(MapPoint* map_point)
{
    // 3D in absolute coordinates
    const Vec4f& mp = map_point->Point(); 

    // 3D in camera coordinates
    const Vec4f mp_in_frame = pose_.inverse()*mp;
    const float &mp_in_frame_x = mp_in_frame[0];
    const float &mp_in_frame_y = mp_in_frame[1];
    const float &mp_in_frame_z = mp_in_frame[2];

    // Check positive depth
    if(mp_in_frame_z < 0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/mp_in_frame_z;
    const float u = intrinsic_(0,0)*mp_in_frame_x*invz+intrinsic_(0,2);
    const float v = intrinsic_(1,1)*mp_in_frame_y*invz+intrinsic_(1,2);

    // std::cout << "pose=" << pose_.inverse() << std::endl << pose_inv_ << std::endl;
    // std::cout << "mp=" << mp.transpose() << "\t" << mp_in_frame.transpose() << std::endl;
    // std::cout << "uv=" << u << " " << v << std::endl << std::endl;

    bool ret = false;
    if(u < 0 || u > static_cast<float>(im_size_[0]) || v < 0 || v > static_cast<float>(im_size_[1]))
    {
        map_point->is_track_in_frame_ = false;
    }
    else
    {
        map_point->ass_key_frames_.push_back(this);
        map_point->is_track_in_frame_ = true;
        ret = true;    
    }
    return ret;
}
}