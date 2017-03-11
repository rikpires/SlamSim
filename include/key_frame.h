#ifndef SLAM_JACOBIAN_INCLUDE_KEY_FRAME_H_
#define SLAM_JACOBIAN_INCLUDE_KEY_FRAME_H_

#include "definition.h"
#include "map_point.h"
namespace slam_sim
{
/**
 * Class KeyFrame includes frame information and associated map points
 */
class KeyFrame
{
public:
    /**
     * Constructor of key frame
     * @param  id           frame id
     * @param  intrinsic    camera intrinsic matrix
     * @param  im_size      camera size
     */
    KeyFrame(FrameId id, Mat3f intrinsic, Vec2i im_size);
    ~KeyFrame();
    KeyFrame(const KeyFrame & kf);
    KeyFrame & operator= (const KeyFrame & kf);
    /**
     * Check whether the map points can be seen by this frame
     * @param mps vector of map points
     */
    void ProjectMapPoints(std::vector<MapPoint*> &mps);
    /**
     * Set pose of this frame
     * @param pose 4 by 4 matrix
     */
    void SetPose(Mat4f pose);
    const FrameId& GetId() const {return id_;}
    const Mat4f& GetPose() const {return pose_;}
    /**
     * Get map points associated with this frame (seen by this frame)
     */
    const std::vector<MapPoint*>& GetAssociateMapPoint() const {return ass_map_points_;}
private:
    /**
     * Check whether this map point can be seen by this frame
     * @param  map_point a map point
     * @return           true for seen, false for not seen
     */
    bool PointInKeyFrame(MapPoint* map_point);
public:
    /// camera intrinsic matrix
    Mat3f intrinsic_;
    /// camera size
    Vec2i im_size_;
private:
    /// associated map points
    std::vector<MapPoint*> ass_map_points_;
    /// frame id
    FrameId id_;
    /// 4 by 4 pose matrix
    Mat4f pose_;
    /// inverse of pose matrix
    Mat4f pose_inv_;
};
}

#endif