#ifndef SLAM_JACOBIAN_INCLUDE_TRAJACTORY_H_
#define SLAM_JACOBIAN_INCLUDE_TRAJACTORY_H_

#include <vector>
#include <string>
#include "definition.h"

namespace slam_sim
{
/**
 * Parameter of one segment in the trajectory, poses in one segment have similar directions
 */
struct SegmentParameter
{
    /// direction of one rotation step
    Vec3f rotation_direction;
    /// direction of one translation step
    Vec3f translation_direction;
    /// map point generated in one pose
    int point_num_per_pose;
    /// total pose number
    int pose_num;
};
/**
 * Class TrajectoryParameter includes parameter of several segments
 */
class TrajectoryParameter
{
public:
    TrajectoryParameter(std::string file_path);
    virtual ~TrajectoryParameter();
private:
    bool Read(std::string file_path);
    bool Write();
    void Print();
public:
    /// camera image size
    Vec2i image_size_;
    /// camera intrinsic matrix
    Mat3f intrinsic_;
    /// z depth range of map point
    Vec2f point_z_range_;
    /// noise of rotation step on minimum representation
    Vec3f rotation_noise_;
    /// noise of translation step
    Vec3f translation_noise_;
    /// noise of map point projection (features) in the image
    float projection_noise_;
    /// vector of segments in the trajectory
    std::vector<SegmentParameter> segments_;
};
}
#endif