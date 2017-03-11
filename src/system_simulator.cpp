#include <iostream>

#include "system_simulator.h"
#include "converter.h"
#include <thread>

using namespace std;
namespace slam_sim
{
Simulator::Simulator ( string file_path )
    : trajectory_para_(file_path)
{
    viewer_.SetMap(&map_);
}
Simulator::~Simulator()
{
    // map_.Print();
}
bool Simulator::Run ()
{
    Mat4f reference_pose, current_pose;
    reference_pose = Mat4f::Identity();
    size_t segment_num = trajectory_para_.segments_.size();
    size_t segment_count = 0;
    while (segment_count < segment_num)
    {
        const SegmentParameter& para = trajectory_para_.segments_[segment_count];
        int pose_count = 0;
        while ( pose_count < para.pose_num )
        {
            SimulatePose(reference_pose, para, current_pose);
            std::vector<MapPoint*> sim_points;
            SimulatePoints ( current_pose, para.point_num_per_pose, sim_points );
            reference_pose = current_pose;
            // generate key frame
            KeyFrame* kf = new KeyFrame(pose_count, trajectory_para_.intrinsic_, trajectory_para_.image_size_);
            kf->SetPose(current_pose);
            map_.AddKeyFrame(kf);
            map_.AddMapPoints(sim_points);
            ++pose_count;
            const KeyFrame* pkf = map_.GetKeyFrames().back();
            map_.DataAssociation();
            usleep(500 * 1000);
        }
        ++segment_count;
    }
    return true;
}
bool Simulator::SimulatePose(const Mat4f& reference_pose, const SegmentParameter& para, Mat4f& current_pose)
{
    Vec3f angles_random;
    
    Mat4f random_pose;
    angles_random.setRandom();
    angles_random[0] = angles_random[0] * trajectory_para_.rotation_noise_[0] + para.rotation_direction[0];
    angles_random[1] = angles_random[1] * trajectory_para_.rotation_noise_[1] + para.rotation_direction[1];
    angles_random[2] = angles_random[2] * trajectory_para_.rotation_noise_[2] + para.rotation_direction[2];
    
    Vec3f translation_random;
    translation_random.setRandom();
    translation_random[0] = translation_random[0] * trajectory_para_.translation_noise_[0] + para.translation_direction[0];
    translation_random[1] = translation_random[1] * trajectory_para_.translation_noise_[1] + para.translation_direction[1];
    translation_random[2] = translation_random[2] * trajectory_para_.translation_noise_[2] + para.translation_direction[2];

    Mat3f rotation = Converter::ConvertAngular2Rotation ( angles_random );
    Vec3f translation = translation_random;
    random_pose = Mat4f::Identity();
    random_pose.topLeftCorner<3, 3>() = rotation;
    random_pose.topRightCorner<3, 1>() = translation;
    current_pose = reference_pose * random_pose;
    return true;
}
bool Simulator::SimulatePoints ( const Mat4f & pose, int point_num_per_pose, std::vector<MapPoint*> & sim_points )
{
    // first simulate points at origin, then multiply pose
    int z_len = int ( trajectory_para_.point_z_range_[1] - trajectory_para_.point_z_range_[0] );
    float fx = trajectory_para_.intrinsic_ ( 0, 0 ), fy = trajectory_para_.intrinsic_ ( 1, 1 ), u0 = trajectory_para_.intrinsic_ ( 0, 2 ), v0 = trajectory_para_.intrinsic_ ( 1, 2 );
    float u, v, d;
    size_t map_point_num = map_.GetMapPointsNum();
    for ( int i = 0; i < point_num_per_pose; ++i )
    {
        u = double ( rand() ) / double ( RAND_MAX ) * trajectory_para_.image_size_[0];
        v = double ( rand() ) / double ( RAND_MAX ) * trajectory_para_.image_size_[1];
        u += double ( rand() ) / double ( RAND_MAX ) * trajectory_para_.projection_noise_;
        v += double ( rand() ) / double ( RAND_MAX ) * trajectory_para_.projection_noise_;
        d = trajectory_para_.point_z_range_[0] + float ( rand() % z_len );
        Vec4f point ( ( u - u0 ) *d / fx, ( v - v0 ) *d / fy, d, 1 );
        point = pose * point;
        sim_points.push_back(new MapPoint(point, map_point_num + i));
    }
    return true;
}
bool Simulator::DataAssociation()
{

}
}
