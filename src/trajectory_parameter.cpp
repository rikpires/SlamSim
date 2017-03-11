#include "trajectory_parameter.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
namespace slam_sim
{
TrajectoryParameter::TrajectoryParameter(string file_path)
{
    Read(file_path);
    Print();
}
TrajectoryParameter::~TrajectoryParameter()
{

}
bool TrajectoryParameter::Read(string file_path)
{
    bool ret = true;
    ifstream myfile(file_path);
    if (!myfile.is_open())
    {
        ret = false;
        cerr << "Unable to open file: " << file_path << endl;
    }
    intrinsic_.setZero();
    intrinsic_ ( 2, 2 ) = 1;
    
    string param_name;
    // # general setting
    // camera_size: 640 480
    // intrinsic: 400 400 320 640
    // point_z_range: 10.0 20.0
    // rotation_noise: 0.1 0.1 0.1
    // translation_noise: 0.2 0.2 0.2
    // projection_noise: 1.0    
    getline(myfile, param_name);
    myfile >> param_name >> image_size_[0] >> image_size_[1];
    myfile >> param_name >> intrinsic_(0,0) >> intrinsic_(1,1) >> intrinsic_(0,2) >> intrinsic_(1,2);
    myfile >> param_name >> point_z_range_[0] >> point_z_range_[1];
    myfile >> param_name >> rotation_noise_[0] >> rotation_noise_[1] >> rotation_noise_[2];
    myfile >> param_name >> translation_noise_[0] >> translation_noise_[1] >> translation_noise_[2];
    myfile >> param_name >> projection_noise_;

    // # trajectory
    // segment: 2
    // pose_num: 5
    // point_num_per_pose: 10
    // rotation_direction: 0.0 0.0 0.0
    // translation_direction: 3.0 0.0 0.0
    // pose_num: 10
    // point_num_per_pose: 10
    // rotation_direction: 0.0 0.0 0.0
    // translation_direction: 0.0 3.0 0.0
    getline(myfile, param_name);    // not sure why an additional getline() is needed
    getline(myfile, param_name);
    size_t segment_num;
    myfile >> param_name >> segment_num;
    segments_.reserve(segment_num);
    size_t iter = 0;
    while (iter < segment_num)
    {
        SegmentParameter segment;
        myfile >> param_name >> segment.pose_num;
        myfile >> param_name >> segment.point_num_per_pose;
        myfile >> param_name >> segment.rotation_direction[0] >> segment.rotation_direction[1] >> segment.rotation_direction[2];
        myfile >> param_name >> segment.translation_direction[0] >> segment.translation_direction[1] >> segment.translation_direction[2];
        segments_.push_back(segment);
        ++iter;
    }
    return ret;
}
void TrajectoryParameter::Print()
{
    cout << "# general setting" << endl;
    cout << "camera_size: " << image_size_[0] << " " << image_size_[1] << endl;
    cout << "intrinsic: " << intrinsic_(0,0) << " " << intrinsic_(1,1) << " " << intrinsic_(0,2) << " " << intrinsic_(1,2) << endl;
    cout << "point_z_range: " << point_z_range_[0] << " " << point_z_range_[1] << endl;
    cout << "rotation_noise: " << rotation_noise_[0] << " " << rotation_noise_[1] << " " << rotation_noise_[2] << endl;
    cout << "translation_noise: " << translation_noise_[0] << " " << translation_noise_[1] << " " << translation_noise_[2] << endl;
    cout << "projection_noise: " << projection_noise_ << endl;

    cout << "# trajectory" << endl;
    size_t segment_num = segments_.size();
    cout << "segment: " << segment_num << endl;
    size_t iter = 0;
    while (iter < segment_num)
    {
        cout << "pose_num: " << segments_[iter].pose_num << endl;
        cout << "point_num_per_pose: " << segments_[iter].point_num_per_pose << endl;
        cout << "rotation_direction: " << segments_[iter].rotation_direction[0] << " " << segments_[iter].rotation_direction[1] << " " << segments_[iter].rotation_direction[1] << endl;
        cout << "translation_direction: " << segments_[iter].translation_direction[0] << " " << segments_[iter].translation_direction[1] << " " << segments_[iter].translation_direction[1] << endl;
        ++iter;
    }
}
bool TrajectoryParameter::Write()
{
    return false;
}
}