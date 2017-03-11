#ifndef SLAM_JACOBIAN_INCLUDE_SYSTEM_SIMULATOR_H_
#define SLAM_JACOBIAN_INCLUDE_SYSTEM_SIMULATOR_H_

#include <vector>
#include <string>
#include "definition.h"
#include "map.h"
#include "viewer.h"
#include "trajectory_parameter.h"
#include <thread>

namespace slam_sim
{
/**
 * Class simulator include methods to simulate a virtual trajectory from file 
 */
class Simulator
{
public:
    /**
     * Read trajectory information from file
     * @param file_path path of trajectory file
     */
    Simulator(std::string file_path);
    virtual ~Simulator();
    const Mat3f & GetIntrinsic() {return trajectory_para_.intrinsic_;}
    /**
     * Simulate poses and map points according to trajectory
     * @return true for simulation finished, false for failure
     */
    bool Run();
private:
    /**
     * Simulate points at one pose and transform points to world coordinates
     * @param  pose               key frame pose
     * @param  point_num_per_pose point will be generated
     * @param  sim_points         output points in world coordinates
     * @return                    true for success, false for failure
     */
    bool SimulatePoints(const Mat4f& pose, int point_num_per_pose, std::vector<MapPoint*>& sim_points);
    /**
     * Simulate one pose from reference pose and trajectory design
     * @param  reference_pose reference pose (last location)
     * @param  para           trajectory para (rotation and translation direction)
     * @param  current_pose   output pose
     * @return                true for success, false for failure
     */
    bool SimulatePose(const Mat4f& reference_pose, const SegmentParameter& para, Mat4f& current_pose);
    /**
     * Associate key frames and map points ( which map points can be seen by one key frame )
     * @return true for success, false for failure
     */
    bool DataAssociation();
private:
    // simulated map
    Map map_;
    // for display
    Viewer viewer_;
    // trajectory parameter
    TrajectoryParameter trajectory_para_;
};
}
#endif