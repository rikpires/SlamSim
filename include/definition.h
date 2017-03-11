#ifndef SLAM_JACOBIAN_INCLUDE_definitions_H_
#define SLAM_JACOBIAN_INCLUDE_definitions_H_

#include "Eigen/Core"
#include "Eigen/LU"

namespace slam_sim
{
#ifdef SLAM_JACOBIAN_EXPORT
#define SLAM_JACOBIAN_API
#else
#define SLAM_JACOBIAN_API
#endif

typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector2i Vec2i;
typedef size_t FrameId;
typedef size_t PointId;
}
#endif