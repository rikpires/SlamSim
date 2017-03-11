#ifndef SLAM_JACOBIAN_INCLUDE_CONVERTER_H_
#define SLAM_JACOBIAN_INCLUDE_CONVERTER_H_

#include <vector>
#include "definition.h"
namespace slam_sim
{
class Converter
{
public:
    /**
     * Convert rotation's minimum representation to rotation matrix
     * @param  vector three axis angle
     * @return        rotation matrix
     */
    static Mat3f ConvertAngular2Rotation(Vec3f vector);
    /**
     * Convert rotation matrix to its minimum representation
     * @param  rotation rotation matrix
     * @return          rotation's minimum representation
     */
    static Vec3f ConvertRotation2Angular(Mat3f rotation);
};
}
#endif