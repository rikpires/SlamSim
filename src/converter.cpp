#include "converter.h"
#include <Eigen/Dense>

namespace slam_sim
{
Mat3f Converter::ConvertAngular2Rotation ( Vec3f vector )
{
    Vec3f v ( vector );
    double theta = v.norm();
    v /= theta;
    Mat3f skew_v;
    skew_v << 0, -v[2], v[1],
           v[2], 0 , -v[0],
           -v[1], v[0], 0;
    Mat3f skew_vv = skew_v * skew_v;
    Mat3f rot = Mat3f::Identity() + sin ( theta ) * skew_v + ( 1 - cos ( theta ) ) * skew_vv;
    return rot;
}
Vec3f Converter::ConvertRotation2Angular ( Mat3f rotation )
{
    if ( fabs ( rotation.determinant() - 1 ) > 1e-5 )
        return Vec3f ( 0, 0, 0 );

    Mat3f carley = rotation - rotation.transpose();
    double rx = ( carley ( 2, 1 ) - carley ( 1, 2 ) ) * 0.5;
    double ry = ( carley ( 0, 2 ) - carley ( 2, 0 ) ) * 0.5;
    double rz = ( carley ( 1, 0 ) - carley ( 0, 1 ) ) * 0.5;
    Vec3f v ( rx, ry, rz );
    double v_norm = v.norm();
    if ( v_norm < 1e-6 )
    {
        v << 0, 0, 0;
    }
    else if ( v_norm < 1.0 - 1e-6 )
    {
        double theta = asin ( v_norm ) / v_norm;
        v *= theta;
    }
    else
    {
        v *= asin ( 1.0 );
    }
    return v;
}
}