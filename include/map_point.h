#ifndef SLAM_JACOBIAN_INCLUDE_MAP_POINT_H_
#define SLAM_JACOBIAN_INCLUDE_MAP_POINT_H_

#include "definition.h"
// #include "key_frame.h"
namespace slam_sim
{
class KeyFrame;
/**
 * Class MapPoint includes map points in world coordinates and other information
 */
class MapPoint
{
public:
    MapPoint(Vec4f p, PointId id);
    virtual ~MapPoint();
    MapPoint(const MapPoint& mp);
    MapPoint& operator= (const MapPoint& mp);
    const Vec4f& Point() const {return point_;}
    const PointId& GetId() const {return id_;}
    // const std::vector<KeyFrame*> & GetAssociateKeyFrame() {return ass_key_frames_;}

public:
    std::vector<KeyFrame*> ass_key_frames_;
    bool is_track_in_frame_;
private:
    Vec4f point_;
    PointId id_;
};
}

#endif