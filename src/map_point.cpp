#include "map_point.h"
// #include <utility>
#include "key_frame.h"
namespace slam_sim
{
MapPoint::MapPoint(Vec4f p, PointId id)
    : point_(p), id_(id)
{
}
MapPoint::~MapPoint()
{
    ass_key_frames_.clear();
}
MapPoint::MapPoint(const MapPoint& mp)
: id_(mp.GetId()), point_(mp.Point())
{
    ass_key_frames_.clear();
    ass_key_frames_.insert(ass_key_frames_.end(), mp.ass_key_frames_.begin(), mp.ass_key_frames_.end());
}
MapPoint& MapPoint::operator= (const MapPoint& mp)
{
    id_ = mp.GetId();
    point_ = mp.Point();
    ass_key_frames_.clear();
    ass_key_frames_.insert(ass_key_frames_.end(), mp.ass_key_frames_.begin(), mp.ass_key_frames_.end());
    return *this;
}
// void MapPoint::AddFeatures(FrameId key_frame_id, Vec2f * feature)
// {
//     features_[key_frame_id] = feature;
// }
}