#include "map.h"
#include <iostream>

namespace slam_sim
{
Map::Map()
{
}
Map::~Map()
{
}
void Map::AddKeyFrame(KeyFrame* new_key_frame )
{
    key_frames_.push_back(new_key_frame);
}
void Map::AddMapPoint(MapPoint* new_map_point )
{
    map_points_.push_back(new_map_point);
}
void Map::AddMapPoints(const std::vector<MapPoint*>& new_map_points )
{
    map_points_.insert(map_points_.end(),new_map_points.begin(), new_map_points.end());
}
const KeyFrame * Map::GetKeyFrame(FrameId id)
{
    KeyFrame * frame = nullptr;
    for (auto kf = key_frames_.begin(); kf != key_frames_.end(); ++kf)
    {
        if ((*kf)->GetId() == id)
            frame = *kf;
    }
    return frame;
}
const MapPoint * Map::GetMapPoint(PointId id)
{
    MapPoint * point = nullptr;
    for (auto mp = map_points_.begin(); mp != map_points_.end(); ++mp)
    {
        if ((*mp)->GetId() == id)
            point = (*mp);
    }
    return point;
}
void Map::DataAssociation()
{
    for (auto kf = key_frames_.begin(); kf != key_frames_.end(); ++kf)
    {
        (*kf)->ProjectMapPoints(map_points_);
    }
}
void Map::Print()
{
    for (auto kf = key_frames_.begin(); kf != key_frames_.end(); ++kf)
    {
        std::cout << "Key frame id[" << (*kf)->GetId() << "]" ;
        const std::vector<MapPoint*> & ass_mps = (*kf)->GetAssociateMapPoint();
        std::cout << "\tAss point number [" << ass_mps.size() << "]" << std::endl;
        // std::cout << (*kf)->GetPose() << std::endl;
        // for (auto mp = ass_mps.begin(); mp != ass_mps.end(); ++mp)
        // {
        //     std::cout << "\tAss MP[" << (*mp)->GetId() << "]\tpoint=[" 
        //     << (*mp)->Point()[0] << ", "
        //     << (*mp)->Point()[1] << ", "
        //     << (*mp)->Point()[2] << "]"
        //     << std::endl;
        // }
        // std::cout << std::endl;
    }
}
}