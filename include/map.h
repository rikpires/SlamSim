#ifndef SLAM_JACOBIAN_INCLUDE_MAP_H_
#define SLAM_JACOBIAN_INCLUDE_MAP_H_

#include <unordered_map>
#include "definition.h"
#include "key_frame.h"
#include "map_point.h"

namespace slam_sim
{
/**
 * Class Map includes key frames and map points
 */
class Map
{
public:
    Map();
    virtual ~Map();
    /**
     * [AddKeyFrame description]
     * @param key_frame [description]
     */
    void AddKeyFrame(KeyFrame* new_key_frame );
    void AddMapPoint(MapPoint* new_point );
    void AddMapPoints(const std::vector<MapPoint*>& new_points );
    const std::vector<KeyFrame*> GetKeyFrames() {return key_frames_;}
    const std::vector<MapPoint*> GetMapPoints() {return map_points_;}
    size_t GetKeyFrameNum() {return key_frames_.size();}
    size_t GetMapPointsNum() {return map_points_.size();}
    const KeyFrame * GetKeyFrame(FrameId id);
    const MapPoint * GetMapPoint(PointId id);
    /**
     * Associate map points with key frames
     */
    void DataAssociation();
    void Print();
private:
    /// key frames
    std::vector<KeyFrame*> key_frames_;
    /// map points
    std::vector<MapPoint*> map_points_;
};
}

#endif