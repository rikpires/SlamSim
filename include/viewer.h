#ifndef SLAM_JACOBIAN_INCLUDE_VIEWER_H_
#define SLAM_JACOBIAN_INCLUDE_VIEWER_H_

#include "definition.h"
#include "map.h"
#include <vector>
#include <mutex>
#include <thread>
#include <pangolin/pangolin.h>

namespace slam_sim
{
/**
 * Viewer to display map points and key frames
 */
class Viewer
{
public:
    Viewer();
    virtual ~Viewer();
    void SetMap(Map *map);
private:
    void Run();
    /// 
    /**
     * Draw all key frames and points.
     * Associated map points of the latest frame will be drawn with green color.
     * Other map points will be drawn with gray color.
     */
    void DrawKeyFramesAndPoints();
    /// draw map points of one key frame
    void DrawPoints( const KeyFrame* key_frame, bool latest_frame );
    /// draw camera of one key frame
    void DrawCamera( const KeyFrame* key_frame, const float* color, float size_factor );
    void Close();
    void Draw();
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    /// viewer size
    int width_, height_;
    /// ui and line width
    int ui_width, line_width_;
    /// size of map point
    float point_size_;
    /// camera intrinsic matrix
    Mat3f intrinsic_;
    /// map data (map points and key frames)
    Map *map_;
    /// mutex
    std::mutex mutex_;
    /// viewer runs in a individual thread
    std::thread thread_;
};
}
#endif
