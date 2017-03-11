
#include "viewer.h"

using namespace std;
namespace slam_sim
{
// colors for display
const float g_color_red[3] = {1, 0, 0};
const float g_color_gray[3] = {0.5, 0.5, 0.5};
const float g_color_green[3] = {0, 1, 0};
const float g_color_blue[3] = {0, 0, 1};

Viewer::Viewer()
    : width_ ( 320 ), height_ ( 240 ), ui_width ( 100 ), line_width_ ( 3 ), point_size_ (3)
{
    intrinsic_ << 400, 0, width_ / 2, 0, 400, height_ / 2, 0, 0, 1;
    // run viewer thread
    thread_ = thread(&Viewer::Run, this);
}
Viewer::~Viewer()
{
    thread_.join();
}
void Viewer::Run()
{
    cout << "Start pangolin!" << endl;

    int w = width_, h = height_;
    pangolin::CreateWindowAndBind ( "Main", w * 2, h * 2 );

    glEnable ( GL_DEPTH_TEST );

    // 3D visualization
    pangolin::OpenGlRenderState visual_camera (
        pangolin::ProjectionMatrix ( w, h, intrinsic_ ( 0, 0 ), intrinsic_ ( 1, 1 ), intrinsic_ ( 0, 2 ), intrinsic_ ( 1, 2 ), 0.1, 1000 ),
        pangolin::ModelViewLookAt ( -0, -0, -50, 0, 0, 0, pangolin::AxisNegY )
    );

    pangolin::View& Visualization3D_display = pangolin::CreateDisplay()
            .SetBounds ( 0.0, 1.0, pangolin::Attach::Pix ( ui_width ), 1.0, -w / ( float ) h )
            .SetHandler ( new pangolin::Handler3D ( visual_camera ) );
    pangolin::CreatePanel ( "ui" ).SetBounds ( 0.0, 1.0, 0.0, pangolin::Attach::Pix ( ui_width ) );
    pangolin::Var<bool> a_button ( "ui.A_Button", false, false );
    pangolin::OpenGlMatrix ogl_mat;

    while ( !pangolin::ShouldQuit() )
    {
        glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        GetCurrentOpenGLCameraMatrix(ogl_mat);
        visual_camera.Follow(ogl_mat);
        // Activate efficiently by object
        Visualization3D_display.Activate ( visual_camera );
        DrawKeyFramesAndPoints();
        //glColor3f ( 1.0,1.0,1.0 );
        //pangolin::glDrawColouredCube();
        pangolin::FinishFrame();
    }
    cout << "Quit pangolin thread!" << endl;
}
void Viewer::Close()
{

}
void Viewer::SetMap(Map *map)
{
    unique_lock<mutex> lock(mutex_);
    map_ = map;
}
void Viewer::DrawPoints ( const KeyFrame* key_frame, bool is_last_frame )
{
    const vector<MapPoint*>& ass_mps = key_frame->GetAssociateMapPoint();
    glColor3f ( g_color_gray[0], g_color_gray[1], g_color_gray[2] );
    glPointSize(point_size_);
    glBegin ( GL_POINTS );
    // cout << "\t" << ass_mps.size() << endl;
    for ( auto iter =  ass_mps.begin(); iter != ass_mps.end(); ++iter)
    {
        if (!(*iter) || (*iter)->is_track_in_frame_)
            continue;
        glVertex3f ( ( float ) (*iter)->Point()[0],
                     ( float ) (*iter)->Point()[1],
                     ( float ) (*iter)->Point()[2] );
    }
    glEnd();
    glColor3f ( g_color_green[0], g_color_green[1], g_color_green[2] );
    glPointSize(point_size_);
    glBegin ( GL_POINTS );
    for ( auto iter =  ass_mps.begin(); iter != ass_mps.end(); ++iter)
    {
        if (!(*iter) || !(*iter)->is_track_in_frame_)
            continue;
        glVertex3f ( ( float ) (*iter)->Point()[0],
                     ( float ) (*iter)->Point()[1],
                     ( float ) (*iter)->Point()[2] );
    }
    glEnd();
}
void Viewer::DrawKeyFramesAndPoints ()
{
    unique_lock<mutex> lock(mutex_);
    const std::vector< KeyFrame* > key_frames = map_->GetKeyFrames();
    const float* color_green = g_color_green;
    glColor3f ( 0, 0, 1 );
    glLineWidth ( line_width_ );

    glBegin ( GL_LINE_STRIP );
    // draw position
    for ( auto iter =  key_frames.begin(); iter != key_frames.end(); ++iter )
    {
        if (!(*iter))
            continue;
        glVertex3f ( ( float ) ( (*iter)->GetPose()( 0, 3 ) ),
                     ( float ) ( (*iter)->GetPose()( 1, 3 ) ),
                     ( float ) ( (*iter)->GetPose()( 2, 3 ) ) );
    }
    glEnd();
    // draw camera
    for ( auto iter =  key_frames.begin(); iter != key_frames.end(); ++iter )
    {
        if (!(*iter))
            continue;
        DrawCamera ( (*iter), color_green, 1 );
    }
    // draw associated map points
    for ( auto iter =  key_frames.begin(); iter != key_frames.end(); ++iter )
    {
        if (!(*iter))
            continue;
        auto iter_temp = iter;
        bool is_last_frame = (++iter_temp) == key_frames.end();
        // cout << (*iter)->GetId() << " is_last_frame" << is_last_frame << endl;
        DrawPoints(*iter, is_last_frame);
    }
}

void Viewer::DrawCamera ( const KeyFrame* key_frame, const float* color, float size_factor )
{
    int width = width_, height = height_, line_width = line_width_;

    if ( width == 0 )
        return;

    float sz = size_factor;
    float cx = key_frame->intrinsic_ ( 0, 2 ), cy = key_frame->intrinsic_ ( 1, 2 );
    float fx = key_frame->intrinsic_ ( 0, 0 ), fy = key_frame->intrinsic_ ( 1, 1 );
    /*
    float pose_temp[16] = { pose ( 0,0 ),pose ( 1,0 ),pose ( 2,0 ),pose ( 3,0 ),
                            pose ( 0,1 ),pose ( 1,1 ),pose ( 2,1 ),pose ( 3,1 ),
                            pose ( 0,2 ),pose ( 1,2 ),pose ( 2,2 ),pose ( 3,2 ),
                            pose ( 0,3 ),pose ( 1,3 ),pose ( 2,3 ),pose ( 3,3 )
                          };
                          */
    glPushMatrix();
    glMultMatrixf ( ( GLfloat* ) key_frame->GetPose().data() );

    if ( color == 0 )
    {
        glColor3f ( 1, 0, 0 );
    }
    else
        glColor3f ( color[0], color[1], color[2] );

    glLineWidth ( line_width );
    glBegin ( GL_LINE_STRIP );

    glVertex3f ( 0, 0, 0 );
    glVertex3f ( sz * ( 0 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );

    glVertex3f ( 0, 0, 0 );
    glVertex3f ( sz * ( 0 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );

    glVertex3f ( 0, 0, 0 );
    glVertex3f ( sz * ( width - 1 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );

    glVertex3f ( 0, 0, 0 );
    glVertex3f ( sz * ( width - 1 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );

    glVertex3f ( sz * ( width - 1 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );
    glVertex3f ( sz * ( width - 1 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );

    glVertex3f ( sz * ( width - 1 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );
    glVertex3f ( sz * ( 0 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );

    glVertex3f ( sz * ( 0 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );
    glVertex3f ( sz * ( 0 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );

    glVertex3f ( sz * ( 0 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );
    glVertex3f ( sz * ( width - 1 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );

    glEnd();
    glPopMatrix();
}

void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{    
    unique_lock<mutex> lock(mutex_);    
    const std::vector< KeyFrame* > key_frames = map_->GetKeyFrames();
    if(!key_frames.empty())
    {
        const Mat4f& last_pose = key_frames.back()->GetPose();
        Mat3f rot(last_pose.topLeftCorner(3, 3));
        Vec3f translate(last_pose.topRightCorner(3, 1));
        // translate = rot * translate;
        // cout << rot << endl << translate << endl << endl;
        M.m[0] = rot(0, 0);
        M.m[1] = rot(1, 0);
        M.m[2] = rot(2, 0);
        M.m[3]  = 0.0;

        M.m[4] = rot(0, 1);
        M.m[5] = rot(1, 1);
        M.m[6] = rot(2, 1);
        M.m[7]  = 0.0;

        M.m[8] = rot(0, 2);
        M.m[9] = rot(1, 2);
        M.m[10] = rot(2, 2);
        M.m[11]  = 0.0;

        M.m[12] = translate(0);
        M.m[13] = translate(1);
        M.m[14] = translate(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}
}