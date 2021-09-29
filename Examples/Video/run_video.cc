#include <string>
#include <chrono>   // for time stamp
#include <iostream>

#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

string parameterFile = "/home/wyy/code/ORB_SLAM2/Examples/Video/video.yaml";
string vocFile = "/home/wyy/code/ORB_SLAM2/Vocabulary/ORBvoc.txt";
string videoFile = "/home/wyy/data/slam/bytedance/xr/210926/210926.mp4";

        
/*
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };
*/

int main(int argc, char **argv) {

    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR);

    cv::VideoCapture cap(videoFile);

    auto start = chrono::system_clock::now();
    int fps = 15;
    int state;
    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if ( frame.data == nullptr )
            break;
        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(960, 540));
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        auto T = SLAM.TrackMonocular(frame_resized, double(timestamp.count())/1000.0, state);
        auto t2 = chrono::system_clock::now();
        double track_time = chrono::duration_cast<chrono::milliseconds>(t2 - now).count();
        cout << 1000.0 / track_time << " " << state << std::endl;
        if (state == 2) {
            cout << T << endl;
        }
        // if (track_time < 1000 - fps) {
        //     cv::waitKey(1000 / fps - track_time);
        // }
    }
    SLAM.Shutdown();
    return 0;
}
