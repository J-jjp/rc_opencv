#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <opencv2/apriltag.hpp>
#include "tracking.cpp"
using namespace cv;
using namespace std;



int main() {

    Ptr<apriltag::AprilTagDetector> detector = apriltag::AprilTagDetector::create(
    apriltag::AprilTagDetector::TagFamily::TAG_36h11);
    Mat frame;
    double tag_size = 0.1; // AprilTag实际边长（单位：米）
    double fx = 800;      // 相机焦距（像素）
    double fy = 800;
    double cx = 320;      // 光心坐标
    double cy = 240;
    Mat cameraMatrix = Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F); // 假设无畸变
    VideoCapture cap;
    
    // 打开默认摄像头（通常为0）
    cap.open(0, CAP_V4L2);  // 在Linux下明确使用V4L2后端
    
    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        cerr << "错误：无法打开摄像头" << endl;
        cerr << "可能原因：" << endl;
        cerr << "1. 摄像头被其他程序占用" << endl;
        cerr << "2. 没有摄像头设备" << endl;
        cerr << "3. 权限不足（Linux下尝试sudo）" << endl;
        return -1;
    }

    // 设置摄像头参数（先设置参数再获取实际值）
    cap.set(CAP_PROP_FRAME_WIDTH, 800);
    cap.set(CAP_PROP_FRAME_HEIGHT, 600);
    cap.set(CAP_PROP_FPS, 90);  // 常见摄像头通常支持30fps
    shared_ptr<Tracking> tracking = make_shared<Tracking>();
    namedWindow("QR Distance Measurement", WINDOW_AUTOSIZE);
    namedWindow("to_string", WINDOW_AUTOSIZE);
 
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        string name = ".jpg";
        static int counter = 1;
        
        string img_path = "/home/ubuntu/smart-car/opencv/res/train/";
        name = img_path + to_string(counter) + ".jpg";

        Mat a =imread(name);
        vector<Point2f> points;
        // String data = qrDecoder.detectAndDecode(frame, points);
        Mat imageGray, imageBinary;
        cvtColor(a, imageGray, COLOR_BGR2GRAY); // RGB转灰度图
        vector<apriltag::AprilTagDetection> detections = detector->detect(gray);
        // 6. 处理检测结果
        for (const auto& detection : detections) {
            // 绘制边界框
            for (int i = 0; i < 4; ++i) {
                line(frame, 
                    Point(detection.corners[i].x, detection.corners[i].y),
                    Point(detection.corners[(i+1)%4].x, detection.corners[(i+1)%4].y),
                    Scalar(0, 255, 0), 2);
            }

            // 7. 计算距离（方法1：相似三角形法）
            double pixel_size = norm(detection.corners[0] - detection.corners[1]);
            double distance = (tag_size * fx) / pixel_size;

            // 方法2：PnP位姿估计（更精确）
            vector<Point3f> objectPoints = {
                {-tag_size/2, -tag_size/2, 0},
                { tag_size/2, -tag_size/2, 0},
                { tag_size/2,  tag_size/2, 0},
                {-tag_size/2,  tag_size/2, 0}
            };
            Mat rvec, tvec;
            solvePnP(objectPoints, detection.corners, cameraMatrix, distCoeffs, rvec, tvec);
            double precise_dist = norm(tvec); // 精确距离

            // 显示结果
            putText(frame, format("Dist: %.2fm", precise_dist),
                Point(detection.corners[0].x, detection.corners[0].y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        }
        threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

        // if (!points.empty()) {
        //     // 绘制二维码边框
        //     for (int i = 0; i < 4; ++i) {
        //         line(frame, points[i], points[(i + 1) % 4], 
        //              Scalar(0, 255, 0), 2);
        //     }

        //     // 计算并显示距离
        //     double distance = calculateDistance(points);
        //     string distanceText = format("Distance: %.2f cm", distance);
            
        //     putText(frame, distanceText, Point(20, 40), 
        //             FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
            
        //     // 在二维码中心绘制十字标记
        //     Point center = (points[0] + points[2]) / 2;
        //     line(frame, Point(center.x-10, center.y), 
        //                  Point(center.x+10, center.y), Scalar(255, 0, 0), 2);
        //     line(frame, Point(center.x, center.y-10), 
        //                  Point(center.x, center.y+10), Scalar(255, 0, 0), 2);
        // }

        imshow("QR Distance Measurement", a);
        imshow("to_string", imageBinary);
        tracking->trackRecognition(imageBinary);
        tracking->drawImage(a);
        imshow("to", a);
        
        int key = waitKey(10);
        if (key == 13){
        //     tracking->savePicture(frame);
                counter++;
        }
        else if(key == 'a'){
            counter--;

        }
        else if (key == 27) break;
        std::cout<<"第"<<counter<<std::endl;

    }

    cap.release();
    destroyAllWindows();
    return 0;
}