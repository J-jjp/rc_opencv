#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

// 相机参数（需要根据实际相机标定结果填写）
const double FOCAL_LENGTH = 700; // 焦距（像素单位）
const double QRCODE_REAL_SIZE = 12.0; // 二维码实际边长（厘米）

double calculateDistance(const vector<Point2f>& points) {
    // 计算二维码在图像中的像素边长
    double pixelLength = norm(points[1] - points[0]);
    
    // 使用相似三角形原理计算距离
    // 距离 = (实际尺寸 × 焦距) / 像素尺寸
    return (QRCODE_REAL_SIZE * FOCAL_LENGTH) / pixelLength;
}

int main() {

    QRCodeDetector qrDecoder;
    Mat frame;

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

    namedWindow("QR Distance Measurement", WINDOW_AUTOSIZE);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        vector<Point2f> points;
        String data = qrDecoder.detectAndDecode(frame, points);
        
        if (!points.empty()) {
            // 绘制二维码边框
            for (int i = 0; i < 4; ++i) {
                line(frame, points[i], points[(i + 1) % 4], 
                     Scalar(0, 255, 0), 2);
            }

            // 计算并显示距离
            double distance = calculateDistance(points);
            string distanceText = format("Distance: %.2f cm", distance);
            
            putText(frame, distanceText, Point(20, 40), 
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
            
            // 在二维码中心绘制十字标记
            Point center = (points[0] + points[2]) / 2;
            line(frame, Point(center.x-10, center.y), 
                         Point(center.x+10, center.y), Scalar(255, 0, 0), 2);
            line(frame, Point(center.x, center.y-10), 
                         Point(center.x, center.y+10), Scalar(255, 0, 0), 2);
        }

        imshow("QR Distance Measurement", frame);

        if (waitKey(30) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}