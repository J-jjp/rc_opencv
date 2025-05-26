#pragma once

#include <fstream>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <numeric>  // 标准库数值算法
#include <cmath>  
// #include "../../include/common.hpp"

using namespace cv;
using namespace std;
#define COLSIMAGE 800    // 图像的列数
#define ROWSIMAGE 600    // 图像的行数
class Tracking
{
public:
    /**
     * @brief 构建二维坐标
     *
     */
    struct POINT
    {
        int x = 0;
        int y = 0;
        float slope = 0.0f;

        POINT(){};
        POINT(int x, int y) : x(x), y(y){};
    };
    vector<POINT> pointsLeft;     // 赛道左边缘点集
    vector<POINT> pointsRight;    // 赛道右边缘点集
    vector<POINT> widthBlock;         // 色块宽度

    double stdevLeft;                 // 边缘斜率方差（左）
    double stdevRight;                // 边缘斜率方差（右）评估赛道线的直线性

    int validRowsLeft = 0;            // 边缘有效行数（左）
    int validRowsRight = 0;           // 边缘有效行数（右）

    uint16_t rowCutUp = 10;           // 图像顶部切行
    uint16_t rowCutBottom = 10;       // 图像底部切行 图像顶部和底部切行的数量，用于裁剪图像以减少处理区域。


    /**
     * @brief 赛道线识别
     *
     * @param isResearch 是否重复搜索
     * @param rowStart 边缘搜索起始行
     */
    void trackRecognition(bool isResearch, uint16_t rowStart)
    {
        bool flagStartBlock = true;                    // 搜索到色块起始行的标志（行）
        int counterSearchRows = pointsLeft.size(); // 搜索行计数
        int L_startBlock[30];                            // 色块起点（行）
        int L_endBlock[30];                              // 色块终点（行）左
        int R_startBlock[30];                            // 色块起点（行）
        int R_endBlock[30];                              // 色块终点（行）右
        int L_counterBlock = 0;                          // 色块计数器（行）
        int R_counterBlock = 0;                          // 色块计数器（行）


        if (rowCutUp > ROWSIMAGE / 2)                 //为了防止切除过多的行数，这里将它们的最大值限制在图像总行数的四分之一
            rowCutUp = ROWSIMAGE / 2;
        if (rowCutBottom > ROWSIMAGE / 2)
            rowCutBottom = ROWSIMAGE / 2;

        if (!isResearch)
        {
            pointsLeft.clear();              // 初始化边缘结果
            pointsRight.clear();             // 初始化边缘结果
            widthBlock.clear();                  // 初始化色块数据
            validRowsLeft = 0;                   // 边缘有效行数（左）
            validRowsRight = 0;                  // 边缘有效行数（右）
            rowStart = ROWSIMAGE - rowCutBottom; // 默认底部起始行
        }
        else
        {
            if (pointsLeft.size() > rowStart)//赛道线识别的重复搜索模式下，调整各个存储边缘点和宽度信息的数据结构的大小,
                                                 // 并更新搜索的起始点和状态，以便从特定的行开始重新识别赛道线。
                pointsLeft.resize(rowStart);
            if (pointsRight.size() > rowStart)
                pointsRight.resize(rowStart);
            if (widthBlock.size() > rowStart)
            {
                widthBlock.resize(rowStart);
                if (rowStart > 1)
                    rowStart = widthBlock[rowStart - 1].x - 2;
            }

            flagStartBlock = false; // 搜索到色块起始行的标志（行）
        }

        //  开始识别赛道左右边缘
        for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
        {

            L_counterBlock = 0; // 色块计数器清空
            R_counterBlock = 0; 

            for (int col = COLSIMAGE/2; col < COLSIMAGE-2; col++) // 搜索出每行的所有色块
            {
                if (imagePath.at<uchar>(row, col) > 127 &&
                    imagePath.at<uchar>(row, col + 1) < 127)//寻找到白->黑的变化色块
                {
                    R_startBlock[R_counterBlock++] = col;//记录色块起始位置
                }

            }
            if (R_counterBlock==0)
            {
                R_startBlock[0] = COLSIMAGE-1;
            }
            
            for (int col = COLSIMAGE/2; col > 1; col--) // 搜索出每行的所有色块
            {
                if (imagePath.at<uchar>(row, col) > 127 &&
                    imagePath.at<uchar>(row, col - 1) < 127)//寻找到白->黑的变化色块
                {
                    L_startBlock[L_counterBlock++] = col;//记录色块起始位置
                }

            }
            if (L_counterBlock==0)
            {
                L_startBlock[0] = 1;
            }

            int widthBlocks = R_startBlock[0] - L_startBlock[0]; // 色块宽度临时变量

            int counterBlock = min(R_counterBlock,L_counterBlock);//取两个色块数量
            if (flagStartBlock)                            // 起始行做特殊处理(第一行rowstart)
            {
                if (row < ROWSIMAGE / 2)// 首行不满足宽度要求(第一行那有这么小的)
                    return;
                int limitWidthBlock = COLSIMAGE * 0.6; // 首行色块宽度限制（不能太小）

                if (widthBlocks > limitWidthBlock) // 满足首行宽度要求
                {
                    flagStartBlock = false;//第一行处理完成
                    POINT pointTmp(row, L_startBlock[0]);//记录最大的色块起始点x,y
                    pointsLeft.push_back(pointTmp);//记录至左点集
                    pointTmp.y = R_startBlock[0];//记录最大的色块结束点x,y
                    pointsRight.push_back(pointTmp);//记录至右点集
                    widthBlock.emplace_back(row, R_startBlock[0] -L_startBlock[0]);//记录色块宽度row, width
                    counterSearchRows++;
                }

            }
            else // 其它行色块坐标处理
            {
                vector<int> L_indexBlocks;               // 色块序号（行）
                vector<int> R_indexBlocks;               

                for (int i = 0; i < L_counterBlock; i++) // 上下行色块的连通性判断
                {
                    if (L_startBlock[i]>pointsLeft[pointsLeft.size() - 1].y&&
                    L_startBlock[i]<pointsRight[pointsRight.size() - 1].y)
                    {
                        L_indexBlocks.push_back(i);
                    }
                }
                for (size_t i = 0; i < R_counterBlock; i++)
                {

                    if (R_startBlock[i]<pointsRight[pointsRight.size() - 1].y&&
                    R_startBlock[i]>pointsLeft[pointsLeft.size() - 1].y)
                    {
                        R_indexBlocks.push_back(i);
                    }
                }

                int indexBlocks = min(L_indexBlocks.size(), R_indexBlocks.size());
                if (indexBlocks == 0) // 如果没有发现联通色块，则当前行不处理
                {
                    break;
                }
                indexBlocks = L_indexBlocks.size()+R_indexBlocks.size();
                if (indexBlocks == 2) // 只存在单个色块，正常情况，提取边缘信息
                {
                    if (R_startBlock[0] - L_startBlock[0] < COLSIMAGE / 10)//色块宽度过小丢弃这一行 
                    {
                        continue;
                    }
                    pointsLeft.emplace_back(row, L_startBlock[L_indexBlocks[0]]);
                    pointsRight.emplace_back(row, R_startBlock[R_indexBlocks[0]]);
                    slopeCal(pointsLeft, pointsLeft.size() - 1); // 边缘斜率计算
                    slopeCal(pointsRight, pointsRight.size() - 1);
                    widthBlock.emplace_back(row, R_startBlock[R_indexBlocks[0]] - L_startBlock[L_indexBlocks[0]]);
                }
                
                else if (indexBlocks > 2) // 存在多个色块，则需要择优处理：选取与上一行最近的色块
                {
                    int centerLast = COLSIMAGE / 2;
                    if (pointsRight.size() > 0 && pointsLeft.size() > 0)
                        centerLast = (pointsRight[pointsRight.size() - 1].y + pointsLeft[pointsLeft.size() - 1].y) / 2; // 上一行色块的中心点横坐标
                    vector<int> centerThis_set;          
                    vector<int> differBlocks;

                    for (size_t i = 0; i < L_indexBlocks.size(); i++)
                    {
                        for (size_t j = 0; j < R_indexBlocks.size(); j++)
                        {
                            int centerThis = (R_startBlock[R_indexBlocks[j]] + L_startBlock[L_indexBlocks[i]]) / 2;   
                            centerThis_set.push_back(centerThis);
                            differBlocks.push_back(abs(centerThis - centerLast)); 
                        }
                    }
                    
                    int min_value = 0;
                    int min_index = 0;// 目标色块的编号
                    for(size_t i = 0;i<differBlocks.size();i++)
                    {
                        if (differBlocks[i] < min_value) {
                            min_value = differBlocks[i];
                            min_index = i;
                        }
                    }
                    int L_block_id=min_index/R_indexBlocks.size();
                    int R_block_id=min_index%L_indexBlocks.size();
                    


                    POINT tmp_point(row, L_startBlock[L_indexBlocks[L_block_id-1]]);
                    pointsLeft.push_back(tmp_point);
                    tmp_point.y = R_startBlock[R_indexBlocks[R_block_id-1]];
                    pointsRight.push_back(tmp_point);
                    widthBlock.emplace_back(row, R_startBlock[R_indexBlocks[R_block_id-1]] - L_startBlock[L_indexBlocks[L_block_id-1]]);
                    slopeCal(pointsLeft, pointsLeft.size() - 1);
                    slopeCal(pointsRight, pointsRight.size() - 1);
                    counterSearchRows++;

                }

                stdevLeft = stdevEdgeCal(pointsLeft, ROWSIMAGE); // 计算边缘方差
                stdevRight = stdevEdgeCal(pointsRight, ROWSIMAGE);

                validRowsCal(); // 有效行计算
            }
        }
    }

    /**
     * @brief 赛道线识别
     *
     * @param imageBinary 赛道识别基准图像
     */
    void trackRecognition(Mat &imageBinary)
    {
        imagePath = imageBinary;
        trackRecognition(false, 0);
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param trackImage 需要叠加显示的图像
     */
    void drawImage(Mat &trackImage)
    {
        for (uint16_t i = 0; i < pointsLeft.size(); i++)
        {
            circle(trackImage, Point(pointsLeft[i].y, pointsLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (uint16_t i = 0; i < pointsRight.size(); i++)
        {
            circle(trackImage, Point(pointsRight[i].y, pointsRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }



        putText(trackImage, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_8U);
        putText(trackImage, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
                FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_8U);
    }

    /**
     * @brief 边缘斜率计算，计算边缘点集的标准差，用来衡量边缘的稳定性或波动程度。
     *
     * @param v_edge
     * @param img_height
     * @return double
     */
    double stdevEdgeCal(vector<POINT> &v_edge, int img_height)
    {
        if (v_edge.size() < img_height / 4)
        {
            return 1000;
        }
        vector<int> v_slope;
        int step = 10; // v_edge.size()/10;
        for (uint16_t i = step; i < v_edge.size(); i += step)
        {
            if (v_edge[i].x - v_edge[i - step].x)
                v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 / (v_edge[i].x - v_edge[i - step].x));
        }
        if (v_slope.size() > 1)
        {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // 均值
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope), [&](const double d)
                     { accum += (d - mean) * (d - mean); });

            return sqrt(accum / (v_slope.size() - 1)); // 方差
        }
        else
            return 0;
    }

public:
    Mat imagePath; // 赛道搜索图像
    /**
     * @brief 赛道识别输入图像类型
     *
     */
    enum ImageType
    {
        Binary = 0, // 二值化
        Rgb,        // RGB
    };

    ImageType imageType = ImageType::Binary; // 赛道识别输入图像类型：二值化图像
    /**
     * @brief 边缘斜率计算
     *
     * @param edge
     * @param index
     */
    void slopeCal(vector<POINT> &edge, int index)
    {
        if (index <= 4)
        {
            return;
        }
        float temp_slop1 = 0.0, temp_slop2 = 0.0;
        if (edge[index].x - edge[index - 2].x != 0)
        {
            temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                         ((edge[index].x - edge[index - 2].x) * 1.0f);
        }
        else
        {
            temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
        }
        if (edge[index].x - edge[index - 4].x != 0)
        {
            temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                         ((edge[index].x - edge[index - 4].x) * 1.0f);
        }
        else
        {
            edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
        }
        if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
        {
            edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
        }
        else if (abs(temp_slop1) != 255)
        {
            edge[index].slope = temp_slop1;
        }
        else
        {
            edge[index].slope = temp_slop2;
        }
    }

    /**
     * @brief 边缘有效行计算：左/右
     *
     */
    void validRowsCal(void)
    {
        // 左边有效行
        validRowsLeft = 0;
        if (pointsLeft.size() > 1)
        {
            for (int i = pointsLeft.size() - 1; i >= 1; i--)
            {
                if (pointsLeft[i].y > 2 && pointsLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsLeft[i].y < 2 && pointsLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

        // 右边有效行
        validRowsRight = 0;
        if (pointsRight.size() > 1)
        {
            for (int i = pointsRight.size() - 1; i >= 1; i--)
            {
                if (pointsRight[i].y <= COLSIMAGE - 2 && pointsRight[i - 1].y <= COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
                if (pointsRight[i].y >= COLSIMAGE - 2 && pointsRight[i - 1].y < COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
            }
        }
    }

    /**
     * @brief 冒泡法求取集合中值
     *
     * @param vec 输入集合
     * @return int 中值
     */
    int getMiddleValue(vector<int> vec)
    {
        if (vec.size() < 1)
            return -1;
        if (vec.size() == 1)
            return vec[0];

        int len = vec.size();
        while (len > 0)
        {
            bool sort = true; // 是否进行排序操作标志
            for (int i = 0; i < len - 1; ++i)
            {
                if (vec[i] > vec[i + 1])
                {
                    swap(vec[i], vec[i + 1]);
                    sort = false;
                }
            }
            if (sort) // 排序完成
                break;

            --len;
        }

        return vec[(int)vec.size() / 2];
    }
    /**
     * @brief 存储图像至本地
     *
     * @param image 需要存储的图像
     */
    void savePicture(Mat &image)
    {
        // 存图
        string name = ".jpg";
        static int counter = 0;
        counter++;
        string img_path = "../res/train/";
        name = img_path + to_string(counter) + ".jpg";
        imwrite(name, image);
        std::cout<<"第"<<counter<<"张\t"<<std::endl;
    }

};
