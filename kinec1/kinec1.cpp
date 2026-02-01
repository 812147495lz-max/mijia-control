#include <iostream>
#include <string>
#include <windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <limits>

#define BODY_COUNT 6

// 米家控制函数
void triggerMijia(const std::string& action) {
    std::cout << ">>> 触发米家指令: " << action << std::endl;

    std::string cmd = "python send_cmd.py ";
    if (action == "light") {
        cmd += "light";
    }
    else if (action == "open_curtain") {
        cmd += "curtain open";
    }
    else if (action == "close_curtain") {
        cmd += "curtain close";
    }
    else {
        return;
    }

    system(cmd.c_str());
}

int main() {
    IKinectSensor* pSensor = nullptr;
    IBodyFrameReader* pBodyReader = nullptr;
    IDepthFrameReader* pDepthReader = nullptr;
    ICoordinateMapper* pMapper = nullptr;

    HRESULT hr = GetDefaultKinectSensor(&pSensor);
    if (FAILED(hr) || !pSensor) {
        std::cerr << "未找到 Kinect 传感器！" << std::endl;
        return -1;
    }

    hr = pSensor->Open();
    if (FAILED(hr)) {
        std::cerr << "无法打开传感器！" << std::endl;
        return -1;
    }

    // 骨骼
    IBodyFrameSource* pBodySource = nullptr;
    pSensor->get_BodyFrameSource(&pBodySource);
    if (pBodySource) {
        pBodySource->OpenReader(&pBodyReader);
        pBodySource->Release();
    }

    // 深度流
    IDepthFrameSource* pDepthSource = nullptr;
    pSensor->get_DepthFrameSource(&pDepthSource);
    if (pDepthSource) {
        pDepthSource->OpenReader(&pDepthReader);
        pDepthSource->Release();
    }

    // 坐标映射
    pSensor->get_CoordinateMapper(&pMapper);

    std::cout << "Kinect C++ 骨骼监控版已启动！\n";
    std::cout << "右手举过头顶 → 开关灯\n";
    std::cout << "左手往右划 → 关窗帘\n左手往左划 → 开窗帘\nESC 退出\n";

    cv::namedWindow("Kinect Jarvis Skeleton Monitor", cv::WINDOW_NORMAL);
    cv::resizeWindow("Kinect Jarvis Skeleton Monitor", 1024, 848);  // 深度图放大2倍

    ULONGLONG lastActionTime = 0;
    const ULONGLONG COOLDOWN_MS = 2000;

    while (true) {
        if (cv::waitKey(1) == 27) break;  // ESC 退出

        cv::Mat display(424, 512, CV_8UC3, cv::Scalar(0, 0, 0));  // 默认黑背景

        // 深度帧作为背景
        IDepthFrame* pDepthFrame = nullptr;
        if (pDepthReader && pDepthReader->AcquireLatestFrame(&pDepthFrame) == S_OK) {
            UINT16* depthBuffer = new UINT16[512 * 424];
            pDepthFrame->CopyFrameDataToArray(512 * 424, depthBuffer);

            for (int y = 0; y < 424; ++y) {
                for (int x = 0; x < 512; ++x) {
                    UINT16 depth = depthBuffer[y * 512 + x];
                    UINT8 intensity = depth < 4500 ? (UINT8)(depth / 4500.0 * 255) : 0;
                    display.at<cv::Vec3b>(y, x) = cv::Vec3b(intensity, intensity, intensity);
                }
            }
            delete[] depthBuffer;
            pDepthFrame->Release();
        }

        // 放大显示
        cv::Mat resized;
        cv::resize(display, resized, cv::Size(1024, 848));

        // 骨骼帧
        bool tracked = false;
        IBodyFrame* pBodyFrame = nullptr;
        if (pBodyReader && pBodyReader->AcquireLatestFrame(&pBodyFrame) == S_OK) {
            IBody* ppBodies[BODY_COUNT] = { 0 };
            if (pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies) == S_OK) {
                for (int i = 0; i < BODY_COUNT; ++i) {
                    IBody* pBody = ppBodies[i];
                    if (pBody) {
                        BOOLEAN isTracked = false;
                        pBody->get_IsTracked(&isTracked);
                        if (isTracked) {
                            tracked = true;
                            Joint joints[JointType_Count];
                            pBody->GetJoints(JointType_Count, joints);

                            // 提取所有关节的 CameraSpacePoint
                            CameraSpacePoint cameraPoints[JointType_Count];
                            for (int j = 0; j < JointType_Count; ++j) {
                                cameraPoints[j] = joints[j].Position;
                            }

                            // 映射到深度空间
                            DepthSpacePoint depthPoints[JointType_Count];
                            hr = pMapper->MapCameraPointsToDepthSpace(JointType_Count, cameraPoints, JointType_Count, depthPoints);

                            if (SUCCEEDED(hr)) {
                                // 画关键点
                                auto drawJoint = [&](int type, cv::Scalar color) {
                                    DepthSpacePoint p = depthPoints[type];
                                    if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity()) {
                                        cv::circle(resized, cv::Point(cvRound(p.X * 2), cvRound(p.Y * 2)), 16, color, -1);
                                    }
                                    };

                                // 画线
                                auto drawLine = [&](int j1, int j2, cv::Scalar color) {
                                    DepthSpacePoint p1 = depthPoints[j1];
                                    DepthSpacePoint p2 = depthPoints[j2];
                                    if (p1.X != -std::numeric_limits<float>::infinity() && p2.X != -std::numeric_limits<float>::infinity()) {
                                        cv::line(resized, cv::Point(cvRound(p1.X * 2), cvRound(p1.Y * 2)),
                                            cv::Point(cvRound(p2.X * 2), cvRound(p2.Y * 2)), color, 8);
                                    }
                                    };

                                // 全身骨骼
                                drawJoint(JointType_Head, cv::Scalar(0, 255, 0));  // 头绿
                                drawJoint(JointType_HandRight, cv::Scalar(0, 0, 255));  // 右手红
                                drawJoint(JointType_HandLeft, cv::Scalar(255, 0, 0));  // 左手蓝

                                drawLine(JointType_Head, JointType_Neck, cv::Scalar(0, 255, 0));
                                drawLine(JointType_Neck, JointType_SpineShoulder, cv::Scalar(0, 255, 0));
                                drawLine(JointType_SpineShoulder, JointType_SpineMid, cv::Scalar(0, 255, 0));
                                drawLine(JointType_SpineMid, JointType_SpineBase, cv::Scalar(0, 255, 0));
                                drawLine(JointType_SpineBase, JointType_HipRight, cv::Scalar(0, 255, 0));
                                drawLine(JointType_SpineBase, JointType_HipLeft, cv::Scalar(0, 255, 0));
                                drawLine(JointType_SpineShoulder, JointType_ShoulderRight, cv::Scalar(0, 255, 0));
                                drawLine(JointType_ShoulderRight, JointType_ElbowRight, cv::Scalar(0, 255, 0));
                                drawLine(JointType_ElbowRight, JointType_WristRight, cv::Scalar(0, 255, 0));
                                drawLine(JointType_WristRight, JointType_HandRight, cv::Scalar(0, 255, 0));
                                drawLine(JointType_SpineShoulder, JointType_ShoulderLeft, cv::Scalar(0, 255, 0));
                                drawLine(JointType_ShoulderLeft, JointType_ElbowLeft, cv::Scalar(0, 255, 0));
                                drawLine(JointType_ElbowLeft, JointType_WristLeft, cv::Scalar(0, 255, 0));
                                drawLine(JointType_WristLeft, JointType_HandLeft, cv::Scalar(0, 255, 0));

                                // 手势逻辑（相机空间）+ 简化日志
                                CameraSpacePoint headPos = joints[JointType_Head].Position;
                                CameraSpacePoint handRightPos = joints[JointType_HandRight].Position;
                                CameraSpacePoint handLeftPos = joints[JointType_HandLeft].Position;
                                CameraSpacePoint spinePos = joints[JointType_SpineShoulder].Position;

                                float deltaX = handLeftPos.X - spinePos.X;

                                ULONGLONG currentTime = GetTickCount64();
                                if (currentTime - lastActionTime > COOLDOWN_MS) {
                                    // 右手举过头顶 → 开关灯
                                    if (handRightPos.Y > headPos.Y + 0.1f) {
                                        std::cout << "检测到右手举过头顶 → 触发开关灯" << std::endl;
                                        triggerMijia("light");
                                        lastActionTime = currentTime;
                                    }
                                    else {
                                        // 左手划动 → 窗帘
                                        if (deltaX > 0.1f) {  // 左手向右划 → 关窗帘（灵敏度调到0.1）
                                            std::cout << "检测到左手向右划 → 触发关窗帘" << std::endl;
                                            triggerMijia("close_curtain");
                                            lastActionTime = currentTime;
                                        }
                                        else if (deltaX < -0.3f) {  // 左手向左划 → 开窗帘
                                            std::cout << "检测到左手向左划 → 触发开窗帘" << std::endl;
                                            triggerMijia("open_curtain");
                                            lastActionTime = currentTime;
                                        }
                                    }
                                }
                            }
                        }
                        pBody->Release();
                    }
                }
            }
            pBodyFrame->Release();
        }

        // 状态文字
        std::string status = tracked ? "已锁定目标 (Tracking Active)" : "正在搜索人体 (Searching)...";
        cv::putText(resized, status, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1.5, tracked ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 100, 100), 3);

        cv::imshow("Kinect Jarvis Skeleton Monitor", resized);
        Sleep(30);
    }

    cv::destroyAllWindows();

    if (pDepthReader) pDepthReader->Release();
    if (pBodyReader) pBodyReader->Release();
    if (pMapper) pMapper->Release();
    if (pSensor) {
        pSensor->Close();
        pSensor->Release();
    }
    return 0;
}