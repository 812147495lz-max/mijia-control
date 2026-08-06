# Kinect v2 + 米家手势控制项目

用 Kinect v2 实时骨骼追踪 + 手势识别，控制米家智能家居（窗帘/灯等）。完全本地运行，延迟低，隐私安全。

## 演示 / Demo
哔站【可能是国内社区第一个把kinect接入米家】 https://www.bilibili.com/video/BV1Rk6pBWEUp/?share_source=copy_web&vd_source=21da6774512a26622cbbdf0f7b761959
小红书【可能是国内社区第一个把kinect接米家 - 五等分的吴起 | 小红书 - 你的生活兴趣社区】 😆 y6wKdXwR3gZCGxl 😆 https://www.xiaohongshu.com/discovery/item/697eb022000000000c037d5c?source=webshare&xhsshare=pc_web&xsec_token=ABaxoTgBWaRSs_2IEDUYbbqv4n9tocE3eNK2DzG6XW6kk=&xsec_source=pc_share


## 功能 / Features
- 实时深度背景 + 全身骨骼可视化（OpenCV 绘制）
- 左手向右划 → 关窗帘
- 左手向左划 → 开窗帘
- 支持扩展更多设备/手势（空调、风扇等）

## 要求 / Requirements
- Windows 10/11
- Kinect for Windows SDK 2.0
- Visual Studio 2022 + OpenCV 4.x
- Python 3.11 + python-miio + python-dotenv

## 使用步骤 / Usage
1. 安装 Kinect SDK 2.0 和 OpenCV。
2. 配置 `.env` 文件（项目根目录，新建文本文件命名为 `.env`）：
3. token从token_extractor获取
4. 编译 C++ 项目（Release x64 模式）。
5. 把 `send_cmd.py` 放 exe 同目录。
6. 运行 exe（管理员身份），挥手测试！

## 其他 / others
代码里留了控制灯的代码，但是没弄好，就先留着

## 作者 / Author
@assummarry 

---

# Kinect v2 + MiHome Gesture Control Project

A real-time skeleton tracking + gesture recognition system using Kinect v2 to control MiHome smart devices (curtains, lights, etc.). Fully local, low latency, privacy-safe.

## Demo Screenshots
哔站【可能是国内社区第一个把kinect接入米家】 https://www.bilibili.com/video/BV1Rk6pBWEUp/?share_source=copy_web&vd_source=21da6774512a26622cbbdf0f7b761959
小红书【可能是国内社区第一个把kinect接米家 - 五等分的吴起 | 小红书 - 你的生活兴趣社区】 😆 y6wKdXwR3gZCGxl 😆 https://www.xiaohongshu.com/discovery/item/697eb022000000000c037d5c?source=webshare&xhsshare=pc_web&xsec_token=ABaxoTgBWaRSs_2IEDUYbbqv4n9tocE3eNK2DzG6XW6kk=&xsec_source=pc_share


## Features
- Real-time depth background + full-body skeleton visualization (OpenCV)
- Swipe left hand right → Close curtain
- Swipe left hand left → Open curtain
- Easy to extend for more devices/gestures

## Requirements
- Windows 10/11
- Kinect for Windows SDK 2.0
- Visual Studio 2022 + OpenCV 4.x
- Python 3 + python-miio + python-dotenv
- get tokens from token_extractor
- Build C++ project (Release x64).
- Place `send_cmd.py` in exe directory.
- Run exe (as administrator) and test gestures!

## Author
@assummarry
1. Install Kinect SDK and OpenCV.
2. Create `.env` file in project root:
