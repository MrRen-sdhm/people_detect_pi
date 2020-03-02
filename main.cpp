// c++ standard library
#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <chrono>
#include <ctime>
#include <mutex>
// c standard library
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>
#include <cerrno>
// precompiled shared library
#include <opencv2/opencv.hpp>
#include <tbb/tbb_stddef.h>
#include <tbb/tbb.h>
#include <raspicam/raspicam.h>
#include <matio.h>
#include <pigpio.h>
// this project
#include "acf/ACFDetector.h"
#include "general/DetectionList.h"
#include "general/NonMaximumSuppression.h"

#include "control/InfraredRemote.h"
#include "control/Relay.h"
#include "control/LinpRemote.h"
#include "control/HumanInfrared.h"

static const int IMAGE_WIDTH = 960;
static const int IMAGE_HEIGHT = 720;
static const int STATUSBAR_HEIGHT = 20;
static const int WINDOW_WIDTH = 480;
static const int WINDOW_HEIGHT = 320;

bool no_window = false;
int score_threshold_low = 55;
int score_threshold_high = 70;
int distance_threshold = 40;
int dur_threshold_low = 3;
int RELAY_DELAY = 5;
int ir_threshold_light = 64;
int aircdt_open_delay = 10;
int aircdt_close_delay = 10;

static bool ImageReady = false;
static bool FakeVideoHasHuman = false;
static bool FakeVideoNoHuman = false;
static bool PauseFlag = false;
static bool ExitFlag = false;
static bool BlackScreen = false;

static bool CaptureThreadDone = false;
static bool ProcessThreadDone = false;

static std::mutex LastImage_Mutex;
static std::shared_ptr<uint8_t> LastImage;

static std::mutex DetectResult_Mutex;
static std::string DetectorInfo;
static DetectionList DetectResult;

typedef enum {
    VIDEO_NO_HUMAN,     // score < 50
    VIDEO_LIKE_HUMAN,   // 50 <= score < 70
    VIDEO_HAS_HUMAN,    // score >= 50 连续超过 3s
} VideoState_t;

static VideoState_t VideoState = VIDEO_NO_HUMAN;

typedef enum {
    STATE_NO_HUMAN,     // STATE_HAS_HUMAN状态下, 摄像头无人超时后, 进入该状态. 该状态全关
    STATE_CHECK_HUMAN, // STATE_NO_HUMAN状态下, 摄像头亮度低, 但人体红外热释电有信号时进入该状态. 该状态 灯亮空调关
    STATE_HAS_HUMAN,    // 任意状态下, 摄像头检测到有人则进入该状态. 该状态全开
} LightState_t;

static LightState_t LightState = STATE_NO_HUMAN;

typedef enum {
    AIRCDT_CLOSED, AIRCDT_DELAY_OPEN, AIRCDT_OPENED, AIRCDT_DELAY_CLOSE,
} AirConditionerState_t;

static AirConditionerState_t AirConditionerState = AIRCDT_CLOSED;
static std::chrono::steady_clock::time_point AirConditionerStateTimer;

void thread_func_capture() {
    CaptureThreadDone = false;

    // 初始化相机
    raspicam::RaspiCam Camera;
    Camera.setWidth(IMAGE_WIDTH);                             // 画面宽度
    Camera.setHeight(IMAGE_HEIGHT);                                // 画面高度
    Camera.setBrightness(60);                         // 提升20%亮度(50+10=60)
    Camera.setContrast(10);
    Camera.setSaturation(30);                            // 提升20%颜色饱和度(0+20%)
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_AUTO);
    Camera.setMetering(raspicam::RASPICAM_METERING_MATRIX);
//    Camera.setShutterSpeed(330000);
    Camera.setFrameRate(15);                          // FPS: 15
    Camera.setAWB(raspicam::RASPICAM_AWB_FLUORESCENT);
    Camera.setFormat(raspicam::RASPICAM_FORMAT_RGB);

    std::cout << "Camera opening..." << std::flush;
    if (!Camera.open()) {
        std::cout << "Fail" << std::endl;
        ExitFlag = true;
    } else {
        std::cout << "OK" << std::endl;
    }

    for (; !ExitFlag;) {
        // 获取图像并保存在缓冲区中
        if (Camera.grab()) {
            uint8_t *RawData = (uint8_t *) aligned_alloc(16,
                    Camera.getImageBufferSize());
            Camera.retrieve(RawData);

            LastImage_Mutex.lock();
            LastImage = std::shared_ptr<uint8_t>(RawData, free);
            LastImage_Mutex.unlock();

            ImageReady = true;
        }

        std::this_thread::yield();
    }

    Camera.release();
    CaptureThreadDone = true;
}

void thread_func_process() {
    ProcessThreadDone = false;

    try {
        // 初始化ACF检测器
        DetectResult_Mutex.lock();
        DetectorInfo = "Loading detector model...";
        DetectResult_Mutex.unlock();
        ACFDetector acf_detector("/home/pi/AcfHSMy18Detector.mat");

        for (; !ExitFlag;) {
            // 等待图像就绪
            if (!ImageReady) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::this_thread::yield();
                continue;
            }

            // 读取图像
            LastImage_Mutex.lock();
            std::shared_ptr<uint8_t> raw_data = LastImage;
            LastImage_Mutex.unlock();

            DetectionList dets, nms_dets;
            // 计算并显示耗时
            auto measure_time = std::chrono::steady_clock::now();

            // ACF目标检测
            dets = acf_detector.applyDetector(
                    cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, raw_data.get()));
            // 非极大值抑制
            nms_dets = NonMaximumSuppression::dollarNMS(dets);

            int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - measure_time).count();

            std::stringstream info;
            info << IMAGE_WIDTH << "x" << IMAGE_HEIGHT << " ";
            info << "ftr:" << std::setw(2) << acf_detector.calc_feature_ms << "ms ";
            info << "clf:" << std::setw(3) << acf_detector.apply_classifier_ms
                    << "ms ";
            info << "total:" << std::setw(3) << ms << "ms ";
            info << "nDet:" << std::setw(2) << dets.getSize() << " ";
            info << "nHS:" << nms_dets.getSize();

            DetectResult_Mutex.lock();
            DetectorInfo = info.str();
            DetectResult = nms_dets;
            DetectResult_Mutex.unlock();

            std::this_thread::yield();
        }
    } catch (const std::exception& err) {
        std::cout << "thread_func_process exit with exception: " << err.what() << std::endl;
    }

    ProcessThreadDone = true;
}

static void onMouseScreen(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (y < WINDOW_HEIGHT - STATUSBAR_HEIGHT) {
            // 点击屏幕中央 暂停
//            PauseFlag = !PauseFlag;
            if (x >= 0 && x < 50) {
                if (y >= 50 && y < 70) {
                    // 延时
                    if (x < 25) {
                        RELAY_DELAY += 1;
                    } else {
                        RELAY_DELAY -= 1;
                    }
                } else if (y >= 70 && y < 90) {
                    // 距离
                    if (x < 25) {
                        distance_threshold += 1;
                    } else {
                        distance_threshold -= 1;
                    }
                } else if (y >= 90 && y < 110) {
                    // 阈值高
                    if (x < 25) {
                        score_threshold_high += 1;
                    } else {
                        score_threshold_high -= 1;
                    }
                } else if (y >= 110 && y < 130) {
                    // 阈值低
                    if (x < 25) {
                        score_threshold_low += 1;
                    } else {
                        score_threshold_low -= 1;
                    }
                } else if (y >= 130 && y < 150) {
                    // 低阈值持续
                    if (x < 25) {
                        dur_threshold_low += 1;
                    } else {
                        dur_threshold_low -= 1;
                    }
                } else if (y >= 150 && y < 170) {
                    // 低阈值持续
                    if (x < 25) {
                        ir_threshold_light += 1;
                    } else {
                        ir_threshold_light -= 1;
                    }
                }
            } else if (x >= WINDOW_WIDTH - 15 && y <= 15) {
                // 切换黑屏模式
                BlackScreen = !BlackScreen;
            }

        } else {
            // 点击屏幕底部 退出
            PauseFlag = false;
            ExitFlag = true;
        }
    }
}

int main(int argc, char **argv) {

    int major, minor, release;
    Mat_GetLibraryVersion(&major, &minor, &release);
    std::cout << "matio version: " << major << '.' << minor << '.' << release
            << std::endl;

    std::cout << "tbb version: " << TBB_VERSION_MAJOR << '.'
            << TBB_VERSION_MINOR << std::endl;

    std::cout << "opencv version: " << CV_VERSION << std::endl;

    std::cout << "pigpio version: " << gpioVersion() << std::endl;
    std::cout << "pigpio hardware revision: " << gpioHardwareRevision()
            << std::endl;

    // 初始化采样线程
    std::cout << "Start capturing..." << std::flush;
    std::thread capture_thread(thread_func_capture);
    std::cout << "OK" << std::endl;

    // 初始化识别线程
    std::cout << "Start processing..." << std::flush;
    std::thread process_thread(thread_func_process);
    std::cout << "OK" << std::endl;

    try {
        int ret = gpioInitialise();
        if (ret < 0) {
            throw std::runtime_error(
                    std::string("PiGPIO init failed with code ")
                            + std::to_string(ret));
        }

        // 初始化领普无线远程控制
        LinpRemote linp_remote;
        std::cout << "linp ping..." << std::flush;
        if (linp_remote.ping(1000) == 0) {
            std::cout << "OK" << std::endl;
            std::string linp_remote_ver;
            std::cout << "linp receiver version: " << std::flush;
            linp_remote.read_fw_ver(linp_remote_ver);
            std::cout << linp_remote_ver << std::endl;
            linp_remote.set_switch(0x80003c32, false);
        } else {
            std::cout << "Failed" << std::endl;
        }

        // 初始化人体红外热释电
        HumanInfrared ir_human(22, true);

        // 初始化红外控制
        InfraredRemote ir_remote(27, true);

        // 初始化继电器控制逻辑
        bool is_relay_on = false;
        auto last_human = std::chrono::steady_clock::now()
                - std::chrono::seconds(RELAY_DELAY);
        auto last_no_human = std::chrono::steady_clock::now()
                - std::chrono::seconds(RELAY_DELAY);
        auto last_like_human = std::chrono::steady_clock::now()
                - std::chrono::seconds(RELAY_DELAY);
        Relay relay;

        // 初始化窗口
        const char WindowImage[] = "人体检测";
        if (!no_window) {
            std::cout << "Creating window..." << std::flush;
            cv::namedWindow(WindowImage, cv::WINDOW_AUTOSIZE);
            cv::moveWindow(WindowImage, -2, -30);
            cv::setMouseCallback(WindowImage, onMouseScreen, NULL);
            std::cout << "OK" << std::endl;
        }

        uint8_t *raw_data = NULL;
        std::string last_info;
        cv::Mat source;
        for (; !ExitFlag;) {
            // 固定窗口位置
            if (!no_window) {
                cv::moveWindow(WindowImage, -2, -30);
            }

            // 等待图像就绪
            if (!ImageReady) {
                if (!no_window) {
                    cv::waitKey(100);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            // 获取图像
            LastImage_Mutex.lock();
            std::shared_ptr<uint8_t> raw_data = LastImage;
            LastImage_Mutex.unlock();

            source = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3,
                    raw_data.get());

            // 计算图像整体亮度
            cv::Scalar avg = cv::mean(source);
            int brightness = (avg.val[0] + avg.val[1] + avg.val[2]) / 3;

            // 获取结果
            DetectResult_Mutex.lock();
            std::string info = DetectorInfo;
            DetectionList result = DetectResult;
            DetectResult_Mutex.unlock();

            // 打印状态信息
//            if (info != last_info && !info.empty()) {
//                std::cout << info << std::endl;
//                last_info = info;
//            }

            // 根据尺寸过滤结果
            result = result.filterSize(source.cols / (float) distance_threshold,
                    source.cols / (float) distance_threshold);

            // 计算最高得分
            float max_score = result.maxScore();

            switch (VideoState) {
            case VIDEO_NO_HUMAN:
                if (max_score >= score_threshold_high) {
                    VideoState = VIDEO_HAS_HUMAN;
                } else if (max_score >= score_threshold_low) {
                    last_like_human = std::chrono::steady_clock::now();
                    VideoState = VIDEO_LIKE_HUMAN;
                }
                break;
            case VIDEO_LIKE_HUMAN:
                if (max_score >= score_threshold_high) {
                    VideoState = VIDEO_HAS_HUMAN;
                } else if (max_score >= score_threshold_low) {
                    if (std::chrono::steady_clock::now() - last_like_human
                            > std::chrono::seconds(dur_threshold_low)) {
                        VideoState = VIDEO_HAS_HUMAN;
                    }
                } else {
                    VideoState = VIDEO_NO_HUMAN;
                }
                break;
            case VIDEO_HAS_HUMAN:
                if (max_score < score_threshold_low) {
                    VideoState = VIDEO_NO_HUMAN;
                }
                break;
            }

            if (FakeVideoHasHuman) {
                VideoState = VIDEO_HAS_HUMAN;
            } else if (FakeVideoNoHuman) {
                VideoState = VIDEO_NO_HUMAN;
            }

            if (VideoState == VIDEO_HAS_HUMAN) {
                last_human = std::chrono::steady_clock::now();
            } else {
                last_no_human = std::chrono::steady_clock::now();
            }
            auto dur_no_human = std::chrono::steady_clock::now() - last_human;
            auto dur_human = std::chrono::steady_clock::now() - last_no_human;

            // 灯光控制逻辑
            switch (LightState) {
            case STATE_NO_HUMAN:
                if (VideoState == VIDEO_HAS_HUMAN) {
                    LightState = STATE_HAS_HUMAN;
                    // turn on relay
                    relay.set(true);
                    // turn on Linp remote relay
                    linp_remote.set_switch(0x80003c32, true);
                    std::cout << "Light ON" << std::endl;
                } else if (ir_human.get() && brightness < ir_threshold_light) {
                    LightState = STATE_CHECK_HUMAN;
                    // turn on relay
                    relay.set(true);
                    // turn on Linp remote relay
                    linp_remote.set_switch(0x80003c32, true);
                    std::cout << "Light ON" << std::endl;
                }
                break;
            case STATE_CHECK_HUMAN:
                if (VideoState == VIDEO_HAS_HUMAN) {
                    LightState = STATE_HAS_HUMAN;
                    std::cout << "Light ON" << std::endl;
                } else if (!ir_human.get()) {
                    LightState = STATE_NO_HUMAN;
                    // turn off relay
                    relay.set(false);
                    // turn off Linp remote relay
                    linp_remote.set_switch(0x80003c32, false);
                    std::cout << "Light OFF" << std::endl;
                }
                break;
            case STATE_HAS_HUMAN:
                if (dur_no_human > std::chrono::seconds(RELAY_DELAY)
                        && !ir_human.get()) {
                    LightState = STATE_NO_HUMAN;
                    // turn off relay
                    relay.set(false);
                    // turn off Linp remote relay
                    linp_remote.set_switch(0x80003c32, false);
                    std::cout << "Light OFF" << std::endl;
                }
                break;
            }

            // 空调控制逻辑
            switch (AirConditionerState) {
            case AIRCDT_CLOSED:
                if (LightState == STATE_HAS_HUMAN) {
                    AirConditionerStateTimer = std::chrono::steady_clock::now();
                    AirConditionerState = AIRCDT_DELAY_OPEN;
                    std::cout << "[AIRCDT_DELAY_OPEN]" << std::endl;
                }
                break;
            case AIRCDT_DELAY_OPEN:
                if (LightState != STATE_HAS_HUMAN) {
                    AirConditionerState = AIRCDT_CLOSED;
                    std::cout << "[AIRCDT_CLOSED]" << std::endl;
                } else if (std::chrono::steady_clock::now()
                        - AirConditionerStateTimer
                        > std::chrono::seconds(aircdt_open_delay)) {
                    AirConditionerState = AIRCDT_OPENED;
                    std::cout << "[AIRCDT_OPENED] Air conditioner ON"
                            << std::endl;
                    ir_remote.set_power(ir_remote.POWER_ON);
                    ir_remote.send();
                }
                break;
            case AIRCDT_OPENED:
                if (LightState != STATE_HAS_HUMAN) {
                    AirConditionerStateTimer = std::chrono::steady_clock::now();
                    AirConditionerState = AIRCDT_DELAY_CLOSE;
                    std::cout << "[AIRCDT_DELAY_CLOSE]" << std::endl;
                }
                break;
            case AIRCDT_DELAY_CLOSE:
                if (LightState == STATE_HAS_HUMAN) {
                    AirConditionerState = AIRCDT_OPENED;
                    std::cout << "[AIRCDT_OPENED]" << std::endl;
                } else if (std::chrono::steady_clock::now()
                        - AirConditionerStateTimer
                        > std::chrono::seconds(aircdt_close_delay)) {
                    AirConditionerState = AIRCDT_CLOSED;
                    std::cout << "[AIRCDT_CLOSED] Air conditioner OFF"
                            << std::endl;
                    ir_remote.set_power(ir_remote.POWER_OFF);
                    ir_remote.send();
                }
                break;
            }

//            if (!is_relay_on && dur_human >= OPEN_DELAY && dur_no_human <= std::chrono::seconds(RELAY_DELAY)) {
//
//                is_relay_on = true;
//                std::cout << "Relay on" << std::endl;
//            } else if (is_relay_on && dur_no_human > std::chrono::seconds(RELAY_DELAY)) {
//
//                is_relay_on = false;
//                std::cout << "Relay off" << std::endl;
//            }

            // 显示画面
            if (!no_window) {
                // 绘制界面
                cv::Mat show = cv::Mat(IMAGE_HEIGHT + 20, IMAGE_WIDTH, CV_8UC3);
                cv::resize(source, show, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));

                // 绘制画面亮度
                cv::putText(show, std::to_string(brightness),
                        cv::Point(WINDOW_WIDTH - 200, 50), 1, 2,
                        cv::Scalar(128, 255, 128), 2);

                // 绘制红外热释电状态
                if (ir_human.get()) {
                    cv::circle(show, cv::Point(30, 30), 15,
                            cv::Scalar(128, 128, 255),
                            CV_FILLED);
                }

                // 绘制参数栏
                cv::putText(show, "[+] [-] tout=" + std::to_string(RELAY_DELAY),
                        cv::Point(0, 70), 1, 1, cv::Scalar(255, 64, 255), 1);
                cv::putText(show,
                        "[+] [-] dist=" + std::to_string(distance_threshold),
                        cv::Point(0, 90), 1, 1, cv::Scalar(255, 64, 255), 1);
                cv::putText(show,
                        "[+] [-] thrH=" + std::to_string(score_threshold_high),
                        cv::Point(0, 110), 1, 1, cv::Scalar(255, 64, 255), 1);
                cv::putText(show,
                        "[+] [-] thrL=" + std::to_string(score_threshold_low),
                        cv::Point(0, 130), 1, 1, cv::Scalar(255, 64, 255), 1);
                cv::putText(show,
                        "[+] [-] delay=" + std::to_string(dur_threshold_low),
                        cv::Point(0, 150), 1, 1, cv::Scalar(255, 64, 255), 1);
                cv::putText(show,
                        "[+] [-] thrIR=" + std::to_string(ir_threshold_light),
                        cv::Point(0, 170), 1, 1, cv::Scalar(255, 64, 255), 1);

                // 绘制头肩检测情况指示灯
                if (VideoState == VIDEO_HAS_HUMAN) {
                    cv::circle(show, cv::Point(80, 30), 15,
                            cv::Scalar(128, 255, 128),
                            CV_FILLED);
                } else if (VideoState == VIDEO_LIKE_HUMAN) {
                    cv::circle(show, cv::Point(80, 30), 15,
                            cv::Scalar(128, 255, 255),
                            CV_FILLED);
                }

                // 绘制人数
                result.resizeDetections(WINDOW_WIDTH / (float) source.cols,
                        WINDOW_HEIGHT / (float) source.rows);
                int count_good = result.Draw(show, 130);
                if (count_good) {
                    std::stringstream num;
                    num << count_good;
                    cv::putText(show, num.str(),
                            cv::Point(WINDOW_WIDTH - 50, 50), 1, 3,
                            cv::Scalar(128, 255, 128), 3);
                }

                // 绘制状态栏
                cv::Mat statusBar(cv::Size(WINDOW_WIDTH, STATUSBAR_HEIGHT),
                        CV_8UC3, cv::Scalar(255, 255, 255));
                if (LightState == STATE_HAS_HUMAN) {
                    cv::rectangle(statusBar, cv::Point(0, 0),
                            cv::Point(
                                    static_cast<int>(WINDOW_WIDTH
                                            - WINDOW_WIDTH * dur_no_human
                                                    / std::chrono::seconds(
                                                            RELAY_DELAY)),
                                    STATUSBAR_HEIGHT),
                            cv::Scalar(128, 255, 128),
                            CV_FILLED);
                }
                cv::putText(statusBar, info, cv::Point(0, statusBar.rows - 5),
                        1, 1, cv::Scalar(0, 0, 0));
                statusBar.copyTo(
                        show(
                                cv::Rect(0, show.rows - statusBar.rows,
                                        statusBar.cols, statusBar.rows)));

                // 黑屏模式
                if (BlackScreen) {
                    show.setTo(cv::Scalar(0, 0, 0));
                }

                // 绘制黑屏按钮
                cv::rectangle(show,
                        cv::Rect(cv::Point(WINDOW_WIDTH - 15, 0),
                                cv::Size(15, 15)), cv::Scalar(64, 64, 64));

                cv::imshow(WindowImage, show);
                int key_pressed = cv::waitKey(200);

                if ((uint8_t) key_pressed - (uint8_t) 'v' == 0) {
                    FakeVideoNoHuman = false;
                    FakeVideoHasHuman = !FakeVideoHasHuman;
                    std::cout << "FakeVideoHasHuman = " << FakeVideoHasHuman
                            << std::endl;
                } else if ((uint8_t) key_pressed - (uint8_t) 'b' == 0) {
                    FakeVideoHasHuman = false;
                    FakeVideoNoHuman = !FakeVideoNoHuman;
                    std::cout << "FakeVideoNoHuman = " << FakeVideoNoHuman
                            << std::endl;
                }

                if (PauseFlag) {
                    cv::setWindowTitle(WindowImage, "人体检测[暂停]");

                    // 保存界面截图
                    char time_str[30];
                    std::time_t t = std::chrono::system_clock::to_time_t(
                            std::chrono::system_clock::now());
                    std::strftime(time_str, sizeof(time_str),
                            "capture_%Y%m%d_%H%M%S.png", std::localtime(&t));
                    std::string folder_path = "/home/pi/acf_detector_capture/";
                    std::string save_path = folder_path + std::string(time_str);
                    std::cout << "Save capture to " + save_path + "..."
                            << std::flush;
                    // 所有用户可读写模式, 创建文件夹
                    if (mkdir(folder_path.c_str(), ALLPERMS)
                            != 0&& errno != EEXIST) {
                        std::cout << "cannot create folder " << folder_path
                                << ": " << strerror(errno) << std::endl;
                    } else {
                        try {
                            if (cv::imwrite(save_path, show)) {
                                std::cout << "OK" << std::endl;
                            } else {
                                std::cout << "Fail" << std::endl;
                            }
                        } catch (std::runtime_error& ex) {
                            std::cout << "Fail " << ex.what() << std::endl;
                        }
                    }

                    while (PauseFlag) {
                        cv::waitKey(1);
                    }
                    cv::setWindowTitle(WindowImage, "人体检测");
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }

        }
        gpioTerminate();
    } catch (const std::exception &err) {
        std::cout << "Exit with exception: " << err.what() << std::endl;
    }

    ExitFlag = true;
    if (!no_window) {
        cv::waitKey(500);
        cv::destroyAllWindows();
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

// wait thread finish
    std::cout << "Waiting thread finish..." << std::endl;
    while (!CaptureThreadDone || !ProcessThreadDone) {
        if (!no_window) {
            cv::waitKey(100);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    std::cout << "All thread finished" << std::endl;
    return 0;
}
