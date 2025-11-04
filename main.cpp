#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include <cmath>

#include "motor.h"
#include "servo.h"
#include "pid.h"
#include "pca9685_driver.h"

#include "timer.h"
#include "message.h"

#include "control.h"
#include "vision.h"

#include "globals.h"

#include <csignal>

// --- Streaming-related includes removed ---
// #include "httplib.h"
// #include "ts_queue.h"

// --- Streaming-related globals removed ---
// ThreadSafeQueue<cv::Mat> g_frame_queue;
// httplib::Server svr;
// ------------------

// --- streamer_loop() function removed ---

//Ctrl+C信号处理函数
void signal_handler(int signum) {
    allfinishflag.store(true);
}
// ------------------
int main() {
    // +++ ADDED TEST MODE SELECTION +++
    int mode = 3;
    bool motor_on = true; // Local flag to control motor_init()

    std::cout << "Please select a test mode:\n";
    std::cout << "  1: Motor OFF, Display Images ON (Set PID to 0,0,0 manually)\n";
    std::cout << "  2: Motor ON, Display Images OFF\n";
    std::cout << "  3: Motor ON, Display Images ON (Default)\n";
    std::cout << "Enter mode (1, 2, or 3): ";
    std::cin >> mode;

    switch (mode) {
    case 1:
        motor_on = false;
        ConeInformation.debug_enabled = true; // From vision.h
        std::cout << "[INFO] Mode 1 selected: Motor OFF, Display Images ON" << std::endl;
        std::cout << "[INFO] Please enter 0 0 0 for PID values to disable servo." << std::endl;
        break;
    case 2:
        motor_on = true;
        ConeInformation.debug_enabled = false; // From vision.h
        std::cout << "[INFO] Mode 2 selected: Motor ON, Display Images OFF" << std::endl;
        break;
    case 3:
    default:
        motor_on = true;
        ConeInformation.debug_enabled = true; // From vision.h
        std::cout << "[INFO] Mode 3 selected: Motor ON, Display Images ON" << std::endl;
        break;
    }
    // +++ END OF TEST MODE SELECTION +++
// (修改) PID 参数输入
    std::cout << "Please enter ORIGINAL PID parameters (Kp Ki Kd Limit): ";
    std::cin >> g_orig_kp >> g_orig_ki >> g_orig_kd >> g_orig_limit;

    // (NEW) Ask for cone PID parameters if motor is on
    if (motor_on) {
        std::cout << "Please enter CONE AVOIDANCE PID parameters (Kp Ki Kd Limit): ";
        std::cin >> g_cone_kp >> g_cone_ki >> g_cone_kd >> g_cone_limit;
    }
    else {
        // Motor is off, just use the first set for cone PID as well
        g_cone_kp = g_orig_kp;
        g_cone_ki = g_orig_ki;
        g_cone_kd = g_orig_kd;
        g_cone_limit = g_orig_limit;
    }

    std::cout << "[INFO] Original PID Limit: " << g_orig_limit << std::endl;
    std::cout << "[INFO] Cone PID Limit: " << g_cone_limit << std::endl;

    //初始化
    //gpio_init();
    signal(SIGINT, signal_handler);    // 注册信号处理函数：捕获 Ctrl+C
    if (!pca_init()) {
        std::cerr << "Failed to initialize PCA9685 driver!" << std::endl;
        return 1;
    }
    pca_set_pwm_freq(50.0);

    // +++ Conditionally initialize motor +++
    if (motor_on) {
        motor_init(); // Will unlock the motor
    }
    else {
        std::cout << "[INFO] motor_init() SKIPPED. Motor will remain off." << std::endl;
    }

    servo_init(105, 90, 120);
    pid_init(0, 0, 0, 15);//先给个0，0，0，15

    //测试
    VisionTaskState = State::ToBlueCone; // From vision.h
    send_message("Start");

    //初始化帧率检测进程
    std::thread thread3(timedelayIT);
    std::thread t_vision(vision_loop);    //视觉处理线程
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));//稳定图像
    std::cout << "<<<<<<<<<<<<<<<<<<<<To stabilize the image<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    // 正式初始化PID
    pid_init(g_orig_kp, g_orig_ki, g_orig_kd, g_orig_limit);
    std::thread t_control(control_loop_timer); //电机，舵机控制线程

    // --- Streaming thread removed ---
    // std::thread t_streamer(streamer_loop);

    // --- 等待退出信号 ---
    while (!allfinishflag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // --- 开始关闭过程 ---
    std::cout << "[INFO] Shutdown signal received." << std::endl;

    // --- Streaming server stop removed ---
    // if (svr.is_running()) { ... }

    // --- 回收线程 ---
    std::cout << "[INFO] Joining threads..." << std::endl;
    t_vision.join();
    std::cout << "[INFO] Vision thread joined." << std::endl;
    t_control.join();
    std::cout << "[INFO] Control thread joined." << std::endl;
    thread3.join();
    std::cout << "[INFO] Timer thread joined." << std::endl;

    // --- Streaming thread join removed ---
    // t_streamer.join();

    // ------------------

    std::cout << "[INFO] All threads joined. Releasing resources..." << std::endl;
    pca_close();
    //gpio_release();
    SaveResultsToCSV("lane_result.csv");
    std::cout << "[INFO] Program finished." << std::endl;
    return 0;
}