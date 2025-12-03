#include <glog/logging.h>  // 添加glog头文件
#include <ros/ros.h>
#include <unistd.h>
#include <csignal>
#include <execinfo.h>

#include "d_point_lio/laser_mapping.h"

using namespace d_point_lio;

std::string BacktracePrint() {
    void* array[20];
    int size = backtrace(array, 20);
    char** strings = backtrace_symbols(array, size);
    std::string result;
    for (int i = 0; i < size; i++) {
        result += strings[i];
    }
    free(strings);
    return result;
}

void SignalHandler(int sig) {
    // std::cout << "Recieve signal [" << sig << "]" << std::endl;
    ROS_INFO("Recieve signal [%d]", sig);
    if (sig == SIGSEGV) {
        ROS_INFO("%s", BacktracePrint().c_str());
    }
    exit(EXIT_FAILURE);
    return;
}

void RegisterSignal() {
    signal(SIGTTIN, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT,  SignalHandler);
    signal(SIGTERM, SignalHandler);
    signal(SIGQUIT, SignalHandler);
    // signal(SIGSEGV, SignalHandler);
    return;
}

int main(int argc, char **argv) {
    const char *home_dir = getenv("HOME");  // Linux/Mac
    std::string log_dir = std::string(home_dir) + "/tmp/log";
    // 初始化glog
    FLAGS_log_dir = log_dir;
    FLAGS_minloglevel = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_max_log_size = 5;  // 单个日志文件大小上限（MB）, 如果设置为0将默认为1
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter([](const char *data, int size) { std::cout << std::string(data, size); });

    RegisterSignal();

    ros::init(argc, argv, "point_lio");
    ros::NodeHandle nh;

    auto laser_mapping = std::make_shared<d_point_lio::LaserMapping>();

    if (!laser_mapping->InitROS(nh)) {
        ROS_ERROR("Failed to initialize LaserMapping");
        return -1;
    }

    ROS_INFO("Point-LIO initialized successfully, starting processing...");

    // Main processing loop (faster-lio style)
    ros::Rate rate(200);  // 200Hz processing frequency
    while (ros::ok()) {
        ros::spinOnce();

        // Process sensor data
        laser_mapping->Run();

        rate.sleep();
    }

    ROS_INFO("Point-LIO processing finished, saving final map...");

    // Save final map and cleanup
    laser_mapping->Finish();

    // 关闭glog
    google::ShutdownGoogleLogging();
    return 0;
}
