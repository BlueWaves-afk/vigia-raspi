#pragma once
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>

struct RtThreadConfig {
    int         sched_priority;  // SCHED_FIFO priority 1–99
    int         cpu_core;        // CPU affinity 0–3
    std::string name;            // pthread name (max 15 chars)
};

inline std::thread launch_rt_node(
    std::shared_ptr<rclcpp::Node> node,
    RtThreadConfig cfg)
{
    return std::thread([node, cfg]() {
        pthread_setname_np(pthread_self(), cfg.name.c_str());

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cfg.cpu_core, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            throw std::runtime_error("pthread_setaffinity_np failed for " + cfg.name);
        }

        sched_param sp{};
        sp.sched_priority = cfg.sched_priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            // Log warning but continue — may lack CAP_SYS_NICE in dev environment
            RCLCPP_WARN(node->get_logger(),
                "SCHED_FIFO not set for %s (need CAP_SYS_NICE or root). Running CFS.",
                cfg.name.c_str());
        }

        rclcpp::executors::StaticSingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });
}

inline rclcpp::NodeOptions make_ipc_options() {
    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    return opts;
}
