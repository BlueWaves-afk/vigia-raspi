#include <rclcpp/rclcpp.hpp>
#include <sys/mman.h>

#include "vigia_edge_node/rt_thread.hpp"
#include "camera_node.hpp"
#include "vision_node.hpp"
#include "depth_node.hpp"
#include "fusion_node.hpp"
#include "sensor_bridge_node.hpp"
#include "anti_death_node.hpp"
#if defined(VIGIA_HAVE_SDBUS)
#  include "ble_gatt_node.hpp"
#endif

int main(int argc, char * argv[])
{
    // Lock all current and future pages into RAM — eliminates page faults during emergency window.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "[vigia] WARNING: mlockall failed — page faults possible under load\n");
    }

    rclcpp::init(argc, argv);

    // All nodes in a single process — mandatory for rclcpp::intra_process_comm zero-copy.
    auto camera_node    = std::make_shared<CameraNode>(make_ipc_options());
    auto vision_node    = std::make_shared<VisionNode>(make_ipc_options());
    auto depth_node     = std::make_shared<DepthNode>(make_ipc_options());
    auto fusion_node    = std::make_shared<FusionNode>(make_ipc_options());
    auto bridge_node    = std::make_shared<SensorBridgeNode>(make_ipc_options());
    auto antideath_node = std::make_shared<AntiDeathNode>(make_ipc_options());
#if defined(VIGIA_HAVE_SDBUS)
    // BleGattNode runs its D-Bus loop on its own internal std::thread (SCHED_OTHER).
    // launch_rt_node is not used — BLE is best-effort, not RT.
    auto ble_node = std::make_shared<BleGattNode>(make_ipc_options());
#endif

    // Launch RT threads — highest priority first to ensure scheduler sees them in order.
    auto t5 = launch_rt_node(antideath_node, {99, 3, "vigia_antideath"});
    auto t4 = launch_rt_node(bridge_node,    {85, 3, "vigia_bridge"});
    auto t0 = launch_rt_node(camera_node,    {80, 0, "vigia_camera"});
    auto t1 = launch_rt_node(vision_node,    {75, 1, "vigia_vision"});
    auto t2 = launch_rt_node(depth_node,     {75, 2, "vigia_depth"});
    auto t3 = launch_rt_node(fusion_node,    {70, 3, "vigia_fusion"});
#if defined(VIGIA_HAVE_SDBUS)
    auto t6 = launch_rt_node(ble_node,       {40, 3, "vigia_ble"});
#endif

    t0.join(); t1.join(); t2.join();
    t3.join(); t4.join(); t5.join();
#if defined(VIGIA_HAVE_SDBUS)
    t6.join();
#endif

    rclcpp::shutdown();
    return 0;
}
