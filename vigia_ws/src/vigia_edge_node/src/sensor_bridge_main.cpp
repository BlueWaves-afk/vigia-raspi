/** Standalone SensorBridgeNode — Phase 2 Pico COBS ingest without full edge stack. */
#include <rclcpp/rclcpp.hpp>

#include "sensor_bridge_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorBridgeNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
