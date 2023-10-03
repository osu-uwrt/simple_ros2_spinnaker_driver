#include "simple_ros2_spinnaker_driver/simple_ros2_spinnaker_driver.hpp"

/**
 * Source file for the ImagePublisher class 
 */

ImagePublisher::ImagePublisher(const PublisherConfig& config, image_transport::ImageTransport& it)
 : publisherConfig(config)
{
    pub = it.advertise(publisherConfig.topic, 1);
}


const PublisherConfig ImagePublisher::config()
{
    return publisherConfig;
}


void ImagePublisher::readAndPublish(rclcpp::Clock::SharedPtr clk)
{
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = publisherConfig.frame;
    
    readImage(msg);

    msg.header.stamp = clk->now();
    pub.publish(msg);
}
