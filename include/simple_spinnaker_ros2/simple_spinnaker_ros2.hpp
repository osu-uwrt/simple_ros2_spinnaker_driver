#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#define FEATURE_ENABLE (1)
#define FEATURE_DISABLE (0)

#define __DEBUG FEATURE_ENABLE

class SpinnakerCamera {
    public:
    SpinnakerCamera(
        image_transport::ImageTransport& it, 
        const std::string& topic,
        const std::string& frame,
        const std::string& serial,
        bool hardwareTrigger,
        Spinnaker::CameraList cameras,
        std::shared_ptr<Spinnaker::ImageProcessor> postprocessor);
    
    void init();
    void reInit();
    void release();
    void readAndPublish(rclcpp::Time now);

    private:
    const std::string 
        topic, 
        frame,
        serial;
    const bool hardwareTrigger;
    image_transport::Publisher pub;
    Spinnaker::CameraPtr ptr;
    Spinnaker::CameraList cameras;
    std::shared_ptr<Spinnaker::ImageProcessor> postprocessor;
};
