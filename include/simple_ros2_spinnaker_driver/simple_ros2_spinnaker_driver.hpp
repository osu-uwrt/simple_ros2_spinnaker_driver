#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#define FEATURE_ENABLE (1)
#define FEATURE_DISABLE (0)

#define __DEBUG FEATURE_ENABLE


struct CameraConfig {
    std::string serial;
    bool isMaster;
};


struct PublisherConfig {
    std::string
        topic,
        frame;
};


class SpinnakerCamera {
    public:
    SpinnakerCamera(
        CameraConfig config,
        Spinnaker::CameraList cameras,
        std::shared_ptr<Spinnaker::ImageProcessor> postprocessor);
    
    void release();
    Spinnaker::ImagePtr getImage();

    private:
    const CameraConfig config;
    std::shared_ptr<Spinnaker::ImageProcessor> postprocessor;
    Spinnaker::CameraPtr ptr;
    Spinnaker::ImagePtr img;
};


class ImagePublisher {
    public:
    ImagePublisher(const PublisherConfig& config, image_transport::ImageTransport& it);
    void readAndPublish(rclcpp::Clock::SharedPtr clk);
    const PublisherConfig config();
    virtual void release() = 0;

    protected:
    /**
     * @brief Pack an image into a message
     * ALREADY POPULATED IN MSG:
     * header.stamp
     * header.frame_id
     * 
     * NEED TO PACK:
     * width
     * height
     * encoding
     * is_bigendian
     * step
     * data
     * 
     * @param img The image to pack
     */
    virtual void readImage(sensor_msgs::msg::Image& img) = 0;

    private:
    image_transport::Publisher pub;
    const PublisherConfig publisherConfig;
};


class MonoImagePublisher : public ImagePublisher {
    public:
    MonoImagePublisher(
        const PublisherConfig& publisherConfig,
        const CameraConfig& cameraConfig,
        image_transport::ImageTransport& it,
        Spinnaker::CameraList cameras,
        std::shared_ptr<Spinnaker::ImageProcessor> postproc);

    void release() override;

    protected:
    void readImage(sensor_msgs::msg::Image& msg) override;

    private:
    const CameraConfig cameraConfig;
    SpinnakerCamera camera;
};


class StereoImagePublisher : public ImagePublisher {
    public:
    StereoImagePublisher(
        const PublisherConfig& publisherConfig,
        const CameraConfig& leftConfig,
        const CameraConfig& rightConfig,
        image_transport::ImageTransport& it,
        Spinnaker::CameraList cameras,
        std::shared_ptr<Spinnaker::ImageProcessor> postproc);
    
    void release() override;

    protected:
    void readImage(sensor_msgs::msg::Image& msg) override;

    private:
    const CameraConfig
        leftConfig,
        rightConfig;
    
    SpinnakerCamera
        leftCamera,
        rightCamera;
};
