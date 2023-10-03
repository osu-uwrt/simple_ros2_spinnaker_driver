#include "simple_ros2_spinnaker_driver/simple_ros2_spinnaker_driver.hpp"
#include <sensor_msgs/image_encodings.hpp>

/**
 * Source file for the MonoImagePublisher class.
 */

MonoImagePublisher::MonoImagePublisher(
    const PublisherConfig& publisherConfig,
    const CameraConfig& cameraConfig,
    image_transport::ImageTransport& it,
    Spinnaker::CameraList cameras,
    std::shared_ptr<Spinnaker::ImageProcessor> postproc)
: ImagePublisher(publisherConfig, it),
  cameraConfig(cameraConfig),
  camera(SpinnakerCamera(cameraConfig, cameras, postproc))
{ }


void MonoImagePublisher::release() 
{
    camera.release();
}


void MonoImagePublisher::readImage(sensor_msgs::msg::Image& msg) 
{
    Spinnaker::ImagePtr image = camera.getImage();

    //pack ros msg
    msg.width = image->GetWidth();
    msg.height = image->GetHeight();
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    msg.is_bigendian = true;
    msg.step = msg.width * 3; //rgb8 so technically * 1

    //copy data into msg.
    //see https://github.com/ros-perception/vision_opencv/blob/humble/cv_bridge/src/cv_bridge.cpp#L389
    size_t sz = msg.step * msg.height;
    msg.data.resize(sz);
    memcpy(reinterpret_cast<char *>(&msg.data[0]), image->GetData(), sz);
}
