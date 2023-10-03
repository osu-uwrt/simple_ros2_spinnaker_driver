#include "simple_ros2_spinnaker_driver/simple_ros2_spinnaker_driver.hpp"
#include <sensor_msgs/image_encodings.hpp>

/**
 * Source file for the StereoImagePublisher class.
 */

StereoImagePublisher::StereoImagePublisher(
    const PublisherConfig& publisherConfig,
    const CameraConfig& leftConfig,
    const CameraConfig& rightConfig,
    image_transport::ImageTransport& it,
    Spinnaker::CameraList cameras,
    std::shared_ptr<Spinnaker::ImageProcessor> postproc)
: ImagePublisher(publisherConfig, it),
  leftConfig(leftConfig),
  rightConfig(rightConfig),
  leftCamera(SpinnakerCamera(leftConfig, cameras, postproc)),
  rightCamera(SpinnakerCamera(rightConfig, cameras, postproc))
{ }


void StereoImagePublisher::release()
{
    leftCamera.release();
    rightCamera.release();
}

void StereoImagePublisher::readImage(sensor_msgs::msg::Image& msg) 
{
    Spinnaker::ImagePtr 
        leftImage = leftCamera.getImage(),
        rightImage = rightCamera.getImage();
    
    //check that image sizes are the same. This doesnt work if they are not
    int
        leftWidth = leftImage->GetWidth(),
        leftHeight = leftImage->GetHeight(),
        rightWidth = rightImage->GetWidth(),
        rightHeight = rightImage->GetHeight();
    
    if(leftWidth != rightWidth || leftHeight != rightHeight)
    {
        std::string msg = "Image sizes are not the same! left is "
            + std::to_string(leftWidth) + "x" + std::to_string(leftHeight) + ", right is "
            + std::to_string(rightWidth) + "x" + std::to_string(rightHeight);
        throw std::runtime_error(msg);
    }

    //we publish the two images as one image (the two images vertically concatenated; left on top, right on bottom)
    //this works for us because we are not using stereo_image_proc and will ensure that the images come through image_transport in sync

    //pack ros msg
    msg.width = leftWidth;
    msg.height = leftHeight * 2; //=== to leftHeight + rightHeight
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    msg.is_bigendian = true;
    msg.step = msg.width * 3; //rgb8 so technically * 1

    //copy data into msg.
    //see https://github.com/ros-perception/vision_opencv/blob/humble/cv_bridge/src/cv_bridge.cpp#L389
    
    size_t sz = msg.step * leftHeight;
    msg.data.resize(sz * 2);
    memcpy(reinterpret_cast<char *>(&msg.data[0]), leftImage->GetData(), sz);
    memcpy(reinterpret_cast<char *>(&msg.data[sz]), rightImage->GetData(), sz);
}
