#include "simple_spinnaker_ros2/simple_spinnaker_ros2.hpp"
#include <sensor_msgs/image_encodings.hpp>

/**
 * Source file for the SpinnakerCamera class.
 */

SpinnakerCamera::SpinnakerCamera(
    image_transport::ImageTransport& it,
    const std::string& topic,
    const std::string& frame,
    const std::string& serial,
    bool hardwareTrigger,
    Spinnaker::CameraList cameras,
    std::shared_ptr<Spinnaker::ImageProcessor> postprocessor)
 : topic(topic),
   frame(frame),
   serial(serial),
   hardwareTrigger(hardwareTrigger),
   cameras(cameras),
   postprocessor(postprocessor)
{
    pub = it.advertise(topic, 1);

    ptr = cameras.GetBySerial(serial);
    if(!ptr)
    {
        std::string msg = "Camera with serial " + serial + " requested but not connected.";
        throw std::invalid_argument(msg);
    }
}


void SpinnakerCamera::init()
{
    ptr->Init();
    ptr->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
    ptr->ExposureMode.SetValue(Spinnaker::ExposureMode_Timed);
    ptr->ExposureTime.SetValue(30000);

    //todo: set this on node map the "proper" way
    ptr->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

    if(hardwareTrigger)
    {
        ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_On);
        ptr->TriggerOverlap.SetValue(Spinnaker::TriggerOverlap_ReadOut);
    }
    
    if(!ptr->IsValid())
    {
        throw std::runtime_error("Camera not valid! (camera->IsValid() != true)");
    }

    ptr->BeginAcquisition();
}


void SpinnakerCamera::reInit()
{
    release();
    
    ptr = cameras.GetBySerial(serial);
    if(!ptr)
    {
        std::string msg = "Camera with serial " + serial + " requested but not connected.";
        throw std::runtime_error(msg);
    }
}


void SpinnakerCamera::release()
{
    ptr->EndAcquisition();
    ptr->DeInit();

    //manually break reference to camera because it will not go out of scope
    //if this is not done, you will get an error at the end of execution saying that something still owns the camera
    ptr = nullptr;
}


void SpinnakerCamera::readAndPublish(rclcpp::Time now)
{
    Spinnaker::ImagePtr nextImg = ptr->GetNextImage(1000);
    if(nextImg->IsIncomplete())
    {
        throw std::runtime_error(
            "Image for \"" + topic + "\" incomplete: " + 
                std::string(Spinnaker::Image::GetImageStatusDescription(nextImg->GetImageStatus()))
        );
    }

    Spinnaker::ImagePtr converted = postprocessor->Convert(nextImg, Spinnaker::PixelFormat_RGB8);

    //pack ros msg
    sensor_msgs::msg::Image rosmsg;
    rosmsg.header.stamp = now;
    rosmsg.header.frame_id = frame;
    rosmsg.width = converted->GetWidth();
    rosmsg.height = converted->GetHeight();
    rosmsg.encoding = sensor_msgs::image_encodings::RGB8;
    rosmsg.is_bigendian = true;
    rosmsg.step = rosmsg.width * 3; //rgb8 so technically * 1

    //copy data into msg.
    //see https://github.com/ros-perception/vision_opencv/blob/humble/cv_bridge/src/cv_bridge.cpp#L389
    size_t sz = rosmsg.step * rosmsg.height;
    rosmsg.data.resize(sz);
    memcpy(reinterpret_cast<char *>(&rosmsg.data[0]), converted->GetData(), sz);

    //now publish
    pub.publish(rosmsg);

    //release image to avoid clogging up the buffer
    nextImg->Release();
}
