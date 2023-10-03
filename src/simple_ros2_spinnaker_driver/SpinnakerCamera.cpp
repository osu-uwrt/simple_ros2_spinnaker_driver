#include "simple_ros2_spinnaker_driver/simple_ros2_spinnaker_driver.hpp"

/**
 * Source file for the SpinnakerCamera class.
 */

SpinnakerCamera::SpinnakerCamera(
    CameraConfig config,
    Spinnaker::CameraList cameras,
    std::shared_ptr<Spinnaker::ImageProcessor> postprocessor)
 : config(config),
   postprocessor(postprocessor)
{
    ptr = cameras.GetBySerial(config.serial);
    if(!ptr)
    {
        std::string msg = "Camera with serial " + config.serial + " requested but not connected.";
        throw std::invalid_argument(msg);
    }

    ptr->Init();
    ptr->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
    ptr->ExposureMode.SetValue(Spinnaker::ExposureMode_Timed);
    ptr->ExposureTime.SetValue(30000);

    //todo: set this on node map the "proper" way
    ptr->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

    if(config.hardwareTrigger)
    {
        ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_On);
        ptr->TriggerOverlap.SetValue(Spinnaker::TriggerOverlap_ReadOut);
    } else 
    {
        ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
    }
    
    if(!ptr->IsValid())
    {
        throw std::runtime_error("Camera not valid! (camera->IsValid() != true)");
    }

    ptr->BeginAcquisition();

    img = nullptr;
}


void SpinnakerCamera::release()
{
    ptr->EndAcquisition();
    ptr->DeInit();

    //manually break reference to camera because it will not go out of scope
    //if this is not done, you will get an error at the end of execution saying that something still owns the camera
    ptr = nullptr;
}


Spinnaker::ImagePtr SpinnakerCamera::getImage()
{
    Spinnaker::ImagePtr nextImg = ptr->GetNextImage(1000);
    if(nextImg->IsIncomplete())
    {
        throw std::runtime_error(
            "Image is incomplete: " + 
                std::string(Spinnaker::Image::GetImageStatusDescription(nextImg->GetImageStatus()))
        );
    }

    //save as local member so ptr survives going out of scope
    img = postprocessor->Convert(nextImg, Spinnaker::PixelFormat_RGB8);

    //release image to avoid clogging up the buffer
    nextImg->Release();

    return img;
}
