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
    if (!ptr)
    {
        std::string msg = "Camera with serial " + config.serial + " requested but not connected.";
        throw std::invalid_argument(msg);
    }

    ptr->Init();

    Spinnaker::GenApi::INodeMap &nodeMap = ptr->GetNodeMap();

    std::cout << nodeMap.GetDeviceName() << std::endl;

    // todo: set this on node map the "proper" way
    ptr->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

    Spinnaker::GenApi::CEnumerationPtr triggerMode = nodeMap.GetNode("TriggerMode");
    triggerMode->SetIntValue(triggerMode->GetEntryByName("On")->GetValue());

    Spinnaker::GenApi::CEnumerationPtr triggerSource = nodeMap.GetNode("TriggerSource");
    triggerSource->SetIntValue(triggerSource->GetEntryByName("Line0")->GetValue());

    if (config.isMaster)
    {

        Spinnaker::GenApi::CEnumerationPtr triggerSelector = nodeMap.GetNode("TriggerSelector");
        triggerSelector->SetIntValue(triggerSelector->GetEntryByName("FrameStart")->GetValue());
        Spinnaker::GenApi::CEnumerationPtr triggerActivation = nodeMap.GetNode("TriggerActivation");
        triggerActivation->SetIntValue(triggerActivation->GetEntryByName("RisingEdge")->GetValue());
        
        ptr->ExposureAuto.SetValue(Spinnaker::ExposureAutoEnums::ExposureAuto_Off);
        ptr->ExposureMode.SetValue(Spinnaker::ExposureModeEnums::ExposureMode_Timed);
        ptr->ExposureTime.SetValue(20000);

        Spinnaker::GenApi::CEnumerationPtr ptrTriggerOverlap = nodeMap.GetNode("TriggerOverlap");
        Spinnaker::GenApi::CEnumEntryPtr ptrTriggerOverlapReadOut = ptrTriggerOverlap->GetEntryByName("ReadOut");

        Spinnaker::GenApi::CEnumerationPtr ptrLineSelector = nodeMap.GetNode("LineSelector");
        Spinnaker::GenApi::CEnumEntryPtr ptrLineSelectorLine1 = ptrLineSelector->GetEntryByName("Line1");
        ptrLineSelector->SetIntValue(ptrLineSelectorLine1->GetValue());

        Spinnaker::GenApi::CEnumerationPtr ptrLineMode = nodeMap.GetNode("LineMode");
        Spinnaker::GenApi::CEnumEntryPtr ptrLineModeOutput = ptrLineMode->GetEntryByName("Output");
        ptrLineMode->SetIntValue(ptrLineModeOutput->GetValue());

        // Spinnaker::GenApi::CEnumerationPtr ptrLineSource = nodeMap.GetNode("LineSource");
        // Spinnaker::GenApi::CEnumEntryPtr ptrLineSourceExposureActive = ptrLineSource->GetEntryByName("ExposureActive");
        // ptrLineSource->SetIntValue(ptrLineSourceExposureActive->GetValue());
    }
    else
    {
        Spinnaker::GenApi::CEnumerationPtr ptrTriggerOverlap = nodeMap.GetNode("TriggerOverlap");
        Spinnaker::GenApi::CEnumEntryPtr ptrTriggerOverlapReadOut = ptrTriggerOverlap->GetEntryByName("ReadOut");

        Spinnaker::GenApi::CEnumerationPtr triggerActivation = nodeMap.GetNode("TriggerActivation");
        triggerActivation->SetIntValue(triggerActivation->GetEntryByName("RisingEdge")->GetValue());
    }

    if (!ptr->IsValid())
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

    // manually break reference to camera because it will not go out of scope
    // if this is not done, you will get an error at the end of execution saying that something still owns the camera
    ptr = nullptr;
}

Spinnaker::ImagePtr SpinnakerCamera::getImage()
{
    Spinnaker::ImagePtr nextImg = ptr->GetNextImage(1000);
    if (nextImg->IsIncomplete())
    {
        throw std::runtime_error(
            "Image is incomplete: " +
            std::string(Spinnaker::Image::GetImageStatusDescription(nextImg->GetImageStatus())));
    }

    // save as local member so ptr survives going out of scope
    img = postprocessor->Convert(nextImg, Spinnaker::PixelFormat_RGB8);

    // release image to avoid clogging up the buffer
    nextImg->Release();

    return img;
}
