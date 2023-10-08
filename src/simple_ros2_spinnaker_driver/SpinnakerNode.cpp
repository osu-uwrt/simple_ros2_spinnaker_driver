#include "simple_ros2_spinnaker_driver/simple_ros2_spinnaker_driver.hpp"

/**
 * ROS2 driver node to drive FLIR BlackFly camera with Spinnaker SDK
 */

using namespace std::chrono_literals;


class SpinnakerNode : public rclcpp::Node {
    public:
    SpinnakerNode()
     : rclcpp::Node("spinnaker_node")
    {
        postproc = std::make_shared<Spinnaker::ImageProcessor>();
        postproc->SetColorProcessing(Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
    }


    template<typename T>
    rclcpp::Parameter declareAndGetParameter(const std::string& name, const T& defaultValue)
    {
        if(!this->has_parameter(name))
        {
            this->declare_parameter(name, defaultValue);
        }

        return this->get_parameter(name);
    }


    CameraConfig readCameraConfigFromParams(const std::string& prefix)
    {
        CameraConfig config;
        config.serial = declareAndGetParameter<std::string>(prefix + ".serial", "").as_string();
        config.isMaster = declareAndGetParameter<bool>(prefix + ".hardware_trigger", "").as_bool();
        return config;
    }


    PublisherConfig readPublisherConfigFromParams(const std::string& prefix)
    {
        PublisherConfig config;
        config.topic = declareAndGetParameter<std::string>(prefix + ".topic", "").as_string();
        config.frame = declareAndGetParameter<std::string>(prefix + ".frame", "").as_string();
        return config;
    }


    bool isPublisherConfigDefined(const PublisherConfig& config)
    {
        return
            config.frame.length() > 0
            && config.topic.length() > 0;
    }


    bool isCameraConfigDefined(const CameraConfig& config)
    {
        return config.serial.length() > 0;
    }


    void init(image_transport::ImageTransport& it)
    {
        //initialize spinnaker and cameras
        RCLCPP_INFO(this->get_logger(), "Initializing Spinnaker");
        system = Spinnaker::System::GetInstance();
        cameras = system->GetCameras();

        RCLCPP_INFO(this->get_logger(), "Reading parameters");

        //dont need cam mutex to access because thread isnt started yet
        PublisherConfig pubConfig = readPublisherConfigFromParams("mono");
        if(isPublisherConfigDefined(pubConfig))
        {
            //properly defined mono config
            CameraConfig camConfig = readCameraConfigFromParams("mono.camera");
            if(isCameraConfigDefined(camConfig))
            {
                RCLCPP_INFO(this->get_logger(), "Initializing as MONO CAMERA. Serial#: %s", camConfig.serial.c_str());
                openedCam = std::make_shared<MonoImagePublisher>(pubConfig, camConfig, it, cameras, postproc);
                RCLCPP_INFO(this->get_logger(), "Opened camera with serial# %s", camConfig.serial.c_str());
            } else
            {
                throw std::runtime_error("Mono camera not properly defined in parameters!");
            }
        } else
        {   
            //improperly defined mono config, but maybe the user wants stereo
            pubConfig = readPublisherConfigFromParams("stereo");
            if(!isPublisherConfigDefined(pubConfig))
            {
                throw std::runtime_error("No mono or stereo config was properly defined in parameters!");
            }

            //properly defined stereo config
            CameraConfig
                leftCamConfig = readCameraConfigFromParams("stereo.left"),
                rightCamConfig = readCameraConfigFromParams("stereo.right");
            
            bool 
                leftCamDefined = isCameraConfigDefined(leftCamConfig),
                rightCamDefined = isCameraConfigDefined(rightCamConfig);

            if(leftCamDefined && rightCamDefined)
            {
                RCLCPP_INFO(this->get_logger(), "Initializing as STEREO CAMERA. Left serial#: %s, right serial#: %s", leftCamConfig.serial.c_str(), rightCamConfig.serial.c_str());
                openedCam = std::make_shared<StereoImagePublisher>(pubConfig, leftCamConfig, rightCamConfig, it, cameras, postproc);
                RCLCPP_INFO(this->get_logger(), "Opened camera with serial# %s as left camera", leftCamConfig.serial.c_str());
                RCLCPP_INFO(this->get_logger(), "Opened camera with serial# %s as right camera", rightCamConfig.serial.c_str());
            } else
            {
                std::string
                    leftStatus = (leftCamDefined ? "good" : "bad"),
                    rightStatus = (rightCamDefined ? "good" : "bad");
                
                throw std::runtime_error("Stereo config improperly defined! Left: " + leftStatus + ", right: " + rightStatus);
            }
        }

        thread = std::make_shared<std::thread>(
            std::bind(&SpinnakerNode::processCamerasAsync, this));

        RCLCPP_INFO(this->get_logger(), "Driver initialized");
    }


    void release()
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up");

        cameras.Clear();

        if(thread)
        {
            thread->join();
        }
        
        if(openedCam)
        {
            openedCam->release();
        }

        if(system)
        {
            system->ReleaseInstance();
        }
    }


    private:
    void processCamerasAsync()
    {
        //this vector will store indices of cameras to attempt deinitialization and reinitialization after an iteration of the loop
        std::vector<int> camerasToReinit; 

        while(rclcpp::ok())
        {
            //instruct all cameras to publish images captured during this trigger
            camMutex.lock();           
            try
            {
                openedCam->readAndPublish(this->get_clock());
            } catch(std::runtime_error& ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "std::runtime_error when updating %s: %s", openedCam->config().topic.c_str(), ex.what());
            } catch(Spinnaker::Exception& ex)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Spinnaker::Exception when updating %s: %s", openedCam->config().topic.c_str(), ex.what());
            }

            camMutex.unlock();
        }
    }

    std::shared_ptr<std::thread> thread = nullptr;

    Spinnaker::SystemPtr system = nullptr;
    Spinnaker::CameraList cameras;
    std::shared_ptr<ImagePublisher> openedCam = nullptr;
    std::shared_ptr<Spinnaker::ImageProcessor> postproc;

    std::mutex camMutex;
};


void listConnectedCameras()
{
    std::cout << "Initializing Spinnaker" << std::endl;
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraList cameras = system->GetCameras();

    //print information about the cameras
    const unsigned int numCameras = cameras.GetSize();
    std::cout << "Detected " << numCameras << " cameras." << std::endl;
    for(size_t i = 0; i < numCameras; i++)
    {
        Spinnaker::CameraPtr camera = cameras.GetByIndex(i);
        Spinnaker::GenApi::CStringPtr cStringSerial = camera->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
        if(Spinnaker::GenApi::IsReadable(cStringSerial))
        {
            Spinnaker::GenICam::gcstring cameraSerial = cStringSerial->GetValue();
            std::cout << "Device " << i << "'s serial#: " << cameraSerial.c_str() << std::endl;
        }
    }

    cameras.Clear();
    system->ReleaseInstance();
}


int main(int argc, char **argv)
{
    if(argc >= 2)
    {
        if(strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
        {
            std::cout << "Usage: ./SpinnakerNode [-h][--help] [-l][--list]\n";
            std::cout << "  Options:\n";
            std::cout << "   -h, --help: Display this help text and exit\n";
            std::cout << "   -l, --list: List connected cameras and their serial numbers and exit." << std::endl;
            return 0;
        } else if(strcmp(argv[1], "-l") == 0 || strcmp(argv[1], "--list") == 0)
        {
            listConnectedCameras();
            return 0;
        }
    }

    rclcpp::init(argc, argv);
    std::shared_ptr<SpinnakerNode> node = std::make_shared<SpinnakerNode>();
    image_transport::ImageTransport it(node);

    try 
    {
        node->init(it);
    } catch(std::runtime_error& ex)
    {
        RCLCPP_ERROR(node->get_logger(), ex.what());
        rclcpp::shutdown();
        node->release();
        exit(1);
    } catch(std::invalid_argument& ex)
    {
        RCLCPP_ERROR(node->get_logger(), ex.what());
        rclcpp::shutdown();
        node->release();
        exit(1);
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    node->release();
    return 0;
}
