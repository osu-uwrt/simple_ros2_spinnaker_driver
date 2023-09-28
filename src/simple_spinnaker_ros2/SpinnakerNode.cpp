#include "simple_spinnaker_ros2/simple_spinnaker_ros2.hpp"

/**
 * ROS2 driver node to drive FLIR BlackFly camera with Spinnaker SDK
 */

using namespace std::chrono_literals;

const char *TMP_SERIAL = "14432788";

struct CameraConfig {
    std::string
        topic,
        frame,
        serial;
};

struct SpinnakerCameraAndConfig {
    CameraConfig config;
    std::shared_ptr<SpinnakerCamera> camera;
};

class SpinnakerNode : public rclcpp::Node {
    public:
    SpinnakerNode()
     : rclcpp::Node("spinnaker_node")
    {
        postproc = std::make_shared<Spinnaker::ImageProcessor>();
        postproc->SetColorProcessing(Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);        
    }


    CameraConfig readParametersForCamera(int idx)
    {
        std::string cameraName = "camera" + std::to_string(idx);

        CameraConfig config;
        config.topic = this->get_parameter(cameraName + ".left").as_string();
        config.frame = this->get_parameter(cameraName + ".frame").as_string(); 
        config.serial = this->get_parameter(cameraName + ".serial").as_string();

        return config;
    }


    std::list<CameraConfig> readParameters()
    {
        RCLCPP_INFO(this->get_logger(), "Reading Parameters");
        std::list<CameraConfig> cameraConfigs;

        //declare initial params
        this->declare_parameter("camera0.left", "");
        this->declare_parameter("camera0.frame", "");
        this->declare_parameter("camera0.serial", "");
        
        int cameraId = 0;
        CameraConfig config = readParametersForCamera(cameraId);
        while(
            config.frame.length() > 0
            && config.topic.length() > 0
            && config.serial.length() > 0)
        {
            cameraConfigs.push_back(config);
            cameraId++;
            std::string cameraName = "camera" + std::to_string(cameraId);
            this->declare_parameter(cameraName + ".left", "");
            this->declare_parameter(cameraName + ".frame", ""); 
            this->declare_parameter(cameraName + ".serial", "");
            config = readParametersForCamera(cameraId);
        }
        
        //TODO: read params here
        CameraConfig tmp;
        tmp.topic = "left";
        tmp.frame = "tmp/left";
        tmp.serial = TMP_SERIAL;
        cameraConfigs.push_back(tmp);
        return cameraConfigs;
    }


    void init(image_transport::ImageTransport& it, std::list<CameraConfig> cameraConfigs)
    {
        //initialize spinnaker and cameras
        RCLCPP_INFO(this->get_logger(), "Initializing Spinnaker");
        system = Spinnaker::System::GetInstance();
        cameras = system->GetCameras();

        RCLCPP_INFO(this->get_logger(), "Initializing cameras");

        camMutex.lock();
        for(CameraConfig config : cameraConfigs)
        {
            try 
            {
                SpinnakerCameraAndConfig newCamera;
                newCamera.config = config;
                newCamera.camera = std::make_shared<SpinnakerCamera>(it, config.topic, config.frame, cameras, config.serial, postproc);
                newCamera.camera->init();
                openedCams.push_back(newCamera);

                RCLCPP_INFO(this->get_logger(), "Opened camera with serial# %s as \"%s\"", config.serial.c_str(), config.topic.c_str());
            } catch(std::invalid_argument& ex)
            {
                RCLCPP_ERROR(this->get_logger(), "FAILED to open camera with serial# %s (named \"%s\")", config.serial.c_str(), config.topic.c_str());
            }
        }

        camMutex.unlock();

        thread = std::make_shared<std::thread>(
            std::bind(&SpinnakerNode::processCamerasAsync, this));

        RCLCPP_INFO(this->get_logger(), "Driver Initialized");
    }


    void release()
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up");

        thread->join();
        for(SpinnakerCameraAndConfig camera : openedCams)
        {
            camera.camera->release();
        }

        cameras.Clear();
        system->ReleaseInstance();
    }


    private:
    void processCamerasAsync()
    {
        //this vector will store indices of cameras to attempt deinitialization and reinitialization after an iteration of the loop
        std::vector<int> camerasToReinit; 

        while(rclcpp::ok())
        {
            //TODO: implement synchronous trigger

            //instruct all cameras to publish images captured during this trigger
            rclcpp::Time now = this->get_clock()->now();
            for(size_t i = 0; i < openedCams.size(); i++)
            {
                try
                {
                    openedCams[i].camera->readAndPublish(now);
                } catch(std::runtime_error& ex)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "std::runtime_error: %s", ex.what());
                } catch(Spinnaker::Exception& ex)
                {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Spinnaker::Exception: %s", ex.what());
                    camerasToReinit.push_back(i);
                }
            }

            //attempt to reinitialize cameras
            while(camerasToReinit.size() > 0)
            {   
                try
                {
                    openedCams[camerasToReinit[0]].camera->reInit();
                } catch(Spinnaker::Exception& ex)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Failed to reconnect to camera with serial# %s: %s",
                        openedCams[camerasToReinit[0]].config.serial.c_str(), ex.what());
                } catch(std::runtime_error& ex)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Failed to reconnect to camera with serial# %s: %s",
                        openedCams[camerasToReinit[0]].config.serial.c_str(), ex.what());
                }

                camerasToReinit.erase(camerasToReinit.begin());
            }
        }
    }


    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<std::thread> thread;

    Spinnaker::SystemPtr system;
    Spinnaker::CameraList cameras;
    std::shared_ptr<Spinnaker::ImageProcessor> postproc;

    std::mutex camMutex;
    std::vector<SpinnakerCameraAndConfig> openedCams;
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
        } else if(strcmp(argv[1], "-l") == 0 || strcmp(argv[1], "--list") == 0)
        {
            listConnectedCameras();
        }

        return 0;
    }

    rclcpp::init(argc, argv);
    std::shared_ptr<SpinnakerNode> node = std::make_shared<SpinnakerNode>();
    image_transport::ImageTransport it(node);
    std::list<CameraConfig> cameraConfigs = node->readParameters();

    if(cameraConfigs.size() == 0)
    {
        RCLCPP_ERROR(node->get_logger(), "No cameras defined in parameters. Aborting");
        rclcpp::shutdown();
        return 1;
    }

    node->init(it, cameraConfigs);
    rclcpp::spin(node);
    rclcpp::shutdown();
    node->release();
    return 0;
}
