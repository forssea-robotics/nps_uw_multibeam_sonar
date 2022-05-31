/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <assert.h>
#include <sys/stat.h>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_raster_based.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

#include <iostream>
#include <chrono>

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRosMultibeamSonar);

// Constructor
NpsGazeboRosMultibeamSonar::NpsGazeboRosMultibeamSonar() :
  SensorPlugin(), width(0), height(0), depth(0)
{
  this->depth_image_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->point_cloud_connect_count_ = 0;
  this->sonar_image_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);

  // frame counter for variational reflectivity
  this->maxDepth_before = 0.0;
  this->maxDepth_beforebefore = 0.0;
  this->maxDepth_prev = 0.0;

  // for csv write logs
  this->writeCounter = 0;
  this->writeNumber = 1;
}


// Destructor
NpsGazeboRosMultibeamSonar::~NpsGazeboRosMultibeamSonar()
{
  this->newDepthFrameConnection.reset();
  this->newImageFrameConnection.reset();
  this->newRGBPointCloudConnection.reset();

  this->parentSensor.reset();
  this->depthCamera.reset();

  // CSV log write stream close
  writeLog.close();
}


// Load the controller
void NpsGazeboRosMultibeamSonar::Load(sensors::SensorPtr _parent,
                                  sdf::ElementPtr _sdf)
{
  clock_t start_timer = clock();
  
  // Get tf frame for output
  this->frame_name_ = gazebo_ros::SensorFrameID(*_parent, *_sdf);

  // Initialize ROS node
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);

  this->sdf_ = _sdf;

  this->parentSensor =
    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_parent);
  this->depthCamera = this->parentSensor->DepthCamera();
  this->world = physics::get_world(parentSensor->WorldName());

  if (!this->parentSensor)
  {
    gzerr << "DepthCameraPlugin not attached to a depthCamera sensor\n";
    return;
  }

  this->width = this->depthCamera->ImageWidth();
  this->height = this->depthCamera->ImageHeight();
  this->depth = this->depthCamera->ImageDepth();
  this->format = this->depthCamera->ImageFormat();
  
  // Buffer size
  if (this->format == "L8" || this->format == "L_INT8") {
    this->type_ = sensor_msgs::image_encodings::MONO8;
    this->skip_ = 1;
  } else if (this->format == "L16" || this->format == "L_INT16") {
    this->type_ = sensor_msgs::image_encodings::MONO16;
    this->skip_ = 2;
  } else if (this->format == "R8G8B8" || this->format == "RGB_INT8") {
    this->type_ = sensor_msgs::image_encodings::RGB8;
    this->skip_ = 3;
  } else if (this->format == "B8G8R8" || this->format == "BGR_INT8") {
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  } else if (this->format == "R16G16B16" || this->format == "RGB_INT16") {
    this->type_ = sensor_msgs::image_encodings::RGB16;
    this->skip_ = 6;
  } else if (this->format == "BAYER_RGGB8") {
    RCLCPP_INFO(this->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip_ = 1;
  } else if (this->format == "BAYER_BGGR8") {
    RCLCPP_INFO(this->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip_ = 1;
  } else if (this->format == "BAYER_GBRG8") {
    RCLCPP_INFO(this->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip_ = 1;
  } else if (this->format == "BAYER_GRBG8") {
    RCLCPP_INFO(this->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip_ = 1;
  } else {
    RCLCPP_ERROR(this->ros_node_->get_logger(), "Unsupported Gazebo ImageFormat, using BGR8\n");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  this->newDepthFrameConnection =
    this->depthCamera->ConnectNewDepthFrame(
        std::bind(&NpsGazeboRosMultibeamSonar::OnNewDepthFrame,
                  this, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5));

  this->newImageFrameConnection =
    this->depthCamera->ConnectNewImageFrame(
        std::bind(&NpsGazeboRosMultibeamSonar::OnNewImageFrame,
                  this, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5));

  this->parentSensor->SetActive(true);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok()) // change to is_initialized() when possible
  {
    RCLCPP_FATAL_STREAM(this->ros_node_->get_logger(), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // topic for camera raw images
  if (!_sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = "ir/image_raw";
  else
    this->image_topic_name_ =
      _sdf->GetElement("imageTopicName")->Get<std::string>();

  // depth image stuff
  if (!_sdf->HasElement("depthImageTopicName"))
    this->depth_image_topic_name_ = "depth/image_raw";
  else
    this->depth_image_topic_name_ =
      _sdf->GetElement("depthImageTopicName")->Get<std::string>();

  if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
    this->depth_image_camera_info_topic_name_ = "depth/camera_info";
  else
    this->depth_image_camera_info_topic_name_ =
      _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudTopicName"))
    this->point_cloud_topic_name_ = "points";
  else
    this->point_cloud_topic_name_ =
        _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudCutoff"))
    this->point_cloud_cutoff_ = 0.01;
  else
    this->point_cloud_cutoff_ =
        _sdf->GetElement("pointCloudCutoff")->Get<double>();

  // sonar stuff
  if (!_sdf->HasElement("sonarImageRawTopicName"))
    this->sonar_image_raw_topic_name_ = "sonar_image_raw";
  else
    this->sonar_image_raw_topic_name_ =
      _sdf->GetElement("sonarImageRawTopicName")->Get<std::string>();
  if (!_sdf->HasElement("sonarImageTopicName"))
    this->sonar_image_topic_name_ = "sonar_image";
  else
    this->sonar_image_topic_name_ =
      _sdf->GetElement("sonarImageTopicName")->Get<std::string>();

  // Read sonar properties from model.sdf
  if (!_sdf->HasElement("verticalFOV"))
    this->verticalFOV = 10;  // Blueview P900 -> 10 degrees
  else
    this->verticalFOV =
      _sdf->GetElement("verticalFOV")->Get<double>();
  if (!_sdf->HasElement("sonarFreq"))
    this->sonarFreq = 900e3;  // Blueview P900 [Hz]
  else
    this->sonarFreq =
      _sdf->GetElement("sonarFreq")->Get<double>();
  if (!_sdf->HasElement("bandwidth"))
    this->bandwidth = 29.5e6;  // Blueview P900 [Hz]
  else
    this->bandwidth =
      _sdf->GetElement("bandwidth")->Get<double>();
  if (!_sdf->HasElement("soundSpeed"))
    this->soundSpeed = 1500;
  else
    this->soundSpeed =
      _sdf->GetElement("soundSpeed")->Get<double>();
  if (!_sdf->HasElement("maxDistance"))
    this->maxDistance = 60;
  else
    this->maxDistance =
      _sdf->GetElement("maxDistance")->Get<double>();
  if (!_sdf->HasElement("sourceLevel"))
    this->sourceLevel = 220;
  else
    this->sourceLevel =
      _sdf->GetElement("sourceLevel")->Get<double>();
  if (!_sdf->HasElement("constantReflectivity"))
    this->constMu = true;
  else
    this->constMu =
      _sdf->GetElement("constantReflectivity")->Get<bool>();
  if (!_sdf->HasElement("artificialVehicleVibration"))
    this->artificialVehicleVibration = false;
  else
    this->artificialVehicleVibration =
      _sdf->GetElement("artificialVehicleVibration")->Get<bool>();
  if (!_sdf->HasElement("customSDFTagReflectivity"))
    this->customTag = false;
  else
    this->customTag =
      _sdf->GetElement("customSDFTagReflectivity")->Get<bool>();
  if (!_sdf->HasElement("raySkips"))
    this->raySkips = 10;
  else
    this->raySkips =
      _sdf->GetElement("raySkips")->Get<int>();
  if (!_sdf->HasElement("plotScaler"))
    this->plotScaler = 10;
  else
    this->plotScaler =
      _sdf->GetElement("plotScaler")->Get<float>();
  if (!_sdf->HasElement("sensorGain"))
    this->sensorGain = 0.02;
  else
    this->sensorGain =
      _sdf->GetElement("sensorGain")->Get<float>();
  // Configure skips
  if (this->raySkips == 0) this->raySkips = 1;

  // --- Variational Reflectivity --- //
  // Read the variational reflectivity database file path from the SDF file
  if (!this->constMu)
  {
    if (!this->customTag)
    {
      if (!_sdf->HasElement("reflectivityDatabaseFile"))
      {
        this->reflectivityDatabaseFileName = "variationalReflectivityDatabase.csv";
      }
      else
      {
        this->reflectivityDatabaseFileName =
          _sdf->GetElement("reflectivityDatabaseFile")->Get<std::string>();
        GZ_ASSERT(!this->reflectivityDatabaseFileName.empty(),
          "Empty variational reflectivity database file name");
      }
    }
    else
    {
      if (!_sdf->HasElement("customSDFTagDatabaseFile"))
      {
        this->customTagDatabaseFileName = "customSDFTagDatabase.csv";
      }
      else
      {
        this->customTagDatabaseFileName =
          _sdf->GetElement("customSDFTagDatabaseFile")->Get<std::string>();
        GZ_ASSERT(!this->customTagDatabaseFileName.empty(),
          "Empty custom SDF Tag database file name");
      }
    }
  }

  this->mu = 1e-3;  // default constant mu

  this->reflectivityDatabaseFilePath = ament_index_cpp::get_package_share_directory("nps_uw_multibeam_sonar") + "/worlds/" + this->reflectivityDatabaseFileName;
  this->customTagDatabaseFilePath = ament_index_cpp::get_package_share_directory("nps_uw_multibeam_sonar") + "/worlds/" + this->customTagDatabaseFileName;

  // Read csv file
  std::ifstream csvFile; std::string line;
  if (!this->customTag)
    csvFile.open(this->reflectivityDatabaseFilePath);
  else
    csvFile.open(this->customTagDatabaseFilePath);
  // skip the 3 lines
  getline(csvFile, line); getline(csvFile, line); getline(csvFile, line);
  while (getline(csvFile, line))
  {
      if (line.empty())  // skip empty lines:
      {
          continue;
      }
      std::istringstream iss(line);
      std::string lineStream;
      std::string::size_type sz;
      std::vector <std::string> row;
      while (getline(iss, lineStream, ','))
      {
          row.push_back(lineStream);
      }
      this->objectNames.push_back(row[0]);
      this->reflectivities.push_back(stof(row[1], &sz));
  }

  // Read coefficient for Biofouling and roughness
  if (this->customTag)
  {
    for (int k=0; k<objectNames.size(); k++)
    {
      if (objectNames[k] == "biofouling_rating")
        this->biofouling_rating_coeff = reflectivities[k];
      if (objectNames[k] == "roughness")
        this->roughness_coeff = reflectivities[k];
    }
  }

  // From FiducialCameraPlugin
  if (this->depthCamera)
  {
    this->scene = this->depthCamera->GetScene();
  }
  if (!this->depthCamera || !this->scene)
  {
    gzerr << "SonarDummy failed to load. "
        << "Camera and/or Scene not found" << std::endl;
  }
  // load the fiducials
  if (_sdf->HasElement("fiducial"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("fiducial");
    while (elem)
    {
      this->fiducials.insert(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  }
  else
  {
    gzmsg << "No fiducials specified. All models will be tracked."
        << std::endl;
    this->detectAll = true;
  }

  // Transmission path properties (typical model used here)
  // More sophisticated model by Francois-Garrison model is available
  this->absorption = 0.0354;  // [dB/m]
  this->attenuation = this->absorption*log(10)/20.0;

  // Range vector
  const float max_T = this->maxDistance*2.0/this->soundSpeed;
  float delta_f = 1.0/max_T;
  const float delta_t = 1.0/this->bandwidth;
  this->nFreq = ceil(this->bandwidth/delta_f);
  delta_f = this->bandwidth/this->nFreq;
  const int nTime = nFreq;
  this->rangeVector = new float[nTime];
  for (int i = 0; i < nTime; i++)
  {
    this->rangeVector[i] = delta_t*i*this->soundSpeed/2.0;
  }

  // FOV, Number of beams, number of rays are defined at model.sdf
  // Currently, this->width equals # of beams, and this->height equals # of rays
  // Each beam consists of (elevation,azimuth)=(this->height,1) rays
  // Beam patterns
  this->nBeams = this->width;
  this->nRays = this->height;
  this->ray_nElevationRays = this->height;
  this->ray_nAzimuthRays = 1;
  this->elevation_angles = new float[this->nRays];

  // Print sonar calculation settings
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "============   SONAR PLUGIN LOADED   =============");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "============      RASTER VERSION     =============");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Maximum view range  [m] = " << this->maxDistance);
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Distance resolution [m] = " <<
                    this->soundSpeed*(1.0/(this->nFreq*delta_f)));
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "# of Beams = " << this->nBeams);
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "# of Rays / Beam (Elevation, Azimuth) = ("
      << ray_nElevationRays << ", " << ray_nAzimuthRays << ")");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Calculation skips (Elevation) = "
      << this->raySkips);
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "# of Time data / Beam = " << this->nFreq);
  if (!this->constMu)
  {
    if (this->customTag)
      RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Reflectivity method : Variational (based on custon SDF tag)");
    else
      RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Reflectivity method : Variational (based on model name)");
  }
  else
  {
      RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Reflectivity method : Constant");
  }
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "");

  // get writeLog Flag
  if (!_sdf->HasElement("writeLog"))
    this->writeLogFlag = true;
  else
  {
    this->writeLogFlag = _sdf->Get<bool>("writeLog");
    if (this->writeLogFlag)
    {
      if (_sdf->HasElement("writeFrameInterval"))
        this->writeInterval = _sdf->Get<int>("writeFrameInterval");
      else
        this->writeInterval = 10;
      RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Raw data at " << "/tmp/SonarRawData_{numbers}.csv");
      RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "every " << this->writeInterval << " frames");
      RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "");

      struct stat buffer;
      std::string logfilename("/tmp/SonarRawData_000001.csv");
      if (stat (logfilename.c_str(), &buffer) == 0)
        system("rm /tmp/SonarRawData*.csv");
    }
  }

  // Get debug flag for computation time display
  if (!_sdf->HasElement("debugFlag"))
    this->debugFlag = true;
  else
    this->debugFlag =
      _sdf->GetElement("debugFlag")->Get<bool>();

  // -- Pre calculations for sonar -- //
  // rand number generator
  this->rand_image = cv::Mat(this->height, this->width, CV_32FC2);
  uint64 randN = static_cast<uint64>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.0f);

  // Hamming window
  this->window = new float[this->nFreq];
  float windowSum = 0;
  for (size_t f = 0; f < this->nFreq; f++)
  {
    this->window[f] = 0.54 - 0.46 * cos(2.0*M_PI*(f+1)/this->nFreq);
    windowSum += pow(this->window[f], 2.0);
  }
  for (size_t f = 0; f < this->nFreq; f++)
    this->window[f] = this->window[f]/sqrt(windowSum);

  // Sonar corrector preallocation
  this->beamCorrector = new float*[nBeams];
  for (int i = 0; i < nBeams; i++)
      this->beamCorrector[i] = new float[nBeams];
  this->beamCorrectorSum = 0.0;

  static const rmw_qos_profile_t my_custom_qos_profile =
  {
      RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      5,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
  };
  // rmw_qos_profile_sensor_data

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization::from_rmw(my_custom_qos_profile),
    my_custom_qos_profile);
  
  // Init publishers
  this->image_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
    this->image_topic_name_, qos);

  this->depth_image_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
    this->depth_image_topic_name_, qos);

  this->depth_image_camera_info_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    this->depth_image_camera_info_topic_name_, qos);

  this->normal_image_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
    this->depth_image_topic_name_+"_normals", qos);

  this->point_cloud_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->point_cloud_topic_name_, qos);

  this->sonar_image_raw_pub_ = this->ros_node_->create_publisher<acoustic_msgs::msg::SonarImage>(
    this->sonar_image_raw_topic_name_, qos);

  this->sonar_image_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
    this->sonar_image_topic_name_, qos);

  clock_t end_timer = clock();
  std::cout<<"Load function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<" msec\n";
}

void NpsGazeboRosMultibeamSonar::PopulateFiducials()
{
  this->fiducials.clear();

  // Check all models for inclusion in the frustum.
  rendering::VisualPtr worldVis = this->scene->WorldVisual();
  for (unsigned int i = 0; i < worldVis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = worldVis->GetChild(i);
    if (childVis->GetType() == rendering::Visual::VT_MODEL)
      this->fiducials.insert(childVis->Name());
  }
}


//----------------------------------------------------------------
// Increment and decriment a connection counter so that the sensor
// is only active and ROS messages being published when required
// TODO: Update once new message (plugin output) is being published
//----------------------------------------------------------------

// Update everything when Gazebo provides a new depth frame (texture)
void NpsGazeboRosMultibeamSonar::OnNewDepthFrame(const float *_image,
                                             unsigned int _width,
                                             unsigned int _height,
                                             unsigned int _depth,
                                             const std::string &_format)
{
  clock_t start_timer = clock();
  
  // std::cout<<"on new depth frame event callback"<<std::endl;
  if (!rclcpp::ok() || this->height <=0 || this->width <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();  

  if (this->parentSensor->IsActive())
  {
    // std::cout<<"parent is active"<<std::endl;

    // Deactivate if no subscribers
    this->image_connect_count_ = this->ros_node_->count_publishers(this->image_topic_name_);

    this->depth_image_connect_count_ = 
    this->ros_node_->count_publishers(this->depth_image_topic_name_) + 
    this->ros_node_->count_publishers(this->depth_image_topic_name_ + "_normals") +
    this->ros_node_->count_publishers(this->sonar_image_topic_name_) +
    this->ros_node_->count_publishers(this->sonar_image_raw_topic_name_);

    this->point_cloud_connect_count_ = 
    this->ros_node_->count_publishers(this->point_cloud_topic_name_);

    if (this->depth_image_connect_count_ <= 0 &&
        this->point_cloud_connect_count_ <= 0 &&
        this->image_connect_count_ <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      // std::cout<<"compute point cloud and sonar image"<<std::endl;

      this->ComputePointCloud(_image);

      if (this->depth_image_connect_count_ > 0)
        this->ComputeSonarImage(_image);
    }
  }
  else
  {
    if (this->depth_image_connect_count_ <= 0 ||
        this->point_cloud_connect_count_ > 0)
      this->parentSensor->SetActive(true);
  }
  this->PublishCameraInfo();

  clock_t end_timer = clock();
  std::cout<<"OnNewDepthFrame function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";
}


// Process the camera image when Gazebo provides one. Do we actually need this?
void NpsGazeboRosMultibeamSonar::OnNewImageFrame(const unsigned char *_image,
                                             unsigned int _width,
                                             unsigned int _height,
                                             unsigned int _depth,
                                             const std::string &_format)
{
  clock_t start_timer = clock();

  // std::cout<<"on new image frame event callback"<<std::endl;
  if (!rclcpp::ok() || this->height <=0 || this->width <=0)
    return;

  auto sensor_update_time = this->parentSensor->LastMeasurementTime();
  this->image_connect_count_ = this->ros_node_->count_publishers(this->image_topic_name_);


  if (!this->parentSensor->IsActive())
  {
    if (this->image_connect_count_ > 0) 
      // do this first so there's chance for sensor
      // to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->image_connect_count_ > 0)
    {
      // Save image
      this->image_msg_.header.frame_id = this->frame_name_;
      this->image_msg_.header.stamp.sec = int(sensor_update_time.sec);
      this->image_msg_.header.stamp.nanosec = uint(sensor_update_time.nsec);

      this->image_msg_.encoding = this->type_;
      this->image_msg_.height = _height;
      this->image_msg_.width = _width;
      this->image_msg_.step = this->skip_ * _width;
      size_t st0 = (this->skip_ * _width * this->image_msg_.height);
      this->image_msg_.data.resize(st0);
      memcpy(&this->image_msg_.data[0], reinterpret_cast<const void *>(_image), st0);

      this->image_msg_.is_bigendian = 0;

      this->image_pub_->publish(this->image_msg_);
    }
  }

  // Calculate only if the maxDepth from depth camera is changed and stabled
  double min; cv::minMaxLoc(this->point_cloud_image_, &min, &this->maxDepth);
  if (this->maxDepth == this->maxDepth_before
      && this->maxDepth == this->maxDepth_beforebefore
      && this->calculateReflectivity == false
      && this->maxDepth != this->maxDepth_prev)
  {
    this->calculateReflectivity = true;
    this->maxDepth_prev = this->maxDepth;

    // Regenerate rand image
    uint64 randN = static_cast<uint64>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);
  }
  else
    this->calculateReflectivity = false;

  this->maxDepth_beforebefore = this->maxDepth_before;
  this->maxDepth_before = this->maxDepth;

  // For variational reflectivity
  if (!this->constMu)
  {
    if (calculateReflectivity)
    {
      // Generate reflectivity opencv image palette
      cv::Mat reflectivity_image = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

      if (!this->selectionBuffer)
      {
        std::string cameraName = this->depthCamera->OgreCamera()->getName();
        this->selectionBuffer.reset(
            new rendering::SelectionBuffer(cameraName,
            this->scene->OgreSceneManager(),
            this->depthCamera->RenderTexture()->getBuffer()->
            getRenderTarget()));
      }

      if (this->detectAll)
        this->PopulateFiducials();

      std::vector<FiducialData> results;
      for (const auto &f : this->fiducials)
      {
        // check if fiducial is visible within the frustum
        rendering::VisualPtr vis = this->scene->GetVisual(f);
        if (!vis)
          continue;

        if (!this->depthCamera->IsVisible(vis))
          continue;

        // Loop over every pixel
        for (int i=0; i<reflectivity_image.rows; i++)
        {
          for (int j=0; j<reflectivity_image.cols; j+=raySkips)
          {
            // target pixel
            ignition::math::Vector2i pt = ignition::math::Vector2i(i, j);

            // use selection buffer to check if visual is occluded by other entities
            // in the camera view
            Ogre::Entity *entity =
              this->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

            rendering::VisualPtr result;
            if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
            {
              try
              {
                result = this->scene->GetVisual(
                    Ogre::any_cast<std::string>(
                    entity->getUserObjectBindings().getUserAny()));
              }
              catch(Ogre::Exception &_e)
              {
                gzerr << "Ogre Error:" << _e.getFullDescription() << "\n";
                continue;
              }
            }

            if (result && result->GetRootVisual() == vis)
            {
              FiducialData fd;
              fd.id = vis->Name();
              fd.pt = pt;

              // this->world;


              // Assign variational reflectivity
              if (!this->customTag)
              {
                for (int k=0; k<objectNames.size(); k++)
                  if (vis->Name() == objectNames[k])
                    reflectivity_image.at<float>(j, i) = reflectivities[k];
              }
              else
              {
                // Read custom tags for surface properties
                sdf::ElementPtr modelElt =
                  this->world->BaseByName(vis->Name())->GetSDF();

                int biofoulingRating = 0; // Biofouling rating, [0, 100]
                if (modelElt->HasElement("surface_props:biofouling_rating"))
                  biofoulingRating = modelElt->Get<int>("surface_props:biofouling_rating");

                double roughness = 0.0; // Surface roughness, [0.0, 1.0]
                if (modelElt->HasElement("surface_props:roughness"))
                  roughness = modelElt->Get<double>("surface_props:roughness");

                std::string material = "default"; // Surface material
                if (modelElt->HasElement("surface_props:material"))
                  material = modelElt->Get<std::string>("surface_props:material");

                for (int k=0; k<objectNames.size(); k++)
                  if (material == objectNames[k])
                    reflectivity_image.at<float>(j, i) =
                      reflectivities[k] * (1.0/(roughness + 1)) / this->roughness_coeff
                      * (1.0/(biofoulingRating + 1)) / this->biofouling_rating_coeff;

              }
              // results.push_back(fd);  // Redundant
            }
          }
        }  // end of pixel loop
      }  // end of selection buffer

      // Save reflectivity image
      this->reflectivityImage = reflectivity_image;
    }  // end of variational reflectivity calculation
  }  // end of variational reflectivity bool

  clock_t end_timer = clock();
  std::cout<<"OnNewImageFrame function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";
          
}

// Most of the plugin work happens here
void NpsGazeboRosMultibeamSonar::ComputeSonarImage(const float *_src)
{
  clock_t start_timer = clock();
  // std::cout<<"Compute sonar image function"<<std::endl;

  this->lock_.lock();
  cv::Mat depth_image = this->point_cloud_image_;
  cv::Mat normal_image = this->ComputeNormalImage(depth_image);
  double vFOV = this->parentSensor->DepthCamera()->VFOV().Radian();
  double hFOV = this->parentSensor->DepthCamera()->HFOV().Radian();
  double vPixelSize = vFOV / this->height;
  double hPixelSize = hFOV / this->width;

  if (this->beamCorrectorSum == 0)
    ComputeCorrector();

  // Default value for reflectivity
  if (this->reflectivityImage.rows == 0)
    this->reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

  // If artifical vehicle vibration flag is on
  if (this->artificialVehicleVibration)
  {
    // Regenerate rand image
    uint64 randN = static_cast<uint64>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);
  }

  // For calc time measure
  auto start = std::chrono::high_resolution_clock::now();
  // ------------------------------------------------//
  // --------      Sonar calculations       -------- //
  // ------------------------------------------------//
  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
                  depth_image,   // cv::Mat& depth_image
                  normal_image,  // cv::Mat& normal_image
                  rand_image,    // cv::Mat& rand_image
                  hPixelSize,    // hPixelSize
                  vPixelSize,    // vPixelSize
                  hFOV,          // hFOV
                  vFOV,          // VFOV
                  hPixelSize,    // _beam_azimuthAngleWidth
                  verticalFOV/180*M_PI,  // _beam_elevationAngleWidth
                  hPixelSize,    // _ray_azimuthAngleWidth
                  this->elevation_angles, // _ray_elevationAngles
                  vPixelSize*(raySkips+1),  // _ray_elevationAngleWidth
                  this->soundSpeed,    // _soundSpeed
                  this->maxDistance,   // _maxDistance
                  this->sourceLevel,   // _sourceLevel
                  this->nBeams,        // _nBeams
                  this->nRays,         // _nRays
                  this->raySkips,      // _raySkips
                  this->sonarFreq,     // _sonarFreq
                  this->bandwidth,     // _bandwidth
                  this->nFreq,         // _nFreq
                  this->reflectivityImage,  // reflectivity_image
                  this->attenuation,   // _attenuation
                  this->window,        // _window
                  this->beamCorrector,      // _beamCorrector
                  this->beamCorrectorSum,   // _beamCorrectorSum
                  this->debugFlag);

  // For calc time measure
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<
                  std::chrono::microseconds>(stop - start);
  if (debugFlag)
  {
    RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "GPU Sonar Frame Calc Time " <<
                    duration.count()/10000 << "/100 [s]\n");
  }

  // Gaussian noise
  // double whiteNoise = ignition::math::Rand::DblNormal(0.0, 0.7);
      // ROS_INFO_STREAM(Intensity[beam][f]);

  // CSV log write stream
  // Each cols corresponds to each beams
  if (this->writeLogFlag)
  {
    this->writeCounter = this->writeCounter + 1;
    if (this->writeCounter == 1
        ||this->writeCounter % this->writeInterval == 0)
    {
      double time = this->parentSensor->LastMeasurementTime().Double();
      std::stringstream filename;
      filename << "/tmp/SonarRawData_" << std::setw(6) <<  std::setfill('0')
               << this->writeNumber << ".csv";
      writeLog.open(filename.str().c_str(), std::ios_base::app);
      filename.clear();
      writeLog << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n";
      writeLog << "# First column is range vector\n";
      writeLog << "#  nBeams : " << nBeams << "\n";
      writeLog << "# Simulation time : " << time << "\n";
      for (size_t i = 0; i < P_Beams[0].size(); i++)
      {
        // writing range vector at first column
        writeLog << this->rangeVector[i];
        for (size_t b = 0; b < nBeams; b ++)
        {
          if (P_Beams[b][i].imag() > 0)
            writeLog << "," << P_Beams[b][i].real()
                     << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog << "," << P_Beams[b][i].real()
                     << P_Beams[b][i].imag() << "i";
        }
        writeLog << "\n";
      }
      writeLog.close();

      this->writeNumber = this->writeNumber + 1;
    }
  }

  // Sonar image ROS msg
  this->sonar_image_raw_msg_.header.frame_id
        = this->frame_name_.c_str();
  this->sonar_image_raw_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->sonar_image_raw_msg_.header.stamp.nanosec
        = this->depth_sensor_update_time_.nsec;
  this->sonar_image_raw_msg_.frequency = this->sonarFreq;
  this->sonar_image_raw_msg_.sound_speed = this->soundSpeed;
  this->sonar_image_raw_msg_.azimuth_beamwidth = hPixelSize;
  this->sonar_image_raw_msg_.elevation_beamwidth = hPixelSize*this->nRays;
  std::vector<float> azimuth_angles;
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  for (size_t beam = 0; beam < nBeams; beam ++)
    azimuth_angles.push_back(atan2(static_cast<double>(beam) -
                    0.5 * static_cast<double>(width), fl));
  this->sonar_image_raw_msg_.azimuth_angles = azimuth_angles;
  std::vector<float> ranges;
  for (size_t i = 0; i < P_Beams[0].size(); i ++)
    ranges.push_back(rangeVector[i]);
  this->sonar_image_raw_msg_.ranges = ranges;

  // this->sonar_image_raw_msg_.is_bigendian = false;
  this->sonar_image_raw_msg_.data_size = 1;  // sizeof(float) * nFreq * nBeams;
  std::vector<uchar> intensities;
  int Intensity[nBeams][nFreq];
  for (size_t f = 0; f < nFreq; f ++)
  {
    for (size_t beam = 0; beam < nBeams; beam ++)
    {
      // Serialize beams in reverse order to flip the data left to right
      const size_t beam_idx = nBeams-beam-1;
      Intensity[beam_idx][f] = static_cast<int>(this->sensorGain * abs(P_Beams[beam_idx][f]));
      uchar counts = static_cast<uchar>(std::min(UCHAR_MAX, Intensity[beam_idx][f]));

      intensities.push_back(counts);
    }
  }
  this->sonar_image_raw_msg_.intensities = intensities;
  this->sonar_image_raw_pub_->publish(this->sonar_image_raw_msg_);

  // Construct visual sonar image for rqt plot in sensor::image msg format
  cv_bridge::CvImage img_bridge;

  // Generate image of 328UC1
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);

  const float rangeMax = maxDistance;
  const float rangeRes = ranges[1]-ranges[0];
  const int nEffectiveRanges = ceil(rangeMax / rangeRes);
  const unsigned int radius = Intensity_image.size().height;
  const cv::Point origin(Intensity_image.size().width/2,
                         Intensity_image.size().height);
  const float binThickness = 2 * ceil(radius / nEffectiveRanges);

  struct BearingEntry
  {
    float begin, center, end;
    BearingEntry(float b, float c, float e)
      : begin(b), center(c), end(e)
        {;}
  };

  std::vector<BearingEntry> angles;
  angles.reserve(nBeams);

  for ( int b = 0; b < nBeams; ++b )
  {
    const float center = azimuth_angles[b];
    float begin = 0.0, end = 0.0;
    if (b == 0)
    {
      end = (azimuth_angles[b + 1] + center) / 2.0;
      begin = 2 * center - end;
    }
    else if (b == nBeams - 1)
    {
      begin = angles[b - 1].end;
      end = 2 * center - begin;
    }
    else
    {
      begin = angles[b - 1].end;
      end = (azimuth_angles[b + 1] + center) / 2.0;
    }
    angles.push_back(BearingEntry(begin, center, end));
  }

  const float ThetaShift = 1.5*M_PI;
  for ( int r = 0; r < ranges.size(); ++r )
  {
    if ( ranges[r] > rangeMax ) continue;
    for ( int b = 0; b < nBeams; ++b )
    {
      const float range = ranges[r];
      const int intensity = floor(10.0*log(abs(P_Beams[nBeams - 1 - b][r])));
      const float begin = angles[b].begin + ThetaShift,
                  end = angles[b].end + ThetaShift;
      const float rad = static_cast<float>(radius) * range/rangeMax;
      // Assume angles are in image frame x-right, y-down
      cv::ellipse(Intensity_image, origin, cv::Size(rad, rad), 0,
                  begin * 180/M_PI, end * 180/M_PI,
                  intensity, binThickness);
    }
  }

  // Normlize and colorize
  cv::normalize(Intensity_image,Intensity_image,
                -255 + this->plotScaler/10*255, 255, cv::NORM_MINMAX);
  cv::Mat Itensity_image_color;
  cv::applyColorMap(Intensity_image, Itensity_image_color, cv::COLORMAP_HOT);

  // Publish final sonar image
  this->sonar_image_msg_.header.frame_id
        = this->frame_name_;
  this->sonar_image_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->sonar_image_msg_.header.stamp.nanosec
        = this->depth_sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header,
                                  sensor_msgs::image_encodings::BGR8,
                                  Itensity_image_color);
  // from cv_bridge to sensor_msgs::Image
  img_bridge.toImageMsg(this->sonar_image_msg_);

  this->sonar_image_pub_->publish(this->sonar_image_msg_);

  // ---------------------------------------- End of sonar calculation

  // Still publishing the depth and normal image (just because)
  // Depth image
  this->depth_image_msg_.header.frame_id
        = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nanosec
        = this->depth_sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->depth_image_msg_.header,
                                  sensor_msgs::image_encodings::TYPE_32FC1,
                                  depth_image);
  // from cv_bridge to sensor_msgs::Image
  img_bridge.toImageMsg(this->depth_image_msg_);
  this->depth_image_pub_->publish(this->depth_image_msg_);


  // Normal image
  this->normal_image_msg_.header.frame_id
        = this->frame_name_;
  this->normal_image_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->normal_image_msg_.header.stamp.nanosec
        = this->depth_sensor_update_time_.nsec;
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  img_bridge = cv_bridge::CvImage(this->normal_image_msg_.header,
                                  sensor_msgs::image_encodings::RGB8,
                                  normal_image8);
  img_bridge.toImageMsg(this->normal_image_msg_);
  // from cv_bridge to sensor_msgs::Image
  this->normal_image_pub_->publish(this->normal_image_msg_);

  this->lock_.unlock();

  clock_t end_timer = clock();
  std::cout<<"ComputeSonarImage function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";

}


void NpsGazeboRosMultibeamSonar::ComputePointCloud(const float *_src)
{
  clock_t start_timer = clock();
  // std::cout<<"Compute point cloud function"<<std::endl;

  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id
        = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nanosec
        = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step
        = this->point_cloud_msg_.point_step * this->width;

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg_);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(this->height * this->width);

  // resize if point cloud image to camera parameters if required
  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();

  point_cloud_msg_.is_dense = true;

  float* toCopyFrom = const_cast<float*>(_src);
  int index = 0;

  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));


  for (uint32_t j = 0; j < this->height; j++)
  {
    double elevation;
    if (this->height > 1)
      elevation = atan2(static_cast<double>(j) -
                        0.5 * static_cast<double>(this->height), fl);
    else
      elevation = 0.0;

    this->elevation_angles[j] = static_cast<float>(elevation);

    for (uint32_t i = 0; i < this->width;
         i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_image)
    {
      double azimuth;
      if (this->width > 1)
        azimuth = atan2(static_cast<double>(i) -
                        0.5 * static_cast<double>(this->width), fl);
      else
        azimuth = 0.0;

      double depth = toCopyFrom[index++];


      // in optical frame hardcoded rotation
      // rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x = depth * tan(azimuth);
      *iter_y = depth * tan(elevation);
      if (depth > this->point_cloud_cutoff_)
      {
        *iter_z = depth;
        *iter_image = sqrt(*iter_x * *iter_x +
                           *iter_y * *iter_y +
                           *iter_z * *iter_z);
      }
      else  // point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_image = 0.0;
        point_cloud_msg_.is_dense = false;
      }


      // put image color data for each point
      uint8_t*  image_src = static_cast<uint8_t*>(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == this->height * this->width*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*this->width*3+0];
        iter_rgb[1] = image_src[i*3+j*this->width*3+1];
        iter_rgb[2] = image_src[i*3+j*this->width*3+2];
      }
      else if (this->image_msg_.data.size() == this->height * this->width)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*this->width];
        iter_rgb[1] = image_src[i+j*this->width];
        iter_rgb[2] = image_src[i+j*this->width];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }
  if (this->point_cloud_connect_count_ > 0)
    this->point_cloud_pub_->publish(this->point_cloud_msg_);

  this->lock_.unlock();

  clock_t end_timer = clock();
  std::cout<<"ComputePointCloud function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";
}

/////////////////////////////////////////////////
// Precalculation of corrector sonar calculation
void NpsGazeboRosMultibeamSonar::ComputeCorrector()
{
  clock_t start_timer = clock();

  double hFOV = this->parentSensor->DepthCamera()->HFOV().Radian();
  double hPixelSize = hFOV / this->width;
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  // Beam culling correction precalculation
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    float beam_azimuthAngle = atan2(static_cast<double>(beam) -
                        0.5 * static_cast<double>(width), fl);
    for (size_t beam_other = 0; beam_other < nBeams; beam_other ++)
    {
      float beam_azimuthAngle_other = atan2(static_cast<double>(beam_other) -
                        0.5 * static_cast<double>(width), fl);
      float azimuthBeamPattern =
        unnormalized_sinc(M_PI * 0.884 / hPixelSize
        * sin(beam_azimuthAngle-beam_azimuthAngle_other));
      this->beamCorrector[beam][beam_other] = abs(azimuthBeamPattern);
      this->beamCorrectorSum += pow(azimuthBeamPattern, 2);
    }
  }
  this->beamCorrectorSum = sqrt(this->beamCorrectorSum);

  clock_t end_timer = clock();
  std::cout<<"ComputeCorrector function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";
}

/////////////////////////////////////////////////
cv::Mat NpsGazeboRosMultibeamSonar::ComputeNormalImage(cv::Mat& depth)
{
  clock_t start_timer = clock();

  // filters
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) << 1,  2,  1,
                                                 0,  0,  0,
                                                -1, -2, -1) / 8;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) << 1, 0, -1,
                                                 2, 0, -2,
                                                 1, 0, -1) / 8;

  cv::Mat f1m, f2m;
  cv::flip(f1, f1m, 0);
  cv::flip(f2, f2m, 1);

  cv::Mat n1, n2;
  cv::filter2D(depth, n1, -1, f1m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(depth, n2, -1, f2m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::Mat no_readings;
  cv::erode(depth == 0, no_readings, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  // cv::dilate(no_readings, no_readings, cv::Mat(),
  //            cv::Point(-1, -1), 2, 1, 1);
  n1.setTo(0, no_readings);
  n2.setTo(0, no_readings);

  std::vector<cv::Mat> images(3);
  cv::Mat white = cv::Mat::ones(depth.rows, depth.cols, CV_32FC1);

  // NOTE: with different focal lengths, the expression becomes
  // (-dzx*fy, -dzy*fx, fx*fy)
  images.at(0) = n1;    // for green channel
  images.at(1) = n2;    // for red channel
  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double focal_length = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));
  images.at(2) = 1.0/focal_length*depth;  // for blue channel

  cv::Mat normal_image;
  cv::merge(images, normal_image);

  for (int i = 0; i < normal_image.rows; ++i)
  {
    for (int j = 0; j < normal_image.cols; ++j)
    {
      cv::Vec3f& n = normal_image.at<cv::Vec3f>(i, j);
      n = cv::normalize(n);
      float& d = depth.at<float>(i, j);
    }
  }
  clock_t end_timer = clock();
  std::cout<<"ComputeNormalImage function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";
  return normal_image;
}


/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::PublishCameraInfo()
{
  clock_t start_timer = clock();

  RCLCPP_DEBUG_STREAM(this->ros_node_->get_logger(), "publishing default camera info, then depth camera info");
  // GazeboRosCamera::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0)
  {
    common::Time sensor_update_time
          = this->parentSensor->LastMeasurementTime();

    // this->sensor_update_time_ = sensor_update_time;
    if (sensor_update_time
          - this->last_depth_image_camera_info_update_time_
          >= this->parentSensor->UpdateRate())
    {
      // C parameters
      auto default_cx = (static_cast<double>(this->width) + 1.0) / 2.0;
      auto cx = this->sdf_->Get<double>("cx", default_cx).first;

      auto default_cy = (static_cast<double>(this->height) + 1.0) / 2.0;
      auto cy = this->sdf_->Get<double>("cy", default_cy).first;

      double hfov = this->depthCamera->HFOV().Radian();
      double computed_focal_length = (static_cast<double>(this->width)) / (2.0 * tan(hfov / 2.0));

      // Focal length
      auto focal_length = this->sdf_->Get<double>("focal_length", 0.0).first;
      if (focal_length == 0) {
        focal_length = computed_focal_length;
      } else if (!ignition::math::equal(focal_length, computed_focal_length)) {
        RCLCPP_WARN(this->ros_node_->get_logger(),
          "The <focal_length> [%f] you have provided for camera [%s]"
          " is inconsistent with specified <image_width> [%d] and"
          " HFOV [%f]. Please double check to see that"
          " focal_length = width / (2.0 * tan(HFOV/2.0))."
          " The expected focal_length value is [%f],"
          " please update your camera model description accordingly.",
          focal_length, this->camera_name_.c_str(), this->width, hfov, computed_focal_length);
      }
      // this->PublishCameraInfo(this->depth_image_camera_info_pub_);
      // CameraInfo
      sensor_msgs::msg::CameraInfo camera_info_msg;
      camera_info_msg.header.frame_id = this->frame_name_;
      camera_info_msg.height = this->height;
      camera_info_msg.width = this->width;
      camera_info_msg.distortion_model = "plumb_bob";
      camera_info_msg.d.resize(5);

      // Publish camera info
      camera_info_msg.header.stamp.sec = int(sensor_update_time.sec);
      camera_info_msg.header.stamp.nanosec = uint(sensor_update_time.nsec);

      // Allow the user to disable automatic cropping (used to remove barrel
      // distortion black border. The crop can be useful, but also skewes
      // the lens distortion, making the supplied k and t values incorrect.
      auto border_crop = this->sdf_->Get<bool>("border_crop", true).first;
      auto hack_baseline = this->sdf_->Get<double>("hack_baseline", 0.0).first;

      // Get distortion from camera
      double distortion_k1{0.0};
      double distortion_k2{0.0};
      double distortion_k3{0.0};
      double distortion_t1{0.0};
      double distortion_t2{0.0};
      // if (this->depthCamera->LensDistortion()) {
      //   this->depthCamera->LensDistortion()->SetCrop(border_crop);

      //   distortion_k1 = this->depthCamera->LensDistortion()->K1();
      //   distortion_k2 = this->depthCamera->LensDistortion()->K2();
      //   distortion_k3 = this->depthCamera->LensDistortion()->K3();
      //   distortion_t1 = this->depthCamera->LensDistortion()->P1();
      //   distortion_t2 = this->depthCamera->LensDistortion()->P2();
      // }

      // D = {k1, k2, t1, t2, k3}, as specified in:
      // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
      // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
      camera_info_msg.d[0] = distortion_k1;
      camera_info_msg.d[1] = distortion_k2;
      camera_info_msg.d[2] = distortion_t1;
      camera_info_msg.d[3] = distortion_t2;
      camera_info_msg.d[4] = distortion_k3;

      // Original camera matrix
      camera_info_msg.k[0] = focal_length;
      camera_info_msg.k[1] = 0.0;
      camera_info_msg.k[2] = cx;
      camera_info_msg.k[3] = 0.0;
      camera_info_msg.k[4] = focal_length;
      camera_info_msg.k[5] = cy;
      camera_info_msg.k[6] = 0.0;
      camera_info_msg.k[7] = 0.0;
      camera_info_msg.k[8] = 1.0;

      // // rectification
      // camera_info_msg.r[0] = 1.0;
      // camera_info_msg.r[1] = 0.0;
      // camera_info_msg.r[2] = 0.0;
      // camera_info_msg.r[3] = 0.0;
      // camera_info_msg.r[4] = 1.0;
      // camera_info_msg.r[5] = 0.0;
      // camera_info_msg.r[6] = 0.0;
      // camera_info_msg.r[7] = 0.0;
      // camera_info_msg.r[8] = 1.0;

      // camera_ projection matrix (same as camera_ matrix due
      // to lack of distortion/rectification) (is this generated?)
      camera_info_msg.p[0] = focal_length;
      camera_info_msg.p[1] = 0.0;
      camera_info_msg.p[2] = cx;
      camera_info_msg.p[3] = -focal_length * hack_baseline;
      camera_info_msg.p[4] = 0.0;
      camera_info_msg.p[5] = focal_length;
      camera_info_msg.p[6] = cy;
      camera_info_msg.p[7] = 0.0;
      camera_info_msg.p[8] = 0.0;
      camera_info_msg.p[9] = 0.0;
      camera_info_msg.p[10] = 1.0;
      camera_info_msg.p[11] = 0.0;

      this->depth_image_camera_info_pub_->publish(camera_info_msg);
      // , sensor_update_time);
      this->last_depth_image_camera_info_update_time_ = sensor_update_time;
    }
  }
  clock_t end_timer = clock();
  std::cout<<"PublishCameraInfo function: "<<((float) end_timer - start_timer)*1000/CLOCKS_PER_SEC<<"msec\n";
}

}
