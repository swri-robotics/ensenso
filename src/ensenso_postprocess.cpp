#include <ensenso/ensenso_postprocess.h>

EnsensoPostprocess::EnsensoPostprocess()
{
  ros::NodeHandle nh("");
  ros::NodeHandle priv_nh("~");

  std::string serial("150534");
  priv_nh.param("serial", serial, serial);

  configureCamera(serial);

  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("depth/points", 1, false);

  image_sync_.reset(new ImageSync(SyncPolicy(10), left_image_sub_, right_image_sub_));
  image_sync_->registerCallback(boost::bind(
    &EnsensoPostprocess::processImages,
    this,
    _1,
    _2));

  left_image_sub_.subscribe(nh, "/camera/left/image_raw", 2);
  right_image_sub_.subscribe(nh, "/camera/right/image_raw", 2);

  reconfigure_server_.setCallback(boost::bind(
    &EnsensoPostprocess::cameraParametersCallback,
    this,
    _1,
    _2));

}

EnsensoPostprocess::~EnsensoPostprocess()
{
  ensenso_ptr_->closeTcpPort();
  ensenso_ptr_->closeDevice();
}

void EnsensoPostprocess::configureCamera(const std::string &camera_id)
{
  ensenso_ptr_.reset(new pcl::EnsensoGrabber);
  ensenso_ptr_->openDevice(camera_id);
  ensenso_ptr_->openTcpPort();
  ensenso_ptr_->storeCalibrationPattern(false);
  if (ensenso_ptr_->isRunning())
  {
    ROS_INFO("Camera already running, stopping");
    ensenso_ptr_->stop();
  }

  ensenso_ptr_->start();
  ensenso_ptr_->setMinimumDisparity(-117);
  ensenso_ptr_->setNumberOfDisparities(170);
  ensenso_ptr_->setOptimizationProfile(std::string("AlignedAndDiagonal"));
  ensenso_ptr_->setScaling(0.99);
  ensenso_ptr_->setUniquenessRatio(30);
  ensenso_ptr_->setMedianFilterRadius(0);
  ensenso_ptr_->setSpeckleComponentThreshold(10);
  ensenso_ptr_->setSpeckleRegionSize(30);
  ensenso_ptr_->setFillBorderSpread(1);
}

void EnsensoPostprocess::processImages(
  const sensor_msgs::ImageConstPtr &left_image,
  const sensor_msgs::ImageConstPtr &right_image)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  bool success;
  std::string operation_status;
  success = ensenso_ptr_->getPointCloudFromImage(
    left_image->data,
    right_image->data,
    left_image->width,
    left_image->height,
    cloud,
    operation_status);
  if (success)
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header = left_image->header;
    cloud_pub_.publish(ros_cloud);
  }

  ROS_ERROR_COND(!success, "Failed to generate point cloud because: %s",
    operation_status.c_str());
}

void EnsensoPostprocess::cameraParametersCallback(ensenso::CameraParametersConfig &config, uint32_t level)
{
  std::string profile;
  switch (config.groups.stereo.OptimizationProfile)
  {
    case 0:
      profile = "Aligned";
      break;

    case 1:
      profile = "Diagonal";
      break;

    case 2:
      profile = "AlignedAndDiagonal";
      break;

    default:
      profile = "AlignedAndDiagonal";
      break;
  }
  // Stereo parameters
  ensenso_ptr_->setMinimumDisparity(config.groups.stereo.MinimumDisparity);
  ensenso_ptr_->setNumberOfDisparities(config.groups.stereo.NumberOfDisparities);
  ensenso_ptr_->setOptimizationProfile(profile);
  ensenso_ptr_->setScaling(config.groups.stereo.Scaling);
  ensenso_ptr_->setDepthChangeCost(config.groups.stereo.DepthChangeCost);
  ensenso_ptr_->setDepthStepCost(config.groups.stereo.DepthStepCost);
  ensenso_ptr_->setShadowingThreshold(config.groups.stereo.ShadowingThreshold);
  //Postprocessing parameters
  ensenso_ptr_->setUniquenessRatio(config.groups.postproc.UniquenessRatio);
  ensenso_ptr_->setMedianFilterRadius(config.groups.postproc.MedianFilterRadius);
  ensenso_ptr_->setSpeckleComponentThreshold(config.groups.postproc.SpeckleComponentThreshold);
  ensenso_ptr_->setSpeckleRegionSize(config.groups.postproc.SpeckleRegionSize);
  ensenso_ptr_->setFillBorderSpread(config.groups.postproc.FillBorderSpread);
  ensenso_ptr_->setFillRegionSize(config.groups.postproc.FillRegionSize);
}


int main(int argc, char **argv)
  {
    ros::init(
      argc,
      argv,
      "ensenso_postprocess",
      ros::init_options::AnonymousName);
    EnsensoPostprocess processor;
    ros::spin();
    return 0;
  }
