#include <ensenso/ensenso_postprocess.h>
#include <cv_bridge/cv_bridge.h>

EnsensoPostprocess::EnsensoPostprocess()
{
  ros::NodeHandle nh("");
  ros::NodeHandle priv_nh("~");
  image_transport::ImageTransport it(nh);

  std::string serial("150534");
  priv_nh.param("serial", serial, serial);

  configureCamera(serial);

  left_rectified_pub_ = it.advertiseCamera("postprocessed_left/image_rect", 1);
  right_rectified_pub_ = it.advertiseCamera("postprocessed_right/image_rect", 1);
  disparity_pub_ = nh.advertise<stereo_msgs::DisparityImage>("disparity", 1, false);
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("depth/points", 1, false);

  image_sync_.reset(new ImageSync(SyncPolicy(10), left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_));
  image_sync_->registerCallback(boost::bind(
    &EnsensoPostprocess::processImages,
    this,
    _1,
    _2,
    _3,
    _4));

  left_image_sub_.subscribe(nh, "/camera/left/image_raw", 2);
  right_image_sub_.subscribe(nh, "/camera/right/image_raw", 2);
  left_info_sub_.subscribe(nh, "/camera/left/camera_info", 2);
  right_info_sub_.subscribe(nh, "/camera/right/camera_info", 2);

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
  ensenso_ptr_->setBinning(1);
  ensenso_ptr_->setFlexView(false, 2);
  ensenso_ptr_->setMinimumDisparity(-117);
  ensenso_ptr_->setNumberOfDisparities(170);
  ensenso_ptr_->setOptimizationProfile(std::string("AlignedAndDiagonal"));
  ensenso_ptr_->setScaling(0.99);
  ensenso_ptr_->setUniquenessRatio(30);
  ensenso_ptr_->setMedianFilterRadius(0);
  ensenso_ptr_->setSpeckleComponentThreshold(10);
  ensenso_ptr_->setSpeckleRegionSize(30);
  ensenso_ptr_->setFillBorderSpread(1);
  ensenso_ptr_->setProjector(true);
}

void EnsensoPostprocess::processImages(
  const sensor_msgs::ImageConstPtr &left_image,
  const sensor_msgs::ImageConstPtr &right_image,
  const sensor_msgs::CameraInfoConstPtr &left_info,
  const sensor_msgs::CameraInfoConstPtr &right_info)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  bool success;
  std::string operation_status;
  std::pair<pcl::PCLImage, pcl::PCLImage> rect_images;
  pcl::PCLImage disparity;
  int min_disparity;
  int max_disparity;
  success = ensenso_ptr_->postProcessImages(
    left_image->data,
    right_image->data,
    left_image->width,
    left_image->height,
    rect_images,
    disparity,
    min_disparity,
    max_disparity,
    cloud,
    operation_status);
  if (success)
  {
    sensor_msgs::CameraInfo left_rect_info;
    sensor_msgs::CameraInfo right_rect_info;
    ensenso_ptr_->getCameraInfoRectified("Left", left_rect_info);
    ensenso_ptr_->getCameraInfoRectified("Right", right_rect_info);
    sensor_msgs::Image left_rect_image = *toImageMsg(
      rect_images.first,
      left_image->header.frame_id,
      left_image->header.stamp);
    left_rectified_pub_.publish(
      left_rect_image,
      left_rect_info,
      left_image->header.stamp);

    right_rectified_pub_.publish(
      *toImageMsg(rect_images.second,
        left_image->header.frame_id,
        left_image->header.stamp),
      right_rect_info,
      left_image->header.stamp);
    sensor_msgs::ImagePtr image;
    stereo_msgs::DisparityImage disparity_msg;
    disparity_msg.min_disparity = min_disparity;
    disparity_msg.max_disparity = max_disparity;
    short *image_array = reinterpret_cast<short *>(disparity.data.data());
    cv::Mat image_mat(disparity.height, disparity.width, CV_16SC1, image_array);
    std_msgs::Header header;
    header.frame_id = left_image->header.frame_id;
    header.stamp = ros::Time(disparity.header.stamp);
    image_mat.convertTo(image_mat, CV_32FC1);
    image = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::TYPE_32FC1,
      image_mat).toImageMsg();
    disparity_msg.header = image->header;
    disparity_msg.image = *image;
    disparity_pub_.publish(disparity_msg);
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

  std::string trigger_mode;
  switch (config.groups.capture.TriggerMode)
  {
    case 0:
      trigger_mode = "Software";
      break;

    case 1:
      trigger_mode = "FallingEdge";
      break;

    case 2:
      trigger_mode = "RisingEdge";
      break;

    default:
      trigger_mode = "Software";
      break;
  }

  ensenso_ptr_->setUseDisparityMapAreaOfInterest(config.groups.capture.DisparityMapAOI);
  // Flexview and binning only work in 'Software' trigger mode and with the projector on
  ensenso_ptr_->setBinning(1);
  ensenso_ptr_->setFlexView(config.groups.capture.FlexView, config.groups.capture.FlexViewImages);

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


sensor_msgs::ImagePtr EnsensoPostprocess::toImageMsg(
  pcl::PCLImage pcl_image,
  const std::string &frame,
  const ros::Time &image_time)
{
  unsigned char *image_array = reinterpret_cast<unsigned char *>(&pcl_image.data[0]);
  int type(CV_8UC1);
  std::string encoding("mono8");
  if (pcl_image.encoding == "CV_8UC3")
  {
    type = CV_8UC3;
    encoding = "bgr8";
  }
  cv::Mat image_mat(pcl_image.height, pcl_image.width, type, image_array);
  std_msgs::Header header;
  header.frame_id = frame;
  header.stamp = image_time;
  return cv_bridge::CvImage(header, encoding, image_mat).toImageMsg();
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
