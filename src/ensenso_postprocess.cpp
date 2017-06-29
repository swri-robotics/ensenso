#include <ensenso/ensenso_postprocess.h>

EnsensoPostprocess::EnsensoPostprocess()
{
  ros::NodeHandle nh("");
  ros::NodeHandle priv_nh("~");

  std::string serial("150534");
  priv_nh.param("serial", serial, serial);

  ensenso_ptr_.reset(new pcl::EnsensoGrabber);
  ensenso_ptr_->openDevice(serial);
  ensenso_ptr_->openTcpPort();
  ensenso_ptr_->storeCalibrationPattern(false);
  configureCamera();
  ensenso_ptr_->start();

  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("depth/points", 1, false);
}

EnsensoPostprocess::~EnsensoPostprocess()
{
  ensenso_ptr_->closeTcpPort();
  ensenso_ptr_->closeDevice();
}

void EnsensoPostprocess::configureCamera()
{
  if (ensenso_ptr_->isRunning())
  {
    ensenso_ptr_->stop();
  }

  ensenso_ptr_->start();

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
