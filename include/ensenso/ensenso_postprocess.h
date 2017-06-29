#include <ros/ros.h>
#include <ensenso/ensenso_grabber.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

class EnsensoPostprocess
{
  public:
    EnsensoPostprocess();
    ~EnsensoPostprocess();

  private:
    void configureCamera();
    image_transport::Subscriber left_image_sub_;
    image_transport::Subscriber right_image_sub_;
    ros::Publisher cloud_pub_;
    pcl::EnsensoGrabber::Ptr ensenso_ptr_;
};
