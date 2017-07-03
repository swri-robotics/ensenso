#include <ros/ros.h>
#include <ensenso/ensenso_grabber.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

class EnsensoPostprocess
{
  public:
    EnsensoPostprocess();
    ~EnsensoPostprocess();

  private:
    void configureCamera(const std::string &camera_id);
    ros::Publisher cloud_pub_;
    pcl::EnsensoGrabber::Ptr ensenso_ptr_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> ImageSync;
    boost::shared_ptr<ImageSync> image_sync_;

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
    void processImages(
      const sensor_msgs::ImageConstPtr &left_image,
      const sensor_msgs::ImageConstPtr &right_image);
};
