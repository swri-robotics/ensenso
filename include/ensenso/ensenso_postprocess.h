#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ensenso/CameraParametersConfig.h>
#include <ensenso/ensenso_grabber.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

class EnsensoPostprocess
{
  public:
    EnsensoPostprocess();
    ~EnsensoPostprocess();

  private:
    void configureCamera(const std::string &camera_id);
    ros::Publisher cloud_pub_;
    image_transport::CameraPublisher left_rectified_pub_;
    image_transport::CameraPublisher right_rectified_pub_;
    ros::Publisher disparity_pub_;
    pcl::EnsensoGrabber::Ptr ensenso_ptr_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> ImageSync;
    boost::shared_ptr<ImageSync> image_sync_;

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;
    void processImages(
      const sensor_msgs::ImageConstPtr &left_image,
      const sensor_msgs::ImageConstPtr &right_image,
      const sensor_msgs::CameraInfoConstPtr &left_info,
      const sensor_msgs::CameraInfoConstPtr &right_info);

    dynamic_reconfigure::Server<ensenso::CameraParametersConfig> reconfigure_server_;
    void cameraParametersCallback(ensenso::CameraParametersConfig &config, uint32_t level);
    sensor_msgs::ImagePtr toImageMsg(
      pcl::PCLImage pcl_image,
      const std::string &frame,
      const ros::Time &image_time);
};
