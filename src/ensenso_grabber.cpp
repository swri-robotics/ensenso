#include "ensenso/ensenso_grabber.h"


void ensensoExceptionHandling (const NxLibException &ex,
                 std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
         ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
  {
    NxLibCommand cmd ("");
    PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
  }
}


pcl::EnsensoGrabber::EnsensoGrabber () :
  device_open_ (false),
  last_stereo_pattern_(""),
  store_calibration_pattern_ (false),
  running_ (false),
  tcp_open_ (false)
{
  point_cloud_signal_ = createSignal<sig_cb_ensenso_point_cloud> ();
  images_signal_ = createSignal<sig_cb_ensenso_images> ();
  disparity_signal_ = createSignal<sig_cb_ensenso_disparity> ();
  point_cloud_images_signal_ = createSignal<sig_cb_ensenso_point_cloud_images> ();
  point_cloud_disparity_signal_ = createSignal<sig_cb_ensenso_point_cloud_disparity> ();
  images_disparity_signal_ = createSignal<sig_cb_ensenso_images_disparity> ();
  point_cloud_images_disparity_signal_ = createSignal<sig_cb_ensenso_point_cloud_images_disparity> ();
  PCL_INFO ("Initialising nxLib\n");
  try
  {
    nxLibInitialize ();
    root_.reset (new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "EnsensoGrabber");
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

pcl::EnsensoGrabber::~EnsensoGrabber () throw ()
{
  try
  {
    stop ();
    root_.reset ();

    disconnect_all_slots<sig_cb_ensenso_point_cloud> ();
    disconnect_all_slots<sig_cb_ensenso_images> ();
    disconnect_all_slots<sig_cb_ensenso_disparity> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_disparity> ();
    disconnect_all_slots<sig_cb_ensenso_images_disparity> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images_disparity> ();

    if (tcp_open_)
    {
      closeTcpPort ();
    }
    nxLibFinalize ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

bool pcl::EnsensoGrabber::calibrateHandEye (const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                                            const Eigen::Affine3d &camera_seed,
                                            const Eigen::Affine3d &pattern_seed,
                                            const std::string setup,
                                            Eigen::Affine3d &estimated_camera_pose,
                                            Eigen::Affine3d &estimated_pattern_pose,
                                            int &iterations,
                                            double &reprojection_error) const
{
  if ( (*root_)[itmVersion][itmMajor] <= 1 && (*root_)[itmVersion][itmMinor] < 3)
  {
    PCL_WARN("EnsensoSDK 1.3.x fixes bugs into calibration optimization\n");
    PCL_WARN("please update your SDK! http://www.ensenso.de/support/sdk-download\n");
  }
  NxLibCommand calibrate (cmdCalibrateHandEye);
  try
  {
    // Check consistency
    if (!boost::iequals (setup, valFixed) && !boost::iequals (setup, valMoving))
    {
      PCL_WARN("Received invalid setup value: %s\n", setup.c_str());
      return (false);
    }
    // Set Hand-Eye calibration parameters
    PCL_DEBUG("Setting calibration parameters\n");
    std::string target;
    if (boost::iequals (setup, valFixed))
      target = valWorkspace;
    else
      target = valHand;
    Eigen::Affine3d eigen_pose;
    // Feed robot transformations
    PCL_DEBUG("Populating robot poses\n");
    std::vector<std::string> json_poses;
    json_poses.resize (robot_poses.size ());
    for (uint i = 0; i < robot_poses.size(); ++i)
    {
      eigen_pose = robot_poses[i];
      eigen_pose.translation () *= 1000.0; // meters -> millimeters
      matrixToJson(eigen_pose, json_poses[i]);
    }
    PCL_DEBUG("Executing...\n");
    // Convert camera seed to Json
    std::string json_camera_seed, json_pattern_seed;
    PCL_DEBUG("Populating seeds\n");
    eigen_pose = camera_seed;
    eigen_pose.translation () *= 1000.0; // meters -> millimeters
    matrixToJson(eigen_pose, json_camera_seed);
    PCL_DEBUG("camera_seed:\n %s\n", json_camera_seed.c_str());
    // Convert pattern seed to Json
    eigen_pose = pattern_seed;
    eigen_pose.translation () *= 1000.0; // meters -> millimeters
    matrixToJson(pattern_seed, json_pattern_seed);
    PCL_DEBUG("pattern_seed:\n %s\n", json_pattern_seed.c_str());
    // Populate command parameters
    // It's very important to write the parameters in alphabetical order and at the same time!
    calibrate.parameters ()[itmLink].setJson(json_camera_seed, false);
    calibrate.parameters ()[itmPatternPose].setJson(json_pattern_seed, false);
    calibrate.parameters ()[itmSetup] = setup;
    calibrate.parameters ()[itmTarget] = target;
    for (uint i = 0; i < json_poses.size(); ++i)
      calibrate.parameters ()[itmTransformations][i].setJson(json_poses[i], false);
    // Execute the command
    calibrate.execute ();  // It might take some minutes
    if (calibrate.successful())
    {
      // It's very important to read the parameters in alphabetical order and at the same time!
      iterations = calibrate.result()[itmIterations].asInt();
      std::string json_camera_pose = calibrate.result()[itmLink].asJson (true);
      std::string json_pattern_pose = calibrate.result()[itmPatternPose].asJson (true);
      reprojection_error = calibrate.result()[itmReprojectionError].asDouble();
      // Estimated camera pose
      jsonToMatrix(json_camera_pose, estimated_camera_pose);
      estimated_camera_pose.translation () /= 1000.0; // millimeters -> meters
      // Estimated pattern pose
      jsonToMatrix(json_pattern_pose, estimated_pattern_pose);
      estimated_pattern_pose.translation () /= 1000.0; // millimeters -> meters
      PCL_DEBUG("Result:\n %s\n", json_camera_pose.c_str());
      return (true);
    }
    else
      return (false);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "calibrateHandEye");
    return (false);
  }
}

bool pcl::EnsensoGrabber::closeDevice ()
{
  if (!device_open_)
    return (false);

  stop ();
  PCL_INFO ("Closing Ensenso stereo camera\n");

  try
  {
    NxLibCommand (cmdClose).execute ();
    device_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeDevice");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::closeTcpPort ()
{
  try
  {
    nxLibCloseTcpPort ();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

int pcl::EnsensoGrabber::collectPattern (const bool buffer) const
{
  if (!device_open_ || running_)
    return (-1);
  try
  {
    NxLibCommand (cmdCapture).execute ();
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters ()[itmBuffer].set (buffer);
    collect_pattern.parameters ()[itmDecodeData].set (false);
    collect_pattern.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "collectPattern");
    return (-1);
  }
  return getPatternCount();
}

double pcl::EnsensoGrabber::decodePattern () const
{
  double grid_spacing = -1.0;
  if (!device_open_ || running_)
    return (-1.0);
  try
  {
    NxLibCommand (cmdCapture).execute ();
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters ()[itmBuffer].set (false);
    collect_pattern.parameters ()[itmDecodeData].set (true);
    collect_pattern.execute ();
    grid_spacing = collect_pattern.result()[itmGridSpacing].asDouble();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "decodePattern");
    return (-1.0);
  }
  return grid_spacing;
}

bool pcl::EnsensoGrabber::discardPatterns () const
{
  try
  {
    NxLibCommand (cmdDiscardPatterns).execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "discardPatterns");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::estimatePatternPose (Eigen::Affine3d &pose, const bool average) const
{
  try
  {
    NxLibCommand estimate_pattern_pose (cmdEstimatePatternPose);
    estimate_pattern_pose.parameters ()[itmAverage].set (average);
    estimate_pattern_pose.execute ();
    NxLibItem tf = estimate_pattern_pose.result ()[itmPatternPose];
    // Convert tf into a matrix
    if (!jsonToMatrix (tf.asJson (), pose))
      return (false);
    pose.translation () /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}

int pcl::EnsensoGrabber::enumDevices () const
{
  int camera_count = 0;
  try
  {
    NxLibItem cams = NxLibItem ("/Cameras/BySerialNo");
    camera_count = cams.count ();
    // Print information for all cameras in the tree
    PCL_INFO ("Number of connected cameras: %d\n", camera_count);
    PCL_INFO ("Serial No    Model   Status\n");
    for (int n = 0; n < cams.count (); ++n)
    {
      PCL_INFO ("%s   %s   %s\n", cams[n][itmSerialNumber].asString ().c_str (),
            cams[n][itmModelName].asString ().c_str (),
            cams[n][itmStatus].asString ().c_str ());
    }
    PCL_INFO ("\n");
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "enumDevices");
  }
  return (camera_count);
}

bool pcl::EnsensoGrabber::getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  try
  {
    cam_info.width = camera_[itmSensor][itmSize][0].asInt();
    cam_info.height = camera_[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distortion factors
    cam_info.D.resize(5);
    for(std::size_t i = 0; i < cam_info.D.size(); ++i)
      cam_info.D[i] = camera_[itmCalibration][itmMonocular][cam][itmDistortion][i].asDouble();
    // K and R matrices
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        cam_info.K[3*i+j] = camera_[itmCalibration][itmMonocular][cam][itmCamera][j][i].asDouble();
        cam_info.R[3*i+j] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmRotation][j][i].asDouble();
      }
    }
    cam_info.P[0] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][0].asDouble();
    cam_info.P[1] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][0].asDouble();
    cam_info.P[2] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][0].asDouble();
    cam_info.P[3] = 0.0;
    cam_info.P[4] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][1].asDouble();
    cam_info.P[5] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][1].asDouble();
    cam_info.P[6] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][1].asDouble();
    cam_info.P[7] = 0.0;
    cam_info.P[10] = 1.0;
    if (cam == "Right")
    {
      double B = camera_[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
      double fx = cam_info.P[0];
      cam_info.P[3] = (-fx * B);
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getCameraInfo");
    return false;
  }
}

bool pcl::EnsensoGrabber::getCameraInfoRectified(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  bool ret_val = false;
  ret_val = getCameraInfo(cam, cam_info);

  if (ret_val)
  {
    try
    {
      int return_code;
      int bin_factor = camera_[itmParameters][itmCapture][itmBinning].asInt();
      cam_info.binning_x = static_cast<uint32_t>(bin_factor);
      cam_info.binning_y = cam_info.binning_x;
      ret_val = true;
    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "getCameraInfoRectified");
    }
  }

  return ret_val;
}

bool pcl::EnsensoGrabber::getLastCalibrationPattern ( std::vector<int> &grid_size, double &grid_spacing,
                                                      std::vector<Eigen::Vector2d> &left_points,
                                                      std::vector<Eigen::Vector2d> &right_points,
                                                      Eigen::Affine3d &pose) const
{
  // Lock
  pattern_mutex_.lock ();
  std::string stereo_pattern_json = last_stereo_pattern_;
  pose = last_pattern_pose_;
  pattern_mutex_.unlock ();
  // Process
  if ( stereo_pattern_json.empty() )
    return false;
  // Use NxLib JSON API to extract the Raw Stereo Pattern information
  NxLibItem stereo_pattern ("/tmpStereoPattern");
  stereo_pattern.setJson(stereo_pattern_json);
  // Get pattern details
  grid_size.resize(2);
  grid_size[0] = stereo_pattern[itmPattern][itmGridSize][0].asInt();
  grid_size[1] = stereo_pattern[itmPattern][itmGridSize][1].asInt();
  grid_spacing = stereo_pattern[itmPattern][itmGridSpacing].asDouble();
  int rows = grid_size[0];
  int cols = grid_size[1];
  left_points.resize(rows*cols);
  right_points.resize(rows*cols);
  for (uint i = 0; i < left_points.size(); ++i)
  {
    left_points[i][0] = stereo_pattern[itmPoints][0][i][0].asDouble();
    left_points[i][1] = stereo_pattern[itmPoints][0][i][1].asDouble();
    right_points[i][0] = stereo_pattern[itmPoints][1][i][0].asDouble();
    right_points[i][1] = stereo_pattern[itmPoints][1][i][1].asDouble();
  }
  stereo_pattern.erase();
  return true;
}

float pcl::EnsensoGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return fps_;
}

std::string pcl::EnsensoGrabber::getName () const
{
  return ("EnsensoGrabber");
}

std::string pcl::EnsensoGrabber::getOpenCVType (const int channels,
                        const int bpe,
                        const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str (boost::format ("CV_%i%cC%i") % bits % type % channels));
}

pcl::uint64_t pcl::EnsensoGrabber::getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

int pcl::EnsensoGrabber::getPatternCount () const
{
  return ( (*root_)[itmParameters][itmPatternCount].asInt ());
}

bool pcl::EnsensoGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (!device_open_)
  {
    return (false);
  }

  if (running_)
  {
    return (false);
  }

  try
  {
    NxLibCommand (cmdCapture).execute ();
    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    int width, height;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;
    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabSingleCloud");
    return (false);
  }
}

bool pcl::EnsensoGrabber::isRunning () const
{
  return (running_);
}

bool pcl::EnsensoGrabber::isTcpPortOpen () const
{
  return (tcp_open_);
}

bool pcl::EnsensoGrabber::jsonToMatrix (const std::string json, Eigen::Affine3d &matrix) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    convert.parameters ()[itmTransformation].setJson (json);
    convert.execute ();
    Eigen::Affine3d tmp (Eigen::Affine3d::Identity ());
    // Rotation
    tmp.linear ().col (0) = Eigen::Vector3d (convert.result ()[itmTransformation][0][0].asDouble (),
                         convert.result ()[itmTransformation][0][1].asDouble (),
                         convert.result ()[itmTransformation][0][2].asDouble ());
    tmp.linear ().col (1) = Eigen::Vector3d (convert.result ()[itmTransformation][1][0].asDouble (),
                         convert.result ()[itmTransformation][1][1].asDouble (),
                         convert.result ()[itmTransformation][1][2].asDouble ());
    tmp.linear ().col (2) = Eigen::Vector3d (convert.result ()[itmTransformation][2][0].asDouble (),
                         convert.result ()[itmTransformation][2][1].asDouble (),
                         convert.result ()[itmTransformation][2][2].asDouble ());
    // Translation
    tmp.translation () = Eigen::Vector3d (convert.result ()[itmTransformation][3][0].asDouble (),
                        convert.result ()[itmTransformation][3][1].asDouble (),
                        convert.result ()[itmTransformation][3][2].asDouble ());
    matrix = tmp;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonToMatrix");
    return (false);
  }
}

bool pcl::EnsensoGrabber::matrixToJson (const Eigen::Affine3d &matrix, std::string &json, 
                                        const bool pretty_format) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    // Rotation
    convert.parameters ()[itmTransformation][0][0].set (matrix.linear ().col (0)[0]);
    convert.parameters ()[itmTransformation][0][1].set (matrix.linear ().col (0)[1]);
    convert.parameters ()[itmTransformation][0][2].set (matrix.linear ().col (0)[2]);
    convert.parameters ()[itmTransformation][0][3].set (0.0);
    convert.parameters ()[itmTransformation][1][0].set (matrix.linear ().col (1)[0]);
    convert.parameters ()[itmTransformation][1][1].set (matrix.linear ().col (1)[1]);
    convert.parameters ()[itmTransformation][1][2].set (matrix.linear ().col (1)[2]);
    convert.parameters ()[itmTransformation][1][3].set (0.0);
    convert.parameters ()[itmTransformation][2][0].set (matrix.linear ().col (2)[0]);
    convert.parameters ()[itmTransformation][2][1].set (matrix.linear ().col (2)[1]);
    convert.parameters ()[itmTransformation][2][2].set (matrix.linear ().col (2)[2]);
    convert.parameters ()[itmTransformation][2][3].set (0.0);
    // Translation
    convert.parameters ()[itmTransformation][3][0].set (matrix.translation ()[0]);
    convert.parameters ()[itmTransformation][3][1].set (matrix.translation ()[1]);
    convert.parameters ()[itmTransformation][3][2].set (matrix.translation ()[2]);
    convert.parameters ()[itmTransformation][3][3].set (1.0);
    convert.execute ();
    json = convert.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "matrixToJson");
    return (false);
  }
}

bool pcl::EnsensoGrabber::openDevice (std::string serial)
{
  if (device_open_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Cannot open multiple devices!");
  PCL_INFO ("Opening Ensenso stereo camera S/N: %s\n", serial.c_str());
  try
  {
    // Create a pointer referencing the camera's tree item, for easier access:
    camera_ = (*root_)[itmCameras][itmBySerialNo][serial];
    if (!camera_.exists () || camera_[itmType] != valStereo)
      PCL_THROW_EXCEPTION (pcl::IOException, "Please connect a single stereo camera to your computer!");
    NxLibCommand open (cmdOpen);
    open.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
    open.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openDevice");
    return (false);
  }
  device_open_ = true;
  return (true);
}

bool pcl::EnsensoGrabber::openTcpPort (const int port)
{
  try
  {
    nxLibOpenTcpPort (port);
    tcp_open_ = true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openTcpPort");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::postProcessImages(
  const std::vector<pcl::uint8_t>& left_image,
  const std::vector<pcl::uint8_t>& right_image,
  const int width,
  const int height,
  PairOfImages& rect_images,
  pcl::PCLImage& disparity,
  int& min_disparity,
  int& max_disparity,
  pcl::PointCloud<pcl::PointXYZ>& cloud,
  std::string& operation_status) const
{
  bool ret_val = false;
  if (!device_open_)
  {
    operation_status = "Device not open";
    return false;
  }
  int return_code;
  std::vector<pcl::uint8_t> left_image_resized = left_image;
  std::vector<pcl::uint8_t> right_image_resized = right_image;
  left_image_resized.resize(width * height * sizeof(uint8_t));
  right_image_resized.resize(width * height * sizeof(uint8_t));

  try
  {
    camera_[itmImages][itmRaw][itmLeft].setBinaryData(
      &return_code,
      left_image_resized,
      width,
      height,
      1,
      false);
    operation_status = std::string(nxLibTranslateReturnCode(return_code));
    if (return_code == NxLibOperationSucceeded)
    {
      camera_[itmImages][itmRaw][itmRight].setBinaryData(
        &return_code,
        right_image_resized,
        width,
        height,
        1,
        false);
      operation_status = std::string(nxLibTranslateReturnCode(return_code));
      if (return_code == NxLibOperationSucceeded)
      {
        // Do the stereo matching
        NxLibCommand(cmdRectifyImages).execute();
        std::vector<float> pointMap;
        int computed_width;
        int computed_height;
        int channels;
        int bpe;
        bool isFloat;

        camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo(
          &computed_width,
          &computed_height,
          &channels,
          &bpe,
          &isFloat,
          0);
        rect_images.first.data.resize(computed_width * computed_height * sizeof(float));
        rect_images.second.data.resize(computed_width * computed_height * sizeof(float));
        camera_[itmImages][itmRectified][itmLeft].getBinaryData(
          rect_images.first.data.data(),
          rect_images.first.data.size(),
          0,
          0);
        camera_[itmImages][itmRectified][itmRight].getBinaryData(
          rect_images.second.data.data(),
          rect_images.second.data.size(),
          0,
          0);
        rect_images.first.encoding = getOpenCVType(channels, bpe, isFloat);
        rect_images.second.encoding = rect_images.first.encoding;
        rect_images.first.width = computed_width;
        rect_images.second.width = rect_images.first.width;
        rect_images.first.height = computed_height;
        rect_images.second.height = rect_images.first.height;

        NxLibCommand(cmdComputeDisparityMap).execute();
        NxLibItem dispMap = camera_[itmImages][itmDisparityMap];
        min_disparity = camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmScaledMinimumDisparity].asInt();
        max_disparity = min_disparity;
        max_disparity += camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmNumberOfDisparities].asInt();
        max_disparity -= 1;
        dispMap.getBinaryDataInfo (&computed_width, &computed_height, 0, 0, 0, 0);
        disparity.width = static_cast<pcl::uint32_t>(computed_width);
        disparity.height = static_cast<pcl::uint32_t>(computed_height);
        disparity.data.resize(computed_width * computed_height * sizeof(short));
        disparity.encoding = "CV_16SC1";
        dispMap.getBinaryData(disparity.data.data(), width * height * sizeof(short), 0, 0);

        NxLibCommand(cmdComputePointMap).execute();
        camera_[itmImages][itmPointMap].getBinaryDataInfo(
          &computed_width,
          &computed_height,
          0,
          0,
          0,
          0);
        camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
        // Copy point cloud and convert in meters
        cloud.resize (computed_height * computed_width);
        cloud.width = computed_width;
        cloud.height = computed_height;
        cloud.is_dense = false;
        // Copy data in point cloud (and convert millimeters to meters)
        for (size_t i = 0; i < pointMap.size (); i += 3)
        {
          cloud.points[i / 3].x = pointMap[i] / 1000.0;
          cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
          cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
        }
        ret_val = true;
      }
    }
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getPointCloudFromImage");
  }
  return ret_val;
}

void pcl::EnsensoGrabber::processGrabbing ()
{
  bool continue_grabbing = running_;
  int min_disparity;
  int max_disparity;
  while (continue_grabbing)
  {
    try
    {
      // Publish cloud / images
      if ((num_slots<sig_cb_ensenso_point_cloud> () > 0) ||
          (num_slots<sig_cb_ensenso_images> () > 0) ||
          (num_slots<sig_cb_ensenso_disparity> () > 0) ||
          (num_slots<sig_cb_ensenso_point_cloud_images> () > 0) ||
          (num_slots<sig_cb_ensenso_point_cloud_disparity> () > 0) ||
          (num_slots<sig_cb_ensenso_images_disparity> () > 0) ||
          (num_slots<sig_cb_ensenso_point_cloud_images_disparity> () > 0))
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLImagePtr disparity (new pcl::PCLImage);
        boost::shared_ptr<PairOfImages> rawimages (new PairOfImages);
        boost::shared_ptr<PairOfImages> rectifiedimages (new PairOfImages);
        // Update FPS
        static double last = pcl::getTime ();
        double now = pcl::getTime ();
        fps_mutex_.lock ();
        fps_ = float( 1.0 / (now - last) );
        fps_mutex_.unlock ();
        last = now;
        
        NxLibCommand (cmdCapture).execute ();
        double timestamp;
        camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);

        // Gather images
        pattern_mutex_.lock ();
        last_stereo_pattern_ = std::string("");
        pattern_mutex_.unlock ();
        if ((num_slots<sig_cb_ensenso_images> () > 0) ||
            (num_slots<sig_cb_ensenso_point_cloud_images> () > 0) ||
            (num_slots<sig_cb_ensenso_images_disparity> () > 0) ||
            (num_slots<sig_cb_ensenso_point_cloud_images_disparity> () > 0))
        {
          // Rectify images
          NxLibCommand (cmdRectifyImages).execute ();
          int width, height, channels, bpe;
          bool isFlt, collected_pattern = false;
          // Try to collect calibration pattern and overlay it to the raw image
          // If store_calibration_pattern_ is true, it estimates the pattern pose and store it at last_stereo_pattern_
          if (store_calibration_pattern_)
            discardPatterns();
          try
          {
            NxLibCommand collect_pattern (cmdCollectPattern);
            collect_pattern.parameters ()[itmBuffer].set (store_calibration_pattern_);
            collect_pattern.parameters ()[itmDecodeData].set (true);
            collect_pattern.execute ();
            if (store_calibration_pattern_)
            {
              // estimatePatternPose() takes ages, so, we use the raw data
              // Raw stereo pattern info
              NxLibCommand get_pattern_buffers (cmdGetPatternBuffers);
              get_pattern_buffers.execute ();
              pattern_mutex_.lock ();
              last_stereo_pattern_ = get_pattern_buffers.result()[itmStereo][0].asJson (true);
              // Pattern pose
              estimatePatternPose (last_pattern_pose_, false);
              pattern_mutex_.unlock ();
            }
            collected_pattern = true;
          }
          catch (const NxLibException &ex)
          {
            // if failed to collect the pattern will read the RAW images anyway.
          }

          if (collected_pattern)
          {
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize (width * height * sizeof(float));
            rawimages->second.data.resize (width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
            camera_[itmImages][itmWithOverlay][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize (width * height * sizeof(float));
            rectifiedimages->second.data.resize (width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);
          }
          else
          {
            camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp (timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize (width * height * sizeof(float));
            rawimages->second.data.resize (width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRaw][itmLeft].getBinaryData (rawimages->first.data.data (), rawimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRaw][itmRight].getBinaryData (rawimages->second.data.data (), rawimages->second.data.size (), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize (width * height * sizeof(float));
            rectifiedimages->second.data.resize (width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType (channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData (rectifiedimages->first.data.data (), rectifiedimages->first.data.size (), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData (rectifiedimages->second.data.data (), rectifiedimages->second.data.size (), 0, 0);
          }
        }

        // Gather point cloud
        if ((num_slots<sig_cb_ensenso_point_cloud> () > 0) ||
            (num_slots<sig_cb_ensenso_disparity> () > 0) ||
            (num_slots<sig_cb_ensenso_point_cloud_images> () > 0) ||
            (num_slots<sig_cb_ensenso_images_disparity> () > 0) ||
            (num_slots<sig_cb_ensenso_point_cloud_images_disparity> () > 0))
        {
          // Stereo matching task. If we are getting the disparity or point
          // cloud we will need this information
          NxLibCommand (cmdComputeDisparityMap).execute ();
          // If we need the point cloud we will convert the disparity
          // information into the point cloud
          if((num_slots<sig_cb_ensenso_disparity> () > 0) ||
             (num_slots<sig_cb_ensenso_images_disparity> () > 0) ||
             (num_slots<sig_cb_ensenso_point_cloud_images_disparity> () > 0))
          {
            // Get info about the computed point map and copy it into a std::vector
            NxLibItem dispMap = camera_[itmImages][itmDisparityMap];
            min_disparity = camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmScaledMinimumDisparity].asInt();
            max_disparity = min_disparity;
            max_disparity += camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmNumberOfDisparities].asInt();
            max_disparity -= 1;
            int width, height;
            dispMap.getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
            disparity->header.stamp = getPCLStamp (timestamp);
            disparity->width = width;
            disparity->height = height;
            disparity->data.resize(width * height * sizeof(short));
            disparity->encoding = "CV_16SC1";
            dispMap.getBinaryData(disparity->data.data(), width * height * sizeof(short), 0, 0);
          }

          // If we need the point cloud we will convert the disparity
          // information into the point cloud
          if((num_slots<sig_cb_ensenso_point_cloud> () > 0) ||
             (num_slots<sig_cb_ensenso_point_cloud_images> () > 0) ||
             (num_slots<sig_cb_ensenso_point_cloud_images_disparity> () > 0))
          {
            // Convert disparity map into XYZ data for each pixel
            NxLibCommand (cmdComputePointMap).execute ();
            // Get info about the computed point map and copy it into a std::vector
            std::vector<float> pointMap;
            int width, height;
            camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
            camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
            // Copy point cloud and convert in meters
            cloud->header.stamp = getPCLStamp (timestamp);
            cloud->points.resize (height * width);
            cloud->width = width;
            cloud->height = height;
            cloud->is_dense = false;
            // Copy data in point cloud (and convert milimeters in meters)
            for (size_t i = 0; i < pointMap.size (); i += 3)
            {
              cloud->points[i / 3].x = pointMap[i] / 1000.0;
              cloud->points[i / 3].y = pointMap[i + 1] / 1000.0;
              cloud->points[i / 3].z = pointMap[i + 2] / 1000.0;
            }
          }
        }
        // Publish signals
        if (num_slots<sig_cb_ensenso_point_cloud_images_disparity> () > 0)
        {
          point_cloud_images_disparity_signal_->operator () (cloud, rawimages, rectifiedimages, disparity, min_disparity, max_disparity);
        }
        else if (num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
        {
          point_cloud_images_signal_->operator () (cloud, rawimages, rectifiedimages);
        }
        else if (num_slots<sig_cb_ensenso_images_disparity> () > 0)
        {
          images_disparity_signal_->operator () (rawimages, rectifiedimages, disparity, min_disparity, max_disparity);
        }
        else if (num_slots<sig_cb_ensenso_point_cloud_disparity> () > 0)
        {
          point_cloud_disparity_signal_->operator () (cloud, disparity, min_disparity, max_disparity);
        }
        else if (num_slots<sig_cb_ensenso_point_cloud> () > 0)
        {
          point_cloud_signal_->operator () (cloud);
        }
        else if (num_slots<sig_cb_ensenso_disparity> () > 0)
        {
          disparity_signal_->operator () (disparity, min_disparity, max_disparity);
        }
        else if (num_slots<sig_cb_ensenso_images> () > 0)
        {
          images_signal_->operator () (rawimages,rectifiedimages);
        }
      }
      continue_grabbing = running_;
    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "processGrabbing");
    }
  }
}

void pcl::EnsensoGrabber::storeCalibrationPattern (const bool enable)
{
  store_calibration_pattern_ = enable;
  if (enable)
    PCL_WARN ("Storing calibration pattern. This will clean the pattern buffer continuously\n");
}

bool pcl::EnsensoGrabber::restoreDefaultConfiguration () const
{
  bool result = true;
  result &= setAutoBlackLevel();
  result &= setAutoExposure();
  result &= setAutoGain();
  result &= setBinning();
  result &= setBlackLevelOffset();
  result &= setExposure();
  result &= setFlexView();
  result &= setFrontLight();
  result &= setGain();
  result &= setGainBoost();
  result &= setHardwareGamma();
  result &= setHdr();
  result &= setMinimumDisparity();
  result &= setNumberOfDisparities();
  result &= setOptimizationProfile();
  result &= setPixelClock();
  result &= setProjector();
  result &= setScaling();
  result &= setTargetBrightness();
  result &= setTriggerMode();
  result &= setUseDisparityMapAreaOfInterest();
  return result;
}

bool pcl::EnsensoGrabber::setAutoBlackLevel (const bool enable) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmAutoBlackLevel].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setAutoBlackLevel");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setAutoExposure (const bool enable) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmAutoExposure].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setAutoExposure");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setAutoGain (const bool enable) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmAutoGain].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setAutoGain");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setBinning (const int binning) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmBinning].set (binning);
  }
  catch (NxLibException &ex)
  {
    // TODO: Handle better setBinning exceptions
    //~ ensensoExceptionHandling (ex, "setBinning");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setBlackLevelOffset (const float offset) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmBlackLevelOffset].set (offset);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setBlackLevelOffset");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setExposure (const float exposure) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmExposure].set (exposure);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setExposure");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFlexView (const bool enable, const int imagepairs) const
{
if (!device_open_)
    return (false);
  try
  {
    if (enable && 2 <= imagepairs && imagepairs <= 8)
      camera_[itmParameters][itmCapture][itmFlexView].set (imagepairs);
    else
      camera_[itmParameters][itmCapture][itmFlexView].set (false);
  }
  catch (NxLibException &ex)
  {
    // TODO: Handle better setFlexView exceptions
    //~ ensensoExceptionHandling (ex, "setFlexView");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFrontLight (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmFrontLight].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setFrontLight");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setGain (const float gain) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmGain].set (gain);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setGain");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setGainBoost (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmGainBoost].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setGainBoost");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setGridSpacing (const double grid_spacing) const
{
if (!device_open_)
    return (false);
  try
  {
    (*root_)[itmParameters][itmPattern][itmGridSpacing].set (grid_spacing);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setGridSpacing");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setHardwareGamma (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmHardwareGamma].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setHardwareGamma");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setHdr (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmHdr].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setHdr");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setMinimumDisparity (const int disparity) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmMinimumDisparity].set (disparity);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setMinimumDisparity");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setNumberOfDisparities (const int number) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmNumberOfDisparities].set (number);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "NumberOfDisparities");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setOptimizationProfile (const std::string profile) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmOptimizationProfile].set (profile);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setOptimizationProfile");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setPixelClock (const int pixel_clock) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmPixelClock].set (pixel_clock);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setPixelClock");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setProjector (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmProjector].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setProjector");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setTargetBrightness (const int target) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmTargetBrightness].set (target);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setTargetBrightness");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setScaling (const float scaling) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmScaling].set (scaling);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setScaling");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setTriggerMode (const std::string mode) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmTriggerMode].set (mode);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setTriggerMode");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setUseDisparityMapAreaOfInterest (const bool enable) const
{
if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest].set (enable);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setUseDisparityMapAreaOfInterest");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setDepthChangeCost(const int changecost) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmDepthChangeCost].set (changecost);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setDepthChangeCost");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setDepthStepCost(const int stepcost) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmDepthStepCost].set (stepcost);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setDepthStepCost");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setShadowingThreshold(const int shadowingthreshold) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmStereoMatching][itmShadowingThreshold].set (shadowingthreshold);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setShadowingThreshold");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setUniquenessRatio(const int ratio) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmUniquenessRatio].set (ratio);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setUniquenessRatio");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setMedianFilterRadius(const int radius) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmMedianFilterRadius].set (radius);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setMedianFilterRadius");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setSpeckleComponentThreshold(const int threshold) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmSpeckleRemoval][itmComponentThreshold].set (threshold);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setComponentThreshold");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setSpeckleRegionSize(const int regionsize) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmSpeckleRemoval][itmRegionSize].set (regionsize);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setRegionSize");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFillBorderSpread(const int maximumspread) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmFilling][itmBorderSpread].set (maximumspread);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setBorderSpread");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::setFillRegionSize(const int regionsize) const
{
  if (!device_open_)
    return (false);
  try
  {
    camera_[itmParameters][itmDisparityMap][itmPostProcessing][itmFilling][itmRegionSize].set (regionsize);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setRegionSize");
    return (false);
  }
  return (true);
}

void pcl::EnsensoGrabber::start ()
{
  if (isRunning ())
    return;
  if (!device_open_)
    openDevice (0);
  fps_ = 0.0;
  running_ = true;
  grabber_thread_ = boost::thread (&pcl::EnsensoGrabber::processGrabbing, this);
}

void pcl::EnsensoGrabber::stop ()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback
    grabber_thread_.join ();
  }
}
