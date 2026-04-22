#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <deque>
#include <filesystem>
#include <limits>
#include <algorithm>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <std_srvs/srv/trigger.hpp>
// #include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/LocalCartesian.hpp>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

std::shared_ptr<rclcpp::Node> node;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

struct GpsSample {
    double stamp = 0.0;
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    double eph = 0.0;
    double epv = 0.0;
};

struct GpsAlignmentSample {
    Pose6D pose;
    GpsSample gps;
};

bool useGPS = false;
bool useScanContextLoopClosure = false;
bool gpsAnchorInitialized = false;
bool gpsAlignmentInitialized = false;
double gpsYawOffset = 0.0;
double gpsOffsetX = 0.0;
double gpsOffsetY = 0.0;
double gpsOffsetZ = 0.0;
double gpsMinFixType = 3.0;
double gpsHorizontalSigmaM = 1.0;
double gpsVerticalSigmaM = 2.0;
double gpsSigmaInflate = 2.0;
double gpsHuberThresholdM = 3.0;
double gpsAnchorWarmupSec = 5.0;
double gpsMatchWindowSec = 0.05;
std::string mapFilePath;
std::string gpsTopic;
std::string cloudTopic;
std::string odomTopic;
std::deque<GpsSample> gpsBuf;
std::vector<GpsAlignmentSample> gpsAlignmentSamples;
std::unique_ptr<GeographicLib::LocalCartesian> gpsProjector;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubMapAftPGO, pubLoopScanLocal, pubLoopSubmapLocal;
std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>  pubOdomAftPGO, pubOdomRepubVerifier;
std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pubPathAftPGO;
std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> mapSaveSrv;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::fstream pgTimeSaveStream;

double toSec(const builtin_interfaces::msg::Time& stamp) {
  const double sec = (double) stamp.sec;
  const double nsec = (double) stamp.nanosec;
  return (double)sec + 1e-9*(double)nsec;
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr _laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr _laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

bool isValidGpsFix(const px4_msgs::msg::SensorGps::ConstSharedPtr& gps)
{
    return gps->fix_type >= static_cast<uint8_t>(gpsMinFixType)
        && std::isfinite(gps->latitude_deg)
        && std::isfinite(gps->longitude_deg)
        && std::isfinite(gps->altitude_ellipsoid_m)
        && std::isfinite(gps->eph)
        && std::isfinite(gps->epv)
        && gps->eph > 0.0
        && gps->epv > 0.0;
}

std::optional<GpsSample> projectGpsFix(const px4_msgs::msg::SensorGps::ConstSharedPtr& gps, double stamp)
{
    if (!isValidGpsFix(gps)) {
        return std::nullopt;
    }

    if (!gpsAnchorInitialized) {
        gpsProjector = std::make_unique<GeographicLib::LocalCartesian>(
            gps->latitude_deg,
            gps->longitude_deg,
            gps->altitude_ellipsoid_m);
        gpsAnchorInitialized = true;
        RCLCPP_INFO(node->get_logger(),
            "GPS anchor initialized at lat=%.9f lon=%.9f alt=%.3f",
            gps->latitude_deg, gps->longitude_deg, gps->altitude_ellipsoid_m);
    }

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    gpsProjector->Forward(
        gps->latitude_deg,
        gps->longitude_deg,
        gps->altitude_ellipsoid_m,
        east,
        north,
        up);

    GpsSample sample;
    sample.stamp = stamp;
    sample.east = east;
    sample.north = north;
    sample.up = up;
    sample.eph = gps->eph;
    sample.epv = gps->epv;
    return sample;
}

void gpsHandler(const px4_msgs::msg::SensorGps::ConstSharedPtr _gps)
{
    if(!useGPS) {
        return;
    }

    const double stamp = node->now().seconds();
    auto projected = projectGpsFix(_gps, stamp);
    if(!projected) {
        return;
    }

    mBuf.lock();
    gpsBuf.push_back(*projected);
    while (gpsBuf.size() > 5000) {
        gpsBuf.pop_front();
    }
    mBuf.unlock();
} // gpsHandler

std::optional<GpsSample> findNearestGps(double stamp)
{
    std::optional<GpsSample> best;
    double best_dt = std::numeric_limits<double>::max();

    while (!gpsBuf.empty() && gpsBuf.front().stamp < stamp - gpsMatchWindowSec) {
        gpsBuf.pop_front();
    }

    for (const auto& sample : gpsBuf) {
        const double dt = std::abs(sample.stamp - stamp);
        if (dt < best_dt) {
            best = sample;
            best_dt = dt;
        }
        if (sample.stamp > stamp + gpsMatchWindowSec) {
            break;
        }
    }

    if (best && best_dt <= gpsMatchWindowSec) {
        return best;
    }
    return std::nullopt;
}

void rotateGpsToMap(const GpsSample& gps, double& x, double& y, double& z)
{
    const double c = std::cos(gpsYawOffset);
    const double s = std::sin(gpsYawOffset);
    x = c * gps.east - s * gps.north + gpsOffsetX;
    y = s * gps.east + c * gps.north + gpsOffsetY;
    z = gps.up + gpsOffsetZ;
}

bool updateGpsAlignment(const Pose6D& pose, const GpsSample& gps, double stamp)
{
    if (gpsAlignmentInitialized) {
        return true;
    }

    gpsAlignmentSamples.push_back({pose, gps});
    if (gpsAlignmentSamples.empty()) {
        return false;
    }

    const auto& first = gpsAlignmentSamples.front();
    const double elapsed = stamp - first.gps.stamp;
    const double pose_dx = pose.x - first.pose.x;
    const double pose_dy = pose.y - first.pose.y;
    const double gps_dx = gps.east - first.gps.east;
    const double gps_dy = gps.north - first.gps.north;
    const double pose_dist = std::hypot(pose_dx, pose_dy);
    const double gps_dist = std::hypot(gps_dx, gps_dy);

    if (pose_dist >= 1.0 && gps_dist >= 1.0) {
        const double pose_heading = std::atan2(pose_dy, pose_dx);
        const double gps_heading = std::atan2(gps_dy, gps_dx);
        gpsYawOffset = pose_heading - gps_heading;
    } else if (elapsed < gpsAnchorWarmupSec) {
        return false;
    } else {
        RCLCPP_WARN(node->get_logger(),
            "GPS warmup had only %.2fm pose motion and %.2fm GPS motion; using zero yaw offset",
            pose_dist, gps_dist);
        gpsYawOffset = 0.0;
    }

    const double c = std::cos(gpsYawOffset);
    const double s = std::sin(gpsYawOffset);
    gpsOffsetX = first.pose.x - (c * first.gps.east - s * first.gps.north);
    gpsOffsetY = first.pose.y - (s * first.gps.east + c * first.gps.north);
    gpsOffsetZ = first.pose.z - first.gps.up;
    gpsAlignmentInitialized = true;

    RCLCPP_INFO(node->get_logger(),
        "GPS-to-map alignment ready: yaw_offset=%.3f rad, offset=(%.3f, %.3f, %.3f)",
        gpsYawOffset, gpsOffsetX, gpsOffsetY, gpsOffsetZ);
    return true;
}

gtsam::noiseModel::Base::shared_ptr gpsNoiseModel(const GpsSample& gps)
{
    const double sigma_xy = std::max(gpsHorizontalSigmaM, gps.eph) * gpsSigmaInflate;
    const double sigma_z = std::max(gpsVerticalSigmaM, gps.epv) * gpsSigmaInflate;
    gtsam::Vector robustNoiseVector3(3);
    robustNoiseVector3 << sigma_xy, sigma_xy, sigma_z;
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(gpsHuberThresholdM),
        gtsam::noiseModel::Diagonal::Sigmas(robustNoiseVector3));
}

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

} // initNoises

Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes.at(node_idx) * 1e9));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        // geometry_msgs::msg::Quaternion q_msg;
        // tf2::convert(q, q_msg);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock(); 
    pubOdomAftPGO->publish(odomAftPGO); // last pose 
    pubPathAftPGO->publish(pathAftPGO); // poses 

    static std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    // tf::Transform transform;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odomAftPGO.header.stamp;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "/aft_pgo";
    // transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    transform.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transform.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transform.transform.translation.z = odomAftPGO.pose.pose.position.z;
    // tf::Quaternion q;
    // q.setW(odomAftPGO.pose.pose.orientation.w);
    // q.setX(odomAftPGO.pose.pose.orientation.x);
    // q.setY(odomAftPGO.pose.pose.orientation.y);
    // q.setZ(odomAftPGO.pose.pose.orientation.z);
    // transform.setRotation(q);
    transform.transform.rotation.x = odomAftPGO.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftPGO.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftPGO.pose.pose.orientation.z;
    transform.transform.rotation.w = odomAftPGO.pose.pose.orientation.w;
    br->sendTransform(transform);
} // pubPath

void updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock(); 
        (void)root_idx;
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock(); 
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud


std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    // loop verification 
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe. 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while(rclcpp::ok())
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && toSec(odometryBuf.front()->header.stamp) < toSec(fullResBuf.front()->header.stamp))
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = toSec(odometryBuf.front()->header.stamp);
            timeLaser = toSec(fullResBuf.front()->header.stamp);
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            std::optional<GpsSample> gpsForThisKF;
            if (useGPS) {
                gpsForThisKF = findNearestGps(timeLaserOdometry);
            }
            mBuf.unlock(); 

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue; 

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock(); 
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

            laserCloudMapPGORedraw = true;
            mKF.unlock(); 

            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    // runISAM2opt();          
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // gps factor 
                    if(useGPS && gpsForThisKF) {
                        if (updateGpsAlignment(pose_curr, *gpsForThisKF, timeLaserOdometry)) {
                            double gx = 0.0;
                            double gy = 0.0;
                            double gz = 0.0;
                            rotateGpsToMap(*gpsForThisKF, gx, gy, gz);
                            gtsam::Point3 gpsConstraint(gx, gy, gz);
                            gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, gpsNoiseModel(*gpsForThisKF)));
                            cout << "GPS factor added at node " << curr_node_idx << endl;
                        }
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);                
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            // save utility 
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan 
            pgTimeSaveStream << timeLaser << std::endl; // path 
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performSCLoopClosure(void)
{
    if (!useScanContextLoopClosure)
        return;

    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change 
    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void process_icp(void)
{
    while(rclcpp::ok())
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(node->get_logger(), "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // runISAM2opt();
                mtxPosegraph.unlock();
            } 
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = 10.0; 
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = 1; 
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            cout << "running isam2 optimization ..." << endl;
            mtxPosegraph.unlock();

            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

void process_viz_map(void)
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    rclcpp::Rate rate(vizmapFrequency);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz

pcl::PointCloud<PointType>::Ptr buildOptimizedMap()
{
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());

    mKF.lock();
    const size_t pose_count = std::min(keyframeLaserClouds.size(), keyframePosesUpdated.size());
    for (size_t node_idx = 0; node_idx < pose_count; ++node_idx) {
        *map += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
    }
    mKF.unlock();

    if (!map->empty()) {
        downSizeFilterMapPGO.setInputCloud(map);
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        downSizeFilterMapPGO.filter(*filtered);
        map = filtered;
    }

    return map;
}

void mapSaveCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (mapFilePath.empty()) {
        response->success = false;
        response->message = "map_file_path is empty";
        return;
    }

    if (!gtSAMgraphMade || keyframeLaserClouds.empty()) {
        response->success = false;
        response->message = "no pose-graph keyframes available";
        return;
    }

    mtxPosegraph.lock();
    try {
        if (initialEstimate.size() > 0 || gtSAMgraph.size() > 0) {
            runISAM2opt();
        }
    } catch (const std::exception& exc) {
        mtxPosegraph.unlock();
        response->success = false;
        response->message = std::string("final ISAM2 optimization failed: ") + exc.what();
        return;
    }
    mtxPosegraph.unlock();

    auto map = buildOptimizedMap();
    if (map->empty()) {
        response->success = false;
        response->message = "optimized map is empty";
        return;
    }

    try {
        const auto parent = std::filesystem::path(mapFilePath).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(mapFilePath, *map);
    } catch (const std::exception& exc) {
        response->success = false;
        response->message = std::string("failed to write PCD: ") + exc.what();
        return;
    }

    response->success = true;
    response->message = "optimized map saved";
    RCLCPP_INFO(node->get_logger(), "Saved optimized PGO map to %s with %zu points",
        mapFilePath.c_str(), map->size());
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("laserPGO");

  node->declare_parameter("use_scan_context_loop_closure", false);
  useScanContextLoopClosure = node->get_parameter("use_scan_context_loop_closure").get_parameter_value().get<bool>();
  node->declare_parameter("use_gps_factor", false);
  useGPS = node->get_parameter("use_gps_factor").get_parameter_value().get<bool>();
  node->declare_parameter("map_file_path", "");
  mapFilePath = node->get_parameter("map_file_path").get_parameter_value().get<std::string>();
  node->declare_parameter("cloud_topic", "/velodyne_cloud_registered_local");
  cloudTopic = node->get_parameter("cloud_topic").get_parameter_value().get<std::string>();
  node->declare_parameter("odom_topic", "/aft_mapped_to_init");
  odomTopic = node->get_parameter("odom_topic").get_parameter_value().get<std::string>();
  node->declare_parameter("gps_topic", "/fmu/out/vehicle_gps_position");
  gpsTopic = node->get_parameter("gps_topic").get_parameter_value().get<std::string>();

  node->declare_parameter("gps_min_fix_type", 3.0);
  gpsMinFixType = node->get_parameter("gps_min_fix_type").get_parameter_value().get<double>();
  node->declare_parameter("gps_horizontal_sigma_m", 1.0);
  gpsHorizontalSigmaM = node->get_parameter("gps_horizontal_sigma_m").get_parameter_value().get<double>();
  node->declare_parameter("gps_vertical_sigma_m", 2.0);
  gpsVerticalSigmaM = node->get_parameter("gps_vertical_sigma_m").get_parameter_value().get<double>();
  node->declare_parameter("gps_sigma_inflate", 2.0);
  gpsSigmaInflate = node->get_parameter("gps_sigma_inflate").get_parameter_value().get<double>();
  node->declare_parameter("gps_huber_threshold_m", 3.0);
  gpsHuberThresholdM = node->get_parameter("gps_huber_threshold_m").get_parameter_value().get<double>();
  node->declare_parameter("gps_anchor_warmup_sec", 5.0);
  gpsAnchorWarmupSec = node->get_parameter("gps_anchor_warmup_sec").get_parameter_value().get<double>();
  node->declare_parameter("gps_match_window_sec", 0.05);
  gpsMatchWindowSec = node->get_parameter("gps_match_window_sec").get_parameter_value().get<double>();

  node->declare_parameter("save_directory", "/"); // pose assignment every k m move 
  save_directory = node->get_parameter("save_directory").get_parameter_value().get<std::string>();
    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out); 
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    pgScansDirectory = save_directory + "Scans/";
    std::filesystem::remove_all(pgScansDirectory);
    std::filesystem::create_directories(pgScansDirectory);

  node->declare_parameter("keyframe_meter_gap", 2.0); // pose assignment every k m move 
  keyframeMeterGap = node->get_parameter("keyframe_meter_gap").get_parameter_value().get<double>();
  node->declare_parameter("keyframe_deg_gap", 10.0); // pose assignment every k deg rot 
  keyframeDegGap = node->get_parameter("keyframe_deg_gap").get_parameter_value().get<double>();
    keyframeRadGap = deg2rad(keyframeDegGap);

  node->declare_parameter("sc_dist_thres", 0.2);
  scDistThres = node->get_parameter("sc_dist_thres").get_parameter_value().get<double>();
  node->declare_parameter("sc_max_radius", 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor 
  scMaximumRadius = node->get_parameter("sc_max_radius").get_parameter_value().get<double>();

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4; 
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
  node->declare_parameter("mapviz_filter_size", 0.4); // pose assignment every k frames 
  mapVizFilterSize = node->get_parameter("mapviz_filter_size").get_parameter_value().get<double>();
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	auto subLaserCloudFullRes = node->create_subscription<sensor_msgs::msg::PointCloud2>(cloudTopic, 100, laserCloudFullResHandler);
	auto subLaserOdometry = node->create_subscription<nav_msgs::msg::Odometry>(odomTopic, 100, laserOdometryHandler);
    auto gpsQos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
	auto subGPS = node->create_subscription<px4_msgs::msg::SensorGps>(gpsTopic, gpsQos, gpsHandler);

	pubOdomAftPGO = node->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = node->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);
	pubPathAftPGO = node->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = node->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);

	pubLoopScanLocal = node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);
    mapSaveSrv = node->create_service<std_srvs::srv::Trigger>("/map_save", mapSaveCallback);

    RCLCPP_INFO(node->get_logger(),
        "laserPGO ready: sc_loop=%s gps_factor=%s cloud=%s odom=%s gps=%s map_file=%s",
        useScanContextLoopClosure ? "true" : "false",
        useGPS ? "true" : "false",
        cloudTopic.c_str(),
        odomTopic.c_str(),
        gpsTopic.c_str(),
        mapFilePath.c_str());

	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection 
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

 	rclcpp::spin(node);

    if (posegraph_slam.joinable()) posegraph_slam.join();
    if (lc_detection.joinable()) lc_detection.join();
    if (icp_calculation.joinable()) icp_calculation.join();
    if (isam_update.joinable()) isam_update.join();
    if (viz_map.joinable()) viz_map.join();
    if (viz_path.joinable()) viz_path.join();

	return 0;
}
