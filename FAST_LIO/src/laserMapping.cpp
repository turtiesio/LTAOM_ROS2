/*This code is the implementation of our paper "LTA-OM: Long-Term Association
 LiDAR-Inertial Odometry and Mapping".

Current Developer: Zuhao Zou < zuhaozou@yahoo.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Zou, Zuhao, et al. "LTA-OM: Long-Term Association LiDAR-Inertial
    Odometry and Mapping"
[2] Yuan, C., et al. "Std: Stable triangle descriptor for 3d place recognition"
[3] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < zuhaozou@yahoo.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fast_lio/msg/states.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"

#ifdef USE_ikdforest
#include <ikd-Forest/ikd_Forest.h>
#else
#include <ikd-Tree/ikd_Tree.h>
#endif

#include "extra_lib.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

// Global node pointer for ROS2
rclcpp::Node::SharedPtr g_node = nullptr;

// Logger function for IMU_Processing.hpp
rclcpp::Logger get_logger() {
    if (g_node) {
        return g_node->get_logger();
    }
    return rclcpp::get_logger("fast_lio");
}

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
double time_offset_from_lidar = 0.0f;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0,\
    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

double res_mean_last = 0.05;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;

// Time Log Variables
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
int kdtree_delete_counter = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double search_time_rec[100000];

double match_time = 0, solve_time = 0, solve_const_H_time = 0;

bool lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited;
bool dense_map_en = true;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr>  lidar_buffer;
deque<double>          time_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
vector<vector<int>> pointSearchInd_surf;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
bool   point_selected_surf[100000] = {0};
float  res_last[100000] = {0.0};
double total_residual ;
uint64_t curr_Mea_lidarbegtime_NS = 0;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

std::shared_ptr<KD_TREE<PointType>> ikdtree_ptr    (std::make_shared<KD_TREE<PointType>>());
std::shared_ptr<KD_TREE<PointType>> ikdtree_swapptr(std::make_shared<KD_TREE<PointType>>());
std::mutex mtx_ikdtreeptr;
std::condition_variable sig_ikdtreeptr;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);

//estimator inputs and output;
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;
Eigen::Matrix3d rot_replace;
Eigen::Vector3d pos_replace;
Eigen::Vector3d vel_replace;
bool do_posecorrection = false;
shared_ptr<Preprocess> p_pre(new Preprocess());

int multisession_mode = 0;
double correction_ver_thr = 0.45;
double correction_dis_interval = 50;
double dy_mapretrival_range = 150;

std::unordered_map<int, SubmapInfo> unmap_submap_info;
deque<nav_msgs::msg::Path::SharedPtr>  path_buffer;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullCor;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pubOdomCorrection;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubOdomCorrection2;
rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pubTimeCorrection;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubCorrectionId;
ofstream fout_pre, fout_out, fout_dbg;
std::fstream time_ikdrebuild_thread;
mutex mtx_sub_;
std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> submap_buffer;
std::deque<nav_msgs::msg::Odometry::SharedPtr> submap_pose_buffer;
PointVector PointSubmap;
std::deque<PointCloudXYZI::Ptr>  PointToAddHistorical;
int submap_id;
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pos_kdtree, pos_kdtree_prior;
bool holding_for_ikdtreerebuild = false;
bool first_correction_set = false;
std::string save_directory;

PointCloudXYZI::Ptr correctd_cloud_submap(new PointCloudXYZI());
state_ikfom tmp_state;

// TF2 broadcaster
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(g_node->get_logger(), "catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);
    int reflection_map = intensity*10000;
}

int points_cache_size = 0;

void points_cache_collect()
{
    PointVector points_history;
    ikdtree_ptr->acquire_removed_points(points_history);
    points_cache_size = points_history.size();
    for (size_t i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree_ptr->Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    double time_offset = 0.0f;
    if (time_offset_from_lidar != 0) time_offset = time_offset_from_lidar;
    double msg_time = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    if (msg_time + time_offset < last_timestamp_lidar)
    {
        RCLCPP_ERROR(g_node->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    if (p_pre->lidar_type == XGRIDS)
        pcl::fromROSMsg(*msg, *ptr);
    else
        p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg_time + time_offset);
    last_timestamp_lidar = msg_time + time_offset;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    fout_dbg << "last_timestamp_lidar: " << std::to_string(last_timestamp_lidar) << std::endl << std::endl;
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    double msg_time = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    if (msg_time < last_timestamp_lidar)
    {
        RCLCPP_ERROR(g_node->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg_time);
    last_timestamp_lidar = msg_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
    publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

    double timestamp = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_ERROR(g_node->get_logger(), "imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double last_time = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if(meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();
            return false;
        }
        if (p_pre->lidar_type == XGRIDS)
        {
            meas.lidar_beg_time = time_buffer.front() ;
            lidar_end_time = meas.lidar_beg_time + 0.1;
        }
        else
        {
            meas.lidar_beg_time = time_buffer.front();
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        }
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    double imu_time = static_cast<double>(imu_buffer.front()->header.stamp.sec) +
                      static_cast<double>(imu_buffer.front()->header.stamp.nanosec) * 1e-9;
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = static_cast<double>(imu_buffer.front()->header.stamp.sec) +
                   static_cast<double>(imu_buffer.front()->header.stamp.nanosec) * 1e-9;
        if(imu_time > lidar_end_time + 0.02) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    last_time = omp_get_wtime();
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointVector PointToAddHistoricalFront;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    PointToAddHistoricalFront.reserve(100000);
    double filter_size_map_mi_d = double(filter_size_map_min);
    for (int i = 0; i < feats_down_size; i++)
    {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;

            double mx, my, mz;
            if (fabs(feats_down_world->points[i].x-round(feats_down_world->points[i].x))<0.0001) feats_down_world->points[i].x = round(feats_down_world->points[i].x)+0.001;
            if (fabs(feats_down_world->points[i].y-round(feats_down_world->points[i].y))<0.0001) feats_down_world->points[i].y = round(feats_down_world->points[i].y)+0.001;
            if (fabs(feats_down_world->points[i].z-round(feats_down_world->points[i].z))<0.0001) feats_down_world->points[i].z = round(feats_down_world->points[i].z)+0.001;
            mx = floor(double(feats_down_world->points[i].x)*2)*filter_size_map_mi_d + 0.5 * filter_size_map_mi_d;
            my = floor(double(feats_down_world->points[i].y)*2)*filter_size_map_mi_d + 0.5 * filter_size_map_mi_d;
            mz = floor(double(feats_down_world->points[i].z)*2)*filter_size_map_mi_d + 0.5 * filter_size_map_mi_d;
            double dist  = calc_dist_ondouble(feats_down_world->points[i], mx, my, mz);
            if (fabs(double(points_near[0].x) - mx) - double(0.5 * filter_size_map_mi_d) > 0 && \
                fabs(double(points_near[0].y) - my) - double(0.5 * filter_size_map_mi_d) > 0 && \
                fabs(double(points_near[0].z) - mz) - double(0.5 * filter_size_map_mi_d) > 0)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                double dist_near = calc_dist_ondouble(points_near[readd_i],  mx, my, mz);
                if (dist_near - dist < 0)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
            if (need_add && (PointToAdd.size()%3) == 0) PointSubmap.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    if (!PointToAddHistorical.empty())
    {
        PointToAddHistoricalFront = PointToAddHistorical.front()->points;
        PointToAddHistorical.pop_front();
    }
    {
        unique_lock<std::mutex> my_unique_lock(mtx_ikdtreeptr);
        sig_ikdtreeptr.wait(my_unique_lock, []{return !holding_for_ikdtreerebuild;});
        add_point_size = ikdtree_ptr->Add_Points(PointToAdd, true);
        ikdtree_ptr->Add_Points(PointNoNeedDownsample, false);
        if (!PointToAddHistoricalFront.empty())
        {
            add_point_size = ikdtree_ptr->Add_Points(PointToAddHistoricalFront, true);
        }
    }
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

template<typename T>
void set_posestamp(T & out)
{
    #ifdef USE_IKFOM
    out.position.x = state_point.pos(0);
    out.position.y = state_point.pos(1);
    out.position.z = state_point.pos(2);
    #else
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
    #endif
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void path_cor_cbk(const nav_msgs::msg::Path::SharedPtr path_in)
{
    path_buffer.push_back(path_in);
}

PointCloudXYZI::Ptr last_correction_scan(new PointCloudXYZI());
int last_submap_id;
void submap_id_cbk(const std_msgs::msg::Int32::SharedPtr id_msg)
{
    last_submap_id = submap_id;
    *last_correction_scan = *feats_down_world;
    SubmapInfo tmp;
    submap_id = id_msg->data;
    tmp.submap_index = submap_id;
    tmp.cloud_ontree = PointSubmap;
    fout_dbg << "Submap " << submap_id << "-th has " << PointSubmap.size() <<  " points " << endl;
    PointVector ().swap(PointSubmap);
    auto iter = unmap_submap_info.find(tmp.submap_index);
    if (iter == unmap_submap_info.end())
        unmap_submap_info[tmp.submap_index] = tmp;
}

void submap_pose_cbk(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    mtx_sub_.lock();
    submap_pose_buffer.push_back(odom_msg);
    mtx_sub_.unlock();
}

bool update_submap_info()
{
    if (submap_pose_buffer.empty()) return false;
    if (submap_pose_buffer.back()->twist.covariance[0]!=submap_id) return false;
    mtx_sub_.lock();
    auto submap_pose_buffer_tmp = submap_pose_buffer;
    submap_pose_buffer.clear();
    for (auto &submap_pose : submap_pose_buffer_tmp)
    {
        int idx = int(submap_pose->twist.covariance[0]);
        auto iter = unmap_submap_info.find(idx);
        if (iter != unmap_submap_info.end() && !iter->second.oriPoseSet)
        {
            auto &submap_info = iter->second;
            ExtraLib::poseMsgToEigenRT(submap_pose->pose.pose, submap_info.lidar_pose_rotM, submap_info.lidar_pose_tran);
            submap_info.oriPoseSet = true;
            double msg_time = static_cast<double>(submap_pose->header.stamp.sec) +
                              static_cast<double>(submap_pose->header.stamp.nanosec) * 1e-9;
            submap_info.msg_time = msg_time;
        }
        else
            submap_pose_buffer.push_back(submap_pose);
    }
    mtx_sub_.unlock();

    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr key_poses(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr key_poses_prior(new pcl::PointCloud<pcl::PointXYZI>());
std::unordered_map<int, SubmapInfo> unmap_submap_info_bkq;

void set_submap_corrected_poses(const nav_msgs::msg::Path::SharedPtr& path_cor)
{
    if (!key_poses->empty()) key_poses->clear();
    fout_dbg<< "--------------------set_submap_corrected_poses--------------------" <<endl;
    unmap_submap_info_bkq = unmap_submap_info;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pos_kdtree_tmp\
        (new pcl::KdTreeFLANN<pcl::PointXYZI>());
    PointVector position_vec;
    pcl::PointXYZI posi;
    fout_dbg<<"Correct KF pose @" ;
    for (size_t i = 0; i < path_cor->poses.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose_cor = path_cor->poses[i];
        int idx = int(pose_cor.header.stamp.sec);  // Using sec as index in ROS2
        if (idx >= 50000) idx = -(idx - 50000);
        auto iter = unmap_submap_info.find(idx);
        if (iter != unmap_submap_info.end())
        {
            auto &submap_info = iter->second;
            M3D R1 = submap_info.lidar_pose_rotM;
            V3D t1 = submap_info.lidar_pose_tran;
            M3D R2;   V3D t2;
            if (idx < 0 )  continue;
            ExtraLib::poseMsgToEigenRT(pose_cor.pose, R2, t2);
            submap_info.corr_pose_rotM = R2;
            submap_info.corr_pose_tran = t2;
            submap_info.corPoseSet = true;
            posi.x = submap_info.corr_pose_tran[0], posi.y = submap_info.corr_pose_tran[1], posi.z = submap_info.corr_pose_tran[2], posi.intensity = idx;
            fout_dbg<< " " << idx ;
            key_poses->push_back(posi);
        }
    }
    fout_dbg<< "----------------------------------------" << endl ;
    pos_kdtree_tmp->setInputCloud(key_poses);
    pos_kdtree = pos_kdtree_tmp->makeShared();
}

void recover_unmap_submap_info()
{
    unmap_submap_info = unmap_submap_info_bkq;
}

void correctLidarPoints(PointType const * const pi, PointType * const po, const M3D &rotM, const V3D &tran)
{
    V3D p_ori(pi->x, pi->y, pi->z);
    V3D p_corrected(rotM*p_ori + tran);
    po->x = p_corrected(0);
    po->y = p_corrected(1);
    po->z = p_corrected(2);
    po->intensity = pi->intensity;
}

void correctLidarPoints(PointType const * const pi, PointType * const po, const M3D &R1, const V3D &t1,
                        const M3D &R2, const V3D &t2)
{
    V3D p_ori(pi->x, pi->y, pi->z);
    V3D p_corrected(R2*R1.transpose()*(p_ori - t1)+t2);
    po->x = p_corrected(0);
    po->y = p_corrected(1);
    po->z = p_corrected(2);
    po->intensity = pi->intensity;
}

MD(4,4) M3D_to_M4D(M3D Min, V3D tin)
{
    MD(4,4) out = MD(4,4)::Identity();
    out.block<3,3>(0,0) = Min;
    out.block<3,1>(0,3) = tin;
    return out;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes, const double curr_timestamp_lidar)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }

    *pcl_wait_pub += *laserCloudWorld;

    if(1)
    {
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(curr_timestamp_lidar * 1e9));
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
        pcl_wait_pub->clear();
    }
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = g_node->now();
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = g_node->now();
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}

void publish_odometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, const double curr_timestamp_lidar)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = rclcpp::Time(static_cast<int64_t>(curr_timestamp_lidar * 1e9));
    set_posestamp(odomAftMapped.pose.pose);

    // Publish TF
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odomAftMapped.header.stamp;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "odom";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation = odomAftMapped.pose.pose.orientation;
    tf_broadcaster->sendTransform(transform);

    pubOdomAftMapped->publish(odomAftMapped);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = g_node->now();
    msg_body_pose.header.frame_id = "camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath->publish(path);
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            ikdtree_ptr->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

            if (points_near.size() < NUM_MATCH_POINTS)
            {
                point_selected_surf[i] = false;
            }
            else
            {
                point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            }
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s_val = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s_val > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }
    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    fout_dbg << "scan to map update avg res_mean_last: " << res_mean_last << " KF pose: " << s.pos.transpose() << std::endl;

    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int mainLIOFunction(rclcpp::Node::SharedPtr node)
{
    g_node = node;

    // Initialize TF broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    // Declare and get parameters
    node->declare_parameter<bool>("dense_map_enable", true);
    node->declare_parameter<int>("max_iteration", 4);
    node->declare_parameter<std::string>("map_file_path", "");
    node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    node->declare_parameter<double>("common.time_offset_from_lidar", 0.0);
    node->declare_parameter<double>("filter_size_corner", 0.5);
    node->declare_parameter<double>("filter_size_surf", 0.5);
    node->declare_parameter<double>("filter_size_map", 0.5);
    node->declare_parameter<double>("cube_side_length", 200.0);
    node->declare_parameter<double>("mapping.det_range", 300.0);
    node->declare_parameter<double>("mapping.fov_degree", 180.0);
    node->declare_parameter<double>("mapping.gyr_cov", 0.1);
    node->declare_parameter<double>("mapping.acc_cov", 0.1);
    node->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    node->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    node->declare_parameter<double>("preprocess.blind", 0.01);
    node->declare_parameter<int>("preprocess.lidar_type", AVIA);
    node->declare_parameter<int>("preprocess.scan_line", 16);
    node->declare_parameter<int>("point_filter_num", 2);
    node->declare_parameter<bool>("feature_extract_enable", false);
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    node->declare_parameter<int>("multisession_mode", 0);
    node->declare_parameter<double>("correction_ver_thr", 0.45);
    node->declare_parameter<std::string>("SaveDir", "");
    node->declare_parameter<double>("correction_dis_interval", 50.0);

    dense_map_en = node->get_parameter("dense_map_enable").as_bool();
    NUM_MAX_ITERATIONS = node->get_parameter("max_iteration").as_int();
    map_file_path = node->get_parameter("map_file_path").as_string();
    lid_topic = node->get_parameter("common.lid_topic").as_string();
    imu_topic = node->get_parameter("common.imu_topic").as_string();
    time_offset_from_lidar = node->get_parameter("common.time_offset_from_lidar").as_double();
    filter_size_corner_min = node->get_parameter("filter_size_corner").as_double();
    filter_size_surf_min = node->get_parameter("filter_size_surf").as_double();
    filter_size_map_min = node->get_parameter("filter_size_map").as_double();
    cube_len = node->get_parameter("cube_side_length").as_double();
    DET_RANGE = static_cast<float>(node->get_parameter("mapping.det_range").as_double());
    fov_deg = node->get_parameter("mapping.fov_degree").as_double();
    gyr_cov = node->get_parameter("mapping.gyr_cov").as_double();
    acc_cov = node->get_parameter("mapping.acc_cov").as_double();
    b_gyr_cov = node->get_parameter("mapping.b_gyr_cov").as_double();
    b_acc_cov = node->get_parameter("mapping.b_acc_cov").as_double();
    p_pre->blind = node->get_parameter("preprocess.blind").as_double();
    p_pre->lidar_type = node->get_parameter("preprocess.lidar_type").as_int();
    p_pre->N_SCANS = node->get_parameter("preprocess.scan_line").as_int();
    p_pre->point_filter_num = node->get_parameter("point_filter_num").as_int();
    p_pre->feature_enabled = node->get_parameter("feature_extract_enable").as_bool();
    extrinT = node->get_parameter("mapping.extrinsic_T").as_double_array();
    extrinR = node->get_parameter("mapping.extrinsic_R").as_double_array();
    multisession_mode = node->get_parameter("multisession_mode").as_int();
    correction_ver_thr = node->get_parameter("correction_ver_thr").as_double();
    save_directory = node->get_parameter("SaveDir").as_string();
    correction_dis_interval = node->get_parameter("correction_dis_interval").as_double();

    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    path.header.stamp = node->now();
    path.header.frame_id = "camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    shared_ptr<ImuProcess> p_imu(new ImuProcess());
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    if (p_pre->lidar_type == XGRIDS) p_imu->disable_undistort = true;

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(save_directory + "lio_debug.txt",ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    time_ikdrebuild_thread.open(save_directory + "times_ikdrebuild_LTAOM.txt",ios::out);
    time_ikdrebuild_thread.precision(std::numeric_limits<double>::max_digits10);

    /*** ROS2 subscribe initialization ***/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;

    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, rclcpp::SensorDataQoS(), livox_pcl_cbk);
    } else {
        sub_pcl_standard = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
    }

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(), imu_cbk);

    auto sub_submap_id = node->create_subscription<std_msgs::msg::Int32>(
        "/submap_ids", 100, submap_id_cbk);
    auto sub_submap_pose = node->create_subscription<nav_msgs::msg::Odometry>(
        "/submap_pose", 100, submap_pose_cbk);
    auto sub_path_corrected = node->create_subscription<nav_msgs::msg::Path>(
        "/aft_pgo_path", 100, path_cor_cbk);

    pubLaserCloudFullCor = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_corrected", 100);
    pubOdomCorrection = node->create_publisher<std_msgs::msg::Float32MultiArray>("/odom_correction_info", 10);
    pubOdomCorrection2 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/odom_correction_info64", 10);
    pubTimeCorrection = node->create_publisher<std_msgs::msg::UInt64>("/time_correction", 10);
    pubCorrectionId = node->create_publisher<visualization_msgs::msg::MarkerArray>("/ids_corr", 10);

    auto pubLaserCloudFullRes = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10000);
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10000);
    auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", 10);

    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    bool status = rclcpp::ok();

    while (status)
    {
        if (flg_exit) break;
        rclcpp::spin_some(node);

        if(sync_packages(Measures))
        {
            if (flg_reset)
            {
                RCLCPP_WARN(node->get_logger(), "reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            fout_dbg << "--------------------------------------------------------frame_num " << frame_num << std::endl;
            t0 = omp_get_wtime();
            {
                unique_lock<std::mutex> my_unique_lock(mtx_ikdtreeptr);
                sig_ikdtreeptr.wait(my_unique_lock, []{return !holding_for_ikdtreerebuild;});
                p_imu->Process(Measures, kf, feats_undistort);
            }
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                cout<<"FAST-LIO not ready"<<endl;
                continue;
            }

            bool is_outrange = false;
            for (size_t i = 0; i < feats_undistort->size(); i++)
            {
                if(feats_undistort->points[i].x < -1000000 || feats_undistort->points[i].x > 1000000 ||
                   feats_undistort->points[i].y < -1000000 || feats_undistort->points[i].y > 1000000 ||
                   feats_undistort->points[i].z < -1000000 || feats_undistort->points[i].z > 1000000)
                {
                    cout << "[ Warn ]: point_in_body is out of reasonable range!" << endl;
                    is_outrange = true;
                    break;
                }
            }
            if (is_outrange) continue;
            curr_Mea_lidarbegtime_NS = static_cast<uint64_t>(Measures.lidar_beg_time * 1e9);

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/

            if(ikdtree_ptr->Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    unique_lock<std::mutex> my_unique_lock(mtx_ikdtreeptr);
                    sig_ikdtreeptr.wait(my_unique_lock, []{return !holding_for_ikdtreerebuild;});
                    ikdtree_ptr->set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree_ptr->Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree_ptr->validnum();
            kdtree_size_st = ikdtree_ptr->size();

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true;

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            {
                tmp_state = kf.get_x();
                unique_lock<std::mutex> my_unique_lock(mtx_ikdtreeptr);
                sig_ikdtreeptr.wait(my_unique_lock, []{return !holding_for_ikdtreerebuild;});
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            }

            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];
            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, Measures.lidar_beg_time);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            kdtree_size_end = ikdtree_ptr->size();

            /******* Publish points *******/
            publish_frame_world(pubLaserCloudFullRes, Measures.lidar_beg_time);
            publish_effect_world(pubLaserCloudEffect);
            publish_path(pubPath);

            fout_dbg << "R_offset" << std::setprecision(10) << state_point.offset_R_L_I.toRotationMatrix() << std::endl;
            fout_dbg << "T_offset" << state_point.offset_T_L_I << std::endl;
            fout_dbg << "bias_a " << std::setprecision(10) << state_point.ba << std::endl;
            fout_dbg << "bias_b " << state_point.bg << std::endl;

            /*** Debug variables ***/
            frame_num ++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
            aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
            aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
            aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_down_size;
            s_plot3[time_log_counter] = kdtree_incremental_time;
            s_plot4[time_log_counter] = kdtree_search_time;
            s_plot5[time_log_counter] = kdtree_delete_counter;
            s_plot6[time_log_counter] = kdtree_delete_time;
            s_plot7[time_log_counter] = kdtree_size_st;
            s_plot8[time_log_counter] = kdtree_size_end;
            s_plot9[time_log_counter] = aver_time_consu;
            s_plot10[time_log_counter] = add_point_size;
            time_log_counter ++;
            printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
            ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
            dump_lio_state_to_log(fp);
            if (do_posecorrection)
            {
                  do_posecorrection = false;
                  tmp_state.pos = pos_replace;
                  tmp_state.rot = rot_replace;
                  tmp_state.vel = vel_replace;
                  kf.change_x(tmp_state);
            }
        }
        status = rclcpp::ok();
        rate.sleep();
    }

    /*** Save time logs ***/
    FILE *fp2;
    string log_dir = save_directory + "fast_lio_time_log.csv";
    fp2 = fopen(log_dir.c_str(),"w");
    fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
    for (int i = 0;i<time_log_counter; i++)
    {
        fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
    }
    fclose(fp2);

    return 0;
}
