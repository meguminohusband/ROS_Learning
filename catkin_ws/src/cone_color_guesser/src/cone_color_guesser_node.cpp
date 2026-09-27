// ============================================================
//  cone_color_guesser_node.cpp
//
//  功能：订阅无色锥桶点云，推测每个锥桶是左(红)还是右(蓝)，
//        发布带有 RGB 颜色的点云。
//
//  原理：赛道左红右蓝。判断一个锥桶在赛道的左侧还是右侧，是个纯几何问题。
//        本节点以「车辆在地图系 imu_map 下走过的路径」作为行进方向的基准：
//        把锥桶投影到这条路径上，用该处路径切向的左侧（z × d）判色。
//        路径从地图原点（车辆出发点）开始逐帧追加，判据因此「以地图原点为基准」。
//
//        为什么不用「帧内配对 + 相邻中点方向」：实测锥桶沿赛道的间隔中位数约
//        2.3 m，而赛道宽约 3.1 m，两个尺度重叠 —— 按距离贪心配对会把同一侧的
//        相邻锥桶配成一对（实测占 74%），配对中点根本不在赛道中线上，由此推出
//        的方向是错的。而「车走过的路径」是行进方向的直接测量，不依赖任何反推。
//
//  显示分三级，做到「边行驶边上色」：
//        UNKNOWN     攒的票太少 → 不显示
//        PROVISIONAL 车前方（≤ head_range，实测 8 m 内可靠）→ 用累积多数票，
//                    显示【降亮半透明】暂定色。这一步解决「要等车开过去才上色」。
//        LOCKED      车已驶过、满权票够 → 显示【实心全亮】锁定色，永久不变。
//
//  数据：timu.bag
//        - /only_lidar_points_pub : PointCloud2, 10 Hz, 每帧 10~96 点
//        - /tf                    : 提供 imu_map → imu_link 的实时位姿
//        - 一个点 = 一个锥桶（不要用欧氏聚类！）
// ============================================================

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include <cmath>

// ============================================================
//  常量
// ============================================================
namespace {
constexpr int UNKNOWN = 0;
constexpr int RED     = 1;   // 赛道左侧
constexpr int BLUE    = 2;   // 赛道右侧
constexpr size_t kMaxPathPts = 20000;   // 车辆路径最多缓存点数（0.5 m 步长 ≈ 10 km）
}  // namespace

// ============================================================
//  数据结构：一个锥桶观测
// ============================================================
struct ConeObs {
    Eigen::Vector3f pos;        // 位置（地图系 imu_map）
    int   color = UNKNOWN;      // 颜色
    bool  is_noise = false;     // 是否被判定为杂点
    float vote_w = 0.0f;        // 本次观测的投票权重（0 = 不可信，不投票）
    bool  provisional = false;  // true = 暂定色（前方、未锁定），false = 已锁定
};

// ============================================================
//  数据结构：世界坐标系（imu_map）下的一条锥桶轨迹
//
//  锥桶在地图系里是【静止】的，跨帧按位置关联。
//  每帧的左右判断会有噪声，靠红/蓝【加权】票数投票，
//  并且要求至少出现过若干次「实测切向」的满权票才允许锁定
//  —— 否则最早那几帧（车辆几乎没动、路径还没建立）会把颜色锁死。
//  锁定后不再改色，彻底消除 RViz 里红蓝逐帧跳动的问题。
// ============================================================
struct ConeTrack {
    Eigen::Vector3f pos{0, 0, 0};   // imu_map 下位置（观测均值）
    float red_w        = 0.0f;      // 红色加权票（含降权的前方票，仅用于暂定色）
    float blue_w       = 0.0f;      // 蓝色加权票（同上）
    float votes        = 0.0f;      // 加权总票数
    float red_rel      = 0.0f;      // ★ 只统计「满权票（实测切向）」的红票
    float blue_rel     = 0.0f;      // ★ 只统计「满权票（实测切向）」的蓝票
    int   reliable_cnt = 0;         // 权重 1.0（实测切向）的票数
    int   obs          = 0;         // 累计观测次数
    int   miss         = 0;         // 连续没看到的帧数
    int   locked       = UNKNOWN;   // 锁定颜色后不再变化
};

// ============================================================
//  主类
// ============================================================
class ConeColorGuesser {
public:
    ConeColorGuesser() : nh_("~"), tf_listener_(tf_buffer_) {
        loadParams();

        // ---- 发布者 ----
        colored_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cones/colored", 1);
        marker_pub_  = nh_.advertise<visualization_msgs::MarkerArray>("/cones/markers", 1);

        // ---- 订阅者 ----
        sub_ = nh_.subscribe(input_topic_, 1, &ConeColorGuesser::cloudCallback, this);

        ROS_INFO("===========================================");
        ROS_INFO(" cone_color_guesser 已启动");
        ROS_INFO("   输入话题  : %s", input_topic_.c_str());
        ROS_INFO("   目标坐标系: %s", target_frame_.c_str());
        ROS_INFO("   杂点阈值  : 最近邻 < %.2f m", noise_nn_thr_);
        ROS_INFO("   路径判据  : 抽稀 %.2f m / 横向门限 %.2f m / 贴中线 %.2f m",
                 path_step_, path_gate_, s_min_);
        ROS_INFO("   前方暂定色: 车头朝向估计，%.1f m 内；加权票 >= %.1f 即显示（降亮）",
                 head_range_, provisional_min_);
        ROS_INFO("   锁定条件  : 加权票 >= %.1f 且实测票 >= %d，占比 >= %.2f",
                 min_votes_, min_reliable_, vote_thr_);
        ROS_INFO("===========================================");
    }

private:
    // ========================================================
    //  从参数服务器读取参数
    // ========================================================
    void loadParams() {
        nh_.param<std::string>("input_topic",  input_topic_,  "/only_lidar_points_pub");
        nh_.param<std::string>("target_frame", target_frame_, "imu_link");

        nh_.param("noise_nn_threshold", noise_nn_thr_, 0.5);
        nh_.param("noise_z_max",        noise_z_max_,  0.90);

        nh_.param("min_range",   min_range_,   2.0);
        nh_.param("max_range",   max_range_,   40.0);
        nh_.param("max_lateral", max_lateral_, 15.0);

        // ---------- 路径切向判据（替代原来的配对参数） ----------
        nh_.param("path_step",       path_step_,     0.5);   // 路径抽稀步长 (米)
        nh_.param("path_gate",       path_gate_,     3.2);   // 锥桶到路径的横向门限 (米)
        nh_.param("s_min",           s_min_,         0.40);  // 侧向偏移小于此值不判色 (米)
        nh_.param("path_margin",     path_margin_,   3.0);   // 投影需落在已驶过路径内部多少米才算可信
        nh_.param("path_tan_half",   path_tan_half_, 1.5);   // 切向中心差分的半窗口 (米)
        nh_.param("head_range",      head_range_,    8.0);   // 车前方多远内可用车头朝向定暂定色 (米)
                                                             //   实测：0-5 m 97%、5-8 m 81%、>8 m 近乎随机
        nh_.param("head_weight",     head_weight_,   0.30);  // 车头朝向观测的投票权重

        // ---------- 时序滤波 ----------
        nh_.param("enable_temporal",       enable_temporal_,  true);
        nh_.param("vote_threshold",        vote_thr_,         0.8);
        nh_.param("min_votes",             min_votes_,        6.0);
        nh_.param("min_reliable_votes",    min_reliable_,     2);
        nh_.param("provisional_min_votes", provisional_min_,  2.0);   // 显示暂定色所需加权票
        nh_.param("map_assoc_dist",        assoc_dist_,       1.2);   // 轨迹关联门限 (米)
        nh_.param("map_max_miss",          max_miss_,         30);
    }

    // ========================================================
    //  车辆路径维护（地图系）
    //
    //  车就是沿着赛道走的，所以「路径在任一点处的切向」
    //  就等于该处的行进方向 —— 这是直接测量，不是反推。
    //  路径从地图原点（车辆出发点）开始逐帧追加，天然以地图原点为基准。
    // ========================================================
    void appendPath(const tf2::Vector3& car_map) {
        const Eigen::Vector2f c(static_cast<float>(car_map.x()),
                                static_cast<float>(car_map.y()));
        if (path_.empty() ||
            (c - path_.back()).norm() >= static_cast<float>(path_step_)) {
            path_.push_back(c);
            if (path_.size() > kMaxPathPts)           // 防内存无限增长（≈10 km）
                path_.erase(path_.begin(), path_.begin() + kMaxPathPts / 4);
        }
    }

    // ========================================================
    //  主回调：每收到一帧点云就执行一次
    // ========================================================
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        const ros::WallTime t_start = ros::WallTime::now();

        // ---------- 步骤 1：把点云转到车体坐标系 ----------
        sensor_msgs::PointCloud2 cloud_body;
        tf2::Transform body_from_map;        // imu_map → imu_link
        if (!transformToBody(msg, cloud_body, body_from_map)) {
            ROS_WARN_THROTTLE(2.0, "坐标变换失败，跳过这一帧");
            return;
        }

        // ---------- 步骤 2：解析成 PCL 点云 ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_body, *cloud);

        if (cloud->size() < 2) {
            ROS_WARN_THROTTLE(2.0, "点云为空或点太少 (%zu 个)", cloud->size());
            return;
        }

        // ---------- 步骤 3：剔除杂点 ----------
        std::vector<ConeObs> cones;
        filterNoise(cloud, cones);

        // ---------- 步骤 4：范围过滤 ----------
        std::vector<ConeObs> valid;
        for (const auto& c : cones) {
            float range = c.pos.head<2>().norm();
            if (range < min_range_ || range > max_range_)  continue;
            if (std::abs(c.pos.y()) > max_lateral_)        continue;
            valid.push_back(c);
        }

        if (valid.empty()) {
            ROS_WARN_THROTTLE(2.0, "过滤后没有有效锥桶");
            return;
        }

        // ---------- 步骤 5：换到地图坐标系 imu_map（锥桶在世界里是静止的） ----------
        const tf2::Transform map_from_body = body_from_map.inverse();
        const tf2::Quaternion& q_b2m = map_from_body.getRotation();
        const tf2::Vector3&    t_b2m = map_from_body.getOrigin();
        const Eigen::Quaternionf q_b2m_e(static_cast<float>(q_b2m.getW()),
                                         static_cast<float>(q_b2m.getX()),
                                         static_cast<float>(q_b2m.getY()),
                                         static_cast<float>(q_b2m.getZ()));
        const Eigen::Vector3f t_b2m_e(static_cast<float>(t_b2m.x()),
                                      static_cast<float>(t_b2m.y()),
                                      static_cast<float>(t_b2m.z()));

        // 车辆在地图系下的位置/朝向；路径以地图原点（车辆出发点）为起点
        appendPath(t_b2m);
        const float car_yaw = static_cast<float>(std::atan2(
            2.0 * (q_b2m.getW() * q_b2m.getZ() + q_b2m.getX() * q_b2m.getY()),
            1.0 - 2.0 * (q_b2m.getY() * q_b2m.getY() + q_b2m.getZ() * q_b2m.getZ())));

        std::vector<ConeObs> map_obs;
        map_obs.reserve(valid.size());
        for (const auto& c : valid) {
            ConeObs o;
            o.pos   = q_b2m_e * c.pos + t_b2m_e;
            o.color = UNKNOWN;
            map_obs.push_back(o);
        }

        // ---------- 步骤 6：左右分色（核心算法，地图系 + 车辆路径切向） ----------
        assignColors(map_obs, t_b2m_e, car_yaw);

        // ---------- 步骤 7：跨帧投票 + 分级显示 ----------
        //
        //  显示与「锁定」解耦，分三级（对应方案 §2 的状态机）：
        //    LOCKED      —— 车已驶过，满权票够 → 实心全亮色，永久不变
        //    PROVISIONAL —— 车前方，票数够但还没锁定 → 降亮半透明暂定色，先显示出来
        //    其余        —— 不显示
        //
        //  ★ 这一步是修复「前方锥桶不上色」的关键：
        //    原实现只发 locked，而 locked 要求满权票，满权票只在锥桶投影落到
        //    「已驶过路径内部」时才产生 —— 于是前方锥桶永远不上色，要等车开过去
        //    才出现。现在前方锥桶先用暂定色显示，驶过后再升级成锁定色。
        std::vector<ConeObs> shown;
        if (enable_temporal_) {
            updateTracks(map_obs);
            // ★ 用最新路径给未锁定轨迹重新评分：补上「只在车前方短暂看到过」的锥桶
            rescoreTracks(Eigen::Vector2f(t_b2m_e.x(), t_b2m_e.y()), car_yaw);
            for (const auto& t : tracks_) {
                ConeObs o;
                o.pos = t.pos;
                if (t.locked != UNKNOWN) {
                    o.color       = t.locked;
                    o.provisional = false;
                } else if (t.votes >= static_cast<float>(provisional_min_)) {
                    // 还没锁定，但攒的票够看了 → 先给暂定色
                    // 用【累积多数票】而不是本帧颜色，保证暂定色也不会逐帧乱跳
                    o.color       = (t.red_w > t.blue_w) ? RED : BLUE;
                    o.provisional = true;
                } else {
                    continue;                       // 票太少，宁可不显示
                }
                shown.push_back(o);
            }
        } else {
            for (const auto& c : map_obs)
                if (c.color != UNKNOWN) shown.push_back(c);
        }

        // ---------- 步骤 8：发布（坐标系 = 输入的 imu_map） ----------
        publishPointCloud(shown, msg->header);
        publishMarkers(shown, msg->header);

        // ---------- 步骤 9：统计 ----------
        int n_red = 0, n_blue = 0, n_locked = 0, n_prov = 0, n_pending = 0;
        for (const auto& c : shown) {
            if (c.provisional) { ++n_prov; continue; }
            ++n_locked;
            if      (c.color == RED)  ++n_red;
            else if (c.color == BLUE) ++n_blue;
        }
        if (enable_temporal_)
            for (const auto& t : tracks_)
                if (t.locked == UNKNOWN) ++n_pending;

        const double ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;
        ROS_INFO_THROTTLE(2.0,
            "发布 %zu 个 [锁定 %d: 红 %d / 蓝 %d | 暂定 %d]  投票中 %d  本帧观测 %zu  耗时 %.2f ms",
            shown.size(), n_locked, n_red, n_blue, n_prov, n_pending, map_obs.size(), ms);

        // 待定轨迹很少时把它们的位置/票数打出来（排查「某个锥桶一直不上色」）
        if (enable_temporal_ && n_pending > 0 && n_pending <= 5) {
            for (const auto& t : tracks_) {
                if (t.locked != UNKNOWN) continue;
                ROS_INFO("    待定 (%.2f, %.2f): 观测 %d  加权票 %.1f  满权票 %d  满权红 %.1f / 蓝 %.1f",
                         t.pos.x(), t.pos.y(), t.obs, t.votes,
                         t.reliable_cnt, t.red_rel, t.blue_rel);
            }
        }
    }

    // ========================================================
    //  步骤 1：坐标变换 imu_map → 车体
    // ========================================================
    bool transformToBody(const sensor_msgs::PointCloud2ConstPtr& in,
                         sensor_msgs::PointCloud2& out,
                         tf2::Transform& body_from_map) {
        // 如果点云本来就在目标坐标系下，不用变换
        if (in->header.frame_id == target_frame_) {
            out = *in;
            body_from_map.setIdentity();
            return true;
        }

        try {
            geometry_msgs::TransformStamped tf_msg;
            try {
                // 只做非阻塞尝试：查不到立刻抛，不要在这里等。
                // 本 bag 的 TF stamp 比点云慢 171.78 天，按 stamp 查询【必然】
                // 落在 TF 的"未来"，如果这里给 0.2 s 的超时，就会每一帧都白等
                // 200 ms —— 实测直接导致节点只能跑到 5 Hz，跟不上 10 Hz 的点云。
                tf_msg = tf_buffer_.lookupTransform(
                    target_frame_,          // 目标坐标系
                    in->header.frame_id,    // 源坐标系
                    in->header.stamp,       // 用点云自己的时间戳（实车上的正确做法）
                    ros::Duration(0.0));    // 不等待
            } catch (const tf2::TransformException&) {
                // timu.bag 专属补丁：发 TF 的那台机器时钟慢了 171.78 天
                // （实测：点云 stamp=2025-09-10，TF stamp=2025-03-22，
                //   但两话题的接收时间完全同步）。
                // 按 stamp 查询必然落在 TF 的"未来"，此时退一步
                // 用「缓冲里最新的一条 TF」——物理上它和这帧点云就是同一时刻。
                tf_msg = tf_buffer_.lookupTransform(
                    target_frame_, in->header.frame_id, ros::Time(0),
                    ros::Duration(0.05));
            }

            // 手动组装 tf2::Transform（避开 tf2::fromMsg 的链接依赖）
            const auto& tr = tf_msg.transform;
            tf2::Quaternion qq(tr.rotation.x, tr.rotation.y,
                               tr.rotation.z, tr.rotation.w);
            qq.normalize();
            body_from_map.setOrigin(tf2::Vector3(tr.translation.x,
                                                 tr.translation.y,
                                                 tr.translation.z));
            body_from_map.setRotation(qq);
            tf2::doTransform(*in, out, tf_msg);
            return true;

        } catch (const tf2::TransformException& e) {
            ROS_WARN_THROTTLE(2.0, "TF 查询失败: %s", e.what());
            return false;
        }
    }

    // ========================================================
    //  步骤 3：剔除杂点
    //
    //  核心判据：正常锥桶之间隔着 2.5 米，孤零零的；
    //           杂点挤在一起，间距只有 0.16 米。
    //           所以「最近邻 < 0.5 米」= 杂点。
    // ========================================================
    void filterNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     std::vector<ConeObs>& out) {
        out.clear();
        const size_t n = cloud->size();

        for (size_t i = 0; i < n; ++i) {
            const auto& pi = cloud->points[i];

            // ---- 高度过滤：太高的肯定不是锥桶 ----
            if (pi.z > noise_z_max_) continue;

            // ---- 最近邻距离过滤 ----
            float min_dist = std::numeric_limits<float>::max();
            for (size_t j = 0; j < n; ++j) {
                if (i == j) continue;
                const auto& pj = cloud->points[j];
                float dx = pi.x - pj.x;
                float dy = pi.y - pj.y;
                float dz = pi.z - pj.z;
                float d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < min_dist) min_dist = d2;
            }
            min_dist = std::sqrt(min_dist);

            // 最近邻太近 → 说明它旁边挤着别的点 → 杂点
            if (min_dist < noise_nn_thr_) continue;

            ConeObs obs;
            obs.pos = Eigen::Vector3f(pi.x, pi.y, pi.z);
            out.push_back(obs);
        }
    }

    // ========================================================
    //  步骤 6：左右分色 ★★★ 核心算法 ★★★（以地图原点为基准）
    //
    //  为什么不再用「帧内配对 + 相邻中点方向」：
    //    实测锥桶沿赛道的间隔中位数约 2.3 m，而赛道宽约 3.1 m，两个尺度重叠
    //    —— 按距离贪心配对会把「同一侧的相邻锥桶」配成一对（实测占 74%），
    //    配对中点根本不在赛道中线上，由此推出的方向是错的；再叠加原本
    //    「mids 已按距离重排、循环里却仍用 pairs[k] 索引」的错位，
    //    方向几乎等同于随机，左右自然翻转。
    //
    //  本实现改为：把锥桶投影到【车辆在地图系 imu_map 下走过的路径】上。
    //    路径在任一点处的切向 = 该处的行进方向 —— 这是直接测量，不靠反推；
    //    左侧 = z × d，侧向偏移 s 的符号即为颜色（s>0 红，s<0 蓝）。
    //    路径自地图原点（车辆出发点）起算，故判据以地图原点为基准。
    //    因为判据用「该锥桶所在位置的路径切向」，所以车前方/后方都成立，
    //    不会出现「车后方的锥桶左右被反过来」的老问题。
    //
    //  投票权重分两档：
    //    投影落在「已驶过路径内部」→ 切向是实测的，取满权 1.0；
    //    车前方的锥桶（路径尚未铺到）只能用当前车头朝向近似 → 降权。
    //    这样最早几帧（车辆几乎没动、无路径可用）不会把颜色锁死。
    // ========================================================
    // ========================================================
    //  判色核心：给定地图系下的一个点，判断它在行进方向的哪一侧
    //
    //  返回 true 表示判定成功，*color / *weight 有效：
    //    weight = 1.0           用「已驶过路径」的实测切向判的 → 可信
    //    weight = head_weight_  车前方，只能用当前车头朝向近似 → 降权
    // ========================================================
    bool sideFor(const Eigen::Vector2f& q, const Eigen::Vector2f& car2,
                 float car_yaw, int* color, float* weight) const {
        const int N = static_cast<int>(path_.size());
        if (N < 2) return false;
        const int margin = std::max(1, static_cast<int>(std::lround(path_margin_ / path_step_)));
        const int span   = std::max(1, static_cast<int>(std::lround(path_tan_half_ / path_step_)));

        // ---------- Step 1: 在路径折线上逐段投影，取最近点 ----------
        float best_d2 = std::numeric_limits<float>::max();
        int   best_i  = 0;
        Eigen::Vector2f near(0.0f, 0.0f);
        for (int i = 0; i + 1 < N; ++i) {
            const Eigen::Vector2f a  = path_[i];
            const Eigen::Vector2f ab = path_[i + 1] - a;
            const float l2 = ab.squaredNorm();
            float t = (l2 > 1e-9f) ? (q - a).dot(ab) / l2 : 0.0f;
            t = std::min(1.0f, std::max(0.0f, t));
            const Eigen::Vector2f pr = a + t * ab;
            const float d2 = (q - pr).squaredNorm();
            if (d2 < best_d2) { best_d2 = d2; best_i = i; near = pr; }
        }

        // ---------- Step 2/3: 该处的行进方向 d（并判断这次判定可不可信） ----------
        const bool  interior = (best_i <= N - 2 - margin);   // 投影落在已驶过路径内部
        const float lat_path = std::sqrt(best_d2);           // 到「已驶过路径」的横向距离
        Eigen::Vector2f d;

        if (interior) {
            // ★ 已驶过：这个横向距离是实测的（到赛道的真实横向偏移），直接用它把关
            if (lat_path > static_cast<float>(path_gate_)) return false;
            // 用 ±span 的中心差分，抑制单段噪声
            const int i0 = std::max(0, best_i - span);
            const int i1 = std::min(N - 1, best_i + 1 + span);
            d = path_[i1] - path_[i0];
            if (d.norm() < 1e-6f) return false;
        } else {
            // ★ 车前方：路径还没铺到，【无法】测「到赛道的横向距离」。
            //   原实现拿「到路径端点的距离」当横向距离，而车前方时端点就在车上
            //   —— 等价于要求「锥桶离车 < path_gate(3.2 m)」，head_range(8 m) 形同虚设：
            //   前方锥桶几乎都不上色。改为用「距车距离」+「相对车头航向线的横向偏移」把关。
            const Eigen::Vector2f rel = q - car2;
            if (rel.norm() > static_cast<float>(head_range_)) return false;
            const Eigen::Vector2f left_car(-std::sin(car_yaw), std::cos(car_yaw));
            if (std::abs(rel.dot(left_car)) > static_cast<float>(path_gate_)) return false;
            d = Eigen::Vector2f(std::cos(car_yaw), std::sin(car_yaw));
        }
        d.normalize();

        // ---------- Step 4: 左侧 = z × d，用侧向偏移的符号定色 ----------
        const Eigen::Vector2f left(-d.y(), d.x());
        const float s = (q - near).dot(left);

        // 贴着中线（|s| 太小）时左右不可靠 → 宁可漏发，不可错发
        if (std::abs(s) < static_cast<float>(s_min_)) return false;

        *color  = (s > 0.0f) ? RED : BLUE;
        *weight = interior ? 1.0f : static_cast<float>(head_weight_);
        return true;
    }

    // ---- 给一条轨迹计一票，并检查是否满足锁定条件 ----
    void addVote(ConeTrack& t, int color, float w) {
        const int before = t.locked;
        t.votes += w;
        if (w >= 1.0f) {                              // 实测切向的满权票
            ++t.reliable_cnt;
            if (color == RED) t.red_rel  += w;
            else              t.blue_rel += w;
        }
        if (color == RED) t.red_w += w; else t.blue_w += w;

        // ★ 锁定比例【只看满权票（实测切向）】：前方锥桶是用「当前车头朝向」投的
        //   降权票，在弯道处可能一半红一半蓝，混进来会把比例拉到 50/50，
        //   导致明明驶过了却永远锁不上 → 赛道上出现缺口。
        const float rel_total = t.red_rel + t.blue_rel;
        if (t.votes >= static_cast<float>(min_votes_) &&
            t.reliable_cnt >= min_reliable_ && rel_total > 0.0f) {
            const float r = t.red_rel  / rel_total;
            const float b = t.blue_rel / rel_total;
            if (r >= vote_thr_)      t.locked = RED;
            else if (b >= vote_thr_) t.locked = BLUE;
            // 票不纯 → 继续攒，宁可慢不可错
        }
        if (before == UNKNOWN && t.locked != UNKNOWN) {
            ROS_INFO("轨迹锁定 (%.2f, %.2f) -> %s", t.pos.x(), t.pos.y(),
                     (t.locked == RED) ? "红" : "蓝");
        }
    }

    // ========================================================
    //  步骤 6：左右分色（对当帧观测）
    // ========================================================
    void assignColors(std::vector<ConeObs>& cones,
                      const Eigen::Vector3f& car_map, float car_yaw) {
        if (path_.size() < 2) {
            ROS_WARN_THROTTLE(2.0, "车辆路径不足 2 点，暂不判色（等待车辆移动）");
        }
        const Eigen::Vector2f car2(car_map.x(), car_map.y());
        for (auto& c : cones) {
            c.color  = UNKNOWN;
            c.vote_w = 0.0f;
            int col = UNKNOWN;
            float w = 0.0f;
            if (sideFor(Eigen::Vector2f(c.pos.x(), c.pos.y()), car2, car_yaw, &col, &w)) {
                c.color  = col;
                c.vote_w = w;
            }
        }
    }

    // ========================================================
    //  步骤 6b：用「当前已驶过的路径」给未锁定轨迹重新评分
    //            ★ 这是补全赛道环的关键
    //
    //  为什么需要：有些锥桶只在车辆【前方】被短暂看到（过弯接近时进入视野，
    //  车一转弯它就出了视野），永远拿不到「实测切向」的满权票，于是永远不上色。
    //  实测有一个锥桶 600 帧里只被看到 27 次、全在车前方 5.8~13.9 m，只攒到 2.1 张
    //  降权票 —— 但车其实【驶过】了它的位置，路径已经覆盖那里。
    //  所以每帧用最新路径给这些未锁定轨迹重评一次，就能把它补上。
    // ========================================================
    void rescoreTracks(const Eigen::Vector2f& car2, float car_yaw) {
        if (path_.size() < 2) return;
        for (auto& t : tracks_) {
            if (t.locked != UNKNOWN) continue;
            if (t.obs < 2) continue;            // 只被观测过一次的位置还不可信
            int col = UNKNOWN;
            float w = 0.0f;
            if (!sideFor(Eigen::Vector2f(t.pos.x(), t.pos.y()), car2, car_yaw, &col, &w))
                continue;
            addVote(t, col, w);
        }
    }

    // ========================================================
    //  步骤 7：跨帧投票（方案 v4，指南 §5.4）
    //
    //  观测在 imu_map（世界系）下按位置关联到已有轨迹，红/蓝各计【加权】票。
    //  锁定条件 = 加权总票过线 且 比例过线 且 至少出现过 min_reliable_votes 次
    //  「实测切向」的满权票。
    //
    //  锁定条件 = 加权总票过线 且 至少出现过 min_reliable_votes 次「实测切向」的
    //  满权票 且 【满权票内部】的领先比例过线。
    //
    //  最后一个条件是修复「颜色被锁死」的关键：
    //    原实现只要观测满 3 次就永久锁定，而这 3 次往往来自车辆几乎没动、
    //    路径尚未建立的最不可靠的帧 —— 一旦锁错，后面观测几百次也不会再改
    //    （实测一条锥桶观测 214 次、红票 0 蓝票 3，就是被锁死在错色上）。
    //  锁定后不再改票 → 颜色永不跳变。
    // ========================================================
    void updateTracks(const std::vector<ConeObs>& map_obs) {
        std::vector<bool> hit(tracks_.size(), false);

        for (const auto& o : map_obs) {
            if (o.color == UNKNOWN || o.vote_w <= 0.0f) continue;   // 不可信观测不投票

            // ---- 找最近轨迹（锥桶在地图系里不动） ----
            int best = -1;
            float best_d = static_cast<float>(assoc_dist_);
            for (size_t k = 0; k < tracks_.size(); ++k) {
                if (tracks_[k].locked == UNKNOWN && hit[k])
                    continue;                       // 未锁定的一个观测只能配一条
                float d = (tracks_[k].pos - o.pos).norm();
                if (d < best_d) { best_d = d; best = static_cast<int>(k); }
            }

            if (best < 0) {                         // 新锥桶，开一条轨迹
                ConeTrack t;
                t.pos = o.pos;
                tracks_.push_back(t);
                best = static_cast<int>(tracks_.size()) - 1;
                hit.push_back(false);
            }
            hit[best] = true;

            ConeTrack& t = tracks_[best];
            t.pos = (t.pos * t.obs + o.pos) / (t.obs + 1);   // 位置滑动平均
            ++t.obs;
            t.miss = 0;

            if (t.locked != UNKNOWN) continue;      // 已锁定：只续命，不改色

            // ---- 按权重计票（含锁定判定，逻辑见 addVote） ----
            addVote(t, o.color, o.vote_w);
        }

        // ---- 没被看到的轨迹记一次 miss ----
        // 已锁定的锥桶是「已确认的地标」，不因看不见而删除：
        // 否则车开过去后这些锥桶会陆续掉出 40 m 视野，发布数量忽上忽下
        // （实测出现过 46 个掉到 37 个），输出就不稳定了。
        // 只清理「还没锁定又长期看不到」的候选轨迹。
        for (size_t k = 0; k < tracks_.size(); ++k)
            if (!hit[k]) ++tracks_[k].miss;
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [this](const ConeTrack& t) {
                               return t.locked == UNKNOWN && t.miss > max_miss_;
                           }),
            tracks_.end());
    }

    // ========================================================
    //  步骤 8a：发布有色点云（地图系 imu_map，RViz Fixed Frame 设 imu_map）
    // ========================================================
    void publishPointCloud(const std::vector<ConeObs>& cones,
                           const std_msgs::Header& hdr) {
        pcl::PointCloud<pcl::PointXYZRGB> out;
        out.header.frame_id = hdr.frame_id;
        out.height = 1;

        for (const auto& c : cones) {
            if (c.color == UNKNOWN) continue;   // 不确定的不发

            pcl::PointXYZRGB p;
            p.x = c.pos.x();
            p.y = c.pos.y();
            p.z = c.pos.z();

            // ★ 必须显式初始化 alpha：PointXYZRGB 的 rgb 是一个 float，
            //   若 a 字节保持未初始化的随机值，打包出的 float 可能是 NaN/Inf，
            //   下游用 skip_nans=True 读点云时会把这些点【整批丢掉】
            //   （实测表现为红色点全部消失，只剩下蓝色点）。
            p.a = 255;

            // 锁定色 = 全亮；暂定色 = 降亮（让用户一眼区分「已确认」和「前方暂定」）
            if (c.color == RED) {
                if (c.provisional) { p.r = 255; p.g = 120; p.b = 120; }   // 暂定红（降亮）
                else               { p.r = 255; p.g = 0;   p.b = 0;   }   // 锁定红
            } else {
                if (c.provisional) { p.r = 120; p.g = 170; p.b = 255; }   // 暂定蓝（降亮）
                else               { p.r = 0;   p.g = 80;  p.b = 255; }   // 锁定蓝
            }
            out.push_back(p);
        }

        out.width = static_cast<uint32_t>(out.size());

        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(out, msg_out);
        msg_out.header = hdr;
        colored_pub_.publish(msg_out);
    }

    // ========================================================
    //  步骤 8b：发布 Marker（RViz 可视化，调试用，地图系）
    // ========================================================
    void publishMarkers(const std::vector<ConeObs>& cones,
                        const std_msgs::Header& hdr) {
        visualization_msgs::MarkerArray arr;
        int id = 0;

        for (const auto& c : cones) {
            visualization_msgs::Marker m;
            m.header = hdr;
            m.ns = "cones";
            m.id = id++;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;

            m.pose.position.x = c.pos.x();
            m.pose.position.y = c.pos.y();
            m.pose.position.z = c.pos.z();
            m.pose.orientation.w = 1.0;

            m.scale.x = m.scale.y = m.scale.z = 0.4;   // 直径 0.4 m

            // 锁定：实心不透明；暂定：降亮 + 半透明（前方锥桶"点亮"成实心 = 状态机在升级）
            if (c.color == RED) {
                if (c.provisional) {
                    m.color.r = 1.00f; m.color.g = 0.45f; m.color.b = 0.45f; m.color.a = 0.55f;
                } else {
                    m.color.r = 1.00f; m.color.g = 0.10f; m.color.b = 0.10f; m.color.a = 0.95f;
                }
            } else if (c.color == BLUE) {
                if (c.provisional) {
                    m.color.r = 0.45f; m.color.g = 0.60f; m.color.b = 1.00f; m.color.a = 0.55f;
                } else {
                    m.color.r = 0.10f; m.color.g = 0.30f; m.color.b = 1.00f; m.color.a = 0.95f;
                }
            } else {
                m.color.r = m.color.g = m.color.b = 0.6f;   // 灰色 = 未确定
                m.color.a = 0.9f;
            }

            arr.markers.push_back(m);
        }

        marker_pub_.publish(arr);
    }

    // ---------- 成员变量 ----------
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  colored_pub_;
    ros::Publisher  marker_pub_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string input_topic_, target_frame_;

    double noise_nn_thr_, noise_z_max_;
    double min_range_, max_range_, max_lateral_;

    // 路径切向判据（替代原来的配对参数 pair_min_/pair_max_）
    double path_step_, path_gate_, s_min_;
    double path_margin_, path_tan_half_, head_range_, head_weight_;

    // 车辆在地图系下走过的路径（以地图原点为起点，逐帧追加）
    std::vector<Eigen::Vector2f> path_;

    // 时序投票（跨帧锁定颜色）
    bool   enable_temporal_  = true;
    double vote_thr_         = 0.8;
    double min_votes_        = 6.0;    // 加权总票门槛
    int    min_reliable_     = 2;      // 至少几次「实测切向」的满权票
    double provisional_min_  = 2.0;    // 显示「暂定色」所需的最少加权票
    double assoc_dist_       = 1.2;
    int    max_miss_         = 60;
    std::vector<ConeTrack> tracks_;
};

// ============================================================
//  main
// ============================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "cone_color_guesser");
    ConeColorGuesser node;
    ros::spin();
    return 0;
}
