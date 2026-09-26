// ============================================================
//  cone_color_guesser_node.cpp
//
//  功能：订阅无色锥桶点云，推测每个锥桶是左(红)还是右(蓝)，
//        发布带有 RGB 颜色的点云。
//
//  原理：赛道左红右蓝。判断一个锥桶在赛道的左侧还是右侧，
//        是个纯几何问题 —— 用「配对 + 赛道方向」求解。
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
}  // namespace

// ============================================================
//  数据结构：一个锥桶观测
// ============================================================
struct ConeObs {
    Eigen::Vector3f pos;        // 位置（车体坐标系下）
    int   color = UNKNOWN;      // 颜色
    bool  is_noise = false;     // 是否被判定为杂点
};

// ============================================================
//  数据结构：世界坐标系（imu_map）下的一条锥桶轨迹
//
//  锥桶在地图系里是【静止】的，跨帧按位置关联。
//  每帧的左右判断会有噪声，靠红/蓝票数投票，
//  票数比例过线才「锁定」颜色 —— 锁定的点不再改色，
//  彻底消除 RViz 里红蓝逐帧跳动的问题。
// ============================================================
struct ConeTrack {
    Eigen::Vector3f pos{0, 0, 0};   // imu_map 下位置（观测均值）
    int red_votes  = 0;
    int blue_votes = 0;
    int obs        = 0;             // 累计观测次数
    int miss       = 0;             // 连续没看到的帧数
    int locked     = UNKNOWN;       // 锁定颜色后不再变化
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
        ROS_INFO("   配对范围  : %.2f ~ %.2f m", pair_min_, pair_max_);
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

        nh_.param("pair_min_dist", pair_min_, 2.0);
        nh_.param("pair_max_dist", pair_max_, 4.5);

        nh_.param("enable_temporal",   enable_temporal_, true);
        nh_.param("vote_threshold",    vote_thr_,        0.8);
        nh_.param("min_observations",  min_obs_,         3);
        nh_.param("map_assoc_dist",    assoc_dist_,      0.5);
        nh_.param("map_max_miss",      max_miss_,        20);
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

        // ---------- 步骤 5：左右分色（核心算法，车体系下） ----------
        assignColors(valid);

        // ---------- 步骤 6：换回地图坐标系 imu_map（锥桶在世界里是静止的） ----------
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

        std::vector<ConeObs> map_obs;
        map_obs.reserve(valid.size());
        for (const auto& c : valid) {
            ConeObs o;
            o.pos   = q_b2m_e * c.pos + t_b2m_e;
            o.color = c.color;
            map_obs.push_back(o);
        }

        // ---------- 步骤 7：跨帧投票锁定颜色（消除逐帧红蓝跳动） ----------
        std::vector<ConeObs> shown;
        if (enable_temporal_) {
            updateTracks(map_obs);
            for (const auto& t : tracks_) {
                if (t.locked == UNKNOWN) continue;      // 没锁定的不发
                ConeObs o;
                o.pos   = t.pos;
                o.color = t.locked;
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
        int n_red = 0, n_blue = 0, n_pending = 0;
        for (const auto& c : shown) {
            if      (c.color == RED)  ++n_red;
            else if (c.color == BLUE) ++n_blue;
        }
        if (enable_temporal_)
            for (const auto& t : tracks_)
                if (t.locked == UNKNOWN) ++n_pending;

        const double ms = (ros::WallTime::now() - t_start).toSec() * 1000.0;
        ROS_INFO_THROTTLE(2.0,
            "发布 %zu 个 [红 %d / 蓝 %d]  投票中 %d  本帧观测 %zu  耗时 %.2f ms",
            shown.size(), n_red, n_blue, n_pending, map_obs.size(), ms);
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
                tf_msg = tf_buffer_.lookupTransform(
                    target_frame_,          // 目标坐标系
                    in->header.frame_id,    // 源坐标系
                    in->header.stamp,       // 用点云自己的时间戳（实车上的正确做法）
                    ros::Duration(0.2));    // 最多等 0.2 秒
            } catch (const tf2::TransformException&) {
                // timu.bag 专属补丁：发 TF 的那台机器时钟慢了 171.78 天
                // （实测：点云 stamp=2025-09-10，TF stamp=2025-03-22，
                //   但两话题的接收时间完全同步）。
                // 按 stamp 查询必然落在 TF 的"未来"，此时退一步
                // 用「缓冲里最新的一条 TF」——物理上它和这帧点云就是同一时刻。
                tf_msg = tf_buffer_.lookupTransform(
                    target_frame_, in->header.frame_id, ros::Time(0));
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
    //  步骤 5：左右分色 ★★★ 核心算法 ★★★
    //
    //  思路：
    //    1. 赛道两侧锥桶成对出现，间距 ≈ 赛道宽度 (实测 3.07 m)
    //    2. 每对连线的中点 = 赛道中线上的点
    //    3. 相邻中点的连线 = 该处的赛道前进方向 d
    //    4. 用 z × d 求出「d 的左侧方向」，判断谁左谁右
    // ========================================================
    void assignColors(std::vector<ConeObs>& cones) {
        const size_t n = cones.size();
        if (n < 2) return;

        // ---------- Step 1: 找候选配对 ----------
        struct Pair { int i, j; float dist; };
        std::vector<Pair> candidates;

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                float d = (cones[i].pos - cones[j].pos).norm();
                if (d >= pair_min_ && d <= pair_max_) {
                    candidates.push_back({(int)i, (int)j, d});
                }
            }
        }

        if (candidates.empty()) {
            ROS_WARN_THROTTLE(2.0, "没有找到任何配对，无法判断左右");
            return;
        }

        // ---------- Step 2: 贪心配对 ----------
        // 越接近标准赛道宽度的配对越可信，所以按距离从小到大排序
        std::sort(candidates.begin(), candidates.end(),
                  [](const Pair& a, const Pair& b) { return a.dist < b.dist; });

        std::vector<bool> used(n, false);
        std::vector<Pair> pairs;
        for (const auto& c : candidates) {
            if (used[c.i] || used[c.j]) continue;
            used[c.i] = used[c.j] = true;
            pairs.push_back(c);
        }

        if (pairs.empty()) return;

        // ---------- Step 3: 计算配对中点，并按距离排序 ----------
        std::vector<std::pair<float, Eigen::Vector3f>> mids;   // (距离, 中点)
        for (const auto& p : pairs) {
            Eigen::Vector3f mid = (cones[p.i].pos + cones[p.j].pos) * 0.5f;
            mids.push_back({mid.head<2>().norm(), mid});
        }
        std::sort(mids.begin(), mids.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // ---------- Step 4: 逐对判定左右 ----------
        for (size_t k = 0; k < pairs.size(); ++k) {
            const Pair& pr = pairs[k];

            // 求这一对所在位置的赛道方向
            Eigen::Vector3f dir;

            if (pairs.size() >= 2) {
                // 用相邻中点连线作为赛道方向
                int k0 = std::max(0, (int)k - 1);
                int k1 = std::min((int)mids.size() - 1, (int)k + 1);
                dir = mids[k1].second - mids[k0].second;
                if (dir.norm() < 1e-4f) {
                    // 中点重合，退化为「车头方向」
                    dir = -mids[k].second;      // 从中点指向车
                }
                dir.z() = 0;                    // 只在水平面上考虑
                dir.normalize();
            } else {
                // 只有一对，用「从中点指向车」作为赛道方向的近似
                dir = -mids[0].second;
                dir.z() = 0;
                dir.normalize();
            }

            // ★★★ 关键：求赛道方向的「左侧」★★★
            // ROS 右手坐标系，z 轴向上
            // left = z × dir
            Eigen::Vector3f left_dir(-dir.y(), dir.x(), 0.0f);
            left_dir.normalize();

            // 判断 i 相对 j 是在左边还是右边
            Eigen::Vector3f diff = cones[pr.i].pos - cones[pr.j].pos;
            float s = diff.dot(left_dir);

            if (s > 0) {
                cones[pr.i].color = RED;        // i 在左 → 红
                cones[pr.j].color = BLUE;       // j 在右 → 蓝
            } else {
                cones[pr.i].color = BLUE;
                cones[pr.j].color = RED;
            }
        }

        // 没配上对的锥桶保持 UNKNOWN（不发布）
        // 理由：宁可漏发，不可错发
    }

    // ========================================================
    //  步骤 7：跨帧投票（方案 v4，指南 §5.4）
    //
    //  观测在 imu_map（世界系）下按位置关联到已有轨迹，
    //  红/蓝各计一票；观测次数和票数比例双双过线才锁定。
    //  锁定后不再改票 → 颜色永不跳变。
    // ========================================================
    void updateTracks(const std::vector<ConeObs>& map_obs) {
        std::vector<bool> hit(tracks_.size(), false);

        for (const auto& o : map_obs) {
            if (o.color == UNKNOWN) continue;      // 没配上对的不投票

            // ---- 找最近轨迹（0.5 m 门限，锥桶在地图系里不动） ----
            int best = -1;
            float best_d = assoc_dist_;
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

            if (o.color == RED) ++t.red_votes; else ++t.blue_votes;
            const int total = t.red_votes + t.blue_votes;
            if (t.obs >= min_obs_ && total > 0) {
                const float r = static_cast<float>(t.red_votes)  / total;
                const float b = static_cast<float>(t.blue_votes) / total;
                if (r >= vote_thr_)      t.locked = RED;
                else if (b >= vote_thr_) t.locked = BLUE;
                // 票不纯 → 继续攒，宁可慢不可错
            }
        }

        // ---- 没被看到的轨迹记一次 miss，超期删除 ----
        for (size_t k = 0; k < tracks_.size(); ++k)
            if (!hit[k]) ++tracks_[k].miss;
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [this](const ConeTrack& t) { return t.miss > max_miss_; }),
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

            if (c.color == RED) {
                p.r = 255; p.g = 0;   p.b = 0;      // 红
            } else {
                p.r = 0;   p.g = 80;  p.b = 255;    // 蓝
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
            m.color.a = 0.9f;

            if (c.color == RED) {
                m.color.r = 1.0f; m.color.g = 0.1f; m.color.b = 0.1f;
            } else if (c.color == BLUE) {
                m.color.r = 0.1f; m.color.g = 0.3f; m.color.b = 1.0f;
            } else {
                m.color.r = m.color.g = m.color.b = 0.6f;   // 灰色 = 未确定
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
    double pair_min_, pair_max_;

    // 时序投票（跨帧锁定颜色）
    bool   enable_temporal_ = true;
    double vote_thr_   = 0.8;
    int    min_obs_    = 3;
    double assoc_dist_ = 0.5;
    int    max_miss_   = 20;
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
