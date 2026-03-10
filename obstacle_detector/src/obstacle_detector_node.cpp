#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode() : Node("obstacle_detector_node")
  {
    // Declare parameters
    declare_parameter("scan_topic",      std::string("/scan"));
    declare_parameter("x_min",           -0.5);
    declare_parameter("x_max",            0.5);
    declare_parameter("y_min",           -0.5);
    declare_parameter("y_max",            0.5);
    declare_parameter("cluster_distance", 0.15);
    declare_parameter("min_cluster_size", 5);
    declare_parameter("min_clusters",     1);
    declare_parameter("vicinity_debounce", 5);   // consecutive frames needed to trigger vicinity
    declare_parameter("vicinity_x_min",  -0.6);
    declare_parameter("vicinity_x_max",   0.6);
    declare_parameter("vicinity_y_min",  -0.6);
    declare_parameter("vicinity_y_max",   0.6);

    // Load parameters
    x_min_          = get_parameter("x_min").as_double();
    x_max_          = get_parameter("x_max").as_double();
    y_min_          = get_parameter("y_min").as_double();
    y_max_          = get_parameter("y_max").as_double();
    cluster_dist_   = get_parameter("cluster_distance").as_double();
    min_cluster_sz_ = get_parameter("min_cluster_size").as_int();
    min_clusters_      = get_parameter("min_clusters").as_int();
    vicinity_debounce_ = get_parameter("vicinity_debounce").as_int();
    vx_min_            = get_parameter("vicinity_x_min").as_double();
    vx_max_         = get_parameter("vicinity_x_max").as_double();
    vy_min_         = get_parameter("vicinity_y_min").as_double();
    vy_max_         = get_parameter("vicinity_y_max").as_double();
    auto scan_topic = get_parameter("scan_topic").as_string();

    // Publishers
    pub_obstacle_  = create_publisher<std_msgs::msg::Bool>("/obstacle_detected",   10);
    pub_vicinity_  = create_publisher<std_msgs::msg::Bool>("/obstacle_in_vicinity",10);
    pub_markers_   = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/obstacle_detector/markers", 10);

    // Subscriber
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 10,
      std::bind(&ObstacleDetectorNode::cb_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Obstacle detector ready\n"
      "  scan:    %s\n"
      "  box:     x[%.2f, %.2f]  y[%.2f, %.2f]  (metres)\n"
      "  cluster: size>=%d  gap<=%.2f m  min_clusters=%d",
      scan_topic.c_str(), x_min_, x_max_, y_min_, y_max_,
      min_cluster_sz_, cluster_dist_, min_clusters_);
  }

private:
  void cb_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 1. Convert polar → Cartesian into a PCL cloud
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    double angle = msg->angle_min;
    for (float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        cloud->emplace_back(
          static_cast<float>(r * std::cos(angle)),
          static_cast<float>(r * std::sin(angle)),
          0.0f);
      }
      angle += msg->angle_increment;
    }

    // 2. Box filter — PassThrough on X then Y
    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(static_cast<float>(x_min_), static_cast<float>(x_max_));
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(static_cast<float>(y_min_), static_cast<float>(y_max_));
    pass.filter(*cloud_filtered);

    // 3. Euclidean clustering
    int num_clusters = 0;
    std::vector<pcl::PointIndices> cluster_indices;
    if (!cloud_filtered->empty()) {
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      tree->setInputCloud(cloud_filtered);

      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(static_cast<float>(cluster_dist_));
      ec.setMinClusterSize(min_cluster_sz_);
      ec.setMaxClusterSize(static_cast<int>(cloud_filtered->size()));
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_filtered);
      ec.extract(cluster_indices);

      num_clusters = static_cast<int>(cluster_indices.size());
    }

    // 4. Publish on state change only
    bool obstacle = num_clusters >= min_clusters_;
    if (obstacle != obstacle_active_) {
      obstacle_active_ = obstacle;
      std_msgs::msg::Bool out;
      out.data = obstacle;
      pub_obstacle_->publish(out);
      if (obstacle) {
        RCLCPP_WARN(get_logger(), "OBSTACLE DETECTED: %d cluster(s) in box", num_clusters);
      } else {
        RCLCPP_INFO(get_logger(), "Obstacle CLEARED — coverage can resume");
      }
    }

    // 5. Vicinity box — larger box, same clustering thresholds
    auto cloud_vicinity = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(static_cast<float>(vx_min_), static_cast<float>(vx_max_));
    pass.filter(*cloud_vicinity);

    pass.setInputCloud(cloud_vicinity);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(static_cast<float>(vy_min_), static_cast<float>(vy_max_));
    pass.filter(*cloud_vicinity);

    int vicinity_clusters = 0;
    if (!cloud_vicinity->empty()) {
      auto tree_v = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      tree_v->setInputCloud(cloud_vicinity);

      std::vector<pcl::PointIndices> vic_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_v;
      ec_v.setClusterTolerance(static_cast<float>(cluster_dist_));
      ec_v.setMinClusterSize(min_cluster_sz_);
      ec_v.setMaxClusterSize(static_cast<int>(cloud_vicinity->size()));
      ec_v.setSearchMethod(tree_v);
      ec_v.setInputCloud(cloud_vicinity);
      ec_v.extract(vic_indices);
      vicinity_clusters = static_cast<int>(vic_indices.size());
    }

    // Debounce: require vicinity_debounce_ consecutive positive frames before going high;
    // clear immediately on the first negative frame.
    bool raw_vicinity = vicinity_clusters >= min_clusters_;
    if (raw_vicinity) {
      vicinity_consec_ = std::min(vicinity_consec_ + 1, vicinity_debounce_);
    } else {
      vicinity_consec_ = 0;
    }
    bool in_vicinity = (vicinity_consec_ >= vicinity_debounce_);
    if (in_vicinity != vicinity_active_) {
      vicinity_active_ = in_vicinity;
      std_msgs::msg::Bool vic_out;
      vic_out.data = in_vicinity;
      pub_vicinity_->publish(vic_out);
      if (in_vicinity) {
        RCLCPP_INFO(get_logger(), "Object entered vicinity: %d cluster(s)", vicinity_clusters);
      } else {
        RCLCPP_INFO(get_logger(), "Vicinity cleared");
      }
    }

    // 6. Publish RViz2 markers
    publish_markers(msg->header, *cloud_filtered, *cloud_vicinity, cluster_indices);
  }

  // ── Marker helpers ────────────────────────────────────────────────────────

  // Returns a point for use in LINE_STRIP / POINTS markers
  static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0)
  {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  }

  void publish_markers(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ> & filtered,
    const pcl::PointCloud<pcl::PointXYZ> & vicinity,
    const std::vector<pcl::PointIndices> & cluster_indices)
  {
    visualization_msgs::msg::MarkerArray arr;

    // ── Marker 0: obstacle box outline (yellow LINE_STRIP) ──────────────────
    {
      visualization_msgs::msg::Marker box;
      box.header  = header;
      box.ns      = "obstacle_detector";
      box.id      = 0;
      box.type    = visualization_msgs::msg::Marker::LINE_STRIP;
      box.action  = visualization_msgs::msg::Marker::ADD;
      box.scale.x = 0.02;
      box.color.r = 1.0f;
      box.color.g = 1.0f;
      box.color.b = 0.0f;
      box.color.a = 0.8f;
      box.pose.orientation.w = 1.0;
      box.points = {
        make_point(x_min_, y_min_),
        make_point(x_max_, y_min_),
        make_point(x_max_, y_max_),
        make_point(x_min_, y_max_),
        make_point(x_min_, y_min_),
      };
      arr.markers.push_back(box);
    }

    // ── Marker 10: vicinity box outline (cyan LINE_STRIP, dashed via thin line) ──
    {
      visualization_msgs::msg::Marker vbox;
      vbox.header  = header;
      vbox.ns      = "obstacle_detector";
      vbox.id      = 10;
      vbox.type    = visualization_msgs::msg::Marker::LINE_STRIP;
      vbox.action  = visualization_msgs::msg::Marker::ADD;
      vbox.scale.x = 0.015;
      vbox.color.r = 0.0f;
      vbox.color.g = 0.9f;
      vbox.color.b = 0.9f;
      vbox.color.a = 0.6f;
      vbox.pose.orientation.w = 1.0;
      vbox.points = {
        make_point(vx_min_, vy_min_),
        make_point(vx_max_, vy_min_),
        make_point(vx_max_, vy_max_),
        make_point(vx_min_, vy_max_),
        make_point(vx_min_, vy_min_),
      };
      arr.markers.push_back(vbox);
    }

    // ── Marker 11: vicinity-only points (cyan POINTS — inside vicinity but outside obstacle box) ──
    {
      visualization_msgs::msg::Marker vpts;
      vpts.header  = header;
      vpts.ns      = "obstacle_detector";
      vpts.id      = 11;
      vpts.type    = visualization_msgs::msg::Marker::POINTS;
      vpts.action  = visualization_msgs::msg::Marker::ADD;
      vpts.scale.x = 0.04;
      vpts.scale.y = 0.04;
      vpts.color.r = 0.0f;
      vpts.color.g = 0.9f;
      vpts.color.b = 0.9f;
      vpts.color.a = 0.7f;
      vpts.pose.orientation.w = 1.0;
      for (const auto & p : vicinity) {
        // skip points that are already shown by the obstacle box markers
        if (p.x >= x_min_ && p.x <= x_max_ && p.y >= y_min_ && p.y <= y_max_) {
          continue;
        }
        vpts.points.push_back(make_point(p.x, p.y));
      }
      arr.markers.push_back(vpts);
    }

    // ── Marker 1: filtered points not in any cluster (grey POINTS) ──────────
    {
      // Build a set of indices that belong to a cluster
      std::vector<bool> in_cluster(filtered.size(), false);
      for (const auto & ci : cluster_indices) {
        for (int idx : ci.indices) {
          in_cluster[idx] = true;
        }
      }

      visualization_msgs::msg::Marker noise_pts;
      noise_pts.header  = header;
      noise_pts.ns      = "obstacle_detector";
      noise_pts.id      = 1;
      noise_pts.type    = visualization_msgs::msg::Marker::POINTS;
      noise_pts.action  = visualization_msgs::msg::Marker::ADD;
      noise_pts.scale.x = 0.04;
      noise_pts.scale.y = 0.04;
      noise_pts.color.r = 0.6f;
      noise_pts.color.g = 0.6f;
      noise_pts.color.b = 0.6f;
      noise_pts.color.a = 0.6f;
      noise_pts.pose.orientation.w = 1.0;

      for (std::size_t i = 0; i < filtered.size(); ++i) {
        if (!in_cluster[i]) {
          noise_pts.points.push_back(make_point(filtered[i].x, filtered[i].y));
        }
      }
      arr.markers.push_back(noise_pts);
    }

    // ── Markers 2+: one POINTS marker per cluster, colour-coded ─────────────
    // Palette: cycles through a few distinct colours
    static const std::array<std::array<float, 3>, 6> kPalette = {{
      {1.0f, 0.2f, 0.2f},   // red
      {0.2f, 0.8f, 0.2f},   // green
      {0.2f, 0.6f, 1.0f},   // blue
      {1.0f, 0.6f, 0.0f},   // orange
      {0.8f, 0.2f, 0.8f},   // magenta
      {0.0f, 0.9f, 0.9f},   // cyan
    }};

    int cluster_id = 0;
    for (const auto & ci : cluster_indices) {
      const auto & col = kPalette[cluster_id % kPalette.size()];

      visualization_msgs::msg::Marker cpt;
      cpt.header  = header;
      cpt.ns      = "obstacle_detector";
      cpt.id      = 2 + cluster_id;
      cpt.type    = visualization_msgs::msg::Marker::POINTS;
      cpt.action  = visualization_msgs::msg::Marker::ADD;
      cpt.scale.x = 0.06;
      cpt.scale.y = 0.06;
      cpt.color.r = col[0];
      cpt.color.g = col[1];
      cpt.color.b = col[2];
      cpt.color.a = 1.0f;
      cpt.pose.orientation.w = 1.0;

      for (int idx : ci.indices) {
        cpt.points.push_back(make_point(filtered[idx].x, filtered[idx].y));
      }
      arr.markers.push_back(cpt);
      ++cluster_id;
    }

    // Delete stale cluster markers from previous frames that had more clusters
    // (send DELETE action for any ids beyond what we just published)
    for (int id = 2 + cluster_id; id < 2 + prev_num_clusters_; ++id) {
      visualization_msgs::msg::Marker del;
      del.header = header;
      del.ns     = "obstacle_detector";
      del.id     = id;
      del.action = visualization_msgs::msg::Marker::DELETE;
      arr.markers.push_back(del);
    }
    prev_num_clusters_ = cluster_id;

    pub_markers_->publish(arr);
  }

  // Parameters — obstacle box
  double x_min_, x_max_, y_min_, y_max_;
  double cluster_dist_;
  int    min_cluster_sz_, min_clusters_;
  int    vicinity_debounce_;

  // Parameters — vicinity box
  double vx_min_, vx_max_, vy_min_, vy_max_;

  // State
  bool obstacle_active_{false};
  bool vicinity_active_{false};
  int  vicinity_consec_{0};
  int  prev_num_clusters_{0};

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           pub_obstacle_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           pub_vicinity_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       sub_scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
