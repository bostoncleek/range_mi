#include <ros/ros.h>

#include <chrono>

#include <nav_msgs/OccupancyGrid.h>
#include <range_mi/MIGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>

#include <range_mi/grid_mi.hpp>

class OccupancyGridMI {

  public:
    OccupancyGridMI()
    {
      // Initialize the node handle
      nh = ros::NodeHandle();
      ros::NodeHandle pnh("~");

      // Fetch the ROS parameters
      // std::string map_topic, mi_topic;
      // n.getParam("map_topic", map_topic);
      // n.getParam("mi_topic", mi_topic);
      // Ray tracing parameters
      pnh.getParam("num_beams", num_beams);
      pnh.getParam("condition_steps", condition_steps);
      // Noise parameters
      pnh.getParam("noise_dev", noise_dev);
      pnh.getParam("noise_truncation", noise_truncation);
      pnh.getParam("noise_integration_step", noise_integration_step);
      // Visualization vvv
      // std::string click_condition_topic, mi_map_topic, conditional_map_topic;
      pnh.getParam("visualize", visualize);
      pnh.getParam("visualize_more", visualize_more);
      // n.getParam("click_condition_topic", click_condition_topic);
      // n.getParam("mi_map_topic", mi_map_topic);
      // n.getParam("conditional_map_topic", conditional_map_topic);

      // Construct a publisher for mutual information
      mi_pub = nh.advertise<range_mi::MIGrid>("mi", 1, true);
      mi_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("mi_map", 1, true);
      conditional_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("conditional_map", 1, true);

      // Subscribe to maps and clicked points
      map_sub = nh.subscribe("map", 1, &OccupancyGridMI::map_callback, this);
      click_sub = nh.subscribe("clicked_point", 1, &OccupancyGridMI::click_callback, this);

      start_server = nh.advertiseService("start_mi", &OccupancyGridMI::setStartService, this);
    }

    bool setStartService(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
      // start_call = true;
      compute_mi();
      return true;
    }

    void map_callback(const nav_msgs::OccupancyGrid & map_msg)
    {
      ROS_INFO("Map received");
      map = map_msg;
    }

    void compute_mi()
    {
      ROS_INFO("Starting MI");
      // Store map information
      map_info = map.info;
      map_header = map.header;

      // Convert to probability
      vacancy = std::vector<double>(map_info.height * map_info.width);
      for (unsigned int i = 0; i < vacancy.size(); i++) {
        vacancy[i] = 1 - map.data[i]/99.;
        if (vacancy[i] < 0  or vacancy[i] > 1) {
          // std::cout << "Vacancy out of bounds! " << vacancy[i] << std::endl;
          vacancy[i] = 0;
        }

        vacancy[i] = std::pow(vacancy[i], map_info.resolution);
        //if (vacancy[i] > 0 and vacancy[i] < 1)  {
          //vacancy[i] = 0.99;
        //}
      }

      // Initialize mutual information computation on the grid
      mi_computer = range_mi::GridMI(
          map_info.height,
          map_info.width,
          noise_dev,
          noise_dev * noise_truncation,
          noise_integration_step);

      mi_computer.reset_mi();

      auto start = std::chrono::high_resolution_clock::now();

      double spatial_interpolation = 0;
      double theta = 0;
      double dtheta = (2 * M_PI)/num_beams;
      while (theta < 2 * M_PI) {
        // Add the beam
        mi_computer.compute_mi_beam(
            vacancy.data(),
            theta,
            dtheta,
            spatial_interpolation);

        if (spatial_interpolation == 0) {
          // Move to the next beam
          theta += dtheta;

          // Draw every time a spatial section is completed
          // if (visualize and visualize_more) {
          //   draw_map();
          // }

          if (theta < 2 * M_PI) {
            //mi_computer.reset_mi();
          }
          if (not ros::ok()) break;
        }
      }

      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      std::cout << "Ending MI computation" << std::endl;
      std::cout << "Computing mutual information of a " <<
        map_info.height << "x" << map_info.width << " map with " <<
        num_beams << " beams took " << elapsed.count() << "seconds." << std::endl;

      // if (visualize)

      draw_map();
      publish_mi();
    }

    void click_callback(const geometry_msgs::PointStamped & click_msg) {
      // Condition the map on the clicked point
      double x = click_msg.point.x/map_info.resolution;
      double y = click_msg.point.y/map_info.resolution;
      double dtheta = (2 * M_PI)/condition_steps;
      mi_computer.condition(
          vacancy.data(),
          x, y,
          0, 2 * M_PI,
          dtheta);

      // Update the mutual information
      compute_mi();
    }

    void publish_mi() {
      // Construct a message for the mutual information
      range_mi::MIGrid mi_msg;
      mi_msg.header = map_header;
      mi_msg.data = mi_computer.mi();
      mi_msg.height = map_info.height;
      mi_msg.width = map_info.width;

      // Publish
      mi_pub.publish(mi_msg);
    }

    void draw_map() {
      // Convert to an occupancy map for visualization
      nav_msgs::OccupancyGrid mi_map_msg;
      mi_map_msg.header = map_header;
      mi_map_msg.info = map_info;
      mi_map_msg.data = std::vector<int8_t>(mi_computer.mi().size());
      double mi_max = *std::max_element(mi_computer.mi().begin(), mi_computer.mi().end());
      for (size_t i = 0; i < mi_computer.mi().size(); i++) {
        // Normalize between 0 and 1
        double normalized = mi_computer.mi()[i]/mi_max;
        // Change to 100%
        mi_map_msg.data[i] = 100 * (1 - normalized);
      }
      mi_map_pub.publish(mi_map_msg);

      // Do the same with p not measured
      nav_msgs::OccupancyGrid conditional_map_msg = mi_map_msg;
      for (size_t i = 0; i < conditional_map_msg.data.size(); i++)
        conditional_map_msg.data[i] = 100 * (1 - mi_computer.p_not_measured()[i]);
      conditional_map_pub.publish(conditional_map_msg);
    }

  private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub, click_sub;
    ros::Publisher mi_pub,
      mi_map_pub,
      conditional_map_pub;

    ros::ServiceServer start_server;

    nav_msgs::OccupancyGrid map;

    // Parameters
    int num_beams;
    int condition_steps;
    bool visualize, visualize_more;

    // Noise parameters
    double noise_dev;
    double noise_truncation;
    double noise_integration_step;

    // Map data
    nav_msgs::MapMetaData map_info;
    std_msgs::Header map_header;
    std::vector<double> vacancy;

    // Computation devices
    range_mi::GridMI mi_computer;
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "occupancy_grid_mi");
  OccupancyGridMI ogmi;
  ros::spin();
  return 0;
}
