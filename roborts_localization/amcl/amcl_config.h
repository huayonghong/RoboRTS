#ifndef ROBORTS_LOCALIZATION_AMCL_CONFIG_H
#define ROBORTS_LOCALIZATION_AMCL_CONFIG_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace roborts_localization {

namespace detail {

template<typename T>
void DeclareIfMissing(rclcpp::Node::SharedPtr node, const std::string &key,
                      const T &default_val, T &out) {
  if (!node->has_parameter(key)) {
    node->declare_parameter(key, default_val);
  }
  node->get_parameter(key, out);
}

}  // namespace detail

struct AmclConfig {


  void GetParam(rclcpp::Node::SharedPtr node) {
    detail::DeclareIfMissing(node, "use_map_topic", true, use_map_topic);
    detail::DeclareIfMissing(node, "first_map_only", true, first_map_only);


    detail::DeclareIfMissing(node, "gui_publish_rate", 10, gui_publish_rate);
    detail::DeclareIfMissing(node, "laser_min_range", 0.15, laser_min_range);



    detail::DeclareIfMissing(node, "laser_max_range", 8.0, laser_max_range);


    detail::DeclareIfMissing(node, "laser_max_beams", 30, laser_max_beams);



    detail::DeclareIfMissing(node, "min_particles", 500, min_particles);



    detail::DeclareIfMissing(node, "max_particles", 5000, max_particles);



    detail::DeclareIfMissing(node, "kld_error", 0.05, kld_err);



    detail::DeclareIfMissing(node, "kld_z", 0.99, kld_z);



    detail::DeclareIfMissing(node, "z_hit", 0.5, z_hit);



    detail::DeclareIfMissing(node, "z_rand", 0.5, z_rand);



    detail::DeclareIfMissing(node, "sigma_hit", 0.2, sigma_hit);



    detail::DeclareIfMissing(node, "lambda_short", 0.1, lambda_short);



    detail::DeclareIfMissing(node, "laser_likelihood_max_dist", 2.0,

                             laser_likelihood_max_dist);



    detail::DeclareIfMissing(node, "do_beamskip", true, do_beamskip);



    detail::DeclareIfMissing(node, "beam_skip_distance", 0.5,

                             beam_skip_distance);



    detail::DeclareIfMissing(node, "beam_skip_threshold", 0.3,

                             beam_skip_threshold);



    detail::DeclareIfMissing(node, "beam_skip_error_threshold", 0.9,

                             beam_skip_error_threshold);



    detail::DeclareIfMissing(node, "odom_alpha1", 0.005, odom_alpha1);



    detail::DeclareIfMissing(node, "odom_alpha2", 0.005, odom_alpha2);



    detail::DeclareIfMissing(node, "odom_alpha3", 0.01, odom_alpha3);



    detail::DeclareIfMissing(node, "odom_alpha4", 0.005, odom_alpha4);



    detail::DeclareIfMissing(node, "odom_alpha5", 0.003, odom_alpha5);



    detail::DeclareIfMissing(node, "update_min_d", 0.2, update_min_d);



    detail::DeclareIfMissing(node, "update_min_a", 0.5, update_min_a);



    detail::DeclareIfMissing(node, "resample_interval", 1, resample_interval);



    detail::DeclareIfMissing(node, "transform_tolerance", 1.0,

                             transform_tolerance);



    detail::DeclareIfMissing(node, "recovery_alpha_slow", 0.1,

                             recovery_alpha_slow);



    detail::DeclareIfMissing(node, "recovery_alpha_fast", 0.001,

                             recovery_alpha_fast);



    detail::DeclareIfMissing(node, "use_global_localization", false,

                             use_global_localization);



    detail::DeclareIfMissing(node, "random_heading", false, random_heading);



    detail::DeclareIfMissing(node, "max_uwb_particles", 10,



                             max_uwb_particles);



    detail::DeclareIfMissing(node, "uwb_cov_x", 0.09, uwb_cov_x);



    detail::DeclareIfMissing(node, "uwb_cov_y", 0.09, uwb_cov_y);



    detail::DeclareIfMissing(node, "resample_uwb_factor", 3.0,

                             resample_uwb_factor);



    detail::DeclareIfMissing(node, "laser_filter_weight", 1.0,

                             laser_filter_weight);



  }



  bool use_map_topic;



  bool first_map_only;



  int gui_publish_rate;



  double laser_min_range;



  double laser_max_range;



  int laser_max_beams;



  int min_particles;



  int max_particles;



  double kld_err;



  double kld_z;



//  LaserModel laser_model = LASER_MODEL_LIKELIHOOD_FIELD_PROB;



  double z_hit;



  double z_rand;



  double sigma_hit;



  double lambda_short;



  double laser_likelihood_max_dist;



  bool do_beamskip;



  double beam_skip_distance;



  double beam_skip_threshold;



  double beam_skip_error_threshold;



//  OdomModel odom_model = ODOM_MODEL_OMNI;



  double odom_alpha1;



  double odom_alpha2;



  double odom_alpha3;



  double odom_alpha4;



  double odom_alpha5;



  double update_min_d;



  double update_min_a;



  int resample_interval;



  double transform_tolerance;



  double recovery_alpha_slow;



  double recovery_alpha_fast;



  bool use_global_localization;



  bool random_heading;



  double laser_filter_weight;



  int max_uwb_particles;



  double uwb_cov_x;



  double uwb_cov_y;



  double resample_uwb_factor;



};







}  // namespace roborts_localization















#endif



