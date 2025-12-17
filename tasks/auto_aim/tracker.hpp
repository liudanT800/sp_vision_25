#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
//应对前哨站需要跨帧选择目标的情况
struct candidate_target{ 
  std::vector<Armor> armors = {};
  std::chrono::steady_clock::time_point first_seen;
  std::chrono::steady_clock::time_point last_seen;
  ArmorName name;
  ArmorType type;
  int priority;
  bool consumed{false};
};

class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  std::chrono::steady_clock::time_point last_switch_time_;
  ArmorPriority omni_target_priority_;

  // Candidate management for outpost
  candidate_target candidates_outpost;
  double candidate_max_age_s_;
  double candidate_match_radius_m_;
  double duplicate_check_window_s_;
  double outpost_height_bucket_size_;
  double outpost_min_height_span_;
  double switch_cooldown_s_;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  // Candidate management helpers
  int get_height_bucket(double z) const;
  void add_or_update_candidate(const Armor & armor, std::chrono::steady_clock::time_point t);
  void cleanup_candidates(std::chrono::steady_clock::time_point t);
  bool try_promote_candidate(std::chrono::steady_clock::time_point t);
  double compute_height_span(const std::vector<Armor> & armors) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP