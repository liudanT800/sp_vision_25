#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  last_switch_time_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;

  // Candidate相关参数//TODO：思考参数合理性
  auto config = YAML::LoadFile(config_path);
  candidate_max_age_s_ = config["tracker"]["candidate_max_age_s"].as<double>(0.8);
  candidate_match_radius_m_ = config["tracker"]["candidate_match_radius_m"].as<double>(0.5);
  duplicate_check_window_s_ = config["tracker"]["duplicate_check_window_s"].as<double>(0.15);
  outpost_height_bucket_size_ = config["tracker"]["outpost_height_bucket_size"].as<double>(0.06);
  outpost_min_height_span_ = config["tracker"]["outpost_min_height_span"].as<double>(0.11);//装甲板上下高度差204-误差30*3
  switch_cooldown_s_ = config["tracker"]["switch_cooldown_s"].as<double>(0.5);
  
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;//TODO:考虑修改这里逻辑实现提前建模前哨站快速切换；记得两个track都要改
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target dive rged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{

  if (armors.empty()) {
    // tools::logger()->debug("[Tracker] No armors to set target from");
    return false;
  }

  for (auto & armor : armors) {
    solver_.solve(armor);

    // For outpost armors, accumulate in candidate system
    if (armor.name == ArmorName::outpost) {
      add_or_update_candidate(armor, t);
    }
  }

  // Clean up old/consumed candidates
  cleanup_candidates(t);

  // Try to promote a candidate to full Target
  if (try_promote_candidate(t)) {
    return true;  // Successfully created outpost target
  }

  // Fallback: single-frame initialization for non-outpost targets
  auto & armor = armors.front();

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
    return true;
  }

  else if (armor.name == ArmorName::outpost) {
    // Outpost requires multi-frame accumulation, return false to wait
    return false;
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
    return true;
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
    return true;
  }
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (
      armor.name != target_.name || armor.type != target_.armor_type
      //  || armor.center.x != min_x
    )
      continue;

    solver_.solve(armor);

    target_.update(armor);
  }

  return true;
}

// Candidate management functions
int Tracker::get_height_bucket(double z) const
{
  return static_cast<int>(std::round(z / outpost_height_bucket_size_));
}

void Tracker::add_or_update_candidate(
  const Armor & armor, std::chrono::steady_clock::time_point t)
{
  const double z = armor.xyz_in_world[2];
  const int height_bucket = get_height_bucket(z);

  auto &candidate = candidates_outpost;
  tools::logger()->debug("[Tracker] Considering armor at z={:.3f}m (bucket={})", z, height_bucket);
  if (candidate.consumed || 
      (!candidate.armors.empty() && 
       (candidate.name != armor.name || candidate.type != armor.type))) {
    return;  
  }

  // Initialize candidate if empty
  if (candidate.armors.empty()) {
    candidate.armors.push_back(armor);
    candidate.first_seen = t;
    candidate.last_seen = t;
    candidate.name = armor.name;
    candidate.type = armor.type;
    candidate.priority = armor.priority;
    candidate.consumed = false;
    tools::logger()->debug("[Tracker] Created outpost candidate at z={:.3f}m (bucket={})", z, height_bucket);
    return;
  }

  // Check if this height bucket already exists
  bool height_exists = false;
  for (const auto & existing : candidate.armors) {
    int existing_bucket = get_height_bucket(existing.xyz_in_world[2]);
    if (existing_bucket == height_bucket) {
      height_exists = true;
      break;
    }
  }

  if (!height_exists) {
    // Calculate min/max z for logging
    double min_z = armor.xyz_in_world[2];
    double max_z = armor.xyz_in_world[2];
    for (const auto & existing : candidate.armors) {
      double existing_z = existing.xyz_in_world[2];
      min_z = std::min(min_z, existing_z);
      max_z = std::max(max_z, existing_z);
    }
    
    candidate.armors.push_back(armor);
    candidate.last_seen = t;
    tools::logger()->debug(
      "[Tracker] Added NEW armor plate: bucket={} z={:.3f}m (total unique: {}, min_z={:.3f}m, max_z={:.3f}m)", 
      height_bucket, z, candidate.armors.size(), min_z, max_z);
  } else {
    // Same height bucket exists - just update timestamp (deduplication)
    candidate.last_seen = t;
  }
}

void Tracker::cleanup_candidates(std::chrono::steady_clock::time_point t)
{
  auto & candidate = candidates_outpost;
  auto age = std::chrono::duration<double>(t - candidate.last_seen).count();
  
  // Clear candidate if consumed or too old
  if (candidate.consumed || age > candidate_max_age_s_) {
    candidate.armors.clear();
    candidate.consumed = false;
  }
}

bool Tracker::try_promote_candidate(std::chrono::steady_clock::time_point t)
{
  auto & candidate = candidates_outpost;
  
  if (candidate.consumed || candidate.armors.empty()) return false;
  if (candidate.name != ArmorName::outpost) return false;

  std::set<int> unique_buckets;
  for (const auto & armor : candidate.armors) {
    unique_buckets.insert(get_height_bucket(armor.xyz_in_world[2]));
  }

  // Validate height span
  double height_span = compute_height_span(candidate.armors);

  if (unique_buckets.size() >= 2 && height_span >= outpost_min_height_span_) {
    // Check preemption conditions
    bool can_switch = true;
    if (state_ == "tracking" || state_ == "detecting") {
      auto cooldown = std::chrono::duration<double>(t - last_switch_time_).count();
      if (cooldown < switch_cooldown_s_) {
        can_switch = false;
      }
      if (candidate.priority >= target_.priority) {
        can_switch = false;
      }
    }

    if (!can_switch) return false;

    // Sort armors by height and remove duplicates based on bucket
    std::map<int, Armor> bucket_map;
    for (const auto & armor : candidate.armors) {
      int bucket = get_height_bucket(armor.xyz_in_world[2]);
      bucket_map.insert({bucket, armor});
    }
    
    std::vector<Armor> unique_armors;
    for (auto & pair : bucket_map) {
      unique_armors.push_back(pair.second);
    }
    
    std::sort(unique_armors.begin(), unique_armors.end(), [](const Armor & a, const Armor & b) {
      return a.xyz_in_world[2] < b.xyz_in_world[2];
    });

    // Create Target with multi-armor constructor
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 1}};
    target_ = Target(unique_armors, t, 0.275, 3, P0_dig);
    candidate.consumed = true;
    last_switch_time_ = t;

    tools::logger()->info(
      "[Tracker] Promoted outpost candidate with {} unique heights (span={:.3f}m)",
      unique_buckets.size(), height_span);
    return true;
  }
  
  return false;
}

double Tracker::compute_height_span(const std::vector<Armor> & armors) const
{
  if (armors.empty()) return 0.0;
  double min_z = armors.front().xyz_in_world[2];
  double max_z = min_z;
  for (const auto & armor : armors) {
    double z = armor.xyz_in_world[2];
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
  }
  return max_z - min_z;
}

}  // namespace auto_aim