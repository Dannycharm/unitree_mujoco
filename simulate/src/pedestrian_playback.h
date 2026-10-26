#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include <vector>

namespace grove_g1 {
// Single simulation-thread owner. Never touches qpos, qvel, ctrl, or simulation time.
class PedestrianPlayback {
 public:
  explicit PedestrianPlayback(const std::string& filename);
  void Bind(const mjModel* model, mjData* data);
  void Advance(const mjModel* model, mjData* data);
  void Command(const std::string& action, const mjModel* model, mjData* data);
  const std::string& State() const { return state_; }
  double Time() const { return elapsed_; }
 private:
  struct Segment { double start, duration, x0, y0, x1, y1, yaw0, yaw1; };
  struct Actor {
    std::string id, role, body;
    double x, y, yaw, radius;
    int mocap = -1;
    std::vector<Segment> segments;
  };
  std::vector<Actor> actors_;
  double bounds_[4], duration_, robot_clearance_, elapsed_ = 0, last_time_ = 0;
  int pelvis_ = -1;
  const mjModel* bound_ = nullptr;
  std::string state_ = "ready";
  void ClearStart(const mjData* data) const;
  void Apply(const mjModel* model, mjData* data) const;
};

// Call only under sim.mtx: once per outer physics iteration (poll=true), and
// after every mj_step (poll=false). No worker thread, DDS, ROS, or mj_resetData.
void UpdatePedestrians(const mjModel* model, mjData* data, bool poll);
}  // namespace grove_g1
