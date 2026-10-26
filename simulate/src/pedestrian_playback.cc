#include "pedestrian_playback.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>

namespace grove_g1 {
namespace {
double Num(const YAML::Node& node) {
  const double value = node.as<double>();
  if (!std::isfinite(value)) throw std::runtime_error("Nonfinite playback number");
  return value;
}
void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
double Smooth(double u) { return std::clamp(u*u*u*(10 + u*(-15 + 6*u)), 0.0, 1.0); }
}  // namespace

PedestrianPlayback::PedestrianPlayback(const std::string& filename) {
  const auto root = YAML::LoadFile(filename);
  Require(root["schema_version"].as<int>() == 1, "Invalid playback version");
  duration_ = Num(root["duration"]);
  robot_clearance_ = Num(root["robot_clearance"]);
  Require(duration_ > 0 && robot_clearance_ > 0, "Invalid playback duration/clearance");
  int k = 0;
  for (const char* axis : {"x", "y"}) {
    auto b = root["bounds"][axis];
    Require(b.IsSequence() && b.size() == 2, "Invalid bounds");
    bounds_[k++] = Num(b[0]); bounds_[k++] = Num(b[1]);
  }
  Require(bounds_[0] < bounds_[1] && bounds_[2] < bounds_[3], "Reversed bounds");
  auto nodes = root["actors"];
  Require(nodes.IsSequence() && nodes.size() >= 1 && nodes.size() <= 3, "Invalid actor count");
  std::set<std::string> ids, bodies;
  int clients = 0;
  for (auto node : nodes) {
    Actor a;
    a.id = node["id"].as<std::string>(); a.role = node["role"].as<std::string>();
    a.body = node["body"].as<std::string>(); a.radius = Num(node["radius"]);
    Require(!a.id.empty() && ids.insert(a.id).second && bodies.insert(a.body).second,
            "Duplicate/empty actor identity");
    Require(a.body == "escort_" + a.id, "Invalid actor body mapping");
    Require(a.role == "client" || a.role == "pedestrian", "Invalid role");
    clients += a.role == "client";
    Require(a.radius > 0 && a.radius <= 0.5, "Invalid radius");
    auto initial = node["initial_xy"];
    Require(initial.IsSequence() && initial.size() == 2, "Invalid initial XY");
    a.x = Num(initial[0]); a.y = Num(initial[1]); a.yaw = Num(node["initial_yaw"]);
    auto bounded = [&](double x, double y) {
      return x >= bounds_[0]+a.radius && x <= bounds_[1]-a.radius &&
             y >= bounds_[2]+a.radius && y <= bounds_[3]-a.radius;
    };
    Require(bounded(a.x, a.y), "Initial actor outside bounds");
    auto segments = node["segments"];
    Require(segments.IsSequence() && segments.size() <= 10000, "Invalid segments");
    double end = 0, x = a.x, y = a.y, yaw = a.yaw;
    for (auto n : segments) {
      Require(n["xy0"].size() == 2 && n["xy1"].size() == 2, "Invalid segment XY");
      Segment s{Num(n["start"]), Num(n["duration"]), Num(n["xy0"][0]), Num(n["xy0"][1]),
                Num(n["xy1"][0]), Num(n["xy1"][1]), Num(n["yaw0"]), Num(n["yaw1"])};
      Require(s.duration > 0 && std::isfinite(s.start+s.duration), "Invalid segment duration");
      Require(std::abs(s.start-end) < 1e-9 && s.x0 == x && s.y0 == y && s.yaw0 == yaw,
              "Discontinuous segments");
      Require(bounded(s.x1, s.y1), "Actor endpoint outside bounds");
      end = s.start+s.duration; x=s.x1; y=s.y1; yaw=s.yaw1;
      a.segments.push_back(s);
    }
    Require(end <= duration_+1e-9, "Actor exceeds scenario duration");
    actors_.push_back(a);
  }
  Require(clients == 1, "Exactly one client required");
}

void PedestrianPlayback::ClearStart(const mjData* data) const {
  for (const auto& a : actors_) {
    Require(std::hypot(a.x-data->xpos[3*pelvis_], a.y-data->xpos[3*pelvis_+1]) >
            robot_clearance_+a.radius, "Reset/start would overlap current G1 footprint");
    for (const auto& b : actors_) {
      if (a.id != b.id) Require(std::hypot(a.x-b.x, a.y-b.y) > a.radius+b.radius,
                               "Initial actor footprints overlap");
    }
  }
}

void PedestrianPlayback::Bind(const mjModel* model, mjData* data) {
  pelvis_ = mj_name2id(model, mjOBJ_BODY, "pelvis");
  Require(pelvis_ >= 0, "G1 pelvis missing");
  std::set<int> slots;
  for (auto& a : actors_) {
    const int id = mj_name2id(model, mjOBJ_BODY, a.body.c_str());
    Require(id > 0 && model->body_mocapid[id] >= 0, "Actor body missing or not mocap");
    a.mocap = model->body_mocapid[id];
    Require(slots.insert(a.mocap).second, "Duplicate mocap slot");
    int geoms = 0;
    for (int g=0; g<model->ngeom; ++g) if (model->geom_bodyid[g] == id) {
      ++geoms;
      Require(model->geom_contype[g] == 0 && model->geom_conaffinity[g] == 0 &&
              model->geom_group[g] == 2, "Actor must be visual-only, sensor group 2");
    }
    Require(geoms > 0, "Actor has no visible geometry");
  }
  ClearStart(data);
  bound_ = model; last_time_ = data->time;
  Apply(model, data);
}

void PedestrianPlayback::Apply(const mjModel* model, mjData* data) const {
  for (const auto& a : actors_) {
    double x=a.x, y=a.y, yaw=a.yaw;
    for (const auto& s : a.segments) {
      if (elapsed_ >= s.start+s.duration) { x=s.x1; y=s.y1; yaw=s.yaw1; continue; }
      if (elapsed_ < s.start) break;
      const double u = Smooth((elapsed_-s.start)/s.duration);
      x=s.x0+(s.x1-s.x0)*u; y=s.y0+(s.y1-s.y0)*u; yaw=s.yaw0+(s.yaw1-s.yaw0)*u;
      break;
    }
    data->mocap_pos[3*a.mocap]=x; data->mocap_pos[3*a.mocap+1]=y;
    data->mocap_pos[3*a.mocap+2]=0;
    data->mocap_quat[4*a.mocap]=std::cos(yaw/2);
    data->mocap_quat[4*a.mocap+1]=data->mocap_quat[4*a.mocap+2]=0;
    data->mocap_quat[4*a.mocap+3]=std::sin(yaw/2);
  }
  // Refresh transforms before the next sensor snapshot. All calls are under sim.mtx.
  mj_kinematics(model, data);
}

void PedestrianPlayback::Advance(const mjModel* model, mjData* data) {
  Require(model == bound_, "Model changed; pedestrian playback disabled until a fresh launch");
  Require(std::isfinite(data->time) && data->time >= last_time_,
          "Simulation time rolled back; pedestrian playback disabled (G1 not reset)");
  if (state_ == "playing") {
    elapsed_ = std::min(duration_, elapsed_ + data->time-last_time_);
    if (elapsed_ >= duration_) state_ = "finished";
  }
  last_time_ = data->time;
  Apply(model, data);
}

void PedestrianPlayback::Command(const std::string& action, const mjModel* model, mjData* data) {
  Require(model == bound_, "Playback model mismatch");
  if (action == "pause") state_ = "paused";
  else if (action == "start") {
    Require(state_ == "ready" || state_ == "paused", "start requires ready/paused");
    // Starting from initial positions also checks the current G1 location.
    if (elapsed_ == 0) ClearStart(data);
    state_ = "playing";
  } else if (action == "reset" || action == "replay") {
    Require(state_ != "playing", "Pause before reset/replay");
    ClearStart(data);
    elapsed_ = 0; state_ = action == "reset" ? "ready" : "playing";
  } else throw std::runtime_error("Unknown scenario lifecycle command");
  last_time_ = data->time;
  Apply(model, data);
}

void UpdatePedestrians(const mjModel* model, mjData* data, bool poll) {
  struct Runtime {
    bool initialized=false, disabled=false;
    std::unique_ptr<PedestrianPlayback> playback;
    std::filesystem::path control, status;
    std::filesystem::file_time_type modified{};
    long long sequence=0;
    void Save(const std::string& error="") {
      if (status.empty()) return;
      YAML::Emitter out;
      out << YAML::BeginMap << YAML::Key << "state" << YAML::Value
          << (disabled ? "disabled" : playback->State())
          << YAML::Key << "scenario_time_s" << YAML::Value << (playback ? playback->Time() : 0)
          << YAML::Key << "sequence" << YAML::Value << sequence
          << YAML::Key << "error" << YAML::Value << error << YAML::EndMap;
      const auto temporary=status.string()+".tmp";
      std::ofstream file(temporary); file << out.c_str() << '\n'; file.close();
      std::filesystem::rename(temporary, status);
    }
  };
  static Runtime runtime;
  if (runtime.disabled) return;
  try {
    if (!runtime.initialized) {
      runtime.initialized=true;
      const char* path=std::getenv("GROVE_G1_PEDESTRIAN_SCENARIO");
      if (!path || !*path) return;
      runtime.control=std::filesystem::path(path).parent_path()/"control.yaml";
      runtime.status=std::filesystem::path(path).parent_path()/"status.yaml";
      runtime.playback=std::make_unique<PedestrianPlayback>(path);
      runtime.playback->Bind(model, data);
      // Never execute a leftover command from an earlier simulator process.
      if (std::filesystem::exists(runtime.control)) {
        runtime.modified=std::filesystem::last_write_time(runtime.control);
        runtime.sequence=YAML::LoadFile(runtime.control.string())["sequence"].as<long long>();
      }
      runtime.Save();
    }
    if (!runtime.playback) return;
    const std::string before=runtime.playback->State();
    runtime.playback->Advance(model, data);
    if (before != runtime.playback->State()) runtime.Save();
    if (poll && std::filesystem::exists(runtime.control)) {
      const auto modified=std::filesystem::last_write_time(runtime.control);
      if (modified != runtime.modified) {
        runtime.modified=modified;
        try {
          Require(std::filesystem::file_size(runtime.control) <= 1024, "Control file too large");
          const auto cmd=YAML::LoadFile(runtime.control.string());
          const auto sequence=cmd["sequence"].as<long long>();
          Require(sequence > runtime.sequence, "Command sequence must increase");
          runtime.sequence=sequence;
          runtime.playback->Command(cmd["action"].as<std::string>(), model, data);
          runtime.Save();
        } catch (const std::exception& error) {
          runtime.Save(error.what());
          std::fprintf(stderr, "[escort] command rejected: %s\n", error.what());
        }
      }
    }
  } catch (const std::exception& error) {
    runtime.disabled=true;
    try { runtime.Save(error.what()); } catch (...) {}
    std::fprintf(stderr, "[escort] disabled without resetting G1: %s\n", error.what());
  }
}
}  // namespace grove_g1
