#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iostream>
#include <memory>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    // Stands in for the onboard controller a real robot runs until ReleaseMode(): holds the robot
    // standing until the control stack starts driving the motors, then hands it over for good.
    // Without this every motor is dead from t=0, because the default-constructed lowcmd carries
    // zero gains, and the robot is on the floor before any controller can claim a joint.
    //
    // Two parts, because a joint PD alone cannot balance a humanoid: it holds joint angles while
    // the robot topples about its feet. The pose comes from the PD, the balance from a weld the
    // scene declares by name.
    static constexpr const char* kStartupWeldName = "pelvis_startup_hold";
    // Joint equalities named with this prefix hold a posture from the first physics step, before
    // this bridge exists. On the first tick the PD takes their angles as targets and they go.
    static constexpr const char* kStartupJointHoldPrefix = "startup_hold_";
    static constexpr std::size_t kStartupJointHoldPrefixLength = 13;
    static constexpr double kStartupHoldKp = 200.0;
    static constexpr double kStartupHoldKd = 5.0;
    // Bridge ticks at 1 kHz, so half a second for a balance policy to reach its stance while the
    // pelvis is still supported.
    static constexpr int kStartupSettleTicks = 500;

    // Per motor, so controllers spawning in any order hand over one group at a time instead of
    // the first to activate dropping every joint the others still hold.
    std::vector<char> startup_hold_;
    std::vector<mjtNum> startup_hold_q_;
    int startup_weld_id_ = -1;
    int startup_settle_ticks_ = 0;

    // Equalities waiting to be switched off. eq_active is written only here, from mjcb_control on
    // the physics thread: flipped from this thread it races mj_makeConstraint, which counts the
    // active equalities before building them and exits the simulator on a mismatch.
    static inline std::unique_ptr<std::atomic<char>[]> pending_release_;
    static inline std::atomic<int> pending_release_count_{0};

    static void _apply_pending_releases(const mjModel* m, mjData* d)
    {
        const int count = std::min(pending_release_count_.load(), m->neq);
        for (int e = 0; e < count; e++) {
            if (pending_release_[e].exchange(0)) { d->eq_active[e] = 0; }
        }
    }

    void _capture_startup_hold()
    {
        startup_hold_.assign(num_motor_, 1);
        startup_hold_q_.resize(num_motor_);
        for (int i = 0; i < num_motor_; i++) {
            const int joint_id = mj_model_->actuator_trnid[2 * i];
            startup_hold_q_[i] = joint_id >= 0
                                     ? mj_model_->qpos0[mj_model_->jnt_qposadr[joint_id]]
                                     : 0.0;
        }
        startup_weld_id_ = mj_name2id(mj_model_, mjOBJ_EQUALITY, kStartupWeldName);

        // The queue exists before the callback that drains it.
        pending_release_ = std::make_unique<std::atomic<char>[]>(mj_model_->neq);
        pending_release_count_ = mj_model_->neq;
        mjcb_control = _apply_pending_releases;
        _take_over_startup_joint_holds();
    }

    // Only equalities on a joint a motor drives are released: the PD has to take the posture over.
    void _take_over_startup_joint_holds()
    {
        for (int e = 0; e < mj_model_->neq; e++) {
            const char* name = mj_id2name(mj_model_, mjOBJ_EQUALITY, e);
            if (mj_model_->eq_type[e] != mjEQ_JOINT || mj_model_->eq_obj2id[e] >= 0 ||
                name == nullptr ||
                std::strncmp(name, kStartupJointHoldPrefix, kStartupJointHoldPrefixLength) != 0) {
                continue;
            }
            const int joint_id = mj_model_->eq_obj1id[e];
            bool driven = false;
            for (int i = 0; i < num_motor_; i++) {
                if (mj_model_->actuator_trntype[i] == mjTRN_JOINT &&
                    mj_model_->actuator_trnid[2 * i] == joint_id) {
                    // A single-joint equality holds qpos at qpos0 + polycoef[0].
                    startup_hold_q_[i] = mj_model_->qpos0[mj_model_->jnt_qposadr[joint_id]] +
                                         mj_model_->eq_data[e * mjNEQDATA];
                    driven = true;
                }
            }
            if (driven) { pending_release_[e] = 1; }
        }
    }

    void _hold_motor(int i)
    {
        mj_data_->ctrl[i] =
            kStartupHoldKp * (startup_hold_q_[i] - mj_data_->sensordata[i]) +
            kStartupHoldKd * (0.0 - mj_data_->sensordata[i + num_motor_]);
    }

    // Released once every motor has been driven for kStartupSettleTicks, so the robot is never
    // left balancing on a half-claimed body or on a policy that has not reached its stance yet.
    // A scene without the weld simply has nothing to release.
    //
    // The settle matters: which controller activates last is a spawner race, so without it the
    // weld sometimes releases on the tick a balance policy first writes, while its blend is still
    // at zero and its command is therefore a static pose. A static pose topples.
    void _release_startup_weld_if_done()
    {
        if (startup_weld_id_ < 0) { return; }
        for (int i = 0; i < num_motor_; i++) {
            if (startup_hold_[i]) { return; }
        }
        if (++startup_settle_ticks_ < kStartupSettleTicks) { return; }
        pending_release_[startup_weld_id_] = 1;
        startup_weld_id_ = -1;
    }

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            if(startup_hold_.empty()) { _capture_startup_hold(); }

            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];

                // Hand this motor over on the first frame that would actually drive it, not on
                // the first frame to arrive: the hardware component publishes continuously from
                // the moment it activates, and sends kp=kd=tau=0 for every joint no controller
                // has claimed yet. Latched, so a motor cannot fall back to the hold.
                if(startup_hold_[i] &&
                   (m.kp() != 0.0f || m.kd() != 0.0f || m.tau() != 0.0f)) {
                    startup_hold_[i] = 0;
                }

                if(startup_hold_[i]) {
                    _hold_motor(i);
                } else {
                    mj_data_->ctrl[i] = m.tau() +
                                        m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                        m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
                }
            }
            _release_startup_weld_if_done();
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
