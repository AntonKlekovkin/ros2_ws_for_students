#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include "gz/sim/Joint.hh"
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gz_ros2_sphero_base
{
    // Структура для хранения состояния линка
    struct LinkState
    {
        gz::math::Vector3d position;     // Позиция в мировой системе
        gz::math::Vector3d euler_angles; // Углы Эйлера (roll, pitch, yaw) в радианах
        double local_pitch;
        gz::math::Quaterniond orientation; // Ориентация в виде кватерниона

        // Скорости
        gz::math::Vector3d linear_velocity;   // Линейная скорость (мировая)
        gz::math::Vector3d angular_velocity;  // Угловая скорость (мировая)
        
        // Ускорения
        gz::math::Vector3d linear_acceleration;   // Линейное ускорение
        gz::math::Vector3d angular_acceleration;  // Угловое ускорение

        gz::math::Vector3d local_linear_velocity;
        gz::math::Vector3d local_angular_velocity;

            LinkState()
            : position(gz::math::Vector3d::Zero),
              euler_angles(gz::math::Vector3d::Zero),
              local_pitch(0.0),
              orientation(gz::math::Quaterniond::Identity),
              linear_velocity(gz::math::Vector3d::Zero),
              angular_velocity(gz::math::Vector3d::Zero),
              linear_acceleration(gz::math::Vector3d::Zero),
              angular_acceleration(gz::math::Vector3d::Zero),
              local_linear_velocity(gz::math::Vector3d::Zero),
              local_angular_velocity(gz::math::Vector3d::Zero) {}
    };

    // Структура для хранения состояния джоинта
    struct JointState
    {
        double position = 0.0;     
        double velocity = 0.0;         
    };

    class AngleTracker
    {
    private:
        bool initialized_ = false;
        double prevAngle_ = 0.0;
        int fullRevolutions_ = 0;
    
    public:
        // Сброс трекера
        void reset()
        {
            initialized_ = false;
            prevAngle_ = 0.0;
            fullRevolutions_ = 0;
        }
        
        // Получение непрерывного угла
        double getContinuous(double currentAngle)
        {
            // currentAngle должен быть в диапазоне [-π, π]
            
            if (!initialized_)
            {
                prevAngle_ = currentAngle;
                initialized_ = true;
                fullRevolutions_ = 0;
                
                // Возвращаем угол в [0, 2π] для первого вызова
                return (currentAngle < 0) ? currentAngle + 2*M_PI : currentAngle;
            }
            
            // Вычисляем разницу
            double diff = currentAngle - prevAngle_;
            
            // Корректируем разрыв через границу ±π
            if (diff > M_PI)
            {
                fullRevolutions_--;
            }
            else if (diff < -M_PI)
            {
                fullRevolutions_++;
            }
            
            prevAngle_ = currentAngle;
            
            // Возвращаем непрерывный угол
            return currentAngle + 2*M_PI * fullRevolutions_;
        }
        
        // Получить текущий непрерывный угол без обновления
        double getCurrentContinuous(double currentAngle) const
        {
            if (!initialized_)
            {
                return (currentAngle < 0) ? currentAngle + 2*M_PI : currentAngle;
            }
            return currentAngle + 2*M_PI * fullRevolutions_;
        }
        
        // Получить количество полных оборотов
        int getRevolutions() const
        {
            return fullRevolutions_;
        }
    };

    class SpheroBasePlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
    {
        private:

            // Переменные
            std::shared_ptr<rclcpp::Node> node_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pendulum_torque_sub_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rotor_torque_sub_;

            //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

            gz::sim::Model model;

            gz::sim::Link sphere_link;            
            gz::sim::Link pendulum_link;
            gz::sim::Link rotor_link;   
            gz::sim::Joint pendulum_joint;    
            gz::sim::Joint rotor_joint;    
            
            double target_pendulum_torque_ = 0.0;
            double target_rotor_torque_ = 0.0;  

            AngleTracker pitch_tracker_;
            AngleTracker yaw_tracker_;

            int first_step = 1;

            // sdf parameters
            double sphere_contact_friction = 0.0;            
            bool applyInitCond = false;
            gz::math::Vector3d init_force_vect = gz::math::Vector3d::Zero;
            gz::math::Vector3d offset_force = gz::math::Vector3d::Zero;
            double init_pendulum_vel = 0.0;
            double init_rotor_vel = 0.0;

            double GetPitch(const gz::math::Quaterniond& q)
            {
                // Получаем матрицу из кватерниона
                std::array<std::array<double, 3>, 3> R;
                double w = q.W(), x = q.X(), y = q.Y(), z = q.Z();
                
                R[0][0] = 1 - 2*(y*y + z*z);  R[0][1] = 2*(x*y - w*z);  R[0][2] = 2*(x*z + w*y);
                R[1][0] = 2*(x*y + w*z);      R[1][1] = 1 - 2*(x*x + z*z);  R[1][2] = 2*(y*z - w*x);
                R[2][0] = 2*(x*z - w*y);      R[2][1] = 2*(y*z + w*x);  R[2][2] = 1 - 2*(x*x + y*y);
                
                return pitch_tracker_.getContinuous(atan2(-R[2][0], R[2][2]));                

                //return pitch_tracker_.getContinuous(q.Pitch());
            }
            
            // Получение локальной скорости 
            gz::math::Vector3d GetLocalVelocity(const gz::sim::Link &link, const gz::sim::EntityComponentManager &ecm)
            {
                if (!link.Valid(ecm))
                    return gz::math::Vector3d::Zero;
                
                auto world_vel_opt = link.WorldLinearVelocity(ecm);
                
                if (!world_vel_opt.has_value())
                    return gz::math::Vector3d::Zero;
                
                gz::math::Vector3d world_vel = world_vel_opt.value();
                
                auto world_pose_opt = link.WorldPose(ecm);
                if (!world_pose_opt.has_value())
                    return gz::math::Vector3d::Zero;
                
                gz::math::Quaterniond link_rot = world_pose_opt.value().Rot();
                
                return link_rot.RotateVectorReverse(world_vel);
            }

            void GetLinks(gz::sim::EntityComponentManager &ecm)
            {
                sphere_link = gz::sim::Link(model.LinkByName(ecm, "base_link"));
                pendulum_link = gz::sim::Link(model.LinkByName(ecm, "pendulum_link"));
                rotor_link = gz::sim::Link(model.LinkByName(ecm, "rotor_link"));

                RCLCPP_INFO(node_->get_logger(), "Available links:");
             
                if(sphere_link.Valid(ecm))
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", sphere_link.Name(ecm).value().c_str());
                }
                if(pendulum_link.Valid(ecm))
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", pendulum_link.Name(ecm).value().c_str());
                }
                if(rotor_link.Valid(ecm))
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", rotor_link.Name(ecm).value().c_str());
                }
            }

            void GetJoints(gz::sim::EntityComponentManager &ecm)
            {
                pendulum_joint = gz::sim::Joint(model.JointByName(ecm, "pendulum_joint"));
                rotor_joint = gz::sim::Joint(model.JointByName(ecm, "rotor_joint"));   
                
                RCLCPP_INFO(node_->get_logger(), "Available joints:");           
                if(pendulum_joint.Valid(ecm))
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", pendulum_joint.Name(ecm).value().c_str());
                }
                if(rotor_joint.Valid(ecm))
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", rotor_joint.Name(ecm).value().c_str());
                }
            }

            void SetJointTorque(gz::sim::EntityComponentManager &ecm, gz::sim::Joint joint, double target_torque)
            {                
                joint.SetForce(ecm, {target_torque});
            }

            JointState GetJointState(const gz::sim::EntityComponentManager &ecm, gz::sim::Joint &joint)
            {
                JointState state;

                auto pos_opt = joint.Position(ecm);
                if (pos_opt && !pos_opt->empty())
                {
                    state.position = pos_opt.value()[0];
                }
                                
                auto vel_opt = joint.Velocity(ecm);
                if (vel_opt.has_value() && !vel_opt.value().empty())
                {
                    state.velocity = vel_opt.value()[0];
                }

                return state;
            }

            LinkState GetLinkState(const gz::sim::EntityComponentManager &ecm, gz::sim::Link &link)
            {
                LinkState state;

                if (!link.Valid(ecm))
                {
                    gzerr << "Link is invalid!" << std::endl;
                    return state;
                }
                                
                // Получаем позицию
                auto world_pose_opt = link.WorldPose(ecm);
                if (world_pose_opt.has_value())
                {
                    state.position = world_pose_opt.value().Pos();
                    state.orientation = world_pose_opt.value().Rot();
                    state.euler_angles = state.orientation.Euler();
                }
                
                // Получаем скорости
                auto world_vel_opt = link.WorldLinearVelocity(ecm);
                if (world_vel_opt.has_value())
                    state.linear_velocity = world_vel_opt.value();
                
                auto world_ang_vel_opt = link.WorldAngularVelocity(ecm);
                if (world_ang_vel_opt.has_value())
                    state.angular_velocity = world_ang_vel_opt.value();
                
                // Получаем ускорения
                auto world_acc_opt = link.WorldLinearAcceleration(ecm);
                if (world_acc_opt.has_value())
                    state.linear_acceleration = world_acc_opt.value();
                
                auto world_ang_acc_opt = link.WorldAngularAcceleration(ecm);
                if (world_ang_acc_opt.has_value())
                    state.angular_acceleration = world_ang_acc_opt.value();
                
                // Вычисляем локальные скорости
                if (world_vel_opt.has_value() && world_pose_opt.has_value())
                {
                    gz::math::Quaterniond rot = world_pose_opt.value().Rot();
                    state.local_linear_velocity = rot.RotateVectorReverse(state.linear_velocity);
                    state.local_angular_velocity = rot.RotateVectorReverse(state.angular_velocity);
                }
                
                return state;
            }

            void GetParamsFromUrdf(const std::shared_ptr<const sdf::Element> &_sdf)
            {                   
                if (_sdf && _sdf->HasElement("sphere_contact_friction"))
                {
                    sphere_contact_friction = _sdf->Get<double>("sphere_contact_friction");
                    gzmsg << "sphere_contact_friction is " << sphere_contact_friction << std::endl;
                }

                if (_sdf && _sdf->HasElement("apply_initial_condition"))
                {
                    applyInitCond = _sdf->Get<bool>("apply_initial_condition");
                    gzmsg << "apply_initial_condition is " << applyInitCond << std::endl;
                }

                if (_sdf && _sdf->HasElement("init_force_vect"))
                {
                    std::string force_str = _sdf->Get<std::string>("init_force_vect");
                    std::istringstream iss(force_str);
                    double x, y, z;
                    if (iss >> x >> y >> z)
                    {
                        init_force_vect.X() = x;
                        init_force_vect.Y() = y;
                        init_force_vect.Z() = z;
                        
                        gzmsg << "init_force_vect: (" << x << ", " << y << ", " << z << ")" << std::endl;
                    }
                    else
                    {
                        gzmsg << "init_force_vect error! " << std::endl;
                    }                    
                }

                if (_sdf && _sdf->HasElement("init_force_offset"))
                {
                    std::string offset_force_str = _sdf->Get<std::string>("init_force_offset");
                    std::istringstream iss(offset_force_str);
                    double x, y, z;
                    if (iss >> x >> y >> z)
                    {
                        offset_force.X() = x;
                        offset_force.Y() = y;
                        offset_force.Z() = z;

                        gzmsg << "init_force_offset: (" << x << ", " << y << ", " << z << ")" << std::endl;
                    }
                    else
                    {
                        gzmsg << "init_force_offset error! " << std::endl;
                    }                    
                }
                

                if (_sdf && _sdf->HasElement("init_pendulum_vel"))
                {
                    init_pendulum_vel = _sdf->Get<double>("init_pendulum_vel");
                    gzmsg << "init_pendulum_vel is " << init_pendulum_vel << std::endl;
                }

                if (_sdf && _sdf->HasElement("init_rotor_vel"))
                {
                    init_rotor_vel = _sdf->Get<double>("init_rotor_vel");
                    gzmsg << "init_rotor_vel is " << init_rotor_vel << std::endl;
                }
            }

            
            void ApplyInitialConditions(gz::sim::EntityComponentManager &_ecm)
            {                                                
                sphere_link.AddWorldForce(_ecm, init_force_vect, offset_force);
                
                // double ms = 0.072;
                // double mb = 0.163 + 0.016;
                // double rs = 0.06;
                // double rho = 0.65;
                // double rb = 0.024;
                // double g1 = 0.1736;
                // double g3 = 0.9848;
                // double ib1 = 0.00021;
                // double ib3 = 0.00016;
                // double is = 0.00017;
                // double v0 = 0.633147;
                // double g = 9.81;
                
                // double pendulum_vel = v0 / (rs * g3);
                // double rotor_vel = -( (-rs*rs*((ms+mb)*rho + mb*rb*g1) - rs*g3*(mb*rb*rho + g1*(ib1-ib3)) + is*rho)*v0*v0 + mb*rb*g*rs*rho*rho*g1) / (rho*rs*v0*ib3*g1) ;
                
                pendulum_joint.SetVelocity(_ecm, {init_pendulum_vel});
                gzmsg << "Pendulum vel: " << init_pendulum_vel << std::endl;
                
                rotor_joint.SetVelocity(_ecm, {init_rotor_vel});
                gzmsg << "Rotor vel: " << init_rotor_vel << std::endl;
            }

            void PublishOdometry(const LinkState& sphere_state, const gz::sim::UpdateInfo& _info)
            {
                nav_msgs::msg::Odometry odom_msg;
                
                // Заполняем заголовок
                odom_msg.header.stamp = node_->now();
                odom_msg.header.frame_id = "odom";  // Мировая система координат
                odom_msg.child_frame_id = "base_link";  // Система координат робота
                
                // Позиция (в мировой системе)
                odom_msg.pose.pose.position.x = sphere_state.position.X();
                odom_msg.pose.pose.position.y = sphere_state.position.Y();
                odom_msg.pose.pose.position.z = sphere_state.position.Z();
                
                // Ориентация (кватернион)
                odom_msg.pose.pose.orientation.x = sphere_state.orientation.X();
                odom_msg.pose.pose.orientation.y = sphere_state.orientation.Y();
                odom_msg.pose.pose.orientation.z = sphere_state.orientation.Z();
                odom_msg.pose.pose.orientation.w = sphere_state.orientation.W();
                
                // Ковариация для позиции (пока заполняем нулями, можно добавить позже)
                odom_msg.pose.covariance.fill(0.0);
                
                // Линейная скорость (в мировой системе)
                odom_msg.twist.twist.linear.x = sphere_state.linear_velocity.X();
                odom_msg.twist.twist.linear.y = sphere_state.linear_velocity.Y();
                odom_msg.twist.twist.linear.z = sphere_state.linear_velocity.Z();
                
                // Угловая скорость (в мировой системе)
                odom_msg.twist.twist.angular.x = sphere_state.angular_velocity.X();
                odom_msg.twist.twist.angular.y = sphere_state.angular_velocity.Y();
                odom_msg.twist.twist.angular.z = sphere_state.angular_velocity.Z();
                
                // Ковариация для скорости (пока заполняем нулями)
                odom_msg.twist.covariance.fill(0.0);
                
                // Публикуем
                odom_pub_->publish(odom_msg);
            }


        public:
            SpheroBasePlugin() = default;
            ~SpheroBasePlugin() override = default;

            void Configure(const gz::sim::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        gz::sim::EntityComponentManager &_ecm,
                        gz::sim::EventManager &_eventMgr) override
            {

                // Получаем модель
                model = gz::sim::Model(_entity);
                if (!model.Valid(_ecm))
                {
                    gzerr << "SpheroBasePlugin: Model is invalid.\n";
                    return;
                }

                // Инициализируем ROS2
                if (!rclcpp::ok())
                {
                    rclcpp::init(0, nullptr);
                }

                node_ = std::make_shared<rclcpp::Node>("sphero_plugin");
                                
                // Получаем все линки и джойнты модели
                GetLinks(_ecm);
                GetJoints(_ecm);

                rotor_joint.EnablePositionCheck(_ecm, true);
                rotor_joint.EnableVelocityCheck(_ecm, true);

                pendulum_joint.EnablePositionCheck(_ecm, true);
                pendulum_joint.EnableVelocityCheck(_ecm, true);
                
                sphere_link.EnableVelocityChecks(_ecm, true);
                sphere_link.EnableAccelerationChecks(_ecm, true);
                
                pendulum_link.EnableVelocityChecks(_ecm, true);
                pendulum_link.EnableAccelerationChecks(_ecm, true);
                
                rotor_link.EnableVelocityChecks(_ecm, true);
                rotor_link.EnableAccelerationChecks(_ecm, true);

                // Получаем параметры из SDF
                GetParamsFromUrdf(_sdf);
                
                // Подписываемся на топики с задаваемым моментом на маятник и ротор
                pendulum_torque_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                    "/pendulum_torque",
                    10,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        target_pendulum_torque_ = msg->data;
                    });

                rotor_torque_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                    "/rotor_torque",  
                    10,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        target_rotor_torque_ = msg->data;
                    });

                // Публикуем состояние
                //state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("sphero_states", 10);
                joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("sphero_joint_states", 10);
                odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("sphero_odom", 10);

                gzmsg << "SpheroBasePlugin initialized successfully." << std::endl;
            }

            void PreUpdate(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm) override
            {
                if (_info.paused) 
                    return;

                if(first_step)
                {
                    first_step=0;
                    if(applyInitCond == true)
                    {
                        //применяем начальные условия
                        ApplyInitialConditions(_ecm);
                        return;
                    }
                }

                // Применяем момент к маятнику
                SetJointTorque(_ecm, pendulum_joint, -target_pendulum_torque_);

                // sphere contact friction
                auto sphere_angular_vel = GetLinkState(_ecm, sphere_link).angular_velocity;
                
                if (sphere_angular_vel.Length() > 0.001) // Минимальный порог для избежания джиттера
                {                    
                    // Момент = -коэффициент_трения * угловая_скорость                    
                    auto sphere_friction_torque = -(sphere_angular_vel * sphere_contact_friction);
                                        
                    sphere_link.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, sphere_friction_torque);
                }

                // Применяем момент к ротору
                SetJointTorque(_ecm, rotor_joint, target_rotor_torque_);
                
                // Обрабатываем ROS2 сообщения
                if (node_ && rclcpp::ok())
                {
                    rclcpp::spin_some(node_);
                }
            }

            void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override
            {
                if (_info.paused)  
                    return;

                if (_info.iterations < 5)
                {
                    auto sphereLinkState = GetLinkState(_ecm, sphere_link);
                    auto pendulumLinkState = GetLinkState(_ecm, pendulum_link);
                    
                    gzmsg << "Iter " << _info.iterations 
                        << " | Sphere lin vel: " << sphereLinkState.linear_velocity
                        << " | Sphere ang vel: " << sphereLinkState.angular_velocity
                        << " | Sphere lin acc: " << sphereLinkState.linear_acceleration
                        << " | Pendulum ang vel: " << pendulumLinkState.angular_velocity
                        << std::endl;
                }

                auto pendulumJointState = GetJointState(_ecm, pendulum_joint);
                auto rotorJointState = GetJointState(_ecm, rotor_joint);
                auto sphereLinkState = GetLinkState(_ecm, sphere_link);
                auto pendulumLinkState = GetLinkState(_ecm, pendulum_link);
                auto rotorLinkState = GetLinkState(_ecm, rotor_link);

                auto pitch = GetPitch(pendulumLinkState.orientation);
                auto sphere_angle_yaw = yaw_tracker_.getContinuous(pendulumLinkState.orientation.Yaw());

                // Публикуем данные в топик
                auto joint_msg = sensor_msgs::msg::JointState();
                joint_msg.header.stamp = node_->now();                
                joint_msg.name = {"pendulum_joint", "rotor_joint", "sphereYaw"};

                // position
                joint_msg.position.clear();
                joint_msg.position.push_back(pitch);      //     pendulumLinkState.euler_angles.Y()     
                joint_msg.position.push_back(rotorJointState.position);
                joint_msg.position.push_back(sphere_angle_yaw);
                
                // velocity
                joint_msg.velocity.clear();
                joint_msg.velocity.push_back(pendulumJointState.velocity);
                joint_msg.velocity.push_back(rotorJointState.velocity);
                joint_msg.velocity.push_back(sphereLinkState.angular_velocity.Z());
                
                // effort
                joint_msg.effort.clear();
                joint_msg.effort.push_back(target_pendulum_torque_);
                joint_msg.effort.push_back(target_rotor_torque_);
                joint_msg.effort.push_back(0.0);

                joint_state_pub_->publish(joint_msg);

                PublishOdometry(sphereLinkState, _info);
            }
    };
} // namespace gz_ros2_sphero_base
GZ_ADD_PLUGIN(
    gz_ros2_sphero_base::SpheroBasePlugin,
    gz::sim::System,
    gz_ros2_sphero_base::SpheroBasePlugin::ISystemConfigure,
    gz_ros2_sphero_base::SpheroBasePlugin::ISystemPreUpdate,
    gz_ros2_sphero_base::SpheroBasePlugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    gz_ros2_sphero_base::SpheroBasePlugin,
    "gz_ros2_sphero_base::SpheroBasePlugin"
)