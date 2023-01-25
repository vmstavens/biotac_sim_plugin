// gazebo plug-in for simulating touch sensors...

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Plugin.hh"

#include <eigen3/Eigen/Dense>
#include <ignition/math.hh>
#include <ignition/msgs/vector3d.pb.h>

#include <ros/ros.h>

#include <sr_robot_msgs/BiotacAll.h>

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "sim_biotac/sim_biotac.h"
#include "gazebo/physics/Model.hh"
#include "ros_utils_cpp/eigen.hpp"

namespace gazebo 
{
	class SimBiotacPlugin : public ModelPlugin
	{

		template <class T> class 
		ExponentialDecayFilter 
		{
				T x = T::Zero();
				double f = 0.0;

			public:
				ExponentialDecayFilter(double f) : f(f) { }
				T operator()(T v) 
				{
					x = x * f + v * (1.0 - f);
					return x;
				}
		};


		ros::NodeHandle                  nh;
		ros::Publisher                   tactilePublisher = nh.advertise<sr_robot_msgs::BiotacAll>("/rh/tactile", 1);
		event::ConnectionPtr             updateConnection;
		gazebo::physics::ModelPtr        model;
		gazebo::physics::WorldPtr        world;
		common::Time                     lastUpdateTime = 0;
		SensorModel                      sensorModel;
		common::Time                     lastSensorModelUpdateTime = 0;
		std::string                      sensorModelData, lastSensorModelData;
		std::array<std::deque<Touch>, 5> touchHistories;
		bool                             surfaceInitialized = false;
		common::Time                     lastUpdateTime0 = 0;
		Profiler                         updateProfiler{"update"};
		std::array<Touch, 5>             touchFrameAccumulator;
		std::array<size_t, 5>            touchFrameDivisor{{0, 0, 0, 0, 0}};
		
		typedef ExponentialDecayFilter<SensorModel::Input> ExpDecFilter;
		std::vector<ExpDecFilter>        sensorFilters = std::vector<ExpDecFilter>(5, ExpDecFilter(0.9));

	
public:

	void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) 
	{

		this->model = _model;
		this->world  = this->model->GetWorld();

		ROS_INFO("GazeboRosTouch::Load");
		ROS_ERROR("I'm loading a plugin---------------------------------------------------------------------------------------------------");

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SimBiotacPlugin::OnUpdate, this, _1));

		std::vector<std::string> collisions;

		for(const auto& link : model->GetLinks()) 
		{
			for(const auto& coll : link->GetCollisions()) 
			{
				collisions.push_back(coll->GetName());
				ROS_INFO("collision %s", coll->GetName().c_str());
			}
		}

		physics::ContactManager *contactManager = world->Physics()->GetContactManager();
		// physics::ContactManager *contactManager = model->GetWorld()->GetPhysicsEngine()->GetContactManager();
		std::string topic = contactManager->CreateFilter("gazebo_ros_touch", collisions);
	}


	void OnUpdate(const common::UpdateInfo &_info) 
	{
		ProfilerScope     profilerScope(updateProfiler);

		// gazebo::physics::WorldPtr world =this->model->GetWorld();

		if(!this->world) 
			return;
		
		if(!this->world->Physics()) 
			return;

		if(!this->world->Physics()->GetContactManager()) 
			return;

		bool publishStep = true;

		auto currentTime = this->world->SimTime();

		if((currentTime - lastUpdateTime0).Double() < 0.001) 
			return;
		else 
			lastUpdateTime0 = currentTime;

		if((currentTime - lastUpdateTime).Double() < 0.01) 
			publishStep = false;
		else 
			lastUpdateTime = currentTime;

		if(!model) 
			return;

		physics::ContactManager *contactManager = world->Physics()->GetContactManager();

		if(!contactManager) 
			return;

		if(!model) 
			return;
		// public: inline virtual const ignition::math::Pose3d &WorldPose() const
		auto t = model->WorldPose();
		auto modelPose = ros_utils_cpp::Eigen::make_tf(model->WorldPose());

		std::vector<std::vector<Touch>> fingerTouches(fingerLinkNames.size());

		for(size_t contactIndex = 0; contactIndex < contactManager->GetContactCount(); contactIndex++) 
		{
			auto *contact = contactManager->GetContact(contactIndex);

			for(size_t pointIndex = 0; pointIndex < contact->count; pointIndex++) 
			{
				for(size_t fingerIndex = 0; fingerIndex < fingerLinkNames.size(); fingerIndex++) 
				{
					auto &fingerLinkName = fingerLinkNames[fingerIndex];

					if(fingerLinkName == contact->collision1->GetLink()->GetName() || fingerLinkName == contact->collision2->GetLink()->GetName()) 
					{
						// ::Eigen::Vector3d force = ::Eigen::Vector3d()::Zeros();
						::Eigen::Affine3d force = ::Eigen::Affine3d();
						
						// TODO : Fix the problems below, the test.cpp is just to demonstrate that (for some reason) commenting out _sdf eliminates the GetWorld() errors ¯\_(ツ)_/¯

                        if(fingerLinkName == contact->collision1->GetLink()->GetName()) 
							force = ros_utils_cpp::Eigen::make_tf(contact->wrench[pointIndex].body1Force);

                        if(fingerLinkName == contact->collision2->GetLink()->GetName()) 
							force = ros_utils_cpp::Eigen::make_tf(contact->wrench[pointIndex].body2Force);

						Touch touch;

						// transform from sensor to finger
						::Eigen::Affine3d T_s_f = fingerToSensorTransforms[fingerIndex].inverse();

						// transform from finger link to world
						::Eigen::Affine3d T_f_w = ros_utils_cpp::Eigen::make_tf(model->GetLink(fingerLinkName)->WorldPose()).inverse();
						// ::Eigen::Affine3d T_f_w = ros_utils_cpp::Eigen::make_tf(model->GetLink(fingerLinkName)->GetWorldPose()).inverse();

						// transform from the world to the sensor
						::Eigen::Affine3d T_w_s = ros_utils_cpp::Eigen::make_tf(contact->positions[pointIndex]);

						// finished transformation from world to cp
						::Eigen::Affine3d T_w_cp = T_s_f * T_f_w * T_w_s; // TODO :understand transformation! + fix vscode auto include!

						// ::Eigen::Vector3d p = T_w_cp.translation(); // use this instead of the one below

                        touch.point = fingerToSensorTransforms[fingerIndex].inverse() * ros_utils_cpp::Eigen::make_tf(model->GetLink(fingerLinkName)->WorldPose()).inverse() * ros_utils_cpp::Eigen::make_tf(contact->positions[pointIndex]).translation();
                        // touch.point = fingerToSensorTransforms[fingerIndex].inverse() * ros_utils_cpp::Eigen::make_tf(model->GetLink(fingerLinkName)->GetWorldPose()).inverse() * ros_utils_cpp::Eigen::make_tf(contact->positions[pointIndex]);

						// wtf is going ooooooooooon
						// f_t [3x1] = R [3x3] * f [3x1]
						// force *= -1.0 // how to do this?
                        touch.force = fingerToSensorTransforms[fingerIndex].inverse().rotation() * ( -force.translation());
                        fingerTouches[fingerIndex].push_back(touch);
                    }
                }
            }
        }

        sr_robot_msgs::BiotacAll tactileAll;
        tactileAll.header.stamp = ros::Time::now();
        for(size_t fingerIndex = 0; fingerIndex < fingerLinkNames.size(); fingerIndex++) 
		{
            auto &fingerLinkName = fingerLinkNames[fingerIndex];

            Touch touchSum;
            touchSum.point = ::Eigen::Vector3d::Zero();
            touchSum.force = ::Eigen::Vector3d::Zero();
            for(auto &touch : fingerTouches[fingerIndex]) 
			{
                touchSum.point += touch.point;
                touchSum.force += touch.force;
            }

            if(fingerTouches[fingerIndex].size()) 
			{
                touchSum.point /= fingerTouches[fingerIndex].size();
            }

            for(size_t i = 0; i < touchFrameAccumulator.size(); i++) 
			{
                touchFrameAccumulator[fingerIndex].point += touchSum.point;
                touchFrameAccumulator[fingerIndex].force += touchSum.force;
                touchFrameDivisor[fingerIndex]++;
            }

            if(publishStep) 
			{
                auto divisor = touchFrameDivisor[fingerIndex];
                if(divisor) 
				{
                    float f = 1.0 / divisor;
                    touchSum.point = touchFrameAccumulator[fingerIndex].point * f;
                    touchSum.force = touchFrameAccumulator[fingerIndex].force * f;
                }
                touchFrameAccumulator[fingerIndex].point = Eigen::Vector3d::Zero();
                touchFrameAccumulator[fingerIndex].force = Eigen::Vector3d::Zero();
                touchFrameDivisor[fingerIndex] = 0;
            }

            if(publishStep) 
			{
                touchHistories[fingerIndex].push_back(touchSum);
                while(touchHistories[fingerIndex].size() > 100) {
                    touchHistories[fingerIndex].pop_front();
                }
            }

            SensorModel::Input input = sensorModel.toInput(touchHistories[fingerIndex]);

            if(publishStep) 
			{
                Eigen::Vector3f pos = input.head(3);
                input = sensorFilters[fingerIndex](input);
                input.head(3) = pos;
            }

            if(publishStep) 
			{
                auto output = sensorModel.predict(input);

                auto &tactile = tactileAll.tactiles[fingerIndex];
                tactile.electrodes.resize(electrodePositions.size());
                tactile.pac0 = output[7];
                tactile.pac1 = output[8];
                tactile.pdc = output[9];
                tactile.tac = output[10];
                tactile.tdc = output[11];
                for(size_t i = 0; i < 19; i++) 
				{
                    tactile.electrodes[i] = output[12 + i];
                }
            }
        }

        if(publishStep) 
		{
            tactilePublisher.publish(tactileAll);
        }
    }

};

GZ_REGISTER_MODEL_PLUGIN(SimBiotacPlugin)

};


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// class SimBiotacPlugin : public ModelPlugin {

//     static Eigen::Vector3d toEigen(const math::Vector3 &v) { return Eigen::Vector3d(v.x, v.y, v.z); }
//     static Eigen::Affine3d toEigen(const math::Pose &p) {
//         Eigen::Affine3d affine;
//         affine.fromPositionOrientationScale(toEigen(p.pos), Eigen::Quaterniond(p.rot.w, p.rot.x, p.rot.y, p.rot.z), Eigen::Vector3d(1, 1, 1));
//         return affine;
//     }
//     static Eigen::Vector3d toEigen(const msgs::Vector3d &v) { return Eigen::Vector3d(v.x(), v.y(), v.z()); }

//     ros::NodeHandle node_handle;
//     ros::Publisher tactilePublisher = node_handle.advertise<sr_robot_msgs::BiotacAll>("/rh/tactile", 1);

//     event::ConnectionPtr updateConnection;
//     physics::ModelPtr model;

//     common::Time lastUpdateTime = 0;

//     SensorModel sensorModel;
//     common::Time lastSensorModelUpdateTime = 0;
//     std::string sensorModelData, lastSensorModelData;
// // ------------------------------------------------------------------------------------------------------------------------------------------
//     template <class T> class ExponentialDecayFilter {
//         T x = T::Zero();
//         double f = 0.0;

//     public:
//         ExponentialDecayFilter(double f) : f(f) {}
//         T operator()(T v) {
//             x = x * f + v * (1.0 - f);
//             return x;
//         }
//     };

//     std::vector<ExponentialDecayFilter<SensorModel::Input>> sensorFilters = std::vector<ExponentialDecayFilter<SensorModel::Input>>(5, ExponentialDecayFilter<SensorModel::Input>(0.9));

//     std::array<std::deque<Touch>, 5> touchHistories;

//     bool surfaceInitialized = false;

//     common::Time lastUpdateTime0 = 0;

//     Profiler updateProfiler{"update"};

//     std::array<Touch, 5> touchFrameAccumulator;
//     std::array<size_t, 5> touchFrameDivisor{{0, 0, 0, 0, 0}};

// public:
//     void OnUpdate(const common::UpdateInfo &_info) {

//         ProfilerScope profilerScope(updateProfiler);

//         physics::WorldPtr world = model->GetWorld();
//         if(!world) return;
//         if(!world->GetPhysicsEngine()) return;
//         if(!world->GetPhysicsEngine()->GetContactManager()) return;

//         bool publishStep = true;
//         auto currentTime = world->GetSimTime();

//         if((currentTime - lastUpdateTime0).Double() < 0.001) {
//             return;
//         } else {
//             lastUpdateTime0 = currentTime;
//         }

//         if((currentTime - lastUpdateTime).Double() < 0.01) {
//             publishStep = false;
//         } else {
//             lastUpdateTime = currentTime;
//         }

//         if(!model) return;

//         physics::ContactManager *contactManager = world->GetPhysicsEngine()->GetContactManager();
//         if(!contactManager) return;

//         if(!model) return;
//         auto modelPose = toEigen(model->GetWorldPose());

//         std::vector<std::vector<Touch>> fingerTouches(fingerLinkNames.size());

//         for(size_t contactIndex = 0; contactIndex < contactManager->GetContactCount(); contactIndex++) {
//             auto *contact = contactManager->GetContact(contactIndex);
//             for(size_t pointIndex = 0; pointIndex < contact->count; pointIndex++) {
//                 for(size_t fingerIndex = 0; fingerIndex < fingerLinkNames.size(); fingerIndex++) {
//                     auto &fingerLinkName = fingerLinkNames[fingerIndex];
//                     if(fingerLinkName == contact->collision1->GetLink()->GetName() || fingerLinkName == contact->collision2->GetLink()->GetName()) {
//                         Eigen::Vector3d force = Eigen::Vector3d::Zero();
//                         if(fingerLinkName == contact->collision1->GetLink()->GetName()) force = toEigen(contact->wrench[pointIndex].body1Force);
//                         if(fingerLinkName == contact->collision2->GetLink()->GetName()) force = toEigen(contact->wrench[pointIndex].body2Force);
//                         Touch touch;
//                         touch.point = fingerToSensorTransforms[fingerIndex].inverse() * toEigen(model->GetLink(fingerLinkName)->GetWorldPose()).inverse() * toEigen(contact->positions[pointIndex]);
//                         touch.force = fingerToSensorTransforms[fingerIndex].inverse().rotation() * -force;
//                         fingerTouches[fingerIndex].push_back(touch);
//                     }
//                 }
//             }
//         }

//         sr_robot_msgs::BiotacAll tactileAll;
//         tactileAll.header.stamp = ros::Time::now();
//         for(size_t fingerIndex = 0; fingerIndex < fingerLinkNames.size(); fingerIndex++) {
//             auto &fingerLinkName = fingerLinkNames[fingerIndex];

//             Touch touchSum;
//             touchSum.point = Eigen::Vector3d::Zero();
//             touchSum.force = Eigen::Vector3d::Zero();
//             for(auto &touch : fingerTouches[fingerIndex]) {
//                 touchSum.point += touch.point;
//                 touchSum.force += touch.force;
//             }

//             if(fingerTouches[fingerIndex].size()) {
//                 touchSum.point /= fingerTouches[fingerIndex].size();
//             }

//             for(size_t i = 0; i < touchFrameAccumulator.size(); i++) {
//                 touchFrameAccumulator[fingerIndex].point += touchSum.point;
//                 touchFrameAccumulator[fingerIndex].force += touchSum.force;
//                 touchFrameDivisor[fingerIndex]++;
//             }

//             if(publishStep) {
//                 auto divisor = touchFrameDivisor[fingerIndex];
//                 if(divisor) {
//                     float f = 1.0 / divisor;
//                     touchSum.point = touchFrameAccumulator[fingerIndex].point * f;
//                     touchSum.force = touchFrameAccumulator[fingerIndex].force * f;
//                 }
//                 touchFrameAccumulator[fingerIndex].point = Eigen::Vector3d::Zero();
//                 touchFrameAccumulator[fingerIndex].force = Eigen::Vector3d::Zero();
//                 touchFrameDivisor[fingerIndex] = 0;
//             }

//             if(publishStep) {
//                 touchHistories[fingerIndex].push_back(touchSum);
//                 while(touchHistories[fingerIndex].size() > 100) {
//                     touchHistories[fingerIndex].pop_front();
//                 }
//             }

//             SensorModel::Input input = sensorModel.toInput(touchHistories[fingerIndex]);

//             if(publishStep) {
//                 Eigen::Vector3f pos = input.head(3);
//                 input = sensorFilters[fingerIndex](input);
//                 input.head(3) = pos;
//             }

//             if(publishStep) {
//                 auto output = sensorModel.predict(input);

//                 auto &tactile = tactileAll.tactiles[fingerIndex];
//                 tactile.electrodes.resize(electrodePositions.size());
//                 tactile.pac0 = output[7];
//                 tactile.pac1 = output[8];
//                 tactile.pdc = output[9];
//                 tactile.tac = output[10];
//                 tactile.tdc = output[11];
//                 for(size_t i = 0; i < 19; i++) {
//                     tactile.electrodes[i] = output[12 + i];
//                 }
//             }
//         }

//         if(publishStep) {
//             tactilePublisher.publish(tactileAll);
//         }
//     }

//     void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
//         ROS_INFO("GazeboRosTouch::Load");
//         model = _parent;
//         updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SimBiotacPlugin::OnUpdate, this, _1));
//         std::vector<std::string> collisions;
//         for(auto &link : model->GetLinks()) {
//             for(auto &coll : link->GetCollisions()) {
//                 collisions.push_back(coll->GetName());
//                 // ROS_INFO("collision %s", coll->GetName().c_str());
//             }
//         }
//         physics::ContactManager *contactManager = model->GetWorld()->GetPhysicsEngine()->GetContactManager();
//         std::string topic = contactManager->CreateFilter("gazebo_ros_touch", collisions);
//     }
// };

// GZ_REGISTER_MODEL_PLUGIN(SimBiotacPlugin)

// } // namespace gazebo
