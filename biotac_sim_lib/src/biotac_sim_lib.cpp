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
#include "gazebo/physics/Model.hh"
#include "ros_utils_cpp/eigen.hpp"

#include "biotac_sim_lib/biotac_sim_lib.hpp"

void gazebo::SimBiotacPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{

	this->model = _model;
	this->world  = this->model->GetWorld();

	ROS_INFO("GazeboRosTouch::Load");

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
	std::string topic = contactManager->CreateFilter("gazebo_ros_touch", collisions);
}

void gazebo::SimBiotacPlugin::OnUpdate(const common::UpdateInfo &_info)
{
	helpers::ProfilerScope profilerScope(updateProfiler);

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

	if(!this->model) 
		return;

	physics::ContactManager *contactManager = world->Physics()->GetContactManager();

	if(!contactManager) 
		return;

	if(!this->model) 
		return;
	
	auto t = model->WorldPose();

	auto modelPose = ros_utils_cpp::Eigen::make_tf(model->WorldPose());

	std::vector<std::vector<helpers::Touch>> fingerTouches(fingerLinkNames.size());

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
					::Eigen::Affine3d force = ::Eigen::Affine3d();
					
					// TODO : Fix the problems below, the test.cpp is just to demonstrate that (for some reason) commenting out _sdf eliminates the GetWorld() errors ¯\_(ツ)_/¯

					if(fingerLinkName == contact->collision1->GetLink()->GetName()) 
						force = ros_utils_cpp::Eigen::make_tf(contact->wrench[pointIndex].body1Force);

					if(fingerLinkName == contact->collision2->GetLink()->GetName()) 
						force = ros_utils_cpp::Eigen::make_tf(contact->wrench[pointIndex].body2Force);

					helpers::Touch touch;

					// transform from sensor to finger
					::Eigen::Affine3d T_s_f = fingerToSensorTransforms[fingerIndex].inverse();

					// transform from finger link to world
					::Eigen::Affine3d T_f_w = ros_utils_cpp::Eigen::make_tf(model->GetLink(fingerLinkName)->WorldPose()).inverse();

					// transform from the world to the sensor
					::Eigen::Affine3d T_w_s = ros_utils_cpp::Eigen::make_tf(contact->positions[pointIndex]);

					// finished transformation from world to cp
					::Eigen::Affine3d T_w_cp = T_s_f * T_f_w * T_w_s; // TODO : understand transformation!

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

		helpers::Touch touchSum;
		touchSum.point = ::Eigen::Vector3d::Zero();
		touchSum.force = ::Eigen::Vector3d::Zero();

		for(const auto& touch : fingerTouches[fingerIndex]) 
		{
			touchSum.point += touch.point;
			touchSum.force += touch.force;
		}

		if(fingerTouches[fingerIndex].size()) 
			touchSum.point /= fingerTouches[fingerIndex].size();

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

			while(touchHistories[fingerIndex].size() > 100) 
				touchHistories[fingerIndex].pop_front();
		}

		helpers::SensorModel::Input input = sensorModel.toInput(touchHistories[fingerIndex]);

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
				tactile.electrodes[i] = output[12 + i];
		}
	}

	if(publishStep) 
		tactilePublisher.publish(tactileAll);
}