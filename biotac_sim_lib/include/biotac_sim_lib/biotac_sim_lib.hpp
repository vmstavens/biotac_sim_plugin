#pragma once

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "neural_network.hpp"

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <string>
#include <thread>


#include <ros/ros.h>
#include "helpers.hpp"
#include "ros_utils_cpp/eigen.hpp"
#include "sr_robot_msgs/BiotacAll.h"

namespace gazebo 
{
class SimBiotacPlugin : public ModelPlugin
{
	// template class member 
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

	// members
	ros::NodeHandle nh;
	ros::Publisher tactilePublisher = nh.advertise<sr_robot_msgs::BiotacAll>("/rh/tactile", 1);

	event::ConnectionPtr updateConnection;

	gazebo::physics::ModelPtr model;
	gazebo::physics::WorldPtr world;

	common::Time lastUpdateTime = 0;
	common::Time lastSensorModelUpdateTime = 0;
	common::Time lastUpdateTime0 = 0;

	helpers::SensorModel sensorModel;
	helpers::Profiler updateProfiler{"update"};

	std::string sensorModelData, lastSensorModelData;

	bool surfaceInitialized = false;

	std::array<std::deque<helpers::Touch>, 5> touchHistories;
	std::array<helpers::Touch, 5> touchFrameAccumulator;
	std::array<size_t, 5> touchFrameDivisor{{0, 0, 0, 0, 0}};

	const std::vector<std::string> fingerLinkNames = {"rh_ffmiddle", "rh_mfmiddle", "rh_rfmiddle", "rh_lfmiddle", "rh_thmiddle"};
	const std::vector<Eigen::Affine3d> fingerToSensorTransforms = {
		ros_utils_cpp::Eigen::make_tf(-0.000, -0.007, 0.051, -0.405, -0.405, -0.580, 0.580), 
		ros_utils_cpp::Eigen::make_tf(-0.000, -0.007, 0.051, -0.405, -0.405, -0.580, 0.580), 
		ros_utils_cpp::Eigen::make_tf(-0.000, -0.007, 0.051, -0.405, -0.405, -0.580, 0.580), 
		ros_utils_cpp::Eigen::make_tf(-0.000, -0.007, 0.051, -0.405, -0.405, -0.580, 0.580), 
		ros_utils_cpp::Eigen::make_tf(-0.007, 0.000, 0.054, 0.573, 0.000, 0.820, -0.000),
	};
	
	const std::vector<Eigen::Vector3d> electrodePositions = []() 
	{
		std::vector<double> xx = {0.993, -2.700, -6.200, -8.000, -10.500, -13.400, 4.763, 3.031, 3.031, 1.299, 0.993, -2.700, -6.200, -8.000, -10.500, -13.400, -2.800, -9.800, -13.600};
		std::vector<double> yy = {-4.855, -3.513, -3.513, -4.956, -3.513, -4.956, 0.000, -1.950, 1.950, 0.000, 4.855, 3.513, 3.513, 4.956, 3.513, 4.956, 0.000, 0.000, 0.000};
		std::vector<double> zz = {-1.116, -3.670, -3.670, -1.116, -3.670, -1.116, -2.330, -3.330, -3.330, -4.330, -1.116, -3.670, -3.670, -1.116, -3.670, -1.116, -5.080, -5.080, -5.080};
		
		std::vector<Eigen::Vector3d> ret;

		for(size_t i = 0; i < xx.size(); i++) 
			ret.push_back(Eigen::Vector3d(xx[i], yy[i], zz[i]) * (1.0 / 1000));

		return ret;
	}();

	std::vector<ExponentialDecayFilter<helpers::SensorModel::Input>> 
	sensorFilters = std::vector<ExponentialDecayFilter<helpers::SensorModel::Input>>(5, ExponentialDecayFilter<helpers::SensorModel::Input>(0.9));

public:

	// public methods
	void 
	Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

	void 
	OnUpdate(const common::UpdateInfo &_info);
	
};
GZ_REGISTER_MODEL_PLUGIN(SimBiotacPlugin)
};