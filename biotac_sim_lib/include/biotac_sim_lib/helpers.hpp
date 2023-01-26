#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include "neural_network.hpp"

namespace helpers 
{

// classes ///////////////////////////////////////////////////////

class Profiler 
{
	double sum = 0.0;
	double div = 0.0;
	const char *name = nullptr;

public:
	Profiler(const char *name) : name(name) {}
	friend class ProfilerScope;
};

class ProfilerScope 
{
	Profiler *profiler;
	ros::Time startTime;

public:
	ProfilerScope(Profiler &profiler) : profiler(&profiler) { startTime = ros::Time::now(); }
	~ProfilerScope() 
	{
		profiler->sum += (ros::Time::now() - startTime).toSec();
		profiler->div++;
	}
};

// template functions ////////////////////////////////////////////

// loads csv file (eg. training data)
template <class Sample> void 
loadcsv(std::istream &f, std::vector<Sample> &samples) 
{
	samples.clear();
	std::string line;
	getline(f, line);
	auto cols = Sample().size();
	while(getline(f, line)) 
	{
		Sample sample;
		char *token = (char *)line.c_str();
		for(size_t i = 0; i < cols; i++) 
		{
			float v = strtof(token, &token);
			token++;
			sample[i] = v;
			// ROS_INFO("%i %f", (int)i, v);
		}
		samples.push_back(sample);
	}
}

// writes a csv file
template <class SampleList> void 
writecsv(std::ostream &f, const SampleList &samples, bool writeHeader = false) 
{
	// if(samples.size() == 0) return;
	if(writeHeader) 
	{
		for(size_t i = 0; i < samples[0].size();) 
		{
			f << i;
			i++;
			if(i < samples[0].size()) 
			{
				f << ",";
			}
		}
		f << "\n";
	}
	for(auto &s : samples) 
	{
		for(size_t i = 0; i < s.size();) 
		{
			f << s[i];
			i++;
			if(i < s.size()) 
			{
				f << ",";
			}
		}
		f << "\n";
	}
}

// structs ///////////////////////////////////////////////////////

struct Touch 
{
	Eigen::Vector3d point = Eigen::Vector3d::Zero();
	Eigen::Vector3d force = Eigen::Vector3d::Zero();
};

// neural network based touch sensor model
struct SensorModel 
{

	// type definitions for training and test data
	// constants
	static constexpr size_t inputColumnCount = 13;
	static constexpr size_t sampleColumns    = 31;

	// types
	typedef Eigen::Matrix<float, sampleColumns, 1>    Sample;
	typedef Eigen::Matrix<float, inputColumnCount, 1> Input;
	typedef std::pair<Input, Sample>                  SamplePair;

	std::vector<size_t> outputColumns = []() 
	{
		std::vector<size_t> outputColumns = {7, 8, 9, 10,};

		for(size_t i = 12; i < 31; i++) 
			outputColumns.push_back(i);

		return outputColumns;
	}();

	// loaded training or test samples
	std::vector<SamplePair> samples;

	NeuralNetwork neuralNetwork;

	Input 
	toInput(const std::deque<Touch> &touchHistory) 
	{
		size_t di = 10;
		Input input = neuralNetwork.inputCenter();
		if(touchHistory.size() > di * 2 + 1) 
		{
			input[0] = (touchHistory.end() - 1 - di)->point.x();
			input[1] = (touchHistory.end() - 1 - di)->point.y();
			input[2] = (touchHistory.end() - 1 - di)->point.z();

			input[3] = (touchHistory.end() - 1 - di)->force.x();
			input[4] = (touchHistory.end() - 1 - di)->force.y();
			input[5] = (touchHistory.end() - 1 - di)->force.z();

			input[6] = (touchHistory.end() - 1 - di - di)->force.x();
			input[7] = (touchHistory.end() - 1 - di - di)->force.y();
			input[8] = (touchHistory.end() - 1 - di - di)->force.z();

			input[9] = (touchHistory.end() - 1)->force.x();
			input[10] = (touchHistory.end() - 1)->force.y();
			input[11] = (touchHistory.end() - 1)->force.z();
		}
		return input;
	}

	SensorModel() 
	{ 
		neuralNetwork.load(ros::package::getPath("sim_biotac") + "/config/model.yaml"); 
	}

	Profiler inferenceProfiler{"inference"};

	// predicts sensor outputs from force and position inputs
	Sample 
	predict(const Input &input) 
	{
		ProfilerScope profilerScope(inferenceProfiler);
		Sample ret = Sample::Zero();
		Eigen::VectorXf out;

		neuralNetwork.run(input, out);

		for(size_t i = 0; i < outputColumns.size(); i++) 
			ret[outputColumns[i]] = out[i];

		return ret;
	}

	// tests the sensor model on the loaded data set
	std::vector<Eigen::VectorXf> 
	test() 
	{
		ROS_INFO("test");
		std::vector<Eigen::VectorXf> results;
		for(auto &s : samples) 
		{
			Eigen::VectorXf xy(s.first.size() + s.second.size() * 2);
			xy << s.first, s.second, predict(s.first);
			results.push_back(xy);
		}
		ROS_INFO("%i", (int)results.size());
		return results;
	}

	std::vector<Eigen::VectorXf> 
	test3() 
	{
		ROS_INFO("test");
		Sample averages = Sample::Zero();

		for(auto &s : samples)
			averages += s.second;

		averages *= 1.0 / samples.size();
		std::vector<Eigen::VectorXf> results;
		double testError = 0.0;
		double refError = 0.0;

		for(size_t i = 0; i < samples.size(); i++) 
		{
			if(i * 100 / samples.size() != (i - 1) * 100 / samples.size()) 
				ROS_INFO("%i %%", (int)(i * 100 / samples.size()));

			auto prediction = predict(samples[i].first);

			Eigen::VectorXf xy(samples[i].first.size() + samples[i].second.size() * 2);

			xy << samples[i].first, samples[i].second, prediction;

			results.push_back(xy);

			for(size_t k = 4; k < outputColumns.size(); k++) 
			{
				size_t j = outputColumns[k];
				testError += std::fabs(prediction[j] - samples[i].second[j]) / neuralNetwork.outputScale()[k];
				refError += 1;
			}

			std::cout << testError / refError << std::endl << std::endl;
		}

		ROS_INFO("test results");
		std::cout << testError << std::endl << std::endl;
		std::cout << refError << std::endl << std::endl;
		std::cout << testError / refError << std::endl << std::endl;

		return results;
	}

	// loads a data set from training and testing
	void loadTrainingData(std::istream &f) 
	{
		ROS_INFO("loading training data");
		samples.clear();
		int iline = 0;
		std::vector<Sample> dd;
		loadcsv(f, dd);
		ROS_INFO("%i", (int)dd.size());
		size_t di = 10;
		for(size_t i = di + 1; i < dd.size() - di - 2; i++) 
		{
			Input input;
			input[0] = dd[i][1];
			input[1] = dd[i][2];
			input[2] = dd[i][3];
			input[3] = dd[i][4];
			input[4] = dd[i][5];
			input[5] = dd[i][6];
			input[6] = dd[i + di][4];
			input[7] = dd[i + di][5];
			input[8] = dd[i + di][6];
			input[9] = dd[i - di][4];
			input[10] = dd[i - di][5];
			input[11] = dd[i - di][6];
			input[12] = dd[i][11];
			samples.emplace_back(input, dd[i]);
		}
		ROS_INFO("%i", (int)samples.size());
		ROS_INFO("ready");
	}
};




}