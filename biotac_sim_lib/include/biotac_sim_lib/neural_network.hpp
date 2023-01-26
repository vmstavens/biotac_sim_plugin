#pragma once

#include <eigen3/Eigen/Dense>
#include <functional>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

// added
#include "ros_utils_cpp/encoding.hpp"
#include "ros_utils_cpp/utils.hpp"

class NeuralNetwork 
{
	// base 64 encoded string for loading weights
	std::string b64;
	
	// layer structure struct
	struct Layer 
	{
		std::string                         name;
		std::vector<std::string>            inputNames;
		std::vector<std::shared_ptr<Layer>> inputLayers;
		std::vector<Eigen::VectorXf>        inputs;
		Eigen::VectorXf                     output;
		std::vector<Eigen::MatrixXf>        weights;
		virtual void run() {}
	};

	std::unordered_map<std::string, std::shared_ptr<Layer>> 
	layerMap;
	
	std::vector<std::shared_ptr<Layer>>
	layerList;
	
	Eigen::VectorXf
	_inputCenter, _inputScale, _outputCenter, _outputScale;

	std::shared_ptr<Layer>
	inputLayer, outputLayer;

	// convert config/model.yaml into a vector
	Eigen::VectorXf yamlToVector(const YAML::Node &yaml); 

	// convert config/model.yaml into a weight vector
	Eigen::VectorXf yamlToWeightVector(const YAML::Node &yaml); 


public:

	// simple get functions
	const Eigen::VectorXf& inputCenter()  { return _inputCenter;  }
	const Eigen::VectorXf& inputScale()   { return _inputScale;   }
	const Eigen::VectorXf& outputCenter() { return _outputCenter; }
	const Eigen::VectorXf& outputScale()  { return _outputScale;  }

	// load in the model architecture and weights from the file filename
	void load(const std::string &filename);

	// run the model with the given input, and produce the corresponding output
	void run(const Eigen::VectorXf &input, Eigen::VectorXf &output); 
};