# `biotac_sim_plugin`

A metapackage for simulating the tactile data from BioTac sensors mounted on a Shadow Dexterous Hand.

## Quick Start

To get started, simply clone the necessary [dependency](##Dependencies) and

1. Clone this repository using the command below
	```bash
	git clone git@github.com:vmstavens/biotac_sim_plugin.git
	```
2. Source your catkin workspace using
	```bash
	source <catkin-workspace>/devel/setup.bash
	```
3. Build the package using
	```bash
	catkin build biotac_sim_plugin
	```
4. Run the provided demo
	```bash
	roslaunch biotac_sim_plugin biotac_sim_demo.launch
	```
## Packages

The project consists of two packages: [`biotac_sim_lib`](###`biotac_sim_lib`) and [`biotac_sim_demo`](###`biotac_sim_demo`).

### [`biotac_sim_lib`](biotac_sim_lib/)
The `biotac_sim_lib` package is responsible for generating the Gazebo model plugin for simulating the BioTac tactile sensor data. The package include the following header files
1. [`biotac_sim_lib.hpp`](biotac_sim_lib/include/biotac_sim_lib/biotac_sim_lib.hpp) which is the header file containing the overall plugin structure.
2. [`neural_network.hpp`](biotac_sim_lib/include/biotac_sim_lib/neural_network.hpp) which contains the `NeuralNetwork` class which is responsible for the deep learning part including loading and running the model.
3. [`helpers.hpp`](biotac_sim_lib/include/biotac_sim_lib/helpers.hpp) which contains the necessary `template functions`, `classes` and `structs` used in `neural_network.hpp` and `biotac_sim_lib.hpp`.

Furthermore, the package contains [`config/model.yaml`](biotac_sim_lib/config/model.yaml) which is the model architecture and trained parameters, [`scripts/set_default_model`](biotac_sim_lib/scripts/set_default_model) which sets the default model to the one mentioned previously and finally [`src/*`](biotac_sim_lib/src/) which contains the source files for the above mentioned headers. <par>

This package is not meant to be run directly and therefore contains no `launch/example` files.
### [`biotac_sim_demo`]((biotac_sim_demo/))

The `biotac_sim_demo` package is responsible for running a demo of the `biotac_sim_lib`. This can only be done in the Shadow Dexterous docker container environment, which can be installed as instructed [here](https://dexterous-hand.readthedocs.io/en/latest/user_guide/1_4_simulated_hand_gazebo.html). <par>

The demonstration can be run using the following command
```bash
roslaunch biotac_sim_plugin biotac_sim_demo.launch
```
This package contains several folders
1. `examples/` which contains [`biotac_sim_demo.py`](biotac_sim_demo/examples/biotac_sim_demo.py), the example node run from the command above.
2. `launch/` which contains [`biotac_sim_demo.launch`](biotac_sim_demo/launch/biotac_sim_demo.launch), the launch file running the node in `biotac_sim_demo.py`.
3. `models/` which contains a [3D model of a black pen](##Acknowledgments), used as a prop for the demonstration.
4. `robots/` which contains wrapper `launch` files and `xacro` files for loading the plugin from [`biotac_sim_lib`](###`biotac_sim_lib`)
5. `worlds/` which contains two worlds: one with and one without gravity. The world you want can be set in the launch file or run using the following command
	```bash
	roslaunch biotac_sim_plugin biotac_sim_demo.launch gravity:=true
	```
	or `gravity:=false` in case you want gravity disabled.

## Dependencies

Outside of standard ROS and Gazebo packages, the following dependency is needed
1. [`ros_utils`](https://github.com/vmstavens/ros_utils), which is a utilities package for ROS used for `base64` encoding and decoding.

## Acknowledgments

This work is based on the [biotac_gezebo_plugin](https://github.com/TAMS-Group/biotac_gazebo_plugin) by Philipp Ruppel, Yannick Jonetzko, Michael Görner, Norman Hendrich and Jianwei Zhang - Simulation of the SynTouch BioTac Sensor, The 15th International Conference on Intelligent Autonomous Systems, IAS-15 2018, Baden Baden, Germany. <par>

The data set used to train the deep learning model can be found [here](https://tams.informatik.uni-hamburg.de/research/datasets/index.php#biotac_single_contact_response) <par>

The 3D model used for demonstration is part of the dataset: [A. Rasouli, J.K. Tsotsos. "The Effect of Color Space Selection on Detectability and Discriminability of Colored Objects."](https://data.nvision2.eecs.yorku.ca/3DGEMS/)