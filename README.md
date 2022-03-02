# Decentralized multi-agent control using GNNs for ICRA 2022
Repository containing the code base for evaluating the multi-agent coordination policy used in the paper "A Framework for Real-World Multi-Robot Systems Running Decentralized GNN-Based Policies".

## Setup
1) [Install ROS2](https://docs.ros.org/en/foxy/Installation.html)
2) Clone with `git clone --recursive` to include submodules
3) Source the system ROS2 installation (typically `. /opt/ros2/galactic/setup.sh`)
4) Build from the root of this repository using `colcon build --symlink-install`. This command has to be executed twice (otherwise the RVO library is not built and included properly), this is a known issue.

## Training the policy
We provide pre-trained models, but if re-training the GNN is desired, this can be done as described [here](https://github.com/proroklab/rl_multi_agent_passage). The path to the model has to be specified in the launch file (`model_path` parameter in the `passage_gnn_simple` executables).
## Evaluation in simulation
1) After the build was successful, source the workspace from the root in two separate terminal windows, using `. install/setup.bash`
2) Start the simulation in the first window by running `ros2 launch launch/simulation_robomasters.launch.py`
3) Start the policy in the second window. There are three options:
   1) RVO only: Uses centralized RVO to navigate robots to their goals, can be used as a baseline. Run `ros2 launch launch/centralized_rvo_passage_robomaster.launch.yaml`
   2) Centralized GNN: Uses centralized RVO for resetting robots to initial positions and centralized GNN policy evaluation to navigate robots to their goal positions. Run `ros2 launch src/passage_gnn_simple/launch/centralized_passage_robomaster.launch.yaml`.
   3) Decentralized GNN: Uses centralized RVO for resetting robots to initial positions and decentralized GNN policy evaluation to navigate robots to their goal positions. Run `ros2 launch src/passage_gnn_simple/launch/decentralized_passage_robomaster.launch.py`.

## Evaluation on real robots
This will be updated soon.

## Citation
If you use any part of this code in your research, please cite our paper:

```
@inproceedings{blumenkamp2022decentralizedgnn,
  title={A Framework for Real-World Multi-Robot Systems Running Decentralized GNN-Based Policies},
  author={Blumenkamp, Jan and Morad, Steven and Gielis, Jennifer and Li, Qingbiao and Prorok, Amanda},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2022},
  organization={IEEE}
}
```