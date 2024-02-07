# Epistemic Prediction and Planning with Implicit Coordination for Multi-Robot Teams in Communication Restricted Environments
This code is a small example implementation of the code from the paper "Epistemic Prediction and Planning with Implicit Coordination for Multi-Robot Teams in Communication Restricted Environments" published in ICRA 2023

#### Running the code (requires MATLAB - previously tested on 2023a)
Clone the repository
```
git clone https://github.com/laurenbramblett/epistemic-planning-mrs-icra23
```
Open the file `main.m` and press run.

Variables can be changed in the `main.m` script on lines `4-34`. Additional variables including the partially-known map, agent parameters and dynamics, and the map can be manipulated in the `utils/InitializeSimEnvironment.m` script

#### Example Video
![](https://github.com/laurenbramblett/epistemic-planning-mrs-icra23/blob/main/Example_020724-093813.gif)


#### Usage
If using this repository, please cite our paper:
```
@INPROCEEDINGS{bramblett-epi-icra23,
  author={Bramblett, Lauren and Gao, Shijie and Bezzo, Nicola},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Epistemic Prediction and Planning with Implicit Coordination for Multi-Robot Teams in Communication Restricted Environments}, 
  year={2023},
  volume={},
  number={},
  pages={5744-5750}
}
```
