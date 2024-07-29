# eventail_simulation

This repository serves as the numerical stability testing ground for the eventail solver, presented in our [CVPR 2024 (Oral)](https://mgaoling.github.io/eventail/) and [ICCV 2023 (Oral)](https://mgaoling.github.io/eventail_iccv23/) papers. Please acknowledge the following publications when using our code or referring to this project in your academic work:

```bibtex
@inproceedings{gao2024eventail,
  title        = {An N-Point Linear Solver for Line and Motion Estimation with Event Cameras},
  author       = {Gao, Ling and Gehrig, Daniel and Su, Hang and and Scaramuzza, Davide and Kneip, Laurent},
  booktitle    = {2024 IEEE/CVF Computer Vision and Pattern Recognition (CVPR)},
  year         = {2024},
  organization = {IEEE}
}

@inproceedings{gao2023eventail,
  title        = {A 5-Point Minimal Solver for Event Camera Relative Motion Estimation},
  author       = {Gao, Ling and Su, Hang and Gehrig, Daniel and Cannici, Marco and Scaramuzza, Davide and Kneip, Laurent},
  booktitle    = {2023 IEEE/CVF International Conference on Computer Vision (ICCV)},
  pages        = {8015--8025},
  year         = {2023},
  organization = {IEEE},
  doi          = {10.1109/ICCV51070.2023.00739}
}
```

## Installation

This guide assumes you are using an Ubuntu system. Ensure you have the following prerequisites installed: 

```
# optional
sudo apt-get install git-lfs libeigen3-dev libgoogle-glog-dev libgflags-dev
```

Once the prerequisites are installed, build the project with the following commands:

```
git clone https://github.com/mgaoling/eventail_simulation_open.git
cd ./eventail_simulation_open 
cmake -B build && cmake --build build
```

## Example

We provide a few examples under the `simulation` directory, with their corresponding configurations defined under the `config` directory.

For instance, to run the *runtime_analysis* example, follow these steps:

1. Modify the parameters in the config file located at `config/runtime_analysis.conf`
2. execute `./build/runtime_analysis --flagfile=config/runtime_analysis.conf`

Depending on the iteration number you choose, this may take a while. Results will be available both in the terminal and under the `log` directory.

## Implementation of both Solvers

As a quick reference, the implementation of the 5-point minimal solver can be found [here](https://github.com/mgaoling/eventail_simulation_open/blob/main/src/polyjam_solver.cpp#L77), and the N-point linear solver is available [here](https://github.com/mgaoling/eventail_simulation_open/blob/main/src/linear_solver.cpp#L62).

## Acknowledgments

This research has been supported by projects 22DZ1201900 and 22ZR1441300 funded by the Natural Science Foundation of Shanghai as well as project 62250610225 by the National Science Foundation of China (NSFC). This work was also supported by the European Research Council (ERC) under grant agreement No. 864042 (AGILEFLIGHT).

