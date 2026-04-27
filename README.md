
## 🎥 Multimedia material

![Real-world Experiment (4 UAVs)](output.gif)

(speed >>4x)

- https://mrs.fel.cvut.cz/rbl

## 🚀 Prerequisites

  Ensure you have the following installed:

  - ** MRS System **  
    Follow setup instructions here for ROS1 (master branch): [MRS Apptainer GitHub](https://github.com/ctu-mrs/mrs_apptainer/tree/master)

  ---

## 🛠 Prepare the Workspace


  ```bash
  cd ~/git/mrs_apptainer/user_ros_workspace/src
  git clone git@github.com:ctu-mrs/rbl.git
  git clone git@github.com:ctu-mrs/laserscan_clusters.git
  git clone git@github.com:manuelboldrer/uvdar_multirobot_simulator.git
  cd ../../
  ./example_wrapper
  cd user_ros_workspace
  catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

 ## ▶️ run simulation 

```bash  
cd rbl_controller/tmux/mrs_9 
./start.sh
```
to activate the algorithm go to the pane "activation" and run the last command in history in synchronized mode.
try out mrs_9, mrs_16, mrs_9_circle.
This code was used for some of the results reported in the following papers: 

- [Rule-Based Lloyd Algorithm for Multi-Robot Motion Planning and Control with Safety and Convergence Guarantees](https://arxiv.org/pdf/2310.19511)
- [Distributed Lloyd-based algorithm for uncertainty-aware multi-robot under-canopy flocking](https://arxiv.org/pdf/2504.18840)
