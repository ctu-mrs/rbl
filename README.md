  ## 🚀 Prerequisites

  Ensure you have the following installed:

  - ** MRS System **  
    Follow setup instructions here: [MRS Apptainer GitHub](https://github.com/ctu-mrs/mrs_apptainer)

  ---

  ## 🛠 Prepare the Workspace


  ```bash
  cd ~/git/mrs_apptainer/user_ros_workspace/src
  git clone git@github.com:ctu-mrs/rbl.git
  git clone git@github.com:ctu-mrs/laserscan_clusters.git
  cd ../../
  ./example_wrapper
  cd user_ros_workspace
  catkin build 
  ```
 ## ▶️ run simulation 

```bash  
cd rbl_controller/tmux/mrs_9_uvdar
./start.sh
```
to activate the algorithm go to the pane "activation" and run the last command in history in synchronized mode.

This code was used for some of the results reported in the following papers: 

- [Rule-Based Lloyd Algorithm for Multi-Robot Motion Planning and Control with Safety and Convergence Guarantees](https://arxiv.org/pdf/2310.19511)
- [Distributed Lloyd-based algorithm for uncertainty-aware multi-robot under-canopy flocking](https://arxiv.org/pdf/2504.18840)
