## 🎥 Multimedia material


- https://mrs.fel.cvut.cz/irbl

## 🚀 Prerequisites

  Ensure you have the following installed:

  - ** MRS System **  
    Follow setup instructions here: [MRS Apptainer GitHub](https://github.com/ctu-mrs/mrs_apptainer)

  ---

## 🛠 Prepare the Workspace


  ```bash
  cd ~/git/mrs_apptainer/user_ros_workspace/src
  git clone --branch irbl git@github.com:ctu-mrs/rbl.git
  cd ../../
  ./example_wrapper
  cd user_ros_workspace
  catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

 ## ▶️ run simulation 

```bash  
cd src/rbl/tmux/forest5
./start.sh
```

wait for takeoff.

to run, go to "goto_end" to set the final goal (run the last command in history in synchronized mode), then to activate the algorithm go to the pane "activation" and run the last command in history in synchronized mode. 

This code was used for some of the results reported in the following papers: 

- [Perception-Aware Communication-Free Multi-UAV Coordination in the Wild]()
