# CS791-AMMProject
CS791-Autonomous Mobile Manipulation Term Project


## Installation and Running Instructions
### Clone
Clone the repo and submodules in the src folder of the catkin workspace:
```bash
cd $HOME/autonomous_mobile_manipulation_ws/src
git clone --recurse-submodules git@github.com:tolgakarakurt/CS791-AMMProject.git
```

**If you have cloned the repository before, simply update the submodules:**
```bash
git submodule update --init --recursive
```

### Build
```bash
cd $HOME/autonomous_mobile_manipulation_ws
catkin build cs791_amm_project
```
### Run
Terminal 1
```bash
roslaunch mm_coverage_planner stack.launch
```

Terminal 2 (After the simulator has launched)
```bash
roslaunch mm_coverage_planner planner.launch
```