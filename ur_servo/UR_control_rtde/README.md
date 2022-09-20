# UR_Control_node

### Functions
1. Digital Input and Output interface 
2. Joint and TCP Control
3. Digital Output 0 and 1 for grasp control

### Before to use
before run this script, please:

1. use python3 and install pip3: `sudo apt install python3-pip`

2. upgrade pip3: `pip3 install --upgrade pip`

3. install ur_rtde: `pip3 install ur_rtde`

4. if using ursim simulation, run: `sudo bash ./start-ursim.sh` 

### How to use
1. finish steps in `Before to use`, test python env. is ok with pack `import rtde_io`
2. configure the IP address stuff in UR and PC
3. Modify the IP address inside python scripts
4. run it


### Problems
* **ur_rtde package installation may be quite tricky** 

If you want to use it in python3, just run `pip3 install ur_rtde` can be fine. However, if willing to implemented in python2 (e.g. ROS 1), `pip install ur_rtde` may go wrong because dependency of pybind11. A robust way is to complie and build install the source on your own following [rtde installation guide](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html). **NOTE: DO REMEMBER** to set the git option (`git submodule update --init --recursive`) and cmake command (`cmake -DPYBIND11_PYTHON_VERSION=2.x ..`) as memetioned in [rtde installation guide](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html).

* **URsim and UR IP**

URsim installation and setup can be disaster. Not because of the steps but the comfusing errors poped up. Here are some key steps for ease your ubuntu:
1. download URsim package in UR official website in linux version 
2. if ubuntu 18, install JAVA8 with apt: `sudo apt install openjdk-8-jre-headless`
3. switch JAVA version to JAVA 8: `sudo update-alternatives --config java`
4. REMOVE the `libcurl3` word inside `./install.sh` in the ursim file
5. sudo run: `sudo bash ./install.sh`
6. cd ursim and change permission of all ursim files: `sudo chmod -R 777 ./ursim-xxx`
7. run (do not use sudo) the start shell: `./start-ursim.sh`
