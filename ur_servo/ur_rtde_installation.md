## Source build in Linux (Ubuntu)
[Install Link](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html)
``` sh
sudo apt-get install libboost-all-dev

git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git submodule update --init --recursive
mkdir build
cd build
# cmake .. # for python3
cmake -DPYBIND11_PYTHON_VERSION=2.7 .. # for python2 ROS
make
sudo make install
```