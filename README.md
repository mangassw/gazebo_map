sudo apt-get install libsdl1.2-dev -y
sudo apt install libsdl-image1.2-dev -y
sudo apt-get install ros-noetic-tf2*
sudo apt install ros-noetic-move-base-msgs
sudo apt install ros-noetic-geographic-msgs
sudo apt install ros-noetic-sbpl 
sudo apt install ros-noetic-costmap-converter
sudo apt-get install libsuitesparse-dev 
sudo apt-get install ros-noetic-libg2o

sudo apt-get install ros-noetic-geographic-*
sudo apt-get install geographiclib-*
sudo apt-get install libgeographic-*
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.10/Modules/

gtsam
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  
sudo apt install libgtsam-dev libgtsam-unstable-dev
