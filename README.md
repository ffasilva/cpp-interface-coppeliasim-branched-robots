# C++ interface for branched robots in CoppeliaSim

Basic C++ interface for dynamic simulations of branched robots in CoppeliaSim using DQ Robotics.

## Required libraries

### Install [DQ Robotics](https://github.com/dqrobotics/cpp) for C++ 

Skip these steps if you already have DQ Robotics installed.

#### MacOS (Apple Silicon)

```shell
brew install eigen
```

```shell
git clone https://github.com/dqrobotics/cpp.git
cd cpp
mkdir build && cd build
cmake ..
make -j16
sudo make install
```

#### Ubuntu 

```shell
sudo add-apt-repository ppa:dqrobotics-dev/development -y
sudo apt-get update
sudo apt-get install libdqrobotics
```
### Install [DQ Dynamics](https://github.com/ffasilva/dynamic-modular-composition-cpp.git) for C++

Skip these steps if you already have DQ Dynamics installed.

#### UNIX

```shell
git clone https://github.com/ffasilva/dynamic-modular-composition-cpp.git
cd dynamic-modular-composition-cpp
mkdir build && cd build
cmake ..
make -j16
sudo make install
```

## Build and Install (UNIX)

```shell
git clone https://github.com/ffasilva/cpp-interface-coppeliasim-branched-robots.git
cd cpp-interface-coppeliasim-branched-robots
mkdir build && cd build
cmake ..
make -j16
sudo make install
```

### To Uninstall 

Go to the build folder, and run:

```shell
sudo xargs rm < install_manifest.txt
```
