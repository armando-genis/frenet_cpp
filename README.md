# frenet_cpp
 frenet frame path planning implementation for vehicle models

## Installation

 ```bash
sudo apt update
sudo apt install g++
sudo apt install libeigen3-dev
sudo apt install python3-matplotlib python3-dev
wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h
```

## Run Locally

```bash
g++ -I/usr/include/eigen3 -I/usr/include/python3.10 BicycleModel.cpp Animation.cpp CubicSpline1D.cpp -o frenet_animation -lpython3.10
./frenet_animation
```


