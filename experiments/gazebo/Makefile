# Build the Tier-2 recorder (gz-transport13 + gz-msgs10, no ROS).
# GZ_PREFIX: root of the Gazebo vendor packages (default: a ROS Jazzy install;
# on the lab machine: $(HOME)/opt/gzroot/opt/ros/jazzy, see README.md).
GZ_PREFIX ?= /opt/ros/jazzy
SYS_PREFIX ?= /opt/ros/jazzy
CXX ?= g++
INC := -I$(GZ_PREFIX)/opt/gz_transport_vendor/include/gz/transport13 \
       -I$(GZ_PREFIX)/opt/gz_msgs_vendor/include/gz/msgs10 \
       -I$(SYS_PREFIX)/opt/gz_utils_vendor/include/gz/utils2 \
       -I$(SYS_PREFIX)/opt/gz_math_vendor/include/gz/math7
LIB := -L$(GZ_PREFIX)/opt/gz_transport_vendor/lib -L$(GZ_PREFIX)/opt/gz_msgs_vendor/lib \
       -L$(SYS_PREFIX)/opt/gz_utils_vendor/lib -L$(SYS_PREFIX)/opt/gz_math_vendor/lib -lgz-transport13 -lgz-msgs10 -lgz-utils2 -lprotobuf -pthread

build/gz_recorder: gz_recorder.cc
	mkdir -p build
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wpedantic $(INC) $< -o $@ $(LIB)

clean:
	rm -rf build
