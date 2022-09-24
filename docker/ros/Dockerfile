FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1

#
# fix broken package.xml in test_pluginlib that crops up if/when rosdep is run again
#
#   Error(s) in package '/opt/ros/foxy/build/pluginlib/prefix/share/test_pluginlib/package.xml':
#   Package 'test_pluginlib' must declare at least one maintainer
#   The package node must contain at least one "license" tag
#
#RUN TEST_PLUGINLIB_PACKAGE="${ROS_ROOT}/build/pluginlib/prefix/share/test_pluginlib/package.xml" && \
#    sed -i '/<\/description>/a <license>BSD<\/license>' $TEST_PLUGINLIB_PACKAGE && \
#    sed -i '/<\/description>/a <maintainer email="michael@openrobotics.org">Michael Carroll<\/maintainer>' $TEST_PLUGINLIB_PACKAGE && \
#    cat $TEST_PLUGINLIB_PACKAGE
    
    
#
# Set the default DDS middleware to cyclonedds
# https://github.com/ros2/rclcpp/issues/1335
#
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Custom
RUN apt update && apt install -y \ 
	build-essential \
	gcc \ 
	make \
	cmake \
	cmake-gui \
	cmake-curses-gui \
	libssl-dev \
	doxygen \
	graphviz
RUN apt install git
WORKDIR /
RUN git clone https://github.com/eclipse/paho.mqtt.c.git && \
	cd paho.mqtt.c && \
	make

RUN cd paho.mqtt.c && \
	git checkout v1.3.8 && \
	cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
    -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON && \
	cmake --build build/ --target install 
RUN ldconfig

RUN git clone https://github.com/eclipse/paho.mqtt.cpp && \
	cd paho.mqtt.cpp && \
 	cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
    -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE && \
 	cmake --build build/ --target install
RUN ldconfig

RUN apt install vim -y

RUN apt install -y mosquitto mosquitto-clients
# 
# setup entrypoint
#
COPY ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/install/setup.bash' >> /root/.bashrcc
ENV ws=/workspace/dev_ws
ENTRYPOINT ["/ros_entrypoint.sh"]

COPY ZED_SDK_Tegra_L4T32.7_v3.7.5.run /ZED_SDK_Tegra_L4T32.7_v3.7.5.run

RUN /ZED_SDK_Tegra_L4T32.7_v3.7.5.run -- silent

WORKDIR /

###########################################################################################
# Upgrade Cmake

WORKDIR /tmp
RUN wget https://github.com/Kitware/CMake/releases/download/v3.19.4/cmake-3.19.4.tar.gz
RUN tar xvf cmake-3.19.4.tar.gz
WORKDIR /tmp/cmake-3.19.4/
RUN mkdir $HOME/install
RUN ./configure --prefix=$HOME/install
RUN make -j$(nproc)
RUN make install

############################################################################################
# Install OpenCV

RUN python3 -m pip install setuptools>=41.0.0
RUN python3 -m pip install --upgrade pip setuptools wheel
RUN python3 -m pip install numpy

RUN apt update && apt install -y build-essential

# ARG OPENCV_VERSION='4.5.2'
# ARG MAKEFLAGS='-j4'

# WORKDIR /root

# RUN curl -L https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -o opencv-${OPENCV_VERSION}.zip && \
#     curl -L https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip -o opencv_contrib-${OPENCV_VERSION}.zip && \
#     unzip opencv-${OPENCV_VERSION}.zip && \
#     unzip opencv_contrib-${OPENCV_VERSION}.zip

# RUN  add-apt-repository ppa:ubuntu-toolchain-r/test -y && apt update && apt install g++-8 -y && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-8

# WORKDIR /root/opencv-${OPENCV_VERSION}/build
# ENV LD_LIBRARY_PATH /usr/local/cuda/lib64
# RUN cmake -D WITH_VTK=OFF -D BUILD_opencv_viz=OFF -DWITH_QT=OFF -DWITH_GTK=OFF -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="5.3,6.2,7.2,7.0,10.2" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${OPENCV_VERSION}/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10 -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local/opencv ..

# RUN make ${MAKEFLAGS}

#RUN make

# RUN apt-get update

# RUN make install

# RUN ldconfig

# RUN make clean

# #RUN python3 -m pip install -v git+https://github.com/opencv/opencv-python
# RUN python3 -m pip install opencv-python
# RUN python3 -m pip install imantics

# RUN python3 -m pip install --upgrade pip

# RUN apt install -y python3.8-dev
 
# RUN apt install -y wget zip
# RUN apt-get update \
#     && apt-get install -y --no-install-recommends \
#     python3-pip \
#     python3-setuptools \
#     python3-h5py \
#     && apt -y autoremove \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# RUN sudo apt-get update && apt-get install -y libltdl-dev

WORKDIR /

# RUN git clone https://github.com/jkjung-avt/jetson_nano.git

# WORKDIR /jetson_nano

# # RUN apt-get install nvidia-jetpack

# RUN python3 -m pip install  numpy==1.18.5

# COPY ./install_opencv-3.4.8.sh /
# COPY ./install_protobuf-3.9.2.sh /

# RUN chmod +x ./install_opencv-3.4.8.sh \
# 	&& ./install_opencv-3.4.8.sh

# RUN chmod +x ./install_protobuf-3.9.2.sh \
# 	&& ./install_protobuf-3.9.2.sh
COPY libnvinfer_plugin.so.8.2.1 /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1
RUN ldconfig
RUN python3 -m pip install protobuf

WORKDIR /

# RUN git clone https://github.com/jkjung-avt/tensorrt_demos.git

ADD tensorrt_demos /tensorrt_demos

# COPY custom-yolov4-tiny-detector.cfg /tensorrt_demos/yolo
# COPY custom-yolov4-tiny-detector.weights /tensorrt_demos.yolo

WORKDIR /tensorrt_demos/yolo

# RUN chmod +x ./install_pycuda.sh \
	# && ./install_pycuda.sh

# *** WE NEED TO USE NUMPY VERSION 1.19.4 OR BAD THINGS WILL HAPPEN (Core dump crash)
RUN python3 -m pip install numpy==1.19.4

RUN python3 -m pip install onnx==1.9.0

# WORKDIR /tensorrt_demos/plugins
RUN python3 -m pip install pycuda
# RUN rm Makefile
# COPY Makefile /tensorrt_demos/plugins/Makefile
# RUN make 

# WORKDIR /tensorrt_demos/yolo

# RUN python3 yolo_to_onnx.py -m custom-yolov4-tiny-detector
# RUN  python3 onnx_to_tensorrt.py -m custom-yolov4-tiny-detector

COPY get_python_api.py  /usr/local/zed/get_python_api.py
RUN python3 /usr/local/zed/get_python_api.py
RUN python3 -m pip install numpy==1.19.4

RUN echo 'source /workspace/dev_ws/install/local_setup.bash' >> /root/.bashrc
ENV computer_vision c_v

COPY SN14389.conf /usr/local/zed/settings/
RUN python3 -m pip install wget
RUN apt-get update && apt-get install psmisc -y

RUN echo "alias r2r=\"ros2 run\"" >> /root/.bashrc
RUN echo "export cv=\"computer_vision\"" >> /root/.bashrc
RUN echo "alias cbp=\"colcon build --packages-select\"" >> /root/.bashrc

############################################################################################
# Installing libmodbus
RUN apt-get update && apt-get install -y autoconf autogen libtool
WORKDIR /
RUN git clone https://github.com/stephane/libmodbus.git && \
cd /libmodbus && ./autogen.sh && ./configure && make install
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

RUN ldconfig

CMD ["bash"]

