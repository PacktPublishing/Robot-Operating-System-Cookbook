FROM osrf/ros:indigo-desktop-full

# using bash instead of sh to be able to source
ENV TERM xterm
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y python-catkin-tools ros-indigo-moveit wget && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get remove -y gazebo2 && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gazebo7 ros-indigo-gazebo7-ros-pkgs ros-indigo-gazebo7-ros-control ros-indigo-controller-manager ros-indigo-ros-controllers python-pip && \
    mkdir -p /workspace/src && \
    cd /workspace/ && \
    source /opt/ros/indigo/setup.bash && \
    catkin init

COPY . /workspace/src/

RUN source /opt/ros/indigo/setup.bash && \
    cd /workspace/src && \
    git clone -b indigo-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git && \
    git clone https://github.com/shadow-robot/pysdf.git && \
    git clone -b F_add_moveit_funtionallity https://github.com/shadow-robot/gazebo2rviz.git && \
    git clone -b F_gazebo_7_docker https://github.com/shadow-robot/universal_robot.git && \
    git clone -b F#182_partial_trajectory_mod  https://github.com/shadow-robot/ros_controllers.git && \
    cd /workspace/src && \
    wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo7/00-gazebo7.list -O /etc/ros/rosdep/sources.list.d/00-gazebo7.list && \
    rosdep update && \
    rosdep install --default-yes --all --ignore-src && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# installing gzweb
RUN curl -sL https://deb.nodesource.com/setup | bash - && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y libjansson-dev nodejs libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential xvfb

RUN /workspace/src/setup_gzweb.sh

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y byobu nano

RUN cd /root && \
    git clone git://github.com/c9/core.git c9sdk && \
    cd c9sdk && \
    scripts/install-sdk.sh && \
    sed -i -e 's_127.0.0.1_0.0.0.0_g' /root/c9sdk/configs/standalone.js
    

RUN apt-get remove -y python-pip && \
    wget https://bootstrap.pypa.io/get-pip.py && \
    python get-pip.py && \
    pip2 install --upgrade packaging jupyter && \
    pip2 install --upgrade jupyter_contrib_nbextensions && \
    jupyter contrib nbextension install --system --symlink && \
    mkdir -p /root/.jupyter && \
    jupyter nbextension enable toc2/main

COPY jupyter_notebook_config.py /root/.jupyter/

RUN apt-get remove -y python-pip && \
    wget https://bootstrap.pypa.io/get-pip.py && \
    python get-pip.py && \
    pip2 install --upgrade packaging jupyter

RUN pip2 install --upgrade tensorflow keras h5py sklearn bokeh bayesian-optimization pandas

# cleanup
RUN rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

EXPOSE 8080 8888 7681
