# syntax=docker/dockerfile:1.7-labs

# Based on released Mote noetic driver container
FROM ghcr.io/empriselab/mote-ros-noetic:latest

# Copy package.xml's first so rosdep can install ROS deps as a cached layer
COPY --parents ./**/package.xml ./
RUN apt-get update && \
    rosdep update --rosdistro noetic  --include-eol-distros && \
    rosdep install --from-paths /catkin_ws/src --ignore-src -r -y --include-eol-distros


# for installing matplotlib and numpy and scipy in hw3 and hw4
RUN apt-get update && apt-get install -y \
    python3-pip \
    python-is-python3 \
    wget \
    git \
    zip \
    unzip

# Automatically source ROS for every new bash session
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "[ -f /catkin_ws/src/mote_curriculum/.env ] && source /catkin_ws/src/mote_curriculum/.env" >> /root/.bashrc

# Install rosdeps
RUN echo "alias mote_install='(cd /catkin_ws && apt update && rosdep update && rosdep install --from-paths src --ignore-src -y)'" >> /root/.bashrc
# Build workspace
RUN echo "alias mote_build='(cd /catkin_ws && catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1)'" >> /root/.bashrc
# Run sample node
RUN echo "alias mote_run='roslaunch mote_demos simple_teleop_viz.launch'" >> /root/.bashrc
# Configure mote's IP
RUN echo 'mote_connect() { echo "export ROBOT_IP=$1" > /catkin_ws/src/mote_curriculum/.env; source /root/.bashrc; }' >> /root/.bashrc

# Build nodes
WORKDIR /catkin_ws
COPY ./src/mote_curriculum/ src/mote_curriculum/
RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make"

ENTRYPOINT ["bash", "-c"]
CMD ["source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roscore"]
