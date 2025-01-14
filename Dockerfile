FROM wisevision/ros_with_wisevision_msgs_and_wisevision_core:humble

WORKDIR /root/wisevision_data_black_box_ws

COPY . /root/wisevision_data_black_box_ws/src/wisevision_data_black_box
RUN test -f /root/wisevision_data_black_box_ws/config.json || \
    cp /root/wisevision_data_black_box_ws/src/wisevision_data_black_box/config.json /root/wisevision_data_black_box_ws/config.json

RUN apt-get update && \
    apt-get install -y libcurl4-openssl-dev libjsoncpp-dev python3-rosdep && \
    sudo rosdep fix-permissions && \
    rosdep update --include-eol-distros --rosdistro humble && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

SHELL ["/bin/bash", "-c"]

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
source /root/wisevision_ws/install/setup.bash && \
colcon build --symlink-install"

ENTRYPOINT ["/bin/bash", "-c", "source /root/wisevision_data_black_box_ws/install/setup.bash && ros2 run wisevision_data_black_box black_box"]
RUN echo 'source /root/wisevision_data_black_box_ws/install/setup.bash && ros2 run wisevision_data_black_box black_box' >> ~/.bashrc