FROM ros:indigo-ros-base

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get -y update
RUN apt-get -y install build-essential

# setup catkin
RUN source /opt/ros/indigo/setup.bash
RUN mkdir -p /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/indigo/setup.bash; catkin_init_workspace /root/catkin_ws/src'
RUN /bin/bash -c '. /opt/ros/indigo/setup.bash; cd /root/catkin_ws; catkin_make'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> /root/.bashrc

# setup rospy_message_converter
RUN git clone https://github.com/baalexander/rospy_message_converter /root/catkin_ws/src/rospy_message_converter
WORKDIR /root/catkin_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro indigo -y
RUN /bin/bash -c ". /opt/ros/indigo/setup.bash; cd /root/catkin_ws; catkin_make"
WORKDIR /

# setup tornado
RUN sudo apt-get -y install python-pip
RUN pip install tornado

# setup roscloud_client
RUN git clone https://github.com/roscloud/roscloud_client /root/catkin_ws/src/roscloud_client
COPY ./roscloud_container.launch /root/catkin_ws/src/roscloud_client/launch/

# setup entrypoint
COPY ./roscloud_entrypoint.sh /
RUN chmod +x /roscloud_entrypoint.sh

ENTRYPOINT ["/roscloud_entrypoint.sh"]
CMD ["bash"]
