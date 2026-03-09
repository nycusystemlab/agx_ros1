# syntax=docker/dockerfile:1.6

ARG BASE_PC=ros:noetic-ros-base

FROM ${BASE_PC} AS final

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-suite \
    ros-noetic-velodyne-msgs \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

EXPOSE 9090

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch port:=9090"]
