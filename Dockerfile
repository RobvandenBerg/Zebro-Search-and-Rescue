# Filename: Dockerfile 
FROM ubuntu:16.04
WORKDIR /usr/src/app
COPY ../argos3_simulator-3.0.0-x86_64-beta56.deb ./
COPY ./* ./

RUN apt-get update \
 && apt-get install -y sudo

RUN adduser --disabled-password --gecos '' docker
RUN adduser docker sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker
RUN sudo apt-get install -y gcc g++ cmake freeglut3-dev qt5-default libxi-dev libxmu-dev libfreeimage-dev libfreeimageplus-dev liblua5.3-dev lua5.3
RUN sudo dpkg -i argos3_simulator-3.0.0-x86_64-beta56.deb
