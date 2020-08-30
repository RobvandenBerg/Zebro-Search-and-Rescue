# Filename: Dockerfile 
FROM ubuntu:16.04
WORKDIR /usr/src/app
COPY ./ ./

RUN apt-get update \
 && apt-get install -y sudo

RUN adduser --disabled-password --gecos '' docker
RUN adduser docker sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker
RUN sudo su

RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository ppa:beineri/opt-qt594-xenial
RUN sudo apt update

#RUN sudo apt-get install -y gcc g++ cmake freeglut3-dev qt59-meta-full libxi-dev libxmu-dev libfreeimage-dev libfreeimageplus-dev liblua5.3-dev lua5.3
RUN sudo apt-get install -y cmake libfreeimage-dev libfreeimageplus-dev qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev lua5.3 doxygen graphviz graphviz-dev asciidoc
RUN sudo touch /etc/xdg/qtchooser/default.conf
RUN sudo printf '%s\n' '/opt/qt59/bin' '/opt/qt59/lib' >/etc/xdg/qtchooser/default.conf
RUN sudo echo 'export LD_LIBRARY_PATH=/opt/qt59/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
RUN sudo source ~/.bashrc
RUN sudo dpkg -i argos3_simulator-3.0.0-x86_64-beta56.deb
RUN cd Simulation
RUN sudo mkdir build
RUN cd build
RUN sudo cmake ..
RUN sudo make
RUN cd ../