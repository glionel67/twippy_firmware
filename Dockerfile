# Official Ubuntu 16.04 image
FROM ubuntu:16.04

MAINTAINER Lionel <lionel.geneve@gmail.com>

RUN apt-get update

RUN dpkg --add-architecture i386
RUN apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y \
	software-properties-common \
	curl \
	cmake \
	build-essential \
	git \
	libncurses5 \
	zip unzip \
	python python-pip python-wheel python-dev python-setuptools \
	python3 python3-pip python3-wheel python3-dev python3-setuptools

#RUN apt-get clean

# Install cross-compilation toolchain
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa
RUN apt-get update && apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi


#RUN curl -fsSLO https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q3-update/+download/gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2 \
#    && tar --strip-components=1 -xvjf gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2 -C /usr/local/

RUN mkdir -p /home/twippy/Workspace/bin && chmod 777 /home/twippy/Workspace/

# Set the working directory to Workspace
#WORKDIR /home/twippy/Workspace

# Copy current directory into the container at Workspace
#COPY . /home/twippy/Workspace

# Make port 80 available to the world outside this container
#EXPOSE 80

# Define environment variable
#ENV NAME World
