# Official Ubuntu 16.04 image
FROM ubuntu:16.04

MAINTAINER Lionel <lionel.geneve@gmail.com>

RUN apt-get update

RUN apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y \
	software-properties-common \
	cmake \
	build-essential \
	git \
	openocd

#RUN apt-get clean

# Install cross-compilation toolchain
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa
RUN apt-get update && apt-get install -y gcc-arm-none-eabi

RUN mkdir -p /home/twippy/Workspace

# Set the working directory to Workspace
#WORKDIR /home/twippy/Workspace

# Copy current directory into the container at Workspace
#COPY . /home/twippy/Workspace

# Make port 80 available to the world outside this container
#EXPOSE 80

# Define environment variable
#ENV NAME World
