FROM ubuntu:16.04

RUN apt-get update
RUN apt-get install -y cmake build-essential git
RUN apt-get clean

# Install cross-compilation toolchain
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa
RUN apt-get update
RUN apt-get install gcc-arm-embedded


