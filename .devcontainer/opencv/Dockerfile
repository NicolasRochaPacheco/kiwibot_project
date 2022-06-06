# -------------------------------------------------------------------
# Dockerfile for building OpenCV from scratch
# -------------------------------------------------------------------
# based on OpenCV 4.5.5 build from source tutorial

# Start from base image
FROM ubuntu:20.04

# I am not here
ENV DEBIAN_FRONTEND noninteractive

# Update repositories and install prerequisites
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    cmake \
    g++ \
    build-essential \
    wget \
    unzip

# Move to opt directory
WORKDIR /opt/

# Download packages and extract them
RUN wget -O opencv.zip  --no-check-certificate https://github.com/opencv/opencv/archive/4.5.5.zip \
    && unzip opencv.zip \
    && cd /opt/opencv/

# Move into OpenCV build directory
WORKDIR /opt/opencv/

# Configure 
RUN cmake ../opencv-4.5.5

# Build
RUN cmake --build .

# Run bash console
CMD [ "/bin/bash" ]
