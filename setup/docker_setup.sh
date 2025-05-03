#!/bin/bash

# build docker folder and docker image
cd ../../
mkdir docker
cp -r src/localization/setup/Dockerfile docker/
docker build -t localization_ws -f docker/Dockerfile .

# run docker container
docker run --rm --name localization_ws -p 192.168.81.81:11311:11311 -it -v $PWD:/localization_ws localization_ws:latest