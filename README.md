# TIAGo ++ tutorials (Noetic)

This package contains the Noetic Dockerfile and Packages for TIAGo ++

**Important Note: This Dockerfile is meant to be used to explore the functionalities of PAL Robotics TIAGo ++, it is not meant for development on the real robots since more functionalities are available in the dockers provided to customers when purchasing a robot.**

To use this Docker you can either:

- Build it locally with the latest version of all our packages

or

- Pull the already build image from DockerHub

## Building your docker locally

```
cd tiago_dual_tutorials

docker build --no-cache -t tiago_dual_tutorials_docker .
```
## Pulling Image

The image can be pulled from [DockeHub](https://hub.docker.com/r/palroboticssl/tiago_dual_tutorials) : 

```
docker pull palroboticssl/tiago_dual_tutorials:noetic
```
or
```
docker pull palroboticssl/tiago_dual_tutorials:melodic
```

## Tutorials

* TIAGo ++: [http://wiki.ros.org/action/info/Robots/TIAGo++/Tutorials](http://wiki.ros.org/action/info/Robots/TIAGo%2B%2B/Tutorials)

## Git Repo & Bugtracker

If issues are encountered during those tutorials please open an issue on the appropriate repository.

* **Repository**: [https://github.com/pal-robotics/tiago_dual_tutorials](https://github.com/pal-robotics/tiago_dual_tutorials)
* **Bugtracker**: [https://github.com/pal-robotics/tiago_dual_tutorials/issues](https://github.com/pal-robotics/tiago_dual_tutorials/issues)