FROM ubuntu:22.04
WORKDIR /pure_prusuit
COPY  . .
RUN apt update && apt install -y g++
RUN g++ src/main.cpp src/ppcontroller.cpp
CMD [ "tail","-f","/dev/null" ]