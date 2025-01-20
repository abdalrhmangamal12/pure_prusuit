FROM gcc
WORKDIR /pure_prusuit
SHELL [ "/bin/bash","-c" ]
COPY  . .
RUN curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | bash && \
    apt update && \
    apt install -y libgtest-dev python3-colcon-common-extensions

RUN colcon build 
#CMD [ "tail","-f","/dev/null" ]
ENTRYPOINT [ "./mycontroller.sh" ]