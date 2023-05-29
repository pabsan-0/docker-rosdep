FROM ros:noetic

WORKDIR /catkin_ws  
RUN mkdir /catkin_ws/src                                       && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'   && \
    echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc        ;

# Devel layer. Just remove it when done!
RUN apt update && apt install -y vim 


ENTRYPOINT ["bash", "-c"]
CMD ["/catkin_ws/src/entrypoint.sh"]
