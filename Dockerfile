FROM cyberbotics/webots.cloud:R2023a-ubuntu20.04
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH
#RUN cd $PROJECT_PATH/controllers/judferee && make clean && make
#RUN cd $PROJECT_PATH/controllers/omni_mobile && make clean && make
