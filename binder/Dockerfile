FROM intel4coro/base-notebook:20.04-noetic-full-xpra

# Set environment
ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
COPY --chown=${NB_USER}:users . pycram/
RUN vcs import --input pycram/binder/pycram-http.rosinstall --recursive

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/binder/requirements.txt --user 

COPY --chown=${NB_USER}:users binder/me ${PYCRAM_WS}/src/me

USER root
RUN apt-get update
RUN apt install -y ros-noetic-pr2-arm-kinematics
RUN apt-get install -y libjson-glib-1.0-0 libjson-glib-dev


# Build pycram workspace
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN catkin build

#============= Install extra software packages =============#
# Install SWI Prolog
# Source: https://www.swi-prolog.org/build/PPA.html
USER root
RUN apt install -y software-properties-common && \
    apt-add-repository -y ppa:swi-prolog/stable
RUN apt update && \
    apt install -y swi-prolog
ENV SWI_HOME_DIR=/usr/lib/swi-prolog

## Knowrob: install SWI prolog
USER root
RUN sudo apt-get install -y software-properties-common -y
#RUN sudo apt-add-repository ppa:swi-prolog/stable -y
RUN sudo apt-get -y update
RUN sudo apt-get install -y swi-prolog -y
#RUN export SWI_HOME_DIR=/usr/lib/swi-prolog
RUN sudo apt-get install -y ros-noetic-roscpp
RUN sudo apt-get install -y libmongoc-1.0
RUN sudo apt-get install -y libmongoc-dev
RUN sudo apt-get install -y libbson-dev
RUN sudo apt-get install -y libbson-1.0-0
RUN sudo apt-get update



# Install MongoDB Community Edition
# Source: https://www.mongodb.com/docs/manual/tutorial/install-mongodb-on-ubuntu/#std-label-install-mdb-community-ubuntu
ARG MONGODEB_VERSION=4.4
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-${MONGODEB_VERSION}.asc | \
    gpg -o /usr/share/keyrings/mongodb-server-${MONGODEB_VERSION}.gpg --dearmor
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-${MONGODEB_VERSION}.gpg ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/${MONGODEB_VERSION} multiverse" | \
    tee /etc/apt/sources.list.d/mongodb-org-${MONGODEB_VERSION}.list
RUN apt update && apt install -y mongodb-org
RUN mkdir -p /data/db && \
    chown -R ${NB_USER}:users /data/db


WORKDIR ${PYCRAM_WS}/src/
RUN git clone --branch master https://github.com/code-iai/iai_common_msgs.git 
RUN git clone --branch noetic https://github.com/knowrob/rosprolog.git
RUN git clone --branch fallschool https://github.com/SUTURO/suturo_knowledge.git
RUN git clone --branch fallschool https://github.com/SUTURO/knowrob.git
RUN git clone --branch fallschool https://github.com/SUTURO/suturo_resources.git
RUN git clone --branch soma-cram https://github.com/hawkina/soma.git
RUN git clone --branch main https://github.com/sasjonge/knowrob_designator.git 
RUN pip install rdflib


RUN pip install service_identity
RUN pip install netifaces
RUN catkin build --verbose --no-status
# Jupyter Config
COPY --chown=${NB_USER}:users binder/jupyter-config.json /opt/conda/share/jupyter/lab/settings/overrides.json
# Create a symbolic link to the ROS workspace under the working directory
RUN ln -s ${ROS_WS} ${PWD}/ROS_WS
WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz
