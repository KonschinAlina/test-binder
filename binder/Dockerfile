FROM intel4coro/base-notebook:20.04-noetic-vnc
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG HOME=/root

USER root 
# This steps seem to be necessary for now, TODO: check if this is still needed
# ─── STEP A: Remove any pre-existing ROS list files ─────────────────────────────────
RUN rm -f /etc/apt/sources.list.d/*ros*.list
# ─── STEP B: Update Ubuntu repos, install curl and gnupg2, and fetch the new ROS key ──
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    mkdir -p /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
# ─── STEP C: Recreate ros-latest.list so it points at our new keyring, then update ───
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros-latest.list && \
      apt-get update
# ─── STEP D: Now that APT trusts packages.ros.org, install python3-pip, vcstool, etc. ──
RUN apt-get install -y \
      python3-pip \
      python3-vcstool \
      git \
      default-jre \
      python3-catkin-tools && \
      pip3 install --upgrade pip

USER root 

RUN apt-get update && apt-get install python3-pip python3-vcstool git default-jre python3-catkin-tools -y && pip3 install pip --upgrade
RUN pip3 install rosdep
RUN rm -rf /etc/ros/rosdep/sources.list.d/* && rosdep init


RUN mkdir -p $OVERLAY_WS/src
COPY binder/pycram-http.rosinstall /tmp/pycram-http.rosinstall
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

# System dependencies for polytope
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-dev \
    libgmp-dev

# Build-Tools for Python
RUN pip install --upgrade pip setuptools wheel \
    && pip uninstall -y backports \
    && pip install backports.tarfile

RUN pip3 install playsound==1.2.2

RUN pip3 install --no-cache-dir trimesh[all] pycollada

RUN pip3 install --upgrade pip
WORKDIR $OVERLAY_WS/src/pycram

COPY binder/requirements.txt /tmp/requirements.txt

RUN sed -i '/playsound/d' /tmp/requirements.txt && \
    pip3 install -r /tmp/requirements.txt
EXPOSE 11311

ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux:$LD_LIBRARY_PATH

RUN apt-get update && apt-get install -y \
    software-properties-common && \
    apt-add-repository ppa:swi-prolog/stable && \
    apt-get update && apt-get install -y \
    gdb \
    g++ \
    clang \
    cmake \
    make \
    wget \
    libeigen3-dev \
    libspdlog-dev \
    libraptor2-dev \
    librdf0-dev \
    libgtest-dev \
    libboost-python-dev \
    libboost-serialization-dev \
    libboost-program-options-dev \
    libfmt-dev \
    mongodb-clients \
    libmongoc-1.0-0 \
    libmongoc-dev \
    doxygen \
    graphviz \
    python3 \
    python3-dev \
    python3-venv \
    python-is-python3 \
    python3-catkin-pkg \
    python3-catkin-tools \
    git \
    ros-noetic-catkin \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf \
    ros-noetic-xacro \
    swi-prolog* \
    ros-noetic-urdf \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-navigation

# install janus
RUN apt-get update & apt-get install -y libpython3-dev
RUN pip3 install janus-swi

RUN apt-get update && apt-get install -y ros-noetic-map-server && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3-pyqt5
RUN apt-get update && apt-get install -y ros-noetic-urdfdom-py 

RUN pip3 install --no-cache-dir pytess


# KnowRob dependencies
RUN apt install -y swi-prolog libspdlog-dev \
    libboost-python-dev libboost-serialization-dev libboost-program-options-dev \
    libraptor2-dev librdf0-dev libgtest-dev \
    libfmt-dev libeigen3-dev libmongoc-dev \
    doxygen graphviz
RUN apt install -y ros-noetic-tf2-geometry-msgs

# Install SOMA DFL reasoner library
WORKDIR $HOME
RUN pip install inflection
RUN git clone https://github.com/ease-crc/ease_lexical_resources.git 
WORKDIR $HOME/ease_lexical_resources/
RUN git fetch && \
git checkout 880248886ed1bccacfbb86a37d2803312aa7122a
WORKDIR $HOME/ease_lexical_resources/src
RUN pip install -e .

RUN /opt/conda/bin/conda install -y setuptools=65.5.0

# Build workspace with knowrob
WORKDIR $OVERLAY_WS/src
ARG CACHEBUST=1 
#RUN git clone https://github.com/knowrob/knowrob.git
RUN git clone https://github.com/KonschinAlina/knowrob.git
RUN git clone https://github.com/knowrob/knowrob_ros.git
RUN git clone https://github.com/SUTURO/hsr_description.git
RUN git clone https://github.com/hsr-project/hsrb_rosnav.git
RUN git clone https://github.com/code-iai/iai_common_msgs.git
RUN git clone --branch robocup https://github.com/KonschinAlina/suturo_resources.git
RUN git clone https://github.com/knowrob/knowrob_designator.git -b add_designator_parsing

RUN git clone https://github.com/ToyotaResearchInstitute/hsr_meshes.git
RUN git clone https://github.com/code-iai/iai_maps.git
RUN git clone https://github.com/ease-crc/soma.git


WORKDIR $OVERLAY_WS
RUN /usr/bin/catkin init
RUN . /opt/ros/noetic/setup.sh && /usr/bin/catkin build

# Build workspace with knowrob_designator
WORKDIR $OVERLAY_WS/src
ADD . $OVERLAY_WS/src/project
WORKDIR $OVERLAY_WS
RUN . /opt/ros/noetic/setup.sh && catkin build
RUN echo "source $OVERLAY_WS/devel/setup.bash" >> ~/.bashrc
COPY run_knowrob.sh /run_knowrob.sh

ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:1

# Jupyter Config
COPY binder/jupyter-config.json /opt/conda/share/jupyter/lab/settings/overrides.json
COPY binder/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]

RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz







