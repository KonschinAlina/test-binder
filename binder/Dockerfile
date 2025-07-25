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
RUN vcs import --input https://raw.githubusercontent.com/Tigul/pycram-1/dev/rosinstall/pycram-https.rosinstall --skip-existing $OVERLAY_WS/src
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

RUN pip3 install --upgrade pip
WORKDIR $OVERLAY_WS/src/pycram
#RUN pip3 install pycram_bullet==3.2.8 pathlib~=1.0.1 numpy==1.24.4 pytransform3d psutil==5.9.7 typing_extensions>=4.10.0 owlready2==0.47 pynput~=1.7.7 dm_control \
                #trimesh==4.6.0 deprecated probabilistic_model>=6.0.2 random_events>=4.1.0 pint>=0.21.1 gymnasium  pin==2.7.0 inflection>=0.5.1 manifold3d==3.0.1 transforms3d python-box urdf_parser_py networkx 
#RUN pip3 install numpy==1.24.4 pytransform3d
#RUN pip3 install pycram_bullet==3.2.8 pathlib~=1.0.1
#RUN pip3 install psutil==5.9.7 typing_extensions>=4.10.0 owlready2==0.47
#RUN pip3 install pynput~=1.7.7 dm_control trimesh==4.6.0
#RUN pip3 install deprecated probabilistic_model>=6.0.2 random_events>=4.1.0 pint>=0.21.1 gymnasium
#RUN pip3 install pin==2.7.0 inflection>=0.5.1 manifold3d==3.0.1 transforms3d python-box urdf_parser_py networkx
RUN pip3 install -r requirements.txt
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
    python3-pip \
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
    ros-noetic-urdf

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

# Build workspace with knowrob
WORKDIR $OVERLAY_WS/src
RUN git clone https://github.com/knowrob/knowrob.git
RUN git clone https://github.com/knowrob/knowrob_ros.git
RUN git clone https://github.com/knowrob/knowrob_designator.git -b add_designator_parsing

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

# Jupyter Config
COPY binder/jupyter-config.json /opt/conda/share/jupyter/lab/settings/overrides.json

COPY binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]

RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz







