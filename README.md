# binder
[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/KonschinAlina/test-binder.git/main?labpath=%2Fsrc%2Fproject%2Fbinder%2FBA_ws.jupyterlab-workspace)
This binder is used for running robotics research Jupyter Notebooks on Binderhub.

Tutorials can be found here: https://vib.ai.uni-bremen.de/page/softwaretools/cloud-based-robotics-platform#zero-to-binder

## Development

### Run and build docker image Locally (Under repo directory)

- To make the current directory writable inside the container:

  ```bash
  chmod -R g+w ./
  ```

- Build and run docker image:

  ```bash
  export DOCKER_UID=$(id -u) && export DOCKER_GID=$(id -g) &&\
  docker compose -f ./binder/docker-compose.yml up --build
  ```

- Open Web browser and go to http://localhost:8888/

- To stop and remove container:

  ```bash
  docker compose -f ./binder/docker-compose.yml down
  ```
