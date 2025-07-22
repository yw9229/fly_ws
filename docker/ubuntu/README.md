# Docker

## Introduction

This project provides a Docker containerized environment, including necessary configuration files and scripts, to help users quickly start and run the project. Please follow the steps below to set up and use it.

## Prerequisites

Before proceeding, ensure that the necessary drivers and NVIDIA Container Toolkit are installed.

### Install NVIDIA Driver

Use the following command to install the NVIDIA driver (replace `550` with the required version):

```bash
sudo apt install nvidia-driver-550
```

### Install NVIDIA Container Toolkit

Follow these steps to install the NVIDIA Container Toolkit:

1. Add the GPG key and repository:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

2. Enable the experimental repository:

```bash
sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

3. Update the package list:

```bash
sudo apt-get update
```

4. Install the NVIDIA Container Toolkit:

```bash
sudo apt-get install -y nvidia-container-toolkit
```

For more detailed instructions, visit the [official NVIDIA documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Download the Project

Clone the project to your local workspace using Git:

```bash
git clone https://github.com/TKUwengkunduo/docker.git
cd docker
```

## File Permissions

Ensure the scripts are executable by running the following commands:

```bash
chmod +x build.sh
chmod +x run.sh
```

## Build Docker Image

Use the provided Dockerfile to build the Docker image. By default, the image name is `my_image`, but you can specify a custom name during the build process.

### Build Docker Image

Run the following command to build the image:

```bash
./build.sh
```

### Customize Image Name

If you need to modify the image name, edit `build.sh`, locate the following line, and replace it:

```bash
IMAGE_NAME="my_image"  # Replace my_image with your desired name
```

## Run Container

The run script will start the container. The default image name is `my_image`. You can start it with the following command:

```bash
./run.sh
```

To modify the image name, edit `run.sh`, locate the following line, and replace it:

```bash
IMAGE_NAME="my_image"  # Replace my_image with your desired image name
```

## Modify Configuration File

The project provides a `tmux.conf` file with the following settings:

- Enable mouse mode: `set -g mouse on`
- Shortcut keys for splitting panes:
  - Ctrl+H for horizontal split: `bind -n C-h split-window -h`
  - Ctrl+V for vertical split: `bind -n C-v split-window -v`

To customize, directly modify the `tmux.conf` file and restart tmux to apply changes.

## Appendix

### Check Running Containers

Use the following command to check for running containers:

```bash
docker ps
```

### Stop a Container

Use the following command to stop a running container:

```bash
docker stop <container_name>
```

### Remove a Container

To remove a container, use:

```bash
docker rm <container_name>
```

### Remove an Image

To delete an image, use the following command:

```bash
docker rmi <image_name>
```


