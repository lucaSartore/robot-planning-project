# Windows Docker Installation

1.  First, install **Windows Subsystem for Linux (WSL)**. Open a command prompt with **Administrator** privileges and type:

    ```powershell
    wsl --install
    ```

2.  Install **Ubuntu 20.04.06 LTS** from the Microsoft Store. The entire procedure is explained in detail here:
    https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overv

3.  Launch **Ubuntu 20.04.06 LTS** from the Start menu. You should use this method to open a new terminal (do not use the generic "WSL" application).

4.  To install the Docker client in your Ubuntu environment and grant your user the privileges to run Docker, run the following script:

    ```bash
    sudo apt install curl
    curl -o install_docker.sh [https://raw.githubusercontent.com/idra-lab/loco_nav/refs/heads/master/install_docker.sh](https://raw.githubusercontent.com/idra-lab/loco_nav/refs/heads/master/install_docker.sh)
    sudo chmod +x install_docker.sh
    ./install_docker.sh
    ```

5.  This should have created the folder `trento_lab_home` in your `$HOME` directory. Now, **reboot the system**.

6.  Once rebooted, you can clone the `loco_nav` code inside the `trento_lab_home/ros_ws/src` folder:

    ```bash
    cd ~/trento_lab_home/ros_ws/src
    git clone [https://github.com/idra-lab/loco_nav.git](https://github.com/idra-lab/loco_nav.git)
    ```

7.  Open a new Ubuntu terminal and download the Docker image:

    ```bash
    docker pull mfocchi/trento_lab_framework:loco_nav
    ```

> [!TIP]
> If you have any issues downloading the image, create an account on Docker Hub and login with your username and password by typing:
> ```bash
> docker login
> ```

8.  Edit the `.bashrc` file inside your Ubuntu home folder:

    ```bash
    cd $HOME
    gedit .bashrc
    ```

9.  Copy the following aliases to the bottom of the `.bashrc` file and save it:

    ```bash
    alias lab_planning='xhost +local:root; docker rm -f docker_container || true; \
    docker run --name docker_container --gpus all \
    --workdir="/root" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri:/dev/dri \
    --network=host --hostname=docker -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged --shm-size 2g --rm \
    --volume $HOME/trento_lab_home:/root \
    mfocchi/trento_lab_framework:loco_nav'

    alias dock-other='docker exec -it docker_container /bin/bash'
    ```

10. Load the `.bashrc` script (this will happen automatically the next time you open a terminal):

    ```bash
    source .bashrc
    ```

11. From now on, follow the wiki on how to configure the ROS environment in the "Configure Code" section: [Configure Code](https://github.com/mfocchi/lab-docker#configure-code).

> [!NOTE]
> The alias `lab_planning` needs to be called only **ONCE** and opens the image. To link other terminals to the same image, you should run `dock-other`. This command will "attach" to the image opened previously. You can call `dock-other` as many times as you need to open multiple terminals.

> [!IMPORTANT]
> To perform git commands, ensure you have added an SSH key to your GitHub account (create one if you don't have it) following the procedure described [here](https://github.com/mfocchi/lab-docker/blob/master/install_docker.md).

## Code Management

To create your own code, install **Visual Studio Code** together with the **WSL extension** as explained [here](https://code.visualstudio.com/docs/remote/wsl). This will enable you to edit the code contained in the `trento_lab_home/ros_ws/src` folder.

To be able to open multiple terminals in the same window, install **terminator**:

```bash
sudo apt install terminator
```

## Nvidia Support

If you have an Nvidia GPU, install the driver associated with your GPU model **directly in Windows** by downloading it from the [Nvidia Website](https://www.nvidia.com/Download/index.aspx?lang=en-us) (**do not** install it inside Ubuntu!).

You can check if everything works with:

```bash
nvidia-smi
```

> [!WARNING]
> If you experience any issues using Nvidia with OpenGL rendering (a common symptom is that you cannot visualize STL meshes in RVIZ), you should update to the latest mesa-driver:
>
> ```bash
> sudo add-apt-repository ppa:kisak/kisak-mesa
> sudo apt update
> sudo apt install mesa-utils
> ```

## Optional Steps

12. To install new packages, open a terminal, call the alias `dock-root`, and install with `apt install` (**without** sudo). To store the changes in the local image, first get the Container ID (a hash number) of the active container with:

    ```bash
    docker ps
    ```

13. Commit the Docker image (the next time you open a new container, it will retain the changes made to the image without losing them):

    ```bash
    docker commit <CONTAINER_ID> mfocchi/trento_lab_framework:loco_nav
    ```
