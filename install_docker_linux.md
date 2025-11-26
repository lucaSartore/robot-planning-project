# Linux Docker Installation

- Run the [install_docker.sh](./docker/loco_nav/install_docker.sh) script. This script is important because it installs the Docker client on your machine and adds the necessary privileges for your user to run Docker images.

```bash
sudo apt install curl
curl -o install_docker.sh [https://raw.githubusercontent.com/idra-lab/loco_nav/refs/heads/master/docker/loco_nav/install_docker.sh](https://raw.githubusercontent.com/idra-lab/loco_nav/refs/heads/master/docker/loco_nav/install_docker.sh)
sudo chmod +x install_docker.sh
./install_docker.sh
```

- If everything went smoothly, you should read: **To start docker, reboot the system!** You can now restart the PC so that all changes are applied.
- If you look into your **host** Ubuntu home directory, you will see that the `trento_lab_home` directory has been created with `/ros_ws/src` subfolders.
- Now you can clone the `loco_nav` code inside the `trento_lab_home/ros_ws/src` folder:

```bash
cd ~/trento_lab_home/ros_ws/src
git clone [https://github.com/idra-lab/loco_nav.git](https://github.com/idra-lab/loco_nav.git)
```

- Now you have two options:

    1. Download the Docker image from here (not working for Mac):

    ```bash
    docker pull mfocchi/trento_lab_framework:loco_nav
    ```

    2. Compile the Docker image yourself:

    ```bash
    cd ~/trento_lab_home/ros_ws/src/loco_nav/docker/loco_nav
    docker build -t mfocchi/trento_lab_framework:loco_nav -f Dockerfile .
    ```

- Next, you need to configure the bash environment of your Ubuntu machine. Open the `.bashrc` file from your home folder:

```bash
gedit ~/.bashrc
```

- Add the following lines at the bottom of the `.bashrc` file:
> [!NOTE]
> If you do not have an Nvidia card in your computer, skip the [driver installation section](#installing-nvidia-drivers-optional) and remove `--gpus all` from the `lab_planning` alias below

```bash
alias lab_planning='xhost +local:root; docker rm -f docker_container || true; \
docker run --name docker_container --gpus all \
--workdir="/root" \
--volume $HOME/.ssh:/root/.ssh \
--device=/dev/ttyUSB0:/dev/ttyUSB0 \
--device=/dev/dri:/dev/dri \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--network=host --hostname=docker -it \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged --shm-size 2g --rm \
--volume $HOME/trento_lab_home:/root \
mfocchi/trento_lab_framework:loco_nav'

alias dock-other='docker exec -it docker_container /bin/bash'
```

- Load the `.bashrc` script (next time you open a terminal, this will be loaded automatically).

```bash
source ~/.bashrc
```
- Open a terminal and run the alias:

```bash
lab_planning
```

- You should see your terminal prompt change from `user@hostname` to `user@docker`.
- The script will mount the folder `~/trento_lab_home` on your **host** computer. Inside the Docker images, this folder is mapped to `$HOME`. This means that any files you place in your Docker `$HOME` folder will survive stopping/starting a new Docker container. All other files and installed programs will disappear on the next run.
- The alias `lab_planning` needs to be called only **ONCE** to open the image. To link other terminals to the same image, you should run `dock-other`. This command will *attach* to the image opened previously. You can call `dock-other` as many times as you need to open multiple terminals.

- Now you can compile the ROS workspace in the `$HOME` directory **inside** Docker:

```bash
cd /root/ros_ws/
catkin_make install
```

- **Only once**, after the first compilation, run:

```bash
source /root/ros_ws/install/setup.bash
```
## Installing NVIDIA Drivers (Optional)

If your PC is equipped with an NVIDIA graphics card, you can install its drivers in Ubuntu by following these steps:

1.  Add the repository:
    ```bash
    sudo add-apt-repository ppa:graphics-drivers/ppa
    ```
2.  Update the repository list:
    ```bash
    sudo apt-get update
    ```
3.  Install the driver. Note that for Ubuntu 20.04, version 515 is recommended; for Ubuntu 22.04, version 535 is recommended, but you can use other versions:
    ```bash
    sudo apt-get install nvidia-driver-X
    ```
4.  Then reboot the system:
    ```bash
    sudo reboot
    ```

Now, tell the system to use that driver:
* Open the **Software & Updates** application.
* Go to "Additional Drivers" and select the latest driver you just installed (look for the "proprietary, tested" description).
* Press "Apply Changes".

You can verify if the drivers are installed by opening a terminal and running:

```bash
nvidia-smi
```

If this does not work, and you are sure you correctly installed the drivers, you might need to deactivate the "Secure Boot" feature in your BIOS, as it usually prevents loading the driver.

## Troubleshooting

<a name="docker_issues"></a>

Check this section only if you encounter issues running Docker.

- When launching any graphical interface inside Docker (e.g., PyCharm or gedit), you might get this error:

```text
No protocol specified
Unable to init server: Could not connect: Connection refused

(gedit:97): Gtk-WARNING **: 08:21:29.767: cannot open display: :0.0
```

This means that Docker is not properly copying the value of your `DISPLAY` environment variable. You can try to solve it this way:

1.  In a terminal **outside Docker**, launch:
    ```bash
    echo $DISPLAY
    ```
2.  You will obtain a value (e.g., `:0`). If you run the same command in a Docker terminal and the value is different, add the following line to the `.bashrc` **inside** the Docker container:
    ```bash
    export DISPLAY=value
    ```
