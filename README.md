## Extended AWSIM-Labs

This repository provides an extended version of **AWSIM-Labs**, adding support for U-turn and swerve maneuvers as well as communication with the enhanced AWSIM-Script clients and AW-RuntimeMonitor.

### Prerequisite Setup
The hardware requirements are listed here:
https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/QuickStartDemo/#pc-specs

To run the simulator with the best performance and without hogging the network, complete the following setup steps (copied from https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/QuickStartDemo).

1. Add the following lines to `~/.bashrc` file:

``` bash
if [ ! -e /tmp/cycloneDDS_configured ]; then
	sudo sysctl -w net.core.rmem_max=2147483647
	sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
	sudo ip link set lo multicast on
	touch /tmp/cycloneDDS_configured
fi
```

Every time you restart this machine, and open a new terminal, the above commands will be executed.

Until you restart the machine, they will not be executed again.

2. Save the following as `cyclonedds.xml` in your home directory `~`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="lo" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>default</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>

```

Make sure the following lines are added to the `~/.bashrc` file:

``` bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/your_username/cyclonedds.xml
```

Replace `your_username` with your actual username.
Note that you should use the absolute path to the `cyclonedds.xml` file.
A system restart is required for these changes to work.
**DO NOT** set `export ROS_LOCALHOST_ONLY=1`. CycloneDDS configuration will be enough.  

### Driver Installation (skip if already installed)

1. Install Nvidia GPU driver
```
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo ubuntu-drivers autoinstall

# or install a specific version (following was tested)
# sudo apt install nvidia-driver-545
```
After finish, reboot your machine to make the installed driver detected by the system.
```
sudo reboot
```
You can open a terminal and check if `nvidia-smi` command is available and outputs correct information.

2. Install Vulkan Graphics Library
```
sudo apt install libvulkan1
```

### Launching Binary Release
You can download a binary release from [here](https://github.com/dtanony/AWSIM-Labs/releases/download/v1.0/awsim_labs.zip), unzip it, and launch the simulator using:
```bash
./awsim_labs.x86_64
```

It may take some time for the application to start the so please wait until image similar to the one presented below is visible in your application window.
The screen looks like this
![AWSIM-Labs screen](screenshot.png)

By default, Gaussian noise is added to the simulated data of LiDAR sensors. Use option `-noise false` to disable this noise.
```bash
./awsim_labs.x86_64 -noise false
```




### Unity Project Setup
We recommend using the binary release for most use cases. However, if you want to run the simulator inside the Unity Editor or make modifications, follow these steps (adapted from https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).

0. Ensure that the **Prerequisite Setup** and **Driver Installation** steps above are completed.

1. Install Unity Hub (see [Instruction](https://docs.unity3d.com/hub/manual/InstallHub.html)), then install Unity Editor via Unity Hub. 
The recommended Editor version is  2022.3.62f1.

2. Clone this repo (skip if it was already cloned). Do not source ROS 2 before running these commands.
```bash
cd ~
git clone https://github.com/dtanony/AWSIM-Labs.git
cd AWSIM-Labs
```

3. Open the project in Unity Editor
```bash
~/Unity/Hub/Editor/[your_editor_version]/Editor/Unity -projectPath .
```

Alternatively, you can open it using Unity Hub GUI.

4. Import external package
- Download the map package [Nishishinjuku_URP_v0.2.0.unitypackage](https://autoware-files.s3.us-west-2.amazonaws.com/awsim-labs/Nishishinjuku_URP_v0.2.0.unitypackage).

- In Unity Editor, from the menu bar at the top, select `Assets -> Import Package -> Custom Package...` and navigate the `Nishishinjuku_URP.unitypackage` file you've downloaded and open.

- Click `Import` button in the popup window to import the package.

- `Nishishinjuku` package should be successfully imported under `Assets/AWSIM/Externals/`directory. You can access the directory from the `Project` window in Unity Editor.

5. Import Vehicle Physics Pro Community Edition Asset
Follow the instructions at: [VPP CE Setup](https://autowarefoundation.github.io/AWSIM-Labs/main/DeveloperGuide/EditorSetup/VPPCommunityEdition/)

6. Import Graphy Asset
Import Graphy by following these instructions: [Graphy Asset Setup](https://autowarefoundation.github.io/AWSIM-Labs/main/DeveloperGuide/EditorSetup/Graphy/)

7. Run simulator in Unity Editor

- Open the `AutowareSimulation.unity` scene placed under `Assets/AWSIM/Scenes/Main` directory
- Run the simulation by clicking `Play` button placed at the top section of Editor.
- The simulation should now be running.
