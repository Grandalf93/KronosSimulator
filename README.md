# KronosSimulator

**Please review master branch for further details**


# KronosSimulator
Kronos Simulator sets out an initial approach by which Kiwibots' environments may be simulated on Gazebo API in order to tests algorithms and train AI systems using realistic scenarios. To such effect, this repository considers Loyola Marymount University (LMU) as a case study and stablishes a procedure to create new costum worlds so as to meet the conditions and model the day-to-day operations of the robots.

**For futher documentation:** [Create and interfacing a world](https://github.com/kiwicampus/KronosSimulator/blob/LMU/CreatingNewWorld.md)

## Dependencies 
A recommendation is to use [VS Code](https://code.visualstudio.com/) as the main IDE for development. Make sure you also have installed in your host:
 1. [docker-ce](https://docs.docker.com/install/)
 2. [docker-compose](https://docs.docker.com/compose/install/) v1.26.2 (Recommeded. The container won't build with higher versions)
 3. [Remote development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extensions for VSCode
 4. [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)(Focal),
 5. [Nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
 6. [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
  


![LMU](https://user-images.githubusercontent.com/104376530/177620432-5b4784a1-b8d2-4311-b969-4da0e36e9c3c.png)

## Structure
```

├── build
│   └── user_interface
│       ├── build
│       │   └── lib
│       └── user_interface.egg-info
├── install
│   └── user_interface
│       ├── lib
│       │   ├── python3.8
│       │   └── user_interface
│       └── share
│           ├── ament_index
│           ├── colcon-core
│           └── user_interface
├── log
├── simulator
│   └── kiwi_gazebo
│       ├── CMakeLists.txt
│       ├── launch
│       │   └── LMU.launch.py
│       ├── lib
│       │   ├── labelCreator.cpp
│       │   ├── labelCreator.h
│       │   ├── tinyxml2.cpp
│       │   └── tinyxml2.h
│       ├── meshes
│       │   ├── Bed.dae
│       │   ├── black_chair.dae
│       │   ├── cuadrante_1_v2.obj
│       │   ├── door.dae
│       │   ├── fridge.dae
│       │   ├── kitchen.dae
│       │   ├── kiwibot4.dae
│       │   ├── outside_couch.dae
│       │   ├── plant.dae
│       │   └── toilet.dae
│       ├── models
│       │   ├── ground_plane
│       │   ├── kiwi
│       │   └── map_satellite_33.9698455_-118.416616_1500_1500
│       ├── package.xml
│       ├── README.md
│       ├── src
│       │   └── polygon_parser.cpp
│       └── worlds
│           ├── 0_dolly_empty.world
│           ├── 1_dolly_city.world
│           ├── 2_mundo_medellin.world
│           ├── 3_LMU.world
│           └── dolly_city_sidewalk_obstacles.world
├── spawning_services
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── spawning.cpp
└── user_interface
    ├── package.xml
    ├── resource
    │   └── user_interface
    ├── setup.cfg
    ├── setup.py
    ├── test
    └── user_interface
        ├── build
        │   └── assets
        ├── GUI.py
        ├── __init__.py
        └── __pycache__
            ├── GUI.cpython-38.pyc
            └── __init__.cpython-38.pyc


```

The repository consists of two ros packages. The `kiwi_gazebo` node is in charge of providing a debug and developed tool for the entire stack of the Kronos Project. It is also where most of the worlds are hosted in the folder `worlds` as well as the executables (plugins and libraries). On the other hand, the `spawning_services` package provides a client by creating a request to `/spawn_entity' service of Gazebo.

### kiwi_gazebo
The package provides a GUI plugin Gazebo developed in the script `poygon_parser.cpp`. The GUI plugin  allows the user to interact with Gazebo interface as it populates a `.sdf` file.

**For futher documentation:** [Create a system plugin](https://classic.gazebosim.org/tutorials?tut=system_plugin&cat=write_plugin)

#### ROS publishers

| **Topic** |                   **Field description**                   |
|-----------------------------|:---------------------------------------------------------:|
|       /xml_topic     |               XML stream message of the model to be spawned        |
|             /name_sender             |             name of the model. It publishes an alphanumeric hash         |


#### ROS subscriber
| **Topic** |                   **Field description**                   |
|-----------------------------|:---------------------------------------------------------:|
|       /selection     |             It subscribes to a string message containing the class of the model: Wall, Building, etc     |





### spawning_services
The package creates a client to the service `/spawn_entity' of gazebo. 

#### Generated services

| **gazebo_msgs/SpawnEntity** |                   **Field description**                   |
|-----------------------------|:---------------------------------------------------------:|
|       name (optional)       |               Name of the model to be spawn               |
|             xml             |            This should be an urdf or gazebo xml           |
|  robot_namespace (optional) |  Spawn robot and all ROS interfaces under this namespace  |
|    inital_pose (optional)   |               Only applied to canonical body              |
|  reference_frame (optional) | If left empty or "world", then gazebo world frame is used |

#### ROS subscriber
| **Topic** |                   **Field description**                   |
|-----------------------------|:---------------------------------------------------------:|
|       /xml_topic     |             It subscribes to a string message containing the XML format (sdf) of the model      |

### user_interface
This package contains a tkinter-based interface so that the user can choose among four options  and set the model to be spawned on the map.

![botones](https://user-images.githubusercontent.com/104376530/177658439-d6edcdb1-c424-4e1b-ace2-aa42bdf84e40.png)


#### ROS publishers
| **Topic** |                   **Field description**                   |
|-----------------------------|:---------------------------------------------------------:|
|       /selection     |             It publishes the user's selecntion of the desired model    |

#### ROS subscriber
| **Topic** |                   **Field description**                   |
|-----------------------------|:---------------------------------------------------------:|
|       /name_sendes     |             It subscribes to a string message containing the alphanumeric identifier of the model. It can be displayed on the interface     |
