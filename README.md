# kone_opc_controller

This repo provides a ROS 2 package that provides a pub/sub mecanism to control Kone lifts through a Kone OPC server and publish the status of the lifts.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. We will guide you step by step to install this kone_opc_controller package as well as the OpenOPC gateway and an optional Matrikon OPC Server Simulation.

## Installing

### Matrikon OPC Simulation Server (Windows)(Optional)

If you want to test the gateway without the actual OPC server but with a simulation tool, you can follow the instruction [here](https://github.com/RMFHOPE/kone_matrikon_simu_cfg).

### OpenOPC Gateway (Windows)

You will need to install the OpenOPC Gateway on the same machine as your OPC Server. To install the Gateway, follow the instuction [here](https://github.com/RMFHOPE/open_opc_gateway).

### Client (Linux)

The client has only been tested on Ubuntu 18.04, using ROS 2 Dashing.
It might work on Windows 10 but haven't tested it yet.  
To install ROS 2 Dashing, follow the instructions [here](https://index.ros.org/doc/ros2/Installation/Dashing/).  
You can now install Pyro4:
```
pip install Pyro4
```

You'll need 3 packages, the *rmf_msgs*, the *open_opc_gateway_client* and the *kone_opc_controller* packages.  
First, get the *rmf_msgs* folder from the [rmf repo](https://github.com/osrf/rmf/tree/iterate_lift_message). You can install the whole thing but we really just need the *rmf_msgs* folder for now.

Then, install the *open_opc_gateway_client* following the instructions [here](https://github.com/RMFHOPE/open_opc_gateway_client).

Once done, you can clone this repo to your src folder in your ROS 2 workspace:
```
cd ~/my_ros2_ws/src
git clone https://github.com/RMFHOPE/kone_opc_controller.git
```
Then build the package :
```
source /opt/ros/dashing/setup.bash
cd ~/my_ros2_ws
colcon build --symlink-install --packages-select kone_opc_controller
```
or build everything :
```
cd ~/my_ros2_ws
colcon build --symlink-install
```
then source :
```
source ~/my_ros2_ws/install/setup.bash
```
## Parameters

Before running the package, you'll need to update the YAML file containing the parameters for this node.
The parameters in the file `kone_client_params.yaml` are an example using the file for the Matrikon Simulation.
The basic node parameters available are as follow:

| Parameter Name | Type | Description |
| ------------ | ------------- | ------------- |
| simu | Boolean | Whether to use the simulation file to simulate the control of a kone lift |
| connection_timeout | Integer | Timeout in seconds after which we give up if we can't connect to the gateway |
| read_timeout | Integer | Timeout in seconds after which we give up waiting for the lift to do an action (move, open door, etc...) |
| lift_request_topic | String | The topic we subscribe to listen to request |
| lift_state_topic | String | The topic where we publish the lifts states |

You also need to provide the lifts info, to do so, we prefix the lift info with `lift_info_X_`, X being the index of the lift (1, 2, 3...):

| Parameter Name | Type | Description |
| ------------ | ------------- | ------------- |
| lift_info_X_id | String | The id of the lift (should be unique) |
| lift_info_X_gateway_ip | String | The IP of the machine where you installed the Gateway |
| lift_info_X_gateway_port | Integer | The port to use to access the Gateway |
| lift_info_X_opc_serv_ip | String | The IP of the OPC server the Gateway will use. The Gateway will most likely be on the same machine as the OPC server so then it would be 'localhost' |
| lift_info_X_opc_serv_type | String | The type of the OPC server, for example if you use the simulation : 'Matrikon.OPC.Simulation' |
| lift_info_X_opc_group | String | The OPC group where the aliases are contained |
| lift_info_X_site | String | The OPC site name |
| lift_info_X_location | String | The OPC location name |
| lift_info_X_group | String | The OPC sub group name |
| lift_info_X_lift | String | The OPC lift name |
| lift_info_X_available_floors | String Array | The list of available floors for this lift |
| lift_info_X_available_door_sides | Integer Array | The list of available door side to open at each floors |
| lift_info_X_available_modes | Integer Array | The list of available modes, the value is based on the *rmf_msgs/LiftRequest.msg* values |
| lift_info_X_default_mode | Integer | The default mode to put the lift into when releasing it |

## Run

Once the OPC Server (or the Matrikon Simulation) has been launched, the OpenOPC Gateway has been installed and all the required packages have been installed, you can finally run the *kone_opc_controller* node :
```
ros2 run kone_opc_controller kone_client __params:=~/my_ros2_ws/src/kone_opc_controller/params/kone_client_params.yaml
```

## How to make a request

You can now publish messages on the given topic to make car calls, for example:

```
ros2 topic pub -1 /lift_request rmf_msgs/LiftRequest "{
    lift_name: 'KoneLift1', 
    session_id: 'some_session_id_1', 
    request_type: 1, 
    destination_floor: '5', 
    door_state: 2}"
```

The parameters of the `LiftRequest` are as follow:

| Parameter Name | Type | Description |
| ------------ | ------------- | ------------- |
| lift_name | String | The lift ID used in the YAML parameter file |
| session_id | String | A unique session ID (a UUID) to identify who currently have a hold of the lift |
| request_type | Integer | The type of the request (calling the lift to a floor or releasing it ?) |
| destination_floor | String | The destination floor where to send the lift |
| door_state | Integer | Whether to hold the door open, unused here as we don't have control over the door |

For more information on the message, check the *rmf_msgs* folder and the *LiftRequest.msg* file.

To release the lift, simply change the `request_type` to 0.

## How to see the status of the lifts

You can subscribe to the topic given in the YAML file to receive a `LiftState` message. For example:
```
ros2 topic echo /lift_state
```

For more information on the message, check the *rmf_msgs* folder and the *LiftState.msg* file.


## Limitations

If you lose the connection to the OPC Gateway, the lift state will only be published every 15 seconds (or X times the connection timeout, X being the number of lifts). Python being single threaded, the timer to check the lift info somehow make the timer to publish hang, therefore not publishing until the first timer was finished (in our case, after trying to get all lifts info).


## Useful links

*open_opc_gateway_client* repo [here](https://github.com/RMFHOPE/open_opc_gateway_client).  
*open_opc_gateway* repo [here](https://github.com/RMFHOPE/open_opc_gateway).  
*kone_opc_interfaces* repo [here](https://github.com/RMFHOPE/kone_opc_interfaces).  
*kone_matrikon_simu_cfg* repo [here](https://github.com/RMFHOPE/kone_matrikon_simu_cfg).  
For more informations about the Kone interface, you can directly contact Kone [here](https://www.kone.com/en/contact.aspx).  
Original OpenOPC (v1.3 for Python 2) website [here](http://openopc.sourceforge.net/).  
OpenOPC 1.2 for Python 3.4 [here](https://github.com/ya-mouse/openopc).  
OpenOPC 1.3 for Python 3.6 from which this work is inspired [here](https://github.com/chernals/openopc).  
Pyro4 tutorials and documentations [here](https://pythonhosted.org/Pyro4/index.html).  
Matrikon tools [here](https://www.matrikonopc.com/products/opc-desktop-tools/index.aspx).  
OPC DA Specifications 2.0 [here](https://www-bd.fnal.gov/controls/opc/OPC_DA_Auto_2.02_Specification.pdf).  
You can find specifications 3.0 [here](https://opcfoundation.org/developer-tools/specifications-classic/data-access) but you'll have to register.
