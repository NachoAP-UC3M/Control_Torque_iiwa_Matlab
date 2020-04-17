# Control_Torque_iiwa_Matlab

## Descripción
Controlar el iiwa simulado en gazebo mediante ros y Matlab


## Topics modificados:

  * /iiwa_gazebo_plugin/joint_state : Se recibe en él:  
                  
                  -Position : posición articular (radianes)
                                                      
                  -Velocity: velocidad articular (radianes/s)
                  
                  -Effort : torque de cada articulación
                  
                  -Header.Stamp.seconds: tiempo en segundos
                                                      
  * /iiwa_gazebo_plugin/joint_command : Envia los torque que se quieren ejecutar en cada articulación
  
## Configuración

Download the whole repo wherever you want to have your workspace, for example, in the ```/home/user folder ```

```
cd ~
git clone https://github.com/NachoAP-UC3M/Control_Torque_iiwa_Matlab
```

* Modify .bashrc to use this workspace and set the ROS_IP and ROS_WORKSPACE env variables. Change the IP for your IP. Open a terminal and write:

```
cd 
sudo gedit .bashrc
```
In the last line to the file copy BUT WITH YOUR DATA. if not it doesn't work. 

```
export ROS_WORKSPACE=/home/nacho/catkin_ws
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WORKSPACE
export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${message}'
export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages

source /opt/ros/kinetic/setup.bash
source /home/nacho/catkin_ws/devel/setup.bash

#Enlace para obtener dirección IP en una variable de entorno:
#https://www.mundotelematico.com/linux-variable-de-entorno-con-direccion-ip-del-equipo/

IP="$(ifconfig |grep -A 1 "wlp2s0" |tail -1 |cut -d ":" -f 2| cut -d " " -f 1)"

export ROS_IP=$IP
export ROS_MASTER_URI=http://$ROS_IP:11311
export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH

```

1. CHANGE nacho BY your user name. 

example:

⋅⋅⋅if your user name is Paco⋅⋅

```export ROS_WORKSPACE=/home/nacho/catkin_ws``` by ```export ROS_WORKSPACE=/home/Paco/catkin_ws```

CHANGE catkin_ws by your_workspace_name
example:
if your user name is MyWorkspace
```export ROS_WORKSPACE=/home/nacho/catkin_ws``` by ```export ROS_WORKSPACE=/home/nacho/MyWorkspace```

CHANGE to your Ubuntu distribution.

```source /opt/ros/kinetic/setup.bash```

CHANGE in IP the name wlp2s0 by your internet connection name. Write in the terminal:
```
ifconfig
```
and check this [link](https://www.mundotelematico.com/linux-variable-de-entorno-con-direccion-ip-del-equipo/) to learn to change it.









