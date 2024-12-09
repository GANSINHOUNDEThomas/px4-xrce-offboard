
# ROS2_PX4_Offboard

Basé sur https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example/tree/master

Image docker prête à l'emploi pour l'environnement PX4, firmware 1.15.2 :
https://gitlab.polytech.umontpellier.fr/eii/cohoma/px4-humble

Dans src, nécessite px4_msgs :
https://github.com/PX4/px4_msgs

Mais à nettoyer en remplaçant les dossiers msg et srv par ceux dans /root/PX4-Autopilot/lsg et srv
(pour être synchro avec la même versio de firmware)


```bash
ros2 launch px4_offboard px4-sim.launch.py
```
Lance :
- joy_node
- gazebo + 1 drone x500
- supervision (pour voir les états)
- MicroXrceAgent
- control (offboard_control_152.py)
- rviz+visualizer qui reconstruit la commande et la trajectoire suivie
