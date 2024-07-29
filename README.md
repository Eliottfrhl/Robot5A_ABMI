# Projet d'Asservissement d'un Robot 5A en simulation UnrealEngine 5

## Dépendances

    ROS2 (Robot Operating System 2) Jazzy    
    OpenCV
    Eigen
    yaml-cpp
    UnrealEngine 5

## Instructions d'Installation

- Cloner le dépôt dans votre espace de travail ROS2.
- Compiler le projet avec colcon :

~~~
colcon build
~~~

- Sourcer l'espace de travail :

~~~
source install/setup.bash
~~~

## Exécution

Commencer par lancer UE5.

Lancer le launch file avec ROS2 pour initier la communication ROS2/UE5 et la détection d'ArUco.

~~~
ros2 launch comm_ue launch_simulation.py
~~~

Assurez-vous que les fichiers de configuration camera_calibration.yaml et transform.yaml sont placés dans le répertoire src/comm_ue/config/.

## Autres

Les packages interfaces et robot_pose_transformer sont inutilisés pour le moment.
