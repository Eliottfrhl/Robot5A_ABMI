# Projet d'Asservissement d'un Robot 5A en simulation UnrealEngine 5

## Dépendances

    ROS2 (Robot Operating System 2)
    OpenCV
    Eigen
    yaml-cpp

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

Pour lancer le nœud de détection Aruco :

~~~
ros2 run comm_ue aruco_detector
~~~

Assurez-vous que les fichiers de configuration camera_calibration.yaml et transform.yaml sont placés dans le répertoire src/comm_ue/config/.