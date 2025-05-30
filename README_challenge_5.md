# Mini-Challenge 5 (Semana 5)
### Sobre el reto
El objetivo principal de este Mini Challenge 4 es repasar e implementaar los conceptos de propagación de incertidumbre vistos esta semana con Manchester Robotics, mostrando cómo el ruido y otras perturbaciones afectan la estimación de la pose de un robot tanto real como simulado, se espera de esta entrega (misma que toma como base el funcionamiento de el Mini-Challenge 3) lo siguiente:

* Completar la matriz de covarianza 3×3 de la pose dentro del mensaje Odometry.

* Generar y plotear el elipsoide de covarianza de la pose.

![Screenshot from 2025-05-08 19-07-15](https://github.com/user-attachments/assets/0b4f52e3-2bce-4569-9451-45f4cafb6f3a)


### ¿Cómo ejecutar el nodo?
Una vez que las dependencias y entorno de trabajo este actualizado y listo en tu ordenador, ejecuta lo siguiente en terminal para correr la implementación del reto:

1. Estando en el workspace, construye el paquete referente a el desafío:
   ```bash
   colcon build --packages-select mini_challenge_5
   ```
2. Configura las variables del entorno a través de:
   ```bash
   source install/setup.bash
   ```
3. Usa el launch para visualizar el elipsoide de covarianza en rviz usando la siguiente línea:
   ```bash
   ros2 launch mini_challenge_5 puzzlebot.launch.py
   ```
   
![Screenshot from 2025-05-08 19-12-13](https://github.com/user-attachments/assets/318b1ca8-58f1-43b6-a927-cb2bc63fa537)


Esto desplegará el RViz con la simulación del puzzlebot describiendo la trayectoria definida en el nodo de *set_point_generator* y mostrando el elipsoide de covarianza.
   ```
