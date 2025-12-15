# Proyecto_rob-tica_EL5206-2
Integrantes: Benjamín Bautista, Sebastián Cáceres y Samuel Bugueño

Estos son los nodos desarrollados para el proyecto final del curso EL5206-2 Laboratorio de Inteligencia Computacional y Robótica de Ingeniería Eléctrica en La FCFM. Los nodos que se ven anteriormente son para el control de un robot Pioneer 3-DX, por lo que se requiere el uso de RosAria. El repositorio que se usó como base para el proyecto es el siguiente: https://github.com/Uchile-Lab-Vision-Computacional/pioneer_ws.

Aquí se tiene descripción de los archivos:
- best4 corresponde a los pesos del mejor modelo entrenado en YOLOv8 para detectar una Mochila y una Lámpara.
- camara_yolov8 es el nodo de ROS 2 que se encarga de reconocer los objetos en la imagen, calcular la distancia respecto a la cámara y al centro de la imagen. Usa el paquete de ultralytics y publica los datos obtenidos al nodo publiser.
- camara_yolov8_2 es una variación del nodo anterior, en donde en lugar de evitar a la mochila para acercarse a la lámpara, se aleja solo de la mochila.
- distance_publiser es un nodo de ROS Melodic que se actúa como intermediario entre el nodo controlador del robot con el de camara_yolov8
- controlador_robot corresponde al nodo de ROS Melodic que calcula las velocidades usando controlador PID y se lo envía al nodo de RosAria.

 Para usar los paquetes de RosAria se usó un contenedor de Docker con ROS Melodic.
