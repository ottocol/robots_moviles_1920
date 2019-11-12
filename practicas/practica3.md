

# Práctica 3. Programación de tareas en robots móviles
## Noviembre 2019
## Otto Colomina Pardo, Sergio Orts Escolano



En esta práctica vamos a programar un robot móvil para que realice una tarea "compuesta" que implique navegar por el entorno realizando una serie de subtareas. La tarea puede ser la que queráis, por ejemplo:

- Patrullar por un edificio pasando por diversos puntos y detectando posibles "intrusos"
- Navegar por una habitación detectando objetos tirados por el suelo y recogiéndolos si el robot tiene un brazo capaz de ello
- Navegar por un entorno en que el robot va detectando señales (por ejemplo flechas) que le van indicando a dónde ir
- Intentar localizar una pelota en el suelo e irla empujando para marcar gol en una portería
- Cualquier otra tarea que se os ocurra...

> NOTA: según sea la tarea que diseñéis es posible que no la podáis probar más que en simulación. Se valorarán las pruebas en los Turtlebot reales (y además son más divertidas que las simulaciones :)) pero también podéis usar otros modelos de robots distintos y simplemente simularlos en ROS.

La tarea puede ser muy diversa pero en general vais a necesitar varios tipos de elementos para implementarla:

- Subtareas de **movimiento**:
    + Algunas subtareas deben mandar comandos de movimiento al robot (por ejemplo: "navega en línea recta", "navega al azar evitando obstáculos". Ya hicisteis algo parecido en la práctica 1.
    + Si tienes un mapa del entorno, algunas subtareas pueden requerir ***navegar* a puntos concretos del mapa** (por ejemplo una tarea de vigilancia). Para eso podéis usar el *stack* de navegación de ROS que se explica en la sección siguiente.   
- Otras subtareas serán de **detección de condiciones** (por ejemplo, "detectar si estoy en un pasillo", o "buscar en la imagen de la cámara una pelota de color rojo"). Para estas tendréis que hacer uso de los sensores del robot.
- Necesitaréis **coordinar las subtareas**: por ejemplo habrá subtareas que se deberán realizar en una secuencia ("primero ve al *waypoint* 1 y luego al 2"), otras serán condicionales ("navega aleatoriamente hasta que te encuentres una pelota"), otras tareas serán en paralelo... En robótica para coordinar este tipo de subtareas se pueden usar varios mecanismos, como las máquinas de estados finitos y los *behavior trees*. También podéis coordinarlas simplemente con los condicionales, bucles, etc. de vuestro código.


> Aunque no es obligatorio, se valorará el uso de alguna librería de máquinas de estados finitos, como [FlexBE](http://wiki.ros.org/flexbe) o [SMACH](http://wiki.ros.org/smach) para coordinar las subtareas

## El *stack* de navegación de ROS

El *stack* de navegación de ROS es un conjunto de nodos, de distintos paquetes, que sirven para gestionar las tareas de navegación en robots móviles. Básicamente el *stack* nos permite mover el robot a un destino siguiendo un camino "razonablemente" corto (puede no ser el óptimo) y a la vez evitando los obstáculos. Estos pueden ser de dos tipos: los que están reflejados en el mapa (paredes, vallas, ...) y los obstáculos "temporales" que se va encontrando por el camino (sillas, personas, ...). Por tanto el *stack* de navegación usará tanto información del mapa (que habéis construido en la práctica anterior) como datos de los sensores (laseres, sonares, cámara,...).

La siguiente figura, tomada de [esta presentación](https://www.dis.uniroma1.it/~nardi/Didattica/CAI/matdid/robot-programming-ROS-introduction-to-navigation.pdf) muestra la estructura del *stack*. Como puede verse es bastante compleja.

![](imag/navigation_stack.png)

Fijáos en que el *stack* necesita localizarse y por tanto necesita un mapa, por lo que **tendréis que disponer del mapa** en formato YAML para el entorno que estéis probando.

### Probar el *stack* de navegación con el Turtlebot simulado

Como se puede apreciar en la figura anterior el *stack* está formado por muchos nodos y ponerlos todos en marcha simultáneamente puede resultar complicado. Afortunadamente en los paquetes de turtlebot que hay instalados tenemos un par de *demos* que nos pueden servir:

Para probar con el simulador Gazebo (en 3D) y el mundo por defecto (`playground.world`)

```bash
#simulador
roslaunch turtlebot_gazebo turtlebot_world.launch
#stack de navegación
roslaunch turtlebot_gazebo amcl_demo.launch
#rviz para visualizar el resultado y poder fijar destinos
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Una vez puesto todo en marcha, puedes probar en `rviz` a fijar un destino para el robot. Clica en el botón que aparece en la barra superior llamado `2D Nav Goal`. Luego clica en el punto del mapa al que quieres que se mueva el robot y opcionalmente arrastra para indicar la orientación final del robot. Si hay un camino factible el robot debería seguirlo y moverse hasta el destino.

Esto también lo puedes hacer por código, bien sea en python o en C++, como verás en el apartado siguiente.

> NOTA: ya veremos en clase de teoría con más detalle cómo funciona el *stack* de navegación. De momento podéis observar en los nodos de rviz (panel izquierdo) que hay un `Local Planning` (evitar obstáculos basándose en lo que detectan los sensores) y un `Global Planning` (planificar la mejor trayectoria según el mapa, sin chocar con los obstáculos reflejados en él)

Si quieres usar un mundo propio tendrás que hacerlo de este modo:

```bash
#simulador
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<trayectoria al fichero con el mundo>
#stack de navegación
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<trayectoria al .yaml del mapa>
#rviz se lanza igual
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

También podéis probar con el simulador *stage*, que es mucho más ligero que gazebo y más fácil de probar.

```bash
#Esto lanza el simulador, el stack de navegación y rviz todo en uno
roslaunch turtlebot_stage turtlebot_in_stage.launch
```

### Usar el stack de navegación en Python

## Entrega de la práctica

### Material a entregar

La documentación de la práctica es una parte muy importante en la puntuación final. 

Debéis no solo documentar la implementación que habéis hecho sino también **todos** los experimentos realizados en simulación y con el robot real (¡aun los que no funcionen!, en estos podéis analizar qué es lo que no ha funcionado y cómo lo pretendéis resolver).  

Se debe entregar documentación (en cualquier formato: PDF, HTML, etc.) con los siguientes puntos:

1. Indice del contenido de la práctica 
2. Contenido de la práctica: 
    - Explicación de la tarea que se quiere abordar
    - Si usáis máquinas de estados finitos
       - Esquema de la máquina de estados y descripción general de su comportamiento
       - Descripción de cada uno de los nodos, detallando en su caso los algoritmos/métodos que haya usado cada uno de ellos: por ejemplo si un nodo busca objetos de color azul decir cómo lo habéis hecho.
    - Si no usáis máquinas de estados, explicad cómo se combinan todas las funciones/clases/módulos de vuestro código para realizar la tarea
    - Experimentos realizados en simulación y si los habéis hecho con el robot real
3. Discusión de los resultados (qué ha funcionado, qué no, por qué creéis que han fallado algunas cosas, cómo se podrían resolver, posibles ampliaciones de la práctica,...)

Además de la documentación anterior también debéis entregar todo el **código fuente** que hayáis realizado. En el código debéis incluir comentarios indicando qué hace cada clase y cada método o función.

### Baremo

> Como es lógico la nota estará relacionada con la dificultad de la tarea pero también con la documentación de la práctica. 

- Para una nota como máximo de 6, podéis limitaros a probar alguno de los proyectos que ya están implementados en libros o tutoriales de internet
- Para una nota como máximo de 8, vuestra tarea debería ser original (propuesta por vosotros y no tomada de un libro/tutorial. Si tomáis la idea de algún sitio al menos la implementación debería ser vuestra)
- Para una nota superior a 8, vuestra tarea debería ser original (propuesta por vosotros y no tomada de un libro/tutorial) y además cumplir alguna de las siguientes condiciones: 
    - (1 punto) Que alguna subtarea hagan uso de visión: colores, formas, reconocimiento de objetos. Podéis usar cualquier paquete ROS/librería de terceros que encontréis. La nota dependerá de la dificultad de uso y también de la experimentación realizada. 
    - (1 punto) Que hayáis probado la tarea en los turtlebot reales

### Plazo y procedimiento de entrega

- La práctica se entregará **antes de las 24 horas del lunes 21 de enero de 2020**.
- La entrega se realizará a través del UACloud de la Universidad de Alicante. Se habilitará una entrega específica para subir el código fuente y la memoria asociada a la práctica.

