# Práctica 1. Introducción a ROS

ROS (Robot Operating System) es un conjunto de bibliotecas y herramientas para ayudar a los desarrolladores de software crear aplicaciones robóticas. Proporciona una abstracción del hardware, de los controladores de dispositivos, las bibliotecas, visualizadores, paso de mensajes, gestión de paquetes y mucho más.

Ya habéis utilizado ROS en otras asignaturas así que nos limitaremos a dar un breve repaso de los conceptos en que se basa, sin explicarlos en profundidad.

A continuación veremos cómo ejecutar programas ROS en el simulador y finalmente veremos cómo programar en ROS usando Python.

## Conceptos básicos

- Tópico (*topic*): Son canales de información entre los nodos. Un nodo puede emitir o suscribirse a un tópico. Por ejemplo, stage (simulador de robots) emite un tópico que es la odometría del robot. Cualquier nodos se puede suscribir. El nodo que emite no controla quién está suscrito. La información es, por tanto, unidireccional (asíncrona). Si lo que queremos es una comunicación síncrona (petición/respuesta) debemos usar servicios. Un tópico no es más que un mensaje que se envía. Podemos usar distintos tipos de clases de mensajes.
- Paquete (*package*): El software en ROS está organizado en paquetes. Un paquete puede contener un nodo, una librería, conjunto de datos, o cualquier cosa que pueda constituir un módulo. Los paquetes pueden organizarse en pilas (stacks).
- Nodo (*node*): Un nodo es un proceso que realiza algún tipo de compu- tación en el sistema. Los nodos se combinan dentro de un grafo, compar- tiendo información entre ellos, para crear ejecuciones complejas. Un nodo puede controlar un sensor láser, otro los motores de un robot y otro la construcción de mapas.
- Pila (*stack*): Conjunto de nodos que juntos proporcionan alguna funcionalidad. Por ejemplo, la *pila de navegación* sirve para que el robot pueda moverse a un punto del mapa por la ruta más corta y evitando obstáculos por el camino.

## Probando ROS en el simulador

Probar una aplicación por primera vez en un robot real suele ser problemático, ya que depurar el código es complicado y además el  robot podría sufrir daños si el código no funciona correctamente . Por eso la práctica habitual es probar el código primero en un simulador y luego trasladarlo al robot real cuando estamos razonablemente seguros de que va a funcionar.

ROS está integrado con diversos simuladores. Nosotros usaremos dos distintos:

- **Stage**: es un simulador 2D, útil porque consume pocos recursos y es suficiente para ciertos casos. Por ejemplo un robot cuyo sensor principal sea un láserr o un anillo de sonares básicamente obtiene información 2D del mundo.
- **Gazebo**: es un simulador multirobot de entornos 3D, mucho más realista que stage, aunque también consume muchos más recursos computacionales.

### Stage

Para ejecutar el simulador necesitamos un fichero de definición de mundo en el que se especifique cómo es el entorno (dimensiones, paredes, obstáculos) y el robot (dimensiones físicas y sensores). En la web de la asignatura podéis descargar un ejemplo de mundo en un zip que hay que descomprimir. 

El fichero de definición del mundo propiamente dicho es el `.world`. Además hay un fichero con un *bitmap* (en este ejemplo un `.pgm`) en el que se define el espacio vacío/ocupado.

Para ejecutar *stage* con el ejemplo, teclear en una terminal (desde el mismo directorio donde hayáis descomprimido el `.zip`):

```bash
roscore &
rosrun stage_ros stageros ejemplo.world
```

Debería aparecer una ventana 2D con el mundo simulado. Si veis el `ejemplo.pgm` notaréis que es realmente el mapa del mundo. En el `.world` se especifican sus dimensiones en metros y las del robot junto con los parámetros físicos del robot y de los sensores.

Por supuesto un robot no ve directamente el entorno sea real o simulado, sino que lo percibe indirectamente a través de sus *sensores*. Como todo en ROS, la información de los sensores es accesible a través de ciertos *topics* en los que Stage publica la información. Si hacemos

```bash
rostopic list
```
Veremos una lista de *topics* publicados, de entre los que `/base_scan` es el sensor de rango del robot. En el `.world` se configura un único sensor de tipo *laser scan* que da las distancias a los objetos más cercanos en un sector de 270º. Podemos ver la información en modo numérico en la terminal con 

```bash
rostopic echo /base_scan
```

No obstante ver impresa la lista de números con las distancias no es muy intuitivo. En general es mucho mejor visualizar la información de los sensores en modo gráfico. Para ello disponemos en ROS de la herramienta `rviz`.

```bash
rosrun rviz rviz
```

Al entrar en `rviz` lo primero es **cambiar en el panel izquierdo la opción `fixed frame` en las `Global Options`**. Este es el sistema de coordenadas que usará `rviz` para dibujar. Ahora está puesto a `map` y da un error porque eso sería para un mapa construido por el robot, cosa que no se ha hecho (lo haremos en la práctica 2). Lo podéis cambiar por cualquiera de las otras opciones que sale al seleccionar el desplegable a la derecha de `fixed frame`, por ejemplo `odom`.

Podemos visualizar el sensor de rango añadiendo un nuevo *display* de tipo *Laserscan* (botón `Add` de la parte inferior del panel izquierdo). Una vez añadido hay que cambiar la opción `topic` para que se corresponda con el que está publicando el robot, en este caso `base_scan`. Debería aparecer dibujado en rojo el entorno que rodea al robot

### Gazebo

[Gazebo](http://gazebosim.org) es un simulador 3D mucho más avanzado que Stage, y es el que necesitaremos para poder simular sensores como cámaras o cámaras RGBD (que detectan la profundidad, tipo kinect) o simular efectos físicos como `choques` o empujar objetos.

Ya que estamos, vamos a lanzar gazebo para simular uno de los robots que tenemos en el laboratorio, un turtlebot 2. Para eso haremos uso del *package* `turtlebot_gazebo`: 

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

Podemos mover al robot mediante el teclado con `turtlebot_teleop`

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

Si lanzáis la herramienta `rviz` como lo hacíamos con stage podréis ver gráficamente la información de los sensores del robot simulado.

```bash
rosrun rviz rviz
```
## Programando en ROS

### *Workspaces* y *packages*

Nuestros programas ROS estarán contenidos en un paquete (*package*) que no es más que un conjunto de programas relacionados entre sí. En ROS hay multitud de paquetes ya implementados . Por ejemplo ya habeís visto `turtlebot_gazebo`, que da soporte para simular el robot turtlebot en Gazebo o `turtlebot_teleop` que permite mover el turtlebot con teclado o mando xbox.

En un nivel superior, los *packages* se agrupan en *workspaces* o espacios de trabajo. Podéis usar durante todo el curso el mismo *workspace*, no es necesario que uséis uno distinto en cada práctica.

### Crear un *workspace*

Primero hay que crear el. *workspace* que no es más que un directorio con una determinada estructura de subdirectorios y unos cuantos archivos de configuración. 

1. **Crea un directorio cualquiera** para alojar el *workspace* (en los ejemplos de ROS se suele usar el nombre `catkin_ws`)

2. Métete el el directorio, dentro de él **crea un subdirectorio `src`** 

    ```bash
      cd catkin_ws
      mkdir src
    ```

3. **Ejecuta la orden `catkin_make`** (desde `catkin_ws`, no desde el ` src`) para crear automáticamente el resto de directorios y ficheros de configuración del *workspace*

> IMPORTANTE: para que el sistema "sepa" que `catkin_ws` es un *workspace* de ROS, cada vez que vayamos a usarlo hay que hacer antes

  ```bash
   source devel/setup.bash
  ```

> Esto tendrá efecto en la terminal actual, si abrimos otra habrá que hacerlo de nuevo

### Crear un *package*

Los paquetes residen el el directorio `src` del *workspace*

1. **Métete en el directorio `src`** del *workspace* que creaste 

      ```bash
      cd catkin_ws/src
      ```
2. **Ejecuta la orden `catkin_create_pkg`** para crear el paquete. Hace falta pasarle el nombre del nuevo paquete y la lista de los paquetes de los que depende. En nuestro caso son dos: `rospy`, que nos permite programar en ROS usando Python y `std_msgs`, donde se definen los tipos de mensajes estándar       
  
      ```bash
      catkin_create_pkg practica1 rospy std_msgs
      ```

### Ejemplo productor/consumidor

Como ROS está basado en el paso de mensajes, algunos nodos publicarán mensajes y otros los consumirán, (aunque puede haber alguno que haga las dos cosas). Vamos a ver el típico ejemplo en el que un nodo produce mensajes y otro nodo los consume.

Coloca el siguiente código en un archivo `productor.py`en `practica1/src`, correspondiendo con el productor de mensajes:

```python
#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String

rospy.init_node('productor')
# inicializamos un productor de mensajes. Parámetros:
# - nombre del topic
# - tipo de datos del topic
# - tamaño de la cola de mensajes
pub = rospy.Publisher('saludo', String, queue_size=10)
# definimos la frecuencia de publicación en Hz (mensajes por segundo)
rate = rospy.Rate(1)
count = 0
while not rospy.is_shutdown():
    # publicamos el mensaje
    pub.publish('saludo numero '  + str(count))
    count += 1
    # esperamos según nos diga la frecuencia (1/rate segundos)
    # en este caso 1 segundo
    rate.sleep()
```

Para que el archivo se pueda ejecutar directamente antes tienes que asignarle permisos de ejecución 

```bash
chmod ugo+x productor.py
```

Esto te permitirá ejecutarlo con `./productor.py`, aunque también podrías hacerlo con `python productor.py`.

IMPORTANTE: **antes** de ejecutar el programa tienes que ssegurarte de que el nodo *master* de ROS está activo. Esto lo puedes hacer ejecutando `roscore`en otra terminal.

Puedes usar la herramienta `rostopic`de ROS para ver los *topics* existentes, información sobre ellos, y los mensajes publicados dentro de cada *topic*. Por ejemplo:

- `rostopic list`mostrará la lista de *topics*. Entre ellos, si está en marcha `productor.py` deberías ver el *topic* llamado `/saludo`
- `rostopic info nombre_del_topic` mostrará información del topic en cuestión, como por ejemplo el tipo de mensajes, y los nodos que publican y consumen este *topic*.
- `rostopic echo nombre_del_topic` mostrará los mensajes que se van publicando. Para terminar, pulsa `Ctrl-C`.

También podemos consumir los mensajes del *topic* con Python, como lo hace el siguiente código, que puedes guardar en `consumidor.py`:

```python
#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String

# Esta función es la que se ejecutará cada vez que se reciba un mensaje
def callback(msg): 
    print msg.data

rospy.init_node('consumidor')
# definimos un consumidor de mensajes. Parámetros:
# - nombre del topic
# - tipo del topic
# - qué función hace de callback, llamándose cada vez que se reciba un mensaje
sub = rospy.Subscriber('saludo', String, callback)
# nos ponemos a escuchar y cedemos el control a ROS
rospy.spin()

```

Recuerda que para ejecutarlo:

```bash
chmod ugo+x consumidor.py
./consumidor.py
```

Si está en marcha el `productor.py`deberían aparecer en pantalla los mensajes a medida que se van recibiendo.

### Leyendo y publicando mensajes del robot

Los ejemplos del apartado anterior son ilustrativos del funcionamiento básico de los mensajes en ROS, pero tienen poco que ver con robots. Vamos a ver ejemplos en los que leamos información de los sensores del robot y mandemos información a los efectores.

Lo primero que necesitamos es un robot real o simulado. Recordad que para ejecutar Gazebo simulando un turtlebot podemos teclear:

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

Vamos a leer y mostrar información sobre el sensor de rango 2D, que detecta distancias a los obstáculos más cercanos alrededor del robot (normalmente en un ángulo centrado al frente). En ROS es típico que la información de este sensor se publique en un topic llamado `/scan` (aunque el nombre puede variar).

NOTA: el turtlebot simulado en realidad no tiene un sensor de rango 2D. Tiene uno 3D que es una cámara RGBD (la Kinect). No obstante hay un nodo que se encarga de transformar estos datos a información 2D y publicarlos en el topic `/scan`, que es el que usamos en este ejemplo.

Paraº escribir el código Python necesitamos saber qué información hay en el topic `/scan`. Esto lo podemos hacer con varios comandos de ROS. Por ejemplo con `rostopic` podemos ver el tipo de datos del topic. **Teniendo la simulación de Gazebo en marcha (o el robot real)** escribe en una terminal:

```bash
rostopic info /scan
```

Aparecerá en pantalla junto con otra información el tipo de datos del *topic*, en este caso `sensor_msgs/LaserScan`. Con el comando `rosmsg`podemos ver la estructura de un mensaje de este tipo

```bash
rosmsg show sensor_msgs/LaserScan
```

En la terminal deberían aparecer los campos que componen el mensaje y sus tipos. Por ejemplo están el alcance mínimo y máximo de las lecturas (`range_min`y `range_max`), el ángulo inicial y final en radianes (`angle_min`y `angle_max`,  por ejemplo -1.5 y 1.5 indicarían aproximadamente que el sensor tiene un campo de visión de 180 grados al frente) y las lecturas del sensor están en un array llamado `ranges`.

Si queréis  imprimir un mensaje de tipo `/scan`para ver el aspecto que tienen los datos reales podéis hacer `rostopic echo /scan -n1` (el `-n1` hace que se imprima solo 1 mensaje). Seguramente tendrás que hacer *scroll* hacia arriba para ver los datos.

Con esta información ya podemos escribir un pequeño programa ROS que muestre los datos del sensor de rango:

```python
#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import LaserScan

# mostramos la distancia al objeto más cercano y más lejano detectados
def callback(msg): 
    masCercano = 10000
    masLejano = 0
    for lectura in msg.ranges:
        if lectura<masCercano:
            masCercano = lectura
        elif lectura>masLejano:
            masLejano = lectura    
    print 'más cercano: ', masCercano, ' más lejano: ', masLejano

rospy.init_node('read_scan')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
```

Como nuestro código depende del paquete `sensor_msgs` deberíamos **añadir en el `package.xml` una etiqueta `<depend>`**indicándolo. Hacia el final del archivo habrá una serie de etiquetas de este tipo, podemos añadir una nueva:

```xml
<depend>sensor_msgs</depend>
```

Podemos controlar el movimiento del robot publicando mensajes en el *topic* `/mobile_base/commands/velocity`. Mediante el comando `rostopic info /mobile_base/commands/velocity`podremos ver que usa mensajes de tipo `geometry_msgs/Twist` y con `rosmsg show geometry_msgs/Twist` podemos ver la estructura del mensaje. 

> En Turtlebot 3 el topic será `cmd/vel` en lugar de /mobile_base/commands/velocity`

Básicamente podemos fijar una velocidad lineal en x,y,z y también una velocidad angular con los mismos componentes. Al publicar un mensaje de este tipo no dejamos fija la velocidad sino que pasado un breve espacio de tiempo la velocidad volverá a ser 0. En el siguiente ejemplo podemos ver cómo mandar mensajes de este tipo desde Python:

```python
#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('teleoperacion')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
while not rospy.is_shutdown():
    dato = raw_input("Escribe un comando:(f:forward/l:left/r:right) ")
    cmd = Twist()
    if dato == 'f':
        cmd.linear.x = 0.25
    elif dato == 'l':
        cmd.angular.z = 0.75
        cmd.linear.x = 0.25  
    elif dato == 'r':
        cmd.angular.z = -0.75
        cmd.linear.x = 0.25        
    pub.publish(cmd)
```

podemos escribir una `f`, `l` o `r` (seguidas de INTRO) para hacer que el robot avance recto, hacia la izquierda o hacia la derecha una corta distancia.

## Tarea a desarrollar en la práctica: moverse evitando obstáculos

Desarrolla un programa que haga que el robot se mueva por el entorno evitando los obstáculos.  No tiene por qué tener un destino determinado, simplemente "vagabundear" por el entorno. Aunque en la asignatura veremos algoritmos de evitación de obstáculos bastante sofisticados, no se pide nada de eso ya que el objetivo es simplemente que os familiaricéis con la programación en ROS.

Como idea sencilla podéis mirar las lecturas que apuntan más o menos a la derecha y las que apuntan más o menos a la izquierda (en lugar de tomar una sola podéis coger la media de varias que estén en un ángulo similar). Si las lecturas de la izquierda son menores que las de la derecha hay que girar hacia la derecha, y viceversa. Podéis añadir mejoras como por ejemplo que el giro sea mayor si la diferencia entre izquierda y derecha es mayor, que el robot se pare si la distancia mínima baja de un umbral de seguridad, etc.

**IMPORTANTE**: el código debería funcionar para cualquier sensor de rango 2D independientemente del número de lecturas y del ángulo de visión del sensor. Únicamente debéis suponer que el ángulo de visión tiene su punto medio en el frente (con lo que la lectura del medio del array será lo que tiene el robot justo enfrente). **Haced el código lo bastante  genérico para que aunque cambie el número de lecturas y el ángulo de visión el programa siga funcionando correctamente**.

Tenéis que entregar:

- Código fuente del programa con comentarios de cómo funciona

- Pruebas realizadas con el robot simulado en varios entornos y con el robot real. Haced un video con los experimentos

- A partir de las pruebas realizadas, escribid de media página a 2 páginas máximo con una evaluación del algoritmo: en qué circunstancias funciona, en cuáles ha fallado, por qué creéis que lo ha hecho y cómo creéis que se podría solucionar. 
  

  **IMPORTANTE**: en evitación de obstáculos (como en todo lo demás) no hay algoritmos perfectos  ni que funcionen siempre en todos los casos. No debéis intentar "esconder" los casos en los que vuestro código no funciona, sino  documentarlos y encontrarle una explicación. Es importante conocer las limitaciones de un algoritmo para poder aplicarlo.