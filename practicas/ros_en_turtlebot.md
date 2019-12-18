# Cómo trabajar con los turtlebot del laboratorio en ROS

Los turtlebot llevan a bordo un PC en el que está instalado ROS Indigo. El PC tiene conexión wifi, aunque no tiene pantalla. Básicamente habría dos formas de trabajar con los robots:

- **Mediante VNC**: con la wifi podemos emplear los PCs del laboratorio (o tu portátil si lo prefieres) como terminales gráficos para el robot (es decir, básicamente como si fueran la pantalla/teclado/ratón del PC del robot). **No hace falta que el PC tenga ROS** ni esté en linux ya que todo el código ROS se estaría ejecutando en el propio robot.
- **Master/Slave**: Ejecutar nuestro código ROS en un PC del laboratorio y configurar su ROS Kinetic para que se comunique de manera transparente con el ROS Indigo del robot, en el que correrían entre otros los nodos de los sensores, publicando información. **El PC debe tener instalado ROS**.

## Conexión con el robot mediante VNC

> Recuerda que TODO el código ROS se ejecutará en el robot, y que el PC solo va a hacer de terminal (pantalla/teclado/ratón)

Lo primero es conectarte a la red inalámbrica local del laboratorio. Hay 3 redes inalámbricas (`labrobot-wifi-5-1`, `labrobot-wifi-5-2`, `labrobot-wifi-2-4`) pero son todas la misma red local (hay 2 redes en 5 G y 1 en 2.4G para dispositivos inalámbricos más antiguos).

> Los PCs del laboratorio no tienen wifi, pídele al profesor un pendrive wifi de los armarios de material 
 
Una vez conectado el PC del laboratorio a la wifi, hay que conectar con un robot concreto. Lo haremos mediante VNC, que es un protocolo que permite conectar terminales gráficos a máquinas remotas. El cliente VNC es el equipo que hace de terminal y el servidor la máquina remota (en nuestro caso el robot)

Arranca el robot con el interruptor de la base. Espera un minuto a que arranque su ordenador de a bordo.

En cada robot turtlebot ya debería estar funcionando un servidor VNC. Cualquier cliente VNC te debería permitir conectarte a los robots, en Ubuntu el visor por defecto se llama **Remmina VNC**.

Los datos de los turtlebot son: (cada robot tiene el número en una pegatina en la parte superior)

- Turtlebot 1. IP: 192.168.1.5 password: turtle_1 
- Turtlebot 2. IP: 192.168.1.6 password: turtle_2 
- Turtlebot 3. IP: 192.168.1.7 password: turtle_3 
- Turtlebot 4. IP: 192.168.1.8 password: turtle_4
- Turtlebot 5. IP: 192.168.1.9 password: turtle_5

Si estás usando Remmina, en el desplegable elige el protocolo VNC y teclea la IP del robot con el que quieras conectar. 

### Arrancar ROS en el robot

Para que los sensores y servicios del robot empiecen a publicar datos en ROS:

```bash
#arranque "mínimo" para que la base se mueva
roslaunch turtlebot_bringup minimal.launch
#laser
roslaunch turtlebot_bringup hokuyo_ust10lx.launch
#cámara RGBD. Nos da la distancia en 3D a la que se encuentra cada pixel de la imagen
roslaunch astra_launch astra.launch
```

> NO es necesario arrancar siempre todos los servicios y sensores, es mejor arrancar los mínimos imprescindibles. Por ejemplo para la práctica 1 no necesitas la cámara RGBD (`astra`), solo el laser, por lo que puedes obviar la última línea

Tras la ejecución de estos comandos el robot estará operativo, publicando información sobre su odometría, sensores de contacto, laser, etc. Para comprobarlo prueba a consultar los topics disponibles, a ver si aparece una lista. Comprueba que aparezca `/scan`, (el laser) que es el que necesitas para la práctica 1

```bash
rostopic list
```
> Si te fijas en el turtlebot simulado verás que no tiene laser y sin embargo también publica un *topic* `/scan`. Esto es porque en la simulación se usa la cámara RGBD como sensor de rango (solo se toma una fila horizontal de pixels). Sin embargo esto no debería afectar a la práctica 1 ya que tu código debería funcionar razonablemente independientemente de la resolución exacta del sensor de rango.

también puedes probar a teleoperar el robot con el teclado para comprobar que se puede mover la base

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

### Copiar archivos entre el PC y el robot

Con la versión actual de VNC instalada en los robots no se pueden enviar archivos entre el PC y el robot. Una forma alternativa de copiar archivos es con la herramienta en línea de comandos `scp`.

El robot tiene un usuario linux que se llama `turtlebot` y cuya contraseña es `ros`. Es el que usaremos en `scp`. CUIDADO: el usuario del turtlebot número 5 es `tb2`, no `turtlebot`

> Para windows puedes copiar archivos gráficamente con una aplicación llamada [WinSCP](https://winscp.net/eng/docs/lang:es)

Copiar desde el PC al robot:

```bash
#Esto se teclea DESDE UNA TERMINAL DEL LINUX del PC, NO en el VNC
scp mi_archivo.zip turtlebot@IP_del_robot:~
```

te pedirá la contraseña para el usuario (que en nuestro caso es `ros`) y lo copiará al directorio `home` de ese usuario.

Copiar desde el robot al PC:

```bash
#Esto se teclea DESDE UNA TERMINAL DEL LINUX del PC, NO en el VNC
scp turtlebot@IP_del_robot:~/mi_archivo.zip .
```
## Conexión con el robot en modo cliente/servidor

> Esta forma ocupa mucho más ancho de banda pero permite usar rviz. Necesitarás ROS en el PC

En resumen, hay que definir dos variables de entorno, ROS_MASTER_URI y ROS_HOSTNAME, tanto en el turtlebot como en el PC

### En el PC

```bash
export ROS_MASTER_URI=http://<ip del turtlebot>:11311
export ROS_HOSTNAME=<ip del PC>
```
### En el turtlebot

Conectar mediante `ssh` con el turtlebot:

```bash
#del 1 al 4
ssh turtlebot@<ip del robot>
#en el turtlebot 5 el usuario es distinto
ssh tb2@<ip del robot>
```
Si la conexión funciona, las cosas que escribas en la terminal se estarán ejecutando en el robot:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=<ip del turtlebot>
```
Ejecutar los nodos de ROS que queramos arrancar, por ejemplo

```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup hokuyo_ust10lx.launch
```

## Visualizar los datos en el PC con RViz

```bash
rosrun rviz rviz
```

Ver el laser:

> Dará un error de fixed frame en Global Options,  (está puesto a map, pero no hay). Cambiarlo a otro, Por ejemplo a `base_link`

Añadir una visualización de tipo laserScan. Cambiar el topic a /scan

Ver la cámara 2D

En turtlebot arrancar la cámara astra: `roslaunch astra_launch astra.launch`
Añadir visualización de tipo Camera, cambiar topic a `/camera/rgb/image_raw`

Ver la nube de puntos de la cámara astra

Hay que tener arrancada la cámara en el turtlebot: `roslaunch astra_launch astra.launch`

Añadir visualización de tipo DepthCloud, cambiar topic a /camera/depth/image. La nube de puntos sale en la ventana principal

