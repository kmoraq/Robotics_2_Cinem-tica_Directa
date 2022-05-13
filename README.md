# Robotics_2_Cinemática_Directa

## Implementación con ROS en Python
Inicialmente se creó un script que publica los tópicos y llama al los servicios correspondientes para mover las diferentes articulaciones del robot, se utilizaron las teclas W S D y A para controlar tanto qué articulación se estaba moviendo como a la posición a la que se quería llegar del siguiente modo:
 + W y S se utilizan para cambiar ascendentemente o descendentemente, respectivamente entre los eslabones.
 + D se utiliza para desplazarse a un punto definido por el grupo.
 + A se utiliza para volver a la posición de home.
 
Para realizar la convención anterior se utilizó la función definida en el primer laboratorio “getKey()” y se utilizó una nueva función llamada”jointCommand()” que recibe como parámetros el id del eslabón que se va a mover, el parámetro a modificar que en este caso será la posición y el tiempo de pausa.

[![Captura-de-pantalla-de-2022-05-13-17-09-49.png](https://i.postimg.cc/FRrhcsKy/Captura-de-pantalla-de-2022-05-13-17-09-49.png)](https://postimg.cc/JyFwC8Z0)

En el main realizamos if que cumplen las condiciones mencionadas anteriormente, y utilizando la función “jointCommand()” definimos en el condicional de la letra a las posiciones correspondientes a la posición de home.
Y en el condicional de la letra d definimos las posiciones correspondientes a la escogida por el grupo.

[![Captura-de-pantalla-de-2022-05-13-17-12-58.png](https://i.postimg.cc/Gp2ChggK/Captura-de-pantalla-de-2022-05-13-17-12-58.png)](https://postimg.cc/0zL3VnsK)

Finalmente, para tener un modo de detener el procesos se hace un condicional con la letra x que cambiará el valor del while que realiza la lectura de las teclas repetidamente.


**Implementación ROS Python.**

**Actualización de los CAD y las mallas en rviz acorde el robot px100.**

Para poder adecuar el entorno de rviz al robot px100 disponible en el laboratorio se optó por consultar las referencias de este robot en la pagina web que lo vende pues esta proporciona planos de ensamble del robot con las cotas necesarias para determianar su cinemática, así como un repositorio con las mallas para el px100 con las dimensiones reales.

Luego de descargar los archivos .stl del robot estos se guardan en el directorio "meshes".

Dentro del urdf se deben editar los archivos px.urdf, px_collision y px_transmision.

Estos archivos tienen en su cuerpo la definición serial de cada eslabón y junta que posee el robot.

Un link se define utilizando el siguiente bloque

```python
<link name="nombre_link">
    
</link>
```

Dentro de este bloque se debe especificar el archivo .stl de la malla del eslabón y la postura del marco de referencia del eslabón.

El marco de referencia de el eslabón se describe con ángulos roll, picht, yaw y posición x,y,z con respecto al marco de referencia de la junta anterior a el.

```python
<visual name="">
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="package://px_robot/meshes/px100_1_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
</visual>
```

El bloque de código que se muestra arriba va dentro del bloque \<link name\> ... \<link\> y prensenta la definición del origen con la sentencia \<origin xyz rpy\> y la geometría que se va a leer para la visulización dentro del bloque \<geometry\> ... \<geometry\>. El bloque \<visual name\> ... \<visual\> funciona para visualizar en rviz el modelo del eslabón con su marco de referencia.

Seguido a cada link se coloca la definición de una junta, en el caso del link de base, este no necesita tener una junta que le preceda.

Las juntas pueden ser definidas como sigue,

```python
  <joint name="world_fixed" type="fixed">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
```

Dentro de su definición se especifica el tipo de junta, en este caso fija, su marco de referencia con respecto al de la junta anterior y tambien los links que debe unir.

La herramienta puede ser definida utilizando un link vació y una junta fija que tenga la posición y la orientación del tcp. La siguiente figura ilustra el modelo visual en el rviz del robot px100.

[![Screenshot-from-2022-05-13-13-14-19.png](https://i.postimg.cc/13mJN8hR/Screenshot-from-2022-05-13-13-14-19.png)](https://postimg.cc/KRWn6v8C)

El resultado se puede ver a continuación, en donde se inicia en el primer eslabón llegando hasta el último modificando las posiciones para llegar a la postura definida y nuevamente se devuelve entre los eslabones volviendo a la posición de home: https://youtu.be/_Qfe3F-dehE

### Conclusiones 
Como medida inicial antes de realizar cualquier proceso físico con los manipuladores, es necesario conocer los parámetros del robot, tales como sus posiciones límite, resoluciones y torques a lo que opera, esto con el fin de dar un buen uso y evitar movimientos bruscos.



## Creación del Robot con el ToolBox

Se hallan los parámetros DH para el robot con base a las dimesiones proporciondas por el siguiente pano de ensamble.

[![Screenshot-from-2022-05-13-13-18-47.png](https://i.postimg.cc/FsXqWFc3/Screenshot-from-2022-05-13-13-18-47.png)](https://postimg.cc/PNK2NjS5)

Marcos de referencia según convención de DHstd.

[![Screenshot-from-2022-05-13-18-07-32.png](https://i.postimg.cc/WzTGZyFF/Screenshot-from-2022-05-13-18-07-32.png)](https://postimg.cc/WFYqLX7T)

Tabla de parámetros DHstd según los marcos mostrados anteriomente.

[![Screenshot-from-2022-05-13-13-48-56.png](https://i.postimg.cc/bN05KXHM/Screenshot-from-2022-05-13-13-48-56.png)](https://postimg.cc/Cz1JqQ2H)


```{matlab}
clear L
clear l
l(1) = 89.45;
l(2) = 105.95;
l(3) = 100;
l(4) = 107.6;

L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,'a',l(2),   'd',0,'offset',atan(100/35),   'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,'a',l(3),   'd',0,'offset',-atan(100/35),   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,'a',l(4),   'd',0,'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');

PhantomX.tool = trotx(pi/2)*troty(pi/2);

% Graficar robot
PhantomX.plot([0 0 0 0],'notiles');
hold on 
PhantomX.teach()
```

Posición 1

[![Screenshot-from-2022-05-12-19-47-38.png](https://i.postimg.cc/SQg2VRHt/Screenshot-from-2022-05-12-19-47-38.png)](https://postimg.cc/5jF2244w)

Posición 2

[![Screenshot-from-2022-05-12-19-45-40.png](https://i.postimg.cc/gk6zR0ph/Screenshot-from-2022-05-12-19-45-40.png)](https://postimg.cc/rKM22MKy)

Posición 3

[![Screenshot-from-2022-05-12-19-41-09.png](https://i.postimg.cc/KvLKbfRC/Screenshot-from-2022-05-12-19-41-09.png)](https://postimg.cc/mtLZNQ3S)

Posición 4

[![Screenshot-from-2022-05-12-19-36-17.png](https://i.postimg.cc/L8D84bRQ/Screenshot-from-2022-05-12-19-36-17.png)](https://postimg.cc/75CkmX5S)

Posición 5

[![Screenshot-from-2022-05-12-19-35-33.png](https://i.postimg.cc/pThWZVj3/Screenshot-from-2022-05-12-19-35-33.png)](https://postimg.cc/Q9DZN30q)

#### Conclusiones

El modelo del robot en matlab es una buena aproximación a la cinemática del robot real como se muestra más adelante al verificar que la pose de la herramienta y de las articulaciones corresponde con las del robot real y su gemelo en rviz. Se podría hacer uso tambien de rigidbody para tener un acercamiento más acertado a la dimensiones reales de los eslabones y articulaciones del robot.

Sería posible implementar una junta prismática que simule una de las dos pinzas del efector final y por simetía determinar la cinemática de la otra pinza, esto puede ser util en caso de tener encuenta la pieza de trabajo que se busca sujetar con el gripper

## Implementación con ROS en MatLab y comprobación con el ToolBox

#### Conexión Con Matlab

Para iniciar la conexión con matlab primero se hace necesario iniciar el nodo maestro que servirá como nodo de comunicación con ROS.

```matlab
rosinit
```

Luego, se debe crear un cliente de un servicio, para poder enviar los valores de posición deseados al robot. Dicho servicio se encuentra dentro del paquete de dynamixel y es llamado dynamixel_command.

```matlab
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
```

También es necesario crear el mensaje que será enviado al servicio.

```matlab
motorCommandMsg= rosmessage(motorSvcClient);
```

Debido a que el comando que deberá ser enviado es de posición, es necesario agregar un addres name llamado Goal_Position como un atributo del mensaje.


```matlab
motorCommandMsg.AddrName="Goal_Position";
```

Se crea un vector de posiciones, que tendrá como datos la posición de cada una de las juntas en grados. En este caso se quiere enviar el robot a la posición de Home.

```matlab
t=[0 0 0 0 0];
```

Luego se realiza un ciclo que recorra cada una de las posiciones y las envía a cada uno de los motores. Del código se puede notar que tanto el ID de cada motor como el valor de posición son atributos del mensaje que será enviado al servicio.
Debido a que los valores de posición que recibe el atributo del mensaje es en bits, se realiza un cambio de grados a bits utilizando un mapeo con la función mapfun. Esta función realiza un mapeo de los límites de giro de los motores (en este caso de -180 hasta 180). El mapeo se realiza teniendo en cuenta la resolución del encoder (4095 bits).
Finalmente se utiliza la función call para enviar el mensaje con sus respectivos atributos de ID y posición al servicio, para que este realice el movimiento de los motores.

```matlab
for i=1:length(t)
    motorCommandMsg.Id=i;
    motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
    call(motorSvcClient,motorCommandMsg);
    pause(1);
end
```

Para generar las posiciones pedidas, se cambia el vector de posiciones y se vuelve a utilizar el ciclo para acceder al servicio.


Ahora se requiere obtener las posiciones actuales del robot, es necesario suscribirse al topico joint_states del paquete dynamizel. Para ello se crea el suscriptor

```matlab
Sub=rossubscriber('/dynamixel_workbench/joint_states');
```
Una vez creado el suscriptor se debe llamar el atributo LatestMessage y el atributo Position, para obtener las posiciones actuales de cada una de las juntas en radianes.
```matlab
Sub.LatestMessage.Position;
```
Podemos ver el resultado de la conexión de MatLab+ROS+ToolBox en el siguiente video:
https://youtu.be/pGyQpZdBjbY

#### Conclusiones

Para conectar matlab con Ros siempre es necesario iniciar el nodo maestro. Teniendo en cuenta que el manejo desde matlab sigue los mismos aspectos conceptuales. Acceder a un servicio para cambiar los valores de posición del robot y suscribirse a un tópico para traer información de la posición actual. 

Para el manejo de las posiciones del robot es indispensable conocer la resolución del encoder, para así realizar correctamente el mapeo de las posiciones.

También resultó de gran utilidad entender los atributos del mensaje, para así enviar la información correcta al servicio para que realizara los movimientos del robot. El manejo de la terminal para revisar los tópicos y servicios activos desde el paquete de Dynamixel resultó de gran importancia para entender la forma en la que se manejan las estructuras de ROS.

