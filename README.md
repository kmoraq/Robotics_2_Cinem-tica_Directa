# Robotics_2_Cinemática_Directa

## Implementación con ROS en Python

## Creación del Robot con el ToolBox

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

#### Conclusiones

Para conectar matlab con Ros siempre es necesario iniciar el nodo maestro. Teniendo en cuenta que el manejo desde matlab sigue los mismos aspectos conceptuales. Acceder a un servicio para cambiar los valores de posición del robot y suscribirse a un tópico para traer información de la posición actual. 

Para el manejo de las posiciones del robot es indispensable conocer la resolución del encoder, para así realizar correctamente el mapeo de las posiciones.

También resultó de gran utilidad entender los atributos del mensaje, para así enviar la información correcta al servicio para que realizara los movimientos del robot. El manejo de la terminal para revisar los tópicos y servicios activos desde el paquete de Dynamixel resultó de gran importancia para entender la forma en la que se manejan las estructuras de ROS.

