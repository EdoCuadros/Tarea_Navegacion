# Tarea: Navegación por planeación de mapas

## Autor: Eduardo Cuadros Montealegre

En la tarea se plantea la creación de una ruta óptima con un mapa dado para una posterior simulación de misión de robot con ruedas. Se tuvieron las siguientes restricciones:

- Robot a usar: DR12.
- Resolución de mapa: 16 celdas / metro.
- Entrada: Superior derecha.
- Salida: Inferior izquierda.

## Modelo del robot
Primero se hace la creación del modelo cinemático del robot en MATLAB. El DR12 es un robot móvil con dos ruedas de tracción y una rueda loca de dirección. Para el modelo se necesita la trocha del robot y el radio de sus ruedas, datos que se obtienen del modelo de Coppelia. Encontramos que el radio de cada rueda es de 0.086 metros, con una trocha de 0.154 metros. El modelo cinemático del modelo se da por las dos siguientes ecuaciones, las cuales representan la velocidad lineal y angular del robot, respectivamente.

$$ v = \frac{r}{2} \dot{\theta_1} + \frac{r}{2} \dot{\theta_2} = \frac{0.086}{2} \dot{\theta_1} + \frac{0.086}{2} \dot{\theta_2}$$

$$ w = \frac{r}{2l} \dot{\theta_1} - \frac{r}{2l} \dot{\theta_2} = \frac{0.086}{2*0.154} \dot{\theta_1} - \frac{0.086}{2*0.154} \dot{\theta_2}$$

## Mapas

[[INSERTAR MAPA ]]

El mapa que se nos da tiene una resolución de 16 celdas por metro. Como se puede ver en el mapa 1, eso equivale a un mapa de aproximadamente $3.2$ x $3.2$ metros. Para calcular el factor de inflado, se tendrán en cuenta los posibles caminos que puede tomar el robot. En la siguiente figura se puede ver la magnitud en pixeles de algunos.

[[INSERTAR MAPA CON DISTANCIAS]]

Para calcular el valor en metros, se usa una equivalencia de $539.63\ px/m$. Con esto se puede ver que el camino más angosto que podría tener el robot es de $30.81\ cm$ aproximadamente. Ya que el robot tiene un ancho de 15.8 cm, se encontrará el factor con la relación entre estas dos medidas. El mapa inflado se puede ver en la *Figura 2*.

[[INSERTAR MAPA INFLADO]]


## Probabilistic Roadmap Method (PRM)

Se implementó el método de planeación de ruta PRM. El procedimiento comenzó con la carga de un mapa previamente inflado para representar las zonas de obstáculos con un margen de seguridad adecuado. A partir de este mapa, se generó un grafo de rutas posibles utilizando el objeto *mobileRobotPRM*.

```C
prm = mobileRobotPRM(inflateMap)
```

Para la construcción del grafo, se configuraron dos parámetros clave: el número máximo de nodos *NumNodes* fue establecido en $10000$, lo que permitió una representación densa del espacio de trabajo; y la distancia máxima de conexión entre nodos *ConnectionDistance* se fijó en $0.05\ metros$, restringiendo la generación de aristas a aquellas que conectaran nodos cercanos.

```mathematica
prm.NumNodes = 10000;
prm.ConnectionDistance = 0.05;
```
 
 Una vez generado el grafo, se procedió a encontrar la trayectoria óptima utilizando la función *findpath*. Para este mapa, el punto de inicio esta localizado en $(3.2,3.0)$ y la meta en $(0.2, 0.0)$.

 ```mathematica
startLocation = [3.2 3];
endLocation = [0.2 0];
path = findpath(prm, startLocation, endLocation)
```

El resultado fue una secuencia de puntos que representan la ruta más corta conectando el origen con el destino, evitando colisiones con los obstáculos presentes en el entorno. El mapa probabilistico y el mapa de ocupación con la ruta pintada se pueden ver en *Figura 3* y *Figura 4* respectivamente.

[[INSERTAR MAPA PROBABILISTIC]]
[[INSERTAR MAPA DE OCUPACIÓN CON RUTA PRM]]

Para cuantificar la eficiencia de la ruta, se utilizó como función de costo la distancia total recorrida a lo largo del trayecto, calculada como la suma de las distancias euclidianas entre puntos consecutivos del camino. 

 ```mathematica
costeTotal = sum(vecnorm(diff(path), 2, 2))
```

El valor obtenido para esta función fue de $5.3293$ metros, representando la longitud total de la ruta planificada.

## Rapidly-Exploring Random Tree (RRT)

Siguiendo con los objetivos, se implementó el método de planeación de rutas RRT. Este algoritmo expande aleatoriamente el espacio de búsqueda desde el punto de inicio hasta alcanzar el objetivo, evitando obstáculos.

Para definir el espacio de trabajo del robot, se utilizó el espacio de estados *SE(2)*, que considera las posiciones en el plano y la orientación. Los límites del espacio se configuraron de acuerdo con el napa inflado *inflatedMap*, el cual incorpora márgenes de seguridad alrededor de los obstáculos:

 ```mathematica
stateSpace = stateSpaceSE2;
stateSpace.StateBounds = [inflateMap.XWorldLimits; 
                          inflateMap.YWorldLimits; 
                          [-pi, pi]];
```

A continuación, se definió un validador de estados basado en el mapa inflado, con una distancia de validación de $0.1$ metros para verificar si un segmento de trayectoria es libre de colisiones:

```mathematica
validator = validatorOccupancyMap(stateSpace);
validator.Map = inflateMap;
validator.ValidationDistance = 0.1;
```

Luego, se configuró el planificador RRT, estableciendo una distancia máxima de conexión de $2.0$ metros entre nodos del árbol de búsqueda y un límite de $3000$ iteraciones para permitir una exploración amplia del espacio:

```mathematica
planner = plannerRRT(stateSpace, validator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 3000;
```

Con la configuración anterior, se ejecutó el proceso de planificación de ruta desde y hasta los mismos puntos, ambos con orientación 0 en radianes. La ruta calculada se puede ver en la *Figura 5*.

[[INSERTAR MAPA CON RUTA RRT]]
```mathematica
[pthObj, solnInfo] = plan(planner, [startLocation 0], [endLocation 0]);
```

El coste fue calculado de igual forma que en el anterior método, teniendo un valor de $7.5199$ metros.

## Simulación MATLAB y CoppeliaSim
Se comenzó con la importación del mapa y el ajuste de las distancias según la resolución requerida. A su vez se importó el modelo del robot y se ubicó en la posición de inicio. Esto se puede ver en la *Figura 6*
A continuación se realiza una simulación tomando la ruta PRM calculada anteriormente en MATLAB. Para este fin, se utilizó el controlador *controllerPurePursuit*, el cual permite generar comandos de velocidad lineal y angular que guían al robot a lo largo de la secuencia de puntos. Para esto se usa el algoritmo *Path Following for a Differential Drive Robot* de MATLAB.

Primero, se inicializó el controlador y se configuraron sus parámetros fundamentales. Se estableció una velocidad lineal deseada de $0.6$ metros por segundo, y una velocidad angular máxima de $2$ radianes por segundo.

```mathematica
controller = controllerPurePursuit;
release(controller);
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6; % m/s
controller.MaxAngularVelocity = 2.0;
```

El punto de inicio y el objetivo fueron definidos como el primer y último punto de la ruta generada. La orientación inicial se fijó en $\pi$ radianes, lo que representa que el robot comienza mirando hacia la izquierda en el mapa.

```mathematica
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = pi;
robotCurrentPose = [robotInitialLocation initialOrientation]';
```

A continuación, se estableció la conexión con CoppeliaSim mediante la API remota. Se inició la simulación de forma **sincrónica**, lo cual permite avanzar el tiempo de simulación paso a paso desde MATLAB.

```mathematica
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
vrep.simxSynchronous(id, true);
vrep.simxStartSimulation(id, vrep.simx_opmode_blocking);
```

Se obtuvo el handle del robot en la escena y se comenzaron a recibir actualizaciones de su posición y orientación.

```mathematica
[~, robotHandle] = vrep.simxGetObjectHandle(id, 'dr12', vrep.simx_opmode_blocking);
vrep.simxGetObjectPosition(id, robotHandle, -1, vrep.simx_opmode_streaming);
```

Durante el ciclo de control, se generaban los comandos de velocidad mediante el controlador, y se convertían en velocidades para las ruedas izquierda y derecha. Estas velocidades fueron enviadas a CoppeliaSim mediante señales. Estas señales se asociaban a velocidades de cada motor mediante un script de *lua* asociado al modelo del robot en Coppelia. Esto debido a que no se encotró manera de que el robot se moviera con datos transmitidos usando el método *simxSetJointTargetVelocity*.

```mathematica
[v, omega] = controller(robotCurrentPose);

left_velocity  = (v - (dist_ruedas/2)*omega) / radio_rueda;
right_velocity = (v + (dist_ruedas/2)*omega) / radio_rueda;

vrep.simxSetFloatSignal(id, 'leftMotorVelocity',  left_velocity,  vrep.simx_opmode_streaming);
vrep.simxSetFloatSignal(id, 'rightMotorVelocity', right_velocity, vrep.simx_opmode_streaming);
vrep.simxSynchronousTrigger(id);
```

Una vez alcanzado el objetivo, se enviaron señales de parada a los motores y se finalizó la simulación en Coppelia. El movimiento y ruta del robot usando el controlador mencionado se visualizó en MATLAB y se puede ver en la *Video 1*. El video del funcionamiento en Coppelia se puede ver en *Video 2*.

## Conclusiones y dificultades presentadas

En general el desarrollo de los objetivos se hizo de forma adecuada y sin mayores complicaciones para la parte de calculos en MATLAB. Se vuelve muy importante trabajar de la mano con la documentación de la aplicación para poder saber y entender cómo funciona cada método. 

Los mayores problemas se tuvieron cuando se hace la conexión entre MATLAB y CoppeliaSim. Se tiene que tener cuidado con los scripts de la escena y el robot, ya que se debe tener paridad entre los puertos usados en ambos programas. También se vuelve muy importante hacer sincronización de tiempo en las simulaciones. En un inicio se tuvieron problemas transmitiendo las velocidades a las ruedas, ya que el robot no recibe los datos por el método convencional. Por esta razón se tuvo que usar señales intermedias para la transmición.
Se deben inicializar las velocidades y torques del motor a 0 en cada inicio de simulación, ya que al parar la simulación en MATLAB, las señales quedan guardadas en el último estado que tuvieron. También se tuvieron muchos problemas adaptando las velocidades que se tienen en MATLAB con el robot simulado, ya que para ciertas velocidades donde la simulación en MATLAB da comportamientos normales, la simulación en Coppelia queda descartada por el funcionamiento errático del robot.