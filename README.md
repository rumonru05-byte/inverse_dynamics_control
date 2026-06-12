# Lab 2: Manipulator Dynamics Simulation

Este repositorio contiene la implementación en ROS 2 (C++) del modelo dinámico para un manipulador RR. El nodo principal resuelve las ecuaciones de movimiento de Euler-Lagrange, calculando en tiempo real las aceleraciones, velocidades y posiciones articulares del robot en respuesta a pares aplicados y fuerzas externas, sujeto a la gravedad.

---

## Compilación y Ejecución
Clona este repositorio dentro de la carpeta `src` de tu espacio de trabajo de ROS 2 y compila el paquete:
```bash
cd ~/ros/amp_rob_ws/src
git clone https://github.com/rumonru05-byte/manipulator_dynamics_simulation.git
cd ~/ros/amp_rob_ws
colcon build --packages-select uma_arm_control
source install/setup.bash
```
Para lanzar el simulador y visualizar el brazo manipulador sujeto a la gravedad:
* En una terminal, el visualizador `RViz`:
```bash
ros2 launch uma_arm_description uma_arm_visualization.launch.py
```
* En otra terminal, el nodo:
```bash
ros2 launch uma_arm_control uma_arm_dynamics.launch.py
```

Los parámetros dinámicos (masas, longitudes, fricciones y gravedad) son totalmente configurables a través del archivo `config/dynamics_params.yaml`.

---

## Descripción de la Implementación
El funcionamiento del nodo principal consiste en calcular iterativamente el estado articular resolviendo la ecuación fundamental de la dinámica (Euler-Lagrange):

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = \tau + J^T(q)F_{ext}$$

Para ello, se ha implementado un bucle de control con una frecuencia parametrizable (por defecto $1000 Hz$) que actualiza la cinemática del robot en cada paso de tiempo temporal (`elapsed_time_` o $\Delta t$). Este proceso se divide en tres funciones secuenciales:

1. **Cálculo de Aceleración (`calculate_acceleration`):** En esta función se construyen y evalúan las matrices del modelo dinámico ($M$, $C$, $F_b$, $J$) y los vectores de fuerzas a partir de la posición y velocidad actuales. Finalmente, se despeja la aceleración articular ($\ddot{q}$) aislando el término e invirtiendo la matriz de inercia:

$$\ddot{q} = M(q)^{-1} \left[ \tau - C(q,\dot{q})\dot{q} - F_b\dot{q} - g(q) + J^T(q)F_{ext} \right]$$

2. **Cálculo de Velocidad (`calculate_velocity`):** Se aplica un método de integración numérica de primer orden (Euler hacia adelante). La nueva velocidad articular se obtiene sumando a la velocidad del ciclo anterior el producto de la aceleración recién calculada por el incremento de tiempo:

$$\dot{q}_{t} = \dot{q}_{t-1} + \ddot{q} \cdot \Delta t$$

3. **Cálculo de Posición (`calculate_position`):** Siguiendo el mismo esquema de integración numérica, se actualiza la posición articular sumando el desplazamiento incremental generado por la velocidad actual:

$$q_{t} = q_{t-1} + \dot{q}_{t-1} \cdot \Delta t$$

---

## Resultados y gráficas
A continuación se muestra el comportamiento libre del manipulador cayendo por acción de la gravedad, visualizado en `RViz`.

![Vídeo Parámetros Default](archivos_informe/lab2/video/video1.gif)

Las dinámicas resultantes de esta respuesta natural del sistema han sido registradas utilizando ROS bags y analizadas estadísticamente. El perfil de estado articular (Posición, Velocidad y Aceleración) se muestra a continuación:

![Gráficas Parámetros Default](archivos_informe/lab2/img/graficas_dinamica.svg)

### Análisis de la simulación
En las gráficas superiores se observa la respuesta del sistema al liberarse desde una configuración inicial $q_0 = [\pi/4, -\pi/4]$. Debido a que no se inyectan pares motrices activos ($\tau = 0$), el movimiento está regido exclusivamente por la aceleración gravitatoria y amortiguado por la fricción viscosa de las articulaciones. Las ondas de velocidad y posición reflejan la transferencia continua entre energía potencial y cinética hasta que el brazo alcanza su posición de reposo colgante (equilibrio estable).
