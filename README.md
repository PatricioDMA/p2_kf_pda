# Práctica 2: Implementación del Filtro de Kalman en ROS 2

Segunda Práctica impartida en la asignatura Ampliación de Robótica de 4º GIERM. 

En esta práctica se implementa un Filtro de Kalman para estimar la posición y velocidad de un robot móvil simulado con ROS 2.

Entrega realizada por Patricio De Mariano Aguilera, DNI: 29555267Z

## Configuración y ejecución
1. Clonar repositorio dentro de un workspace:

```bash
git clone https://github.com/PatricioDMA/p2_kf_pda
cd p2_kf_pda
```
2. Construir el paquete:

```bash
colcon build --packages-select p2_kf_adr
source install/setup.bash
```

3. Lanzar la simulación en un terminal:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

4. En otra terminal, ejecutar el nodo del filtro de Kalman (modelo 1 o modelo 2):


```bash
ros2 run p2_kf_adr kf_estimation
ros2 run p2_kf_adr kf_estimation_vel
```
## Cambio entre configuraciones de ruido
Como especificado en las instrucciones de la práctica,se han simulado tres configuraciones distintas de ruido para el filtro de Kalman:
- Ruido bajo
- Ruido alto en la medición (Q grande)
- Ruido alto en el proceso (R grande)

Para simular fácilmente cada caso, se deberá acceder al archivo `filters/kalman_filter.py` y comentar/descomentar las siguientes líneas para el filtro 1 y 2 de Kalman respectivamente:
```python
def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.02, 0.02, 0.01]): # Ruido bajo
# def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.5, 0.5, 0.2]): # Ruido alto en la medición (Q grande)
# def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.5, 0.5, 0.2], obs_noise_std = [0.02, 0.02, 0.01]): # Ruido alto en el proceso (R grande)
```

```python
def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.02]*6, obs_noise_std=[0.02]*6): # Ruido bajo
# def __init__(self, initial_state, initial_covariance,
#              proc_noise_std=[0.02]*6, obs_noise_std=[0.5, 0.5, 0.2, 0.5, 0.5, 0.2]): # Ruido alto en la medición (Q grande)
# def __init__(self, initial_state, initial_covariance,
#              proc_noise_std=[0.5, 0.5, 0.2, 0.5, 0.5, 0.2], obs_noise_std=[0.02]*6): # Ruido alto en el proceso (R grande)
```

## Simulación de cada caso con el filtro completo
