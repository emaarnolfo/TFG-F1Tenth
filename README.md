# TFG-F1Tenth

## 📋 Descripción General

Este repositorio contiene la implementación completa de algoritmos de navegación autónoma para vehículos F1TENTH como parte del Trabajo de Fin de Grado (TFG) de la Universidad de Alcalá (UAH). El proyecto incluye tres algoritmos fundamentales de control autónomo implementados en ROS 2.

## 🏗️ Estructura del Repositorio

```
TFG-F1Tenth/
├── WallFollowing/          # Algoritmo de seguimiento de pared con control PID
│   └── wall_follow/
│       ├── scripts/
│       │   └── wall_follow_node.py
│       ├── wall_follow/
│       ├── package.xml
│       └── setup.py
├── FollowTheGap/          # Algoritmo Follow the Gap reactivo
│   └── gap_follow/
│       ├── scripts/
│       │   └── reactive_node.py
│       ├── gap_follow/
│       ├── package.xml
│       └── setup.py
└── SafetyNode/            # Sistema de frenado automático de emergencia (AEB)
    └── f1tenth_lab2_template/
        └── safety_node/
```

## 🌿 Ramas del Repositorio

Este repositorio está organizado en **dos ramas principales**, cada una optimizada para un entorno específico de ejecución:

### 🖥️ Rama `f1tenth_gym` - Simulación
- **Propósito**: Ejecutar nodos en el simulador F1TENTH Gym
- **Entorno**: Simulación completa con f1tenth_gym_ros
- **Hardware**: Cualquier PC/laptop con ROS 2
- **Características**:
  - Configuraciones adaptadas para el simulador
  - Topics y parámetros optimizados para f1tenth_gym
  - Ideal para desarrollo, pruebas y validación de algoritmos
  - Permite iteración rápida y debugging seguro

### 🏎️ Rama `f1tenth_ws` - Hardware Real
- **Propósito**: Ejecutar nodos directamente en el vehículo F1TENTH
- **Entorno**: Hardware real con NVIDIA Jetson
- **Hardware**: Vehículo F1TENTH con Jetson Nano/Xavier
- **Características**:
  - Configuraciones optimizadas para hardware real
  - Topics y parámetros ajustados para sensores físicos
  - Manejo de latencias reales y limitaciones de hardware
  - Control directo del vehículo físico

## 🚗 Algoritmos Implementados

### 1. Wall Following (Seguimiento de Pared)
- **Archivo principal**: `wall_follow_node.py`
- **Descripción**: Control PID para mantener distancia constante a la pared
- **Método**: 
  - Cálculo de error usando proyección futura (lookahead)
  - Control PID con componentes proporcional, integral y derivativo
  - Velocidad adaptativa según ángulo de dirección

### 2. Follow the Gap (Navegación Reactiva)
- **Archivo principal**: `reactive_node.py`
- **Descripción**: Algoritmo reactivo para evitación de obstáculos
- **Método**:
  - Preprocesamiento de datos LiDAR
  - Creación de burbuja de seguridad alrededor del punto más cercano
  - Búsqueda del gap más largo en espacio libre
  - Selección del mejor punto objetivo

### 3. Safety Node (Frenado de Emergencia)
- **Archivo principal**: Sistema AEB
- **Descripción**: Frenado automático de emergencia
- **Método**: Detección de colisiones inminentes y frenado preventivo

## 🛠️ Requisitos del Sistema

### Requisitos Comunes
- **ROS 2** (Humble Hawksbill o superior)
- **Python 3.8+**
- **Paquetes ROS 2**:
  - `sensor_msgs`
  - `ackermann_msgs`
  - `rclpy`
- **Dependencias Python**:
  - `numpy`

### Requisitos Específicos por Rama

#### Para `f1tenth_gym` (Simulación)
- **f1tenth_gym_ros**: Paquete del simulador F1TENTH
- **Gazebo** (opcional, según configuración)
- **PC/Laptop** con capacidad gráfica adecuada

#### Para `f1tenth_ws` (Hardware Real)
- **NVIDIA Jetson** (Nano, Xavier, Orin)
- **Lidar** (configurado y calibrado)
- **VESC** (controlador de velocidad electrónico)
- **Hardware F1TENTH** completo

## 📦 Instalación y Uso

### Paso 1: Clonar el Repositorio
```bash
git clone <repository-url>
cd TFG-F1Tenth
```

### Paso 2: Seleccionar la Rama Apropiada

#### Para Simulación (f1tenth_gym)
```bash
git checkout f1tenth_gym

# Configurar entorno ROS 2
source /opt/ros/humble/setup.bash

# Compilar paquetes
colcon build
source install/setup.bash

# Ejecutar nodos
ros2 run wall_follow wall_follow_node.py
# o
ros2 run gap_follow reactive_node.py
```

#### Para Hardware Real (f1tenth_ws)
```bash
git checkout f1tenth_ws

# En la Jetson del vehículo
source /opt/ros/humble/setup.bash

# Compilar paquetes
colcon build
source install/setup.bash

# Ejecutar nodos
ros2 run wall_follow wall_follow_node.py
# o
ros2 run gap_follow reactive_node.py
```

## ⚙️ Configuración Específica por Entorno

### Configuración para Simulación (f1tenth_gym)
- **Topics**:
  - `/scan` - Datos LiDAR del simulador
  - `/drive` - Comandos de control al simulador
- **Parámetros**: Optimizados para respuesta en simulación
- **Frecuencia**: Alta frecuencia de actualización

### Configuración para Hardware Real (f1tenth_ws)
- **Topics**:
  - `/scan` - Datos LiDAR del hardware real
  - `/drive` - Comandos al VESC del vehículo
- **Parámetros**: Ajustados para latencias y limitaciones reales
- **Frecuencia**: Optimizada para capacidad de procesamiento de Jetson

## 🎯 Casos de Uso

| Escenario | Rama Recomendada | Justificación |
|-----------|------------------|---------------|
| **Desarrollo inicial** | `f1tenth_gym` | Entorno seguro, iteración rápida |
| **Pruebas de algoritmos** | `f1tenth_gym` | Sin riesgo de daño al hardware |
| **Validación final** | `f1tenth_ws` | Condiciones reales de operación |
| **Demostración** | `f1tenth_ws` | Rendimiento en vehículo real |
| **Investigación** | Ambas | Comparación simulación vs realidad |

## 📊 Diferencias Clave Entre Ramas

| Aspecto | f1tenth_gym | f1tenth_ws |
|---------|-------------|------------|
| **Entorno** | Simulación | Hardware Real |
| **Procesamiento** | PC/Laptop | Jetson |
| **Sensores** | Simulados | Físicos |
| **Latencia** | Mínima | Variable |
| **Seguridad** | Total | Requiere precaución |
| **Costo de Error** | Ninguno | Potencial daño físico |
| **Iteración** | Muy rápida | Más lenta |
| **Validación** | Teórica | Real |

## 🔧 Desarrollo y Contribución

### Workflow Recomendado
1. **Desarrollar** en `f1tenth_gym`
2. **Probar** extensivamente en simulación
3. **Adaptar** parámetros para `f1tenth_ws`
4. **Validar** en hardware real
5. **Iterar** según resultados

### Mejores Prácticas
- Mantener sincronización entre ramas para funcionalidad core
- Documentar diferencias específicas de configuración
- Probar siempre en simulación antes del hardware real
- Mantener parámetros de seguridad conservadores en hardware real

## 📈 Estado del Proyecto

- ✅ **Estructura base**: Completa
- ✅ **Wall Following**: Template implementado
- ✅ **Follow the Gap**: Template implementado
- 🔄 **Safety Node**: En desarrollo
- 🔄 **Optimizaciones**: Mejoras continuas
- 📋 **Documentación**: En progreso

## 👥 Información del TFG

- **Estudiante**: Emanuel
- **Universidad**: Universidad de Alcalá (UAH)
- **Proyecto**: Trabajo de Fin de Grado - Navegación Autónoma F1TENTH
- **Tecnologías**: ROS 2, Python, F1TENTH, NVIDIA Jetson

## 📚 Referencias

- [F1TENTH Course](https://f1tenth.org/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym_ros)
- [Universidad de Alcalá](https://www.uah.es/)

---

## 🚀 Quick Start

### Para Simulación
```bash
git checkout f1tenth_gym
colcon build && source install/setup.bash
ros2 run wall_follow wall_follow_node.py
```

### Para Hardware Real
```bash
git checkout f1tenth_ws
colcon build && source install/setup.bash  
ros2 run wall_follow wall_follow_node.py
```

**⚠️ Importante**: Siempre verificar la rama activa antes de ejecutar en hardware real.
