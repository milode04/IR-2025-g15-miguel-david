#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        # P
        P = self.Kp * error
        
        # I (Con anti-windup simple: si el error es pequeño, no acumulamos)
        self.integral += error
        I = self.Ki * self.integral
        
        # D
        D = self.Kd * (error - self.prev_error)
        self.prev_error = error
        
        return P + I + D

class ProjectFirstNode(Node):
    def __init__(self):
        super().__init__('project_first_node')
        
        self.left = 0.0
        self.right = 0.0
        self.front = 0.0
        self.ranges = []
        
        # CONFIGURACIÓN
        self.target_dist = 0.65  # Distancia a la pared frontal para detenerse
        self.max_sensor_val = 1.5 # "Muro virtual": ignoramos distancias mayores a esto para el PID
        self.turn_speed = 1.0    # Velocidad de giro fija
        self.cruise_speed = 0.25 # Velocidad de avance

        # PID AJUSTADO
        # Kp: Reactividad (0.6 suele ir bien)
        # Ki: 0.0 para evitar oscilaciones (eses)
        # Kd: 0.8 para amortiguar y suavizar el movimiento
        self.pid = PID(Kp=0.6, Ki=0.0, Kd=0.8)

        # Suscriptor LIDAR
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher de velocidad
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer (0.1s = 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("ProjectFirstNode has been started with STOP & TURN logic")

    def scan_callback(self, msg):
        self.ranges = msg.ranges
        n = len(msg.ranges)

        # Calcular índices (asumiendo que el láser cubre delante y lados)
        # NOTA: Dependiendo de tu robot, el índice 0 podría ser atrás o adelante.
        # Este código asume configuración estándar ROS (0 frente, +pi/2 izq, -pi/2 der)
        # Si tu robot usa índice 0 atrás, ajusta estos cálculos.
        front_idx = int((0 - msg.angle_min) / msg.angle_increment)
        left_idx  = int((math.pi/2 - msg.angle_min) / msg.angle_increment)
        right_idx = int((-math.pi/2 - msg.angle_min) / msg.angle_increment)

        # Helper para limpiar datos
        def get_clean_dist(indices):
            valid = []
            for i in range(indices - 5, indices + 5):
                # Usamos modulo n para asegurar que no nos salimos del array
                dist = msg.ranges[i % n]
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.0:
                    # CLAMPING: Aquí está la magia. Si ve infinito, le decimos que es 1.5m
                    # Esto evita que el error se dispare en los cruces.
                    valid.append(min(dist, self.max_sensor_val))
            
            if not valid:
                return self.max_sensor_val
            return sum(valid) / len(valid)

        # Helper especial para el frente (queremos la distancia REAL, no la recortada)
        def get_front_dist(center_idx):
            valid = []
            for i in range(center_idx - 5, center_idx + 5):
                dist = msg.ranges[i % n]
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.0:
                    valid.append(dist)
            if not valid:
                return 5.0 # Lejos
            return min(valid) # Usamos el mínimo por seguridad

        self.left = get_clean_dist(left_idx)
        self.right = get_clean_dist(right_idx)
        self.front = get_front_dist(front_idx)

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher_.publish(msg)

    def control_loop(self):
        if not self.ranges:
            return

        # 1. ESTADO: DETECCIÓN DE PARED FRONTAL (PARAR Y GIRAR)
        if self.front < self.target_dist:
            # FRENAMOS EL AVANCE
            linear_x = 0.0
            
            # Decidimos dirección de giro (Open Loop simple)
            # Giramos hacia donde haya más espacio
            if self.left > self.right:
                # Girar izquierda
                angular_z = -self.turn_speed
            else:
                # Girar derecha
                angular_z = self.turn_speed
                
            self.get_logger().info(f"WALL! Turning... F:{self.front:.2f}")

        # 2. ESTADO: CRUCERO (CENTRADO DE PASILLO)
        else:
            linear_x = self.cruise_speed
            
            # Calculamos error: (Izquierda - Derecha)
            # Si Izq > Der, error positivo -> PID devuelve corrección positiva -> Gira Izq para alejarse pared derecha? 
            # NO, el signo depende de tu robot.
            # Lógica estándar: Queremos error = 0.
            # Si Left (1.0) > Right (0.5), error = 0.5. Estamos cerca de la derecha.
            # Necesitamos girar a la Izquierda (Z positivo).
            
            error = self.left - self.right
            correction = self.pid.compute(error)
            
            # IMPORTANTE: Limitar la velocidad angular máxima del PID
            # Para que no de bandazos violentos
            angular_z = max(min(correction, 1.0), -1.0)
            
            # Debug
            # self.get_logger().info(f"Cruise: Err={error:.2f} Corr={angular_z:.2f}")

        # Enviar comandos
        self.move_robot(linear_x, angular_z)

def main(args=None):
    rclpy.init(args=args)
    node = ProjectFirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()