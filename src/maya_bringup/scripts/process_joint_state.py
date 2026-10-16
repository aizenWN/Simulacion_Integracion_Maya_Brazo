#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from rclpy.qos import qos_profile_sensor_data

class JointStateUnwrapper(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre
        super().__init__('joint_states_unwrapped')
        
        # Define los nombres de tus joints
        self.joint_names = ['motor_1_joint', 'motor_2_joint', 'motor_3_joint', 'motor_4_joint', 'motor_5_joint']

        # Recibe las posiciones
        self.sub = self.create_subscription(JointState, '/joint_states_arm', self.joint_callback, qos_profile_sensor_data)
        # Envía las porsiciones ya calculadas
        self.pub = self.create_publisher(JointState, '/joint_states_unwrapped', 10)
        
        self.last_positions = None
        self.global_positions = None
        
        self.max_val = 4 * math.pi # +12.56 aprox
        self.min_val = -4 * math.pi # -12.56 aprox
        self.range_val = 8 * math.pi  # distancia entre extremos (~25.13)
        self.get_logger().info("Nodo 'joint_state_unwrapper' iniciado. Esperando /joint_states_arm...")
        
        # Publicar estado inicial de ceros
        self.publicar_estado_inicial()

    def publicar_estado_inicial(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0] # Posición inicial
        self.pub.publish(msg)
        self.get_logger().info("Publicado estado inicial de reposo.")

    def joint_callback(self, msg):
        # Convertimos a numpy array para operaciones vectorizadas
        raw_pos = np.array(msg.position)
    
        if self.last_positions is None:
            self.last_positions = raw_pos
            self.global_positions = np.zeros_like(raw_pos)
            self.get_logger().info("Inicializando posiciones...")
            return

        diff = raw_pos - self.last_positions

        for i in range(len(diff)):
            if diff[i] < -self.range_val / 2:
                # Pasó de +4π a -4π por lo que se incrementa vuelta completa
                self.global_positions[i] += self.range_val
            elif diff[i] > self.range_val / 2:
                # Pasó de -4π a +4π por lo que se resta vuelta completa
                self.global_positions[i] -= self.range_val
        
        delta_unwrapped = (raw_pos - self.last_positions + self.range_val/2) % self.range_val - self.range_val/2
        self.global_positions += (raw_pos - self.last_positions) 
        #self.global_positions += delta_unwrapped 
        
        self.last_positions = raw_pos

        factors = np.array([16.0, 16.0, 16.0, 10.0, 1.0])
        real_world_pos = self.global_positions / factors
        
        # Creación del mensaje de salida
        out_msg = JointState()
        #out_msg.header = msg.header
        out_msg.header = Header()
        #out_msg.name = msg.name
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.name = self.joint_names 
        #out_msg.position = visible_pos.tolist()
        out_msg.position = real_world_pos.tolist()
        
        out_msg.velocity = msg.velocity
        out_msg.effort = msg.effort

        self.pub.publish(out_msg)

        # Throttle en ROS 2 se maneja a través del logger con un contexto de tiempo
        # self.get_logger().info(f"Pos global: {np.round(self.global_positions, 2)}", throttle_duration_sec=1.0)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateUnwrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destrucción del nodo
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()