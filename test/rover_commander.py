import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RoverCommander(Node):
    def __init__(self):
        super().__init__('rover_commander')
        self.publisher_ = self.create_publisher(JointState, '/target_joint_states', 10)
        self.joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        
        # Diccionario con tus posiciones predeterminadas
        self.posiciones = {
            'h':  [0.0, -1.5, 2.5, 0.5, 1.57],
            '00': [0.0, 0.0, 0.0, 0.0, 0.0],
            '0':  [0.0, -1.0, 2.0, 0.5, 1.57],
            '1':  [0.0, 0.9, 1.5, 0.8, 0.0],
            '2':  [1.57, 0.9, 1.5, 0.8, 0.0],
            '3':  [-1.57, 0.9, 1.5, 0.8, 0.0]
        }

    def enviar_posicion(self, tecla):
        if tecla in self.posiciones:
            msg = JointState()
            msg.name = self.joint_names
            msg.position = self.posiciones[tecla]
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Enviando comando -> Posición {tecla.upper()}: {msg.position}')
        else:
            print("⚠️ Comando no reconocido. Intenta de nuevo.")

def main(args=None):
    rclpy.init(args=args)
    nodo = RoverCommander()

    menu = """
    =========================================
      PANEL DE CONTROL - ROVER ADRC
    =========================================
    Presiona la tecla y da Enter:
    
      [h]  -> Posición Home
      [00] -> Posición 00 (Ceros)
      [0]  -> Posición 0
      [1]  -> Posición 1
      [2]  -> Posición 2
      [3]  -> Posición 3
      
      [q]  -> Salir del programa
    =========================================
    """
    print(menu)

    try:
        while rclpy.ok():
            comando = input("Selecciona un comando: ").strip().lower()
            
            if comando == 'q':
                print("Cerrando panel de control...")
                break
            
            nodo.enviar_posicion(comando)
            
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
