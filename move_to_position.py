import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ROSInterruptException
import math

class NavigateToPosition(Node):
    def __init__(self):
        super().__init__('navigate_to_position')
        # Créer un ActionClient pour l'action NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Créer un timer pour vérifier régulièrement la distance
        self._goal_position = None
        self._timer = self.create_timer(0.2, self.check_distance)  # Vérifier toutes les 1 seconde

    def send_goal(self, x, y):
        # Créer un message Goal pour la navigation
        goal_msg = NavigateToPose.Goal()
        
        # Définir la position de l'objectif
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Orientation neutre (facing forward)

        # Envoyer l'objectif et attendre le résultat
        self._goal_position = (x, y)  # Enregistrer l'objectif
        self._action_client.wait_for_server()
        self.get_logger().info(f"Envoi de l'objectif à la position: ({x}, {y})")
        self._action_client.send_goal_async(goal_msg)

    def check_distance(self):
        """Vérifie la distance par rapport à l'objectif."""
        if self._goal_position is None:
            return

        # Récupérer la position actuelle du robot
        # Ici, il vous faut obtenir la pose du robot, par exemple à partir d'un topic /amcl_pose ou similaire
        # Remplacez ce code par votre méthode pour obtenir la pose du robot (par exemple en utilisant une subscription à /amcl_pose)
        
        # Exemple d'une position robot fictive (remplacez-la par une vraie récupération de position)
        current_position = Pose()
        current_position.position.x = 1.0  # Exemple de position x du robot
        current_position.position.y = 1.0  # Exemple de position y du robot

        # Calculer la distance entre le robot et l'objectif
        goal_x, goal_y = self._goal_position
        distance = math.sqrt((goal_x - current_position.position.x) ** 2 + (goal_y - current_position.position.y) ** 2)

        # Vérifier si la distance est inférieure à un seuil
        if distance < 0.1:  # Si le robot est à moins de 0.1 mètre de l'objectif
            self.get_logger().info("Yo! Le robot a atteint l'objectif.")
            self._goal_position = None  # Réinitialiser l'objectif
            self._action_client.cancel_goal_async()  # Annuler l'objectif en cours
            self.get_logger().info("Le robot s'est arrêté.")

def main():
    rclpy.init()

    # Instancier le nœud
    nav_node = NavigateToPosition()

    try:
        # Demander à l'utilisateur la position cible
        x = float(input("Entrez la position X de l'objectif: "))
        y = float(input("Entrez la position Y de l'objectif: "))

        # Envoyer l'objectif
        nav_node.send_goal(x, y)

        # Lancer le spin pour exécuter la vérification de la distance
        rclpy.spin(nav_node)

    except ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
