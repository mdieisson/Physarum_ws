#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import math

class PathCoordinator:
    def __init__(self, robot_names, inflation_radius=0.4, point_step_deg=10):
        self.paths = {name: [] for name in robot_names}
        self.inflation_radius = inflation_radius
        self.point_step_deg = point_step_deg

        # Subscreve aos planos globais de cada robô
        for name in robot_names:
            rospy.Subscriber(f"/{name}/move_base/NavfnROS/plan", Path, self.make_cb(name))

        # Publica zonas de custo como PointCloud
        self.pub = rospy.Publisher("/other_paths", PointCloud, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.publish_reservations)

    def make_cb(self, name):
        def cb(msg):
            # Armazena a rota global como lista de pontos (x, y)
            self.paths[name] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        return cb

    def publish_reservations(self, event):
        cloud = PointCloud()
        cloud.header.frame_id = "map"    # Coordenadas globais
        cloud.header.stamp = rospy.Time.now()

        # Gera pontos inflados para cada segmento da rota
        for path_points in self.paths.values():
            for (x, y) in path_points:
                # Círculo preenchido de pontos para evitar buracos
                steps = int(360 / self.point_step_deg)
                for angle in range(steps):
                    rad = math.radians(angle * self.point_step_deg)
                    cloud.points.append(Point32(
                        x=x + self.inflation_radius * math.cos(rad),
                        y=y + self.inflation_radius * math.sin(rad),
                        z=0.0
                    ))

        self.pub.publish(cloud)

if __name__ == "__main__":
    rospy.init_node("path_coordinator")
    robot_list = rospy.get_param("~robot_names", ["robot_0", "robot_1", "robot_2", "robot_3"])
    PathCoordinator(robot_list)
    rospy.spin()
