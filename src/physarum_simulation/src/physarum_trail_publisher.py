#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

def publish_pheromone_map():
    pub = rospy.Publisher('/pheromone_map', OccupancyGrid, queue_size=10)
    rospy.init_node('pheromone_map_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    width, height = 100, 100  # defina igual ao seu Physarum
    resolution = 0.05  # 5cm por célula

    while not rospy.is_shutdown():
        try:
            data = np.load('/tmp/pheromone_map.npy')
            data = (255 * (data / np.max(data))).astype(np.uint8)  # normaliza de 0 a 255

            grid = OccupancyGrid()
            grid.header = Header()
            grid.header.stamp = rospy.Time.now()
            grid.header.frame_id = "map"

            grid.info.resolution = resolution
            grid.info.width = width
            grid.info.height = height
            grid.info.origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

            grid.data = data.flatten().tolist()

            pub.publish(grid)
        except Exception as e:
            rospy.logwarn(f"Erro ao carregar mapa: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pheromone_map()
    except rospy.ROSInterruptException:
        pass

