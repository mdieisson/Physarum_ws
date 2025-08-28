#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("publicador_containers")
    pub = rospy.Publisher("/lista_containers", String, queue_size=1)
    rate = rospy.Rate(0.1)  # publica a cada 10s

    lista = "caixa_0_0,caixa_0_1,caixa_0_2"  # Edite conforme necessário

    while not rospy.is_shutdown():
        pub.publish(lista)
        rospy.loginfo(f"Publicando lista de containers: {lista}")
        rate.sleep()

if __name__ == "__main__":
    main()
