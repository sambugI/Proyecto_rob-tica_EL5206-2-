#!/usr/bin/env python3
import socket
import json
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("distancia_server")
    pub = rospy.Publisher("/distancia_objeto", String, queue_size=10)

    HOST = ""     # escucha todas las interfaces
    PORT = 5005

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)

    rospy.loginfo("Servidor TCP escuchando en puerto 5005")

    while not rospy.is_shutdown():
        conn, addr = server.accept()
        data = conn.recv(1024).decode()

        if data:
            try:
                d = json.loads(data)
                msg = "{},{},{}".format(d['clase'], d['distancia'], d['error_x'])
                pub.publish(String(data=msg))
                rospy.loginfo("Publicado: " + msg)
            except Exception as e:
                rospy.logerr("Error procesando mensaje: %s", e)

        conn.close()

    server.close()

if __name__ == "__main__":
    main()
