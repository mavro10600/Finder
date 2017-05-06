#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16

# imports para la lectura ACPI
import re
import os

class Battery_monitor:
    
    def __init__(self, node_name_override = 'finder_battery_monitor'):
		
        # El nombre del node deberìa poder especificarse como override desde alguna
        # instancia superior, por si acaso, se establece un default razonable.
        # De todos modos, el nombre del nodo que se guarda "internamente" no puede
        # asumirse igual al proporcionado, por si se invocasen al mismo tiempo varios
        # nodos del mismo "tipo", pues ROS les anexa un número al final como ID
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # loginfo sirve como medio de DEBUGGING, y para guardar un registro de la
        # ejecución del programa. Por ejemplo, algón overnodo podría hacer uso del log
        # para arreglar errores
        rospy.loginfo("finder_battery_monitor starting with name %s", self.nodename) 
        # Quizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 1 Hz 
        self.rate = rospy.get_param("finder_battery_monitor_rate", 1)
        # Publicación de la batería como entero
        self.batPub = rospy.Publisher('battery_level', Int16)        
        
    def update(self):
		
        self.bat_level = int((re.search('[0-9]*\%',(os.popen('acpi -b', 'r')).readline())).group(0)[:-1])
        self.batPub.publish(self.bat_level)

    def spin(self):
		
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    finder_battery_monitor = Battery_monitor()
    finder_battery_monitor.spin()
    

