#!/usr/bin/env python3
import rospy
import sys

import interface
from db_node.msg import item, itemList

from db_node.srv import GenerateMit, GenerateMitResponse
from db_node.srv import RetrieveData, RetrieveDataResponse
from db_node.srv import Write, WriteResponse

class db_services:
    def __init__(self,robotName="X1"):
        self.name = "/" + robotName
        print("Connecting to Postgres database...")
        #self.db_conn = self.connect_db()
        print("Completed\nPublishing Services...")
        self.gen_ser = rospy.Service(self.name +'/db_generate_mit', GenerateMit, self.generate_mit)
        self.read_ser = rospy.Service(self.name +'/db_retrieve_data', RetrieveData, self.retrieve_data)
        self.wrote_ser = rospy.Service(self.name +'/db_write', Write, self.write_to_db)
        print("READY")

    def connect_db(self):
        return interface.connect

    def generate_mit(self,req):
        return 

    def retrieve_data(self,req):
        return

    def write_to_db(self,req):
        return

if __name__ == "__main__":
    args = len(sys.argv) - 1
    if (args != 1):
        print("This class requires 1 argument for the robotName.")
        sys.exit()
    
    rospy.init_node('db_node')
    robotName = sys.argv[1]
    print("Robot Name:",robotName)
    n = db_services(robotName)
    rospy.spin()