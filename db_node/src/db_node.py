#!/usr/bin/env python3
import rospy
import sys

#from db_node.msg import item, itemList

from db_node.srv import Compare, CompareResponse
from db_node.srv import GenerateMit, GenerateMitResponse
from db_node.srv import RetrieveData, RetrieveDataResponse
from db_node.srv import Write, WriteResponse

class db_node:
    def __init__(self,robotName="X1"):
        self.name = "/" + robotName
        print("Connecting to Postgres database...")
        self.db_conn = self.connect_db()
        print("Done.\nPublishing Services")
        self.comp_ser = rospy.Service(self.name +'/comparison', self.comparison, Compare)
        self.gen_ser = rospy.Service(self.name +'/generate_mit', self.generate_mit, GenerateMit)
        self.read_ser = rospy.Service(self.name +'/retrieve_data', self.retrieve_data, RetrieveData)
        self.wrote_ser = rospy.Service(self.name +'/write', self.write, Write)
        print("READY")

    def connect_db(self):
        return psycopg2.connect(host="localhost",database=self.name[1:],user="postgres",password="Abcd1234")

    def comparison(self,req):
        mit = self.generate_mit()
        return 

    def generate_mit(self,req):
        return 

    def retrieve_data(self,req):
        return

    def write(self,req):
        return

if __name__ == "__main__":
    args = len(sys.argv) - 1
    if (args != 1):
        print("This class requires 1 argument for the robotName.")
        sys.exit()
    
    rospy.init_node('db_node')
    robotName = sys.argv[1]
    print("Robot Name:",robotName)
    n = db_node(robotName)
    rospy.spin()