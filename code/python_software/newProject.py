import time
import serial
import threading
import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib import animation
import random
from datetime import datetime
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import calendar
ARRAY_SIZE = 5

class BaseProcessing:

    def __init__(self):
        self.isConnected = 0
        self.serial_port = None
        self.lock_data = threading.Lock()
        self.lock_connection_status = threading.Lock()
        self.lock_rssi_coords = threading.Lock()
        self.beacon_x1 = 0
        self.beacon_y1 = 0
        self.beacon_xvals = np.empty(13)
        self.beacon_yvals = np.empty(13)

        self.mobile_x = 0
        self.mobile_y = 0
        self.nodeA_index = 0
        self.nodeB_index = 0
        self.nodeC_index = 0
        self.nodeD_index = 0
        self.nodeE_index = 0
        self.nodeF_index = 0
        self.nodeG_index = 0
        self.nodeH_index = 0
        self.nodeI_index = 0
        self.nodeJ_index = 0
        self.nodeK_index = 0
        self.nodeL_index = 0
        self.rssi_data_samples_nodeA = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeB = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeC = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeD = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeE = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeF = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeG = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeH = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeI = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeJ = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeK = np.ones(ARRAY_SIZE)
        self.rssi_data_samples_nodeL = np.ones(ARRAY_SIZE)
        self.size_received_beacons = 0
        self.x0 = 3
        self.y0 = 2
        self.mobileNode1x = 0
        self.mobileNode1y = 0
        self.mobileNode2x = 0
        self.mobileNode2y = 0
        self.receivedXPositions = np.ones(5)
        self.receivedYPositions = np.ones(5)
        self.receivedRSSI = np.ones(5)
        self.receivedMobileNodeNumber = 0
        self.fig, self.ax = plt.subplots()
        img = plt.imread("floorplan.PNG")
        self.ax.imshow(img, extent=[0,29.9,0,27.6])
        #self.graph, = self.ax.plot([], [],'ro')

        self.mobilenode1, = self.ax.plot([],[],'go', markersize='12')
        self.mobilenode2, = self.ax.plot([],[],'ro', markersize='12')
        
        self.ax.axis([0, 41, 0, 21])
        self.setup_beacons()
        self.writeFile = open('resultsStatic.csv','w')
    def setup_beacons(self):
        # inputfile = open('BeaconData.csv','r')
        # for row in inputfile:
        #     print(row)
        #     print(type(row))
        csv_reader = csv.reader(open('BeaconData.csv'), delimiter=',')
        line_count = 0
        for row in csv_reader:
        #for row in range(10):
            
            self.beacon_xvals[line_count] = row[1]
            self.beacon_yvals[line_count] = row[2]
            line_count += 1
        print(self.beacon_xvals)
        #print(self.beacon_yvals)
        #print(self.beacon_yvals[7])
        print(np.mean(self.beacon_xvals))

    def update(self,extra):
        self.lock_rssi_coords.acquire()
        #self.graph.set_data(self.x0,self.y0)
        self.mobilenode1.set_data(self.mobileNode1x, self.mobileNode1y)
        self.mobilenode2.set_data(self.mobileNode2x, self.mobileNode2y)
        self.fig.suptitle("At X:%f, Y:%f, MobileNode:(%f,%f)" %(self.x0, self.y0, self.mobileNode1x, self.mobileNode1y))

        self.lock_rssi_coords.release()
        

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.update,interval=200, blit=False)


    def calculate_positions(self):
        self.lock_rssi_coords.acquire()
        self.lock_data.acquire()
        if (self.size_received_beacons == 3):
            r1 = self.receivedRSSI[0]
            r2 = self.receivedRSSI[1]
            r3 = self.receivedRSSI[2]
            
            x1 = self.receivedXPositions[0]
            x2 = self.receivedXPositions[1]
            x3 = self.receivedXPositions[2]
            y1 = self.receivedYPositions[0]
            y2 = self.receivedYPositions[1]
            y3 = self.receivedYPositions[2]

            b1 = (r1**2) -(r3**2) - (x1**2) -(y1**2) +(x3**2) + (y3**2)
            b2 = (r2**2) -(r3**2) - (x2**2) -(y2**2) +(x3**2) + (y3**2)

            Ax1 = (2*(x3-x1))
            Ax2 = (2*(x3-x2))

            Ay1 = (2*(y3-y1))
            Ay2 = (2*(y3-y2))
            Ax_array = np.array([Ax1, Ax2])
            Ay_array = np.array([Ay1, Ay2])
            B_array = np.array([b1, b2])
            A = np.vstack([Ax_array, Ay_array]).T
       
            x0, y0 = np.linalg.lstsq(A,B_array,rcond=None)[0]
            if(self.receivedMobileNodeNumber == 1):
                self.mobileNode1x = x0
                self.mobileNode1y = y0
            else:
                self.mobileNode2x = x0
                self.mobileNode2y = y0
        elif (self.size_received_beacons == 4):
            r1 = self.receivedRSSI[0]
            r2 = self.receivedRSSI[1]
            r3 = self.receivedRSSI[2]
            r4 = self.receivedRSSI[3]
            x1 = self.receivedXPositions[0]
            x2 = self.receivedXPositions[1]
            x3 = self.receivedXPositions[2]
            x4 = self.receivedXPositions[3]
            y1 = self.receivedYPositions[0]
            y2 = self.receivedYPositions[1]
            y3 = self.receivedYPositions[2]
            y4 = self.receivedYPositions[3]

            b1 = (r1**2) -(r4**2) - (x1**2) -(y1**2) +(x4**2) + (y4**2)
            b2 = (r2**2) -(r4**2) - (x2**2) -(y2**2) +(x4**2) + (y4**2)
            b3 = (r3**2) -(r4**2) - (x3**2) -(y3**2) +(x4**2) + (y4**2)
            Ax1 = (2*(x4-x1))
            Ax2 = (2*(x4-x2))
            Ax3 = (2*(x4-x3))
            Ay1 = (2*(y4-y1))
            Ay2 = (2*(y4-y2))
            Ay3 = (2*(y4-y3))
            Ax_array = np.array([Ax1, Ax2, Ax3])
            Ay_array = np.array([Ay1, Ay2, Ay3])
            B_array = np.array([b1, b2, b3])
            A = np.vstack([Ax_array, Ay_array]).T
       
            x0, y0 = np.linalg.lstsq(A,B_array,rcond=None)[0]
            if(self.receivedMobileNodeNumber == 1):
                self.mobileNode1x = x0
                self.mobileNode1y = y0
            else:
                self.mobileNode2x = x0
                self.mobileNode2y = y0
        elif (self.size_received_beacons == 5):
            r1 = self.receivedRSSI[0]
            r2 = self.receivedRSSI[1]
            r3 = self.receivedRSSI[2]
            r4 = self.receivedRSSI[3]
            r5 = self.receivedRSSI[4]

            x1 = self.receivedXPositions[0]
            x2 = self.receivedXPositions[1]
            x3 = self.receivedXPositions[2]
            x4 = self.receivedXPositions[3]
            x5 = self.receivedXPositions[4]

            y1 = self.receivedYPositions[0]
            y2 = self.receivedYPositions[1]
            y3 = self.receivedYPositions[2]
            y4 = self.receivedYPositions[3]
            y5 = self.receivedYPositions[4]

            b1 = (r1**2) -(r5**2) - (x1**2) -(y1**2) +(x5**2) + (y5**2)
            b2 = (r2**2) -(r5**2) - (x2**2) -(y2**2) +(x5**2) + (y5**2)
            b3 = (r3**2) -(r5**2) - (x3**2) -(y3**2) +(x5**2) + (y5**2)
            b4 = (r4**2) -(r5**2) - (x4**2) -(y4**2) +(x5**2) + (y5**2)

            Ax1 = (2*(x5-x1))
            Ax2 = (2*(x5-x2))
            Ax3 = (2*(x5-x3))
            Ax3 = (2*(x5-x4))
            
            Ay1 = (2*(y5-y1))
            Ay2 = (2*(y5-y2))
            Ay3 = (2*(y5-y3))
            Ay3 = (2*(y5-y4))

            Ax_array = np.array([Ax1, Ax2, Ax3, Ax4])
            Ay_array = np.array([Ay1, Ay2, Ay3, Ay4])
            B_array = np.array([b1, b2, b3, b4])
            A = np.vstack([Ax_array, Ay_array]).T
       
            x0, y0 = np.linalg.lstsq(A,B_array,rcond=None)[0]
            if(self.receivedMobileNodeNumber == 1):
                self.mobileNode1x = x0
                self.mobileNode1y = y0
            else:
                self.mobileNode2x = x0
                self.mobileNode2y = y0
        self.lock_rssi_coords.release()
        self.lock_data.release()


    def test(self):
        print(self.x0 * self.y0)
def perform_least_square(BaseProcessing):
    while True:
        BaseProcessing.lock_data.acquire()
        r1 = np.mean(BaseProcessing.rssi_data_samples_nodeA)
        r2 = np.mean(BaseProcessing.rssi_data_samples_nodeB)
        r3 = np.mean(BaseProcessing.rssi_data_samples_nodeC)
        r4 = np.mean(BaseProcessing.rssi_data_samples_nodeD)
        r5 = np.mean(BaseProcessing.rssi_data_samples_nodeE)
        r6 = np.mean(BaseProcessing.rssi_data_samples_nodeF)
        r7 = np.mean(BaseProcessing.rssi_data_samples_nodeG)
        r8 = np.mean(BaseProcessing.rssi_data_samples_nodeH)
        r9 = np.mean(BaseProcessing.rssi_data_samples_nodeI)
        r10 = np.mean(BaseProcessing.rssi_data_samples_nodeJ)
        r11= np.mean(BaseProcessing.rssi_data_samples_nodeK)
        r12= np.mean(BaseProcessing.rssi_data_samples_nodeL)
        x1 = BaseProcessing.beacon_xvals[0]
        x2 = BaseProcessing.beacon_xvals[1]
        x3 = BaseProcessing.beacon_xvals[2]
        x4 = BaseProcessing.beacon_xvals[3]
        x5 = BaseProcessing.beacon_xvals[4]
        x6 = BaseProcessing.beacon_xvals[5]
        x7 = BaseProcessing.beacon_xvals[6]
        x8 = BaseProcessing.beacon_xvals[7]
        x9 = BaseProcessing.beacon_xvals[8]
        x10 = BaseProcessing.beacon_xvals[9]
        x11 = BaseProcessing.beacon_xvals[10]
        x12 = BaseProcessing.beacon_xvals[11]
        y1 = BaseProcessing.beacon_yvals[0]
        y2 = BaseProcessing.beacon_yvals[1]
        y3 = BaseProcessing.beacon_yvals[2]
        y4 = BaseProcessing.beacon_yvals[3]
        y5 = BaseProcessing.beacon_yvals[4]
        y6 = BaseProcessing.beacon_yvals[5]
        y7 = BaseProcessing.beacon_yvals[6]
        y8 = BaseProcessing.beacon_yvals[7]
        y9 = BaseProcessing.beacon_yvals[8]
        y10 = BaseProcessing.beacon_yvals[9]
        y11 = BaseProcessing.beacon_yvals[10]
        y12 = BaseProcessing.beacon_yvals[11]
        # print("A")
        # print(BaseProcessing.rssi_data_samples_nodeA)
        # print("B")
        # print(BaseProcessing.rssi_data_samples_nodeB)
        # print("C")
        # print(BaseProcessing.rssi_data_samples_nodeC)
        # print("D")
        # print(BaseProcessing.rssi_data_samples_nodeD)
        # print("E")
        # print(BaseProcessing.rssi_data_samples_nodeE)
        # print("F")
        # print(BaseProcessing.rssi_data_samples_nodeF)
        # print("G")
        # print(BaseProcessing.rssi_data_samples_nodeG)
        # print("H")
        # print(BaseProcessing.rssi_data_samples_nodeH)
        # print("I")
        # print(BaseProcessing.rssi_data_samples_nodeI)
        # print("J")
        # print(BaseProcessing.rssi_data_samples_nodeJ)
        # print("K")
        # print(BaseProcessing.rssi_data_samples_nodeK)
        # print("L")
        # print(BaseProcessing.rssi_data_samples_nodeL)

        b1 = (r1**2) -(r12**2) - (x1**2) -(y1**2) +(x12**2) + (y12**2)
        b2 = (r2**2) -(r12**2) - (x2**2) -(y2**2) +(x12**2) + (y12**2)
        b3 = (r3**2) -(r12**2) - (x3**2) -(y3**2) +(x12**2) + (y12**2)
        b4 = (r4**2) -(r12**2) - (x4**2) -(y4**2) +(x12**2) + (y12**2)
        b5 = (r5**2) -(r12**2) - (x5**2) -(y5**2) +(x12**2) + (y12**2)
        b6 = (r6**2) -(r12**2) - (x6**2) -(y6**2) +(x12**2) + (y12**2)
        b7 = (r7**2) -(r12**2) - (x7**2) -(y7**2) +(x12**2) + (y12**2)
        b8 = (r8**2) -(r12**2) - (x8**2) -(y8**2) +(x12**2) + (y12**2)
        b9 = (r9**2) -(r12**2) - (x9**2) -(y9**2) +(x12**2) + (y12**2)
        b10 = (r10**2) -(r12**2) - (x10**2) -(y10**2) +(x12**2) + (y12**2)
        b11 = (r11**2) -(r12**2) - (x11**2) -(y11**2) +(x12**2) + (y12**2)
        Ax1 = (2*(x12-x1))
        Ax2 = (2*(x12-x2))
        Ax3 = (2*(x12-x3))
        Ax4 = (2*(x12-x4))
        Ax5 = (2*(x12-x5))
        Ax6 = (2*(x12-x6))
        Ax7 = (2*(x12-x7))
        Ax8 = (2*(x12-x8))
        Ax9 = (2*(x12-x9))
        Ax10 = (2*(x12-x10))
        Ax11 = (2*(x12-x11))
        Ay1 = (2*(y12-y1))
        Ay2 = (2*(y12-y2))
        Ay3 = (2*(y12-y3))
        Ay4 = (2*(y12-y4))
        Ay5 = (2*(y12-y5))
        Ay6 = (2*(y12-y6))
        Ay7 = (2*(y12-y7))
        Ay8 = (2*(y12-y8))
        Ay9 = (2*(y12-y9))
        Ay10 = (2*(y12-y10))
        Ay11 = (2*(y12-y11))
        Ax_array = np.array([Ax1, Ax2, Ax3, Ax4, Ax5, Ax6, Ax7, Ax8, Ax9, Ax10, Ax11])
        Ay_array = np.array([Ay1, Ay2, Ay3, Ay4, Ay5, Ay6, Ay7, Ay8, Ay9, Ay10, Ay11])
        B_array = np.array([b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11])
        A = np.vstack([Ax_array, Ay_array]).T
        BaseProcessing.lock_rssi_coords.acquire()
        x0, y0 = np.linalg.lstsq(A,B_array,rcond=None)[0]
        if(x0 > 4):
            x0 = 4
        if(x0 < 0):
            x0 = 0
        if(y0 > 4):
            y0 = 4
        if(y0 < 0):
            y0 = 0
        #FIXME 
        x0 = random.randint(15,25)
        y0 = random.randint(8,10)

        BaseProcessing.x0 = x0
        BaseProcessing.y0 = y0

        BaseProcessing.mobileNode1x = random.randint(5,15)
        BaseProcessing.mobileNode1y = random.randint(8,10)
        print("Coords:%f,%f" % (BaseProcessing.x0, BaseProcessing.y0))
        print("MobileNode1:%f,%f" % (BaseProcessing.mobileNode1x, BaseProcessing.mobileNode1y))
        
        BaseProcessing.lock_data.release()
        BaseProcessing.lock_rssi_coords.release()
        print("inside Least squartes")
        time.sleep(1)




def update_data(BaseProcessing):
    while True:
        
        if not (BaseProcessing.isConnected):
            try:
                    #print("HER")
                    # setup serial port
                    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
                    BaseProcessing.serial_port = ser
                    # change connection status
                    BaseProcessing.lock_connection_status.acquire()
                    BaseProcessing.isConnected = 1
                    BaseProcessing.lock_connection_status.release()

            except Exception:
                #print("ERROR1")
                pass
        else:
            try:

                output = BaseProcessing.serial_port.readline()
                #print(output)
                output_converted = ((str(output, 'utf-8'))[:-1])
                if len(output_converted) > 1:
                    print(output_converted)
                    #output_converted = '{"Node": 52, "rssi":93}'
                    #output_converted = '{"Node": 52, "rssi":93, "ultra":53}'

                    #format = 
                    outputs = output_converted[1:-1].replace(" ", "").split(",")
                    # 3 rssi readings
                    BaseProcessing.lock_data.acquire()
                    if(len(outputs) == 4):
                       
                        mobile_node_number = int((outputs[0])[-1:])
                        BaseProcessing.receivedMobileNodeNumber = mobile_node_number
                        BaseProcessing.size_received_beacons =(len(outputs) -1)
                        #nodeValues are "0-11"
                        nodeInfo1 = outputs[1].split(":")
                        nodeInfo2 = outputs[2].split(":")
                        nodeInfo3 = outputs[3].split(":")
                        
                        nodeNumber1 = int(nodeInfo1[0])
                        nodeNumber2 = int(nodeInfo2[0])
                        nodeNumber3 = int(nodeInfo3[0])
                        BaseProcessing.receivedXPositions[0] = self.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = self.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = self.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = self.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = self.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = self.beacon_yvals[nodeNumber3]
                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        receivedRSSI_actual1 = 10**((-53-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-53-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-53-float(receivedRSSI3))/ (10*2))


                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        BaseProcessing.calculate_positions()
                        
                    if(len(outputs) == 5):
                        
                        mobile_node_number = int((outputs[0])[-1:])
                        BaseProcessing.receivedMobileNodeNumber = mobile_node_number
                        BaseProcessing.size_received_beacons =(len(outputs) -1)
                        #nodeValues are "0-11"
                        nodeInfo1 = outputs[1].split(":")
                        nodeInfo2 = outputs[2].split(":")
                        nodeInfo3 = outputs[3].split(":")
                        nodeInfo4 = outputs[4].split(":")
                        nodeNumber1 = int(nodeInfo1[0])
                        nodeNumber2 = int(nodeInfo2[0])
                        nodeNumber3 = int(nodeInfo3[0])
                        nodeNumber4 = int(nodeInfo4[0])
                        BaseProcessing.receivedXPositions[0] = self.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = self.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = self.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = self.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = self.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = self.beacon_yvals[nodeNumber3]
                        BaseProcessing.receivedXPositions[3] = self.beacon_xvals[nodeNumber4]
                        BaseProcessing.receivedYPositions[3] = self.beacon_yvals[nodeNumber4]



                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        receivedRSSI4= (nodeInfo4[1])
                        receivedRSSI_actual1 = 10**((-53-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-53-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-53-float(receivedRSSI3))/ (10*2))
                        receivedRSSI_actual4 = 10**((-53-float(receivedRSSI4))/ (10*2))

                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        BaseProcessing.receivedRSSI[3] = receivedRSSI_actual4

                        BaseProcessing.calculate_positions()


                        
                    if(len(outputs) == 6):
                        
                        mobile_node_number = int((outputs[0])[-1:])
                        BaseProcessing.receivedMobileNodeNumber = mobile_node_number
                        BaseProcessing.size_received_beacons =(len(outputs) -1)
                        #nodeValues are "0-11"
                        nodeInfo1 = outputs[1].split(":")
                        nodeInfo2 = outputs[2].split(":")
                        nodeInfo3 = outputs[3].split(":")
                        nodeInfo4 = outputs[4].split(":")
                        nodeInfo5 = outputs[5].split(":")
                        nodeNumber1 = int(nodeInfo1[0])
                        nodeNumber2 = int(nodeInfo2[0])
                        nodeNumber3 = int(nodeInfo3[0])
                        nodeNumber4 = int(nodeInfo4[0])
                        nodeNumber5 = int(nodeInfo5[0])
                        BaseProcessing.receivedXPositions[0] = self.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = self.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = self.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = self.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = self.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = self.beacon_yvals[nodeNumber3]
                        BaseProcessing.receivedXPositions[3] = self.beacon_xvals[nodeNumber4]
                        BaseProcessing.receivedYPositions[3] = self.beacon_yvals[nodeNumber4]
                        BaseProcessing.receivedXPositions[4] = self.beacon_xvals[nodeNumber5]
                        BaseProcessing.receivedYPositions[4] = self.beacon_yvals[nodeNumber5]


                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        receivedRSSI4= (nodeInfo4[1])
                        receivedRSSI5= (nodeInfo5[1])
                        receivedRSSI_actual1 = 10**((-53-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-53-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-53-float(receivedRSSI3))/ (10*2))
                        receivedRSSI_actual4 = 10**((-53-float(receivedRSSI4))/ (10*2))
                        receivedRSSI_actual5 = 10**((-53-float(receivedRSSI5))/ (10*2))

                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        BaseProcessing.receivedRSSI[3] = receivedRSSI_actual4

                        BaseProcessing.receivedRSSI[4] = receivedRSSI_actual5




                        BaseProcessing.calculate_positions()
                    BaseProcessing.lock_data.release()
                            

               






                    
                    # if(len(outputs) == 3):
                    #     node = outputs[0].split(":")[1]
                    #     time1 = datetime.utcnow()

                    #     time2 = (calendar.timegm(time1.utctimetuple()) * (10**9))
                    #     strtime = str(time2)
                    #     rssi = outputs[1].split(":")[1]
                    #     ultra = outputs[2].split(":")[1]
                    #     if(node =='12'):
                            
                    #         result_rssi = ("StaticNodeA" + (" RSSI=\"%s\" " %rssi) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_rssi)
                    #         result_ultra = ("StaticNodeA" + (" Ultrasound=\"%s\" " %ultra) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_ultra)
                    #     elif(node =='13'):
                    #         result_rssi = ("StaticNodeB" + (" RSSI=\"%s\" " %rssi) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_rssi)
                    #         result_ultra = ("StaticNodeB" + (" Ultrasound=\"%s\" " %ultra) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_ultra)
                    #     elif(node =='14'):
                    #         result_rssi = ("StaticNodeC" + (" RSSI=\"%s\" " %rssi) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_rssi)
                    #         result_ultra = ("StaticNodeC" + (" Ultrasound=\"%s\" " %ultra) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_ultra)
                    #     elif(node =='15'):
                    #         result_rssi = ("StaticNodeD" + (" RSSI=\"%s\" " %rssi) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_rssi)
                    #         result_ultra = ("StaticNodeD" + (" Ultrasound=\"%s\" " %ultra) + strtime + '\n')
                    #         BaseProcessing.writeFile.write(result_ultra)
                        
                    #     BaseProcessing.writeFile.flush()




            except Exception as e:
                #print(output_converted)
                #print("Hit ERROR2")
                print(e)
                
                BaseProcessing.serial_port.close()
                BaseProcessing.lock_connection_status.acquire()
                BaseProcessing.isConnected = 0
                BaseProcessing.lock_connection_status.release()
                pass
        time.sleep(0.001)
    
def updateGraph(BaseProcessing):
    pass

def main():
    
    baseProgram = BaseProcessing()
    
    reading_thr = threading.Thread(target = update_data, args=(baseProgram,))
    reading_thr.setDaemon(True)
    reading_thr.start()
    lsq_thr = threading.Thread(target = perform_least_square, args=(baseProgram,))
    lsq_thr.setDaemon(True)
    #lsq_thr.start()
    #updategraph_thr = threading.Thread(target = updateGraph, args=(baseProgram,))
    #updategraph_thr.setDaemon(True)
    #updategraph_thr.start()
    baseProgram.animate()
    plt.show()

    while True:
        time.sleep(100)
        
    #     #update_data(baseProgram)





if __name__ == '__main__':
    main()