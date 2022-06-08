import time
import serial
import threading
import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib import animation
import math
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
        #beacon positions
        self.beacon_xvals = np.empty(13)
        self.beacon_yvals = np.empty(13)


        self.size_received_beacons = 0
        #mobile x and y coordinates
        self.mobileNode1x = 0
        self.mobileNode1y = 0
        self.mobileNode2x = 0
        self.mobileNode2y = 0
        # x and y positions of nodes that have sent rssi data
        self.receivedXPositions = np.ones(5)
        self.receivedYPositions = np.ones(5)
        # received rssi data values
        self.receivedRSSI = np.ones(5)
        # last mobile device to transmit
        self.receivedMobileNodeNumber = 0
        self.fig, self.ax = plt.subplots()
        img = plt.imread("floorplan.PNG")
        self.ax.imshow(img, extent=[0,29.9,0,27.6])

        self.mobilenode1, = self.ax.plot([],[],'go', markersize='12')
        self.mobilenode2, = self.ax.plot([],[],'ro', markersize='12')
        
        self.ax.axis([0, 32, 0, 21])
        self.setup_beacons()
        self.ax.plot(self.beacon_xvals,self.beacon_yvals,'bo', markersize='4')
        self.writeFile = open('resultsStatic.csv','w')
        self.tempcount =0

    #Get beacon x and y positions from the csv
    def setup_beacons(self):

        csv_reader = csv.reader(open('BeaconData.csv'), delimiter=',')
        line_count = 0
        for row in csv_reader:
            
            self.beacon_xvals[line_count] = row[1]
            self.beacon_yvals[line_count] = row[2]
            line_count += 1
        
    # update the location of mobile positions on plot
    def update(self,extra):
        self.lock_rssi_coords.acquire()
        self.mobilenode1.set_data(self.mobileNode1x, self.mobileNode1y)
        self.mobilenode2.set_data(self.mobileNode2x, self.mobileNode2y)
        self.fig.suptitle(" MobileNode1:(%f,%f), MobileNode2:(%f,%f)" %(self.mobileNode1x, self.mobileNode1y, self.mobileNode2x, self.mobileNode2y))

        self.lock_rssi_coords.release()
        
    #initialize matlib plot animation
    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.update,interval=400, blit=False)

    # calculte position of mobile based on variable number of rssi values
    def calculate_positions(self):
        self.lock_rssi_coords.acquire()
        # calculate positions if 3 recieved rssi values
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

            tempIndex = 7
            for value in range(3):
                if(self.receivedRSSI[value] < 0.32):
                    tempIndex = value

            if (tempIndex != 7):
                nodeX = self.receivedXPositions[tempIndex]
                nodeY = self.receivedYPositions[tempIndex]
                deltaX = abs(x0 - nodeX)
                deltaY = abs(y0 - nodeY)
                smallX = deltaX * 0.5 / math.sqrt((x0 - nodeX)**2 + (y0 - nodeY)**2)
                smallY = deltaX * 0.5 / math.sqrt((x0 - nodeX)**2 + (y0 - nodeY)**2)
                x0 = smallX + nodeX
                y0 = smallY + nodeY


            #ADD KNN

            if (x0 < 2):
                x0 = 2
            if (y0 < 4.1):
                y0 = 4.1


            if(self.receivedMobileNodeNumber == 1):
                self.mobileNode1x = x0
                self.mobileNode1y = y0
            else:
                self.mobileNode2x = x0
                self.mobileNode2y = y0
        # calculate positions if 4 recieved rssi values
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

            tempIndex = 7
            for value in range(4):
                if(self.receivedRSSI[value] < 0.32):
                    tempIndex = value
                    print("entered")

            if (tempIndex != 7):
                nodeX = self.receivedXPositions[tempIndex]
                nodeY = self.receivedYPositions[tempIndex]
                deltaX = abs(x0 - nodeX)
                deltaY = abs(y0 - nodeY)
                smallX = deltaX * 0.5 / math.sqrt(((x0 - nodeX)**2) + ((y0 - nodeY)**2))
                smallY = deltaX * 0.5 / math.sqrt(((x0 - nodeX)**2) + ((y0 - nodeY)**2))
                x0 = smallX + nodeX
                y0 = smallY + nodeY
            #ADD KNN

            if (x0 < 2):
                x0 = 2
            if (y0 < 4.1):
                y0 = 4.1


            if(self.receivedMobileNodeNumber == 1):
                self.mobileNode1x = x0
                self.mobileNode1y = y0
            else:
                self.mobileNode2x = x0
                self.mobileNode2y = y0
        # calculate positions if 5 recieved rssi values
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
            Ax4 = (2*(x5-x4))
            
            Ay1 = (2*(y5-y1))
            Ay2 = (2*(y5-y2))
            Ay3 = (2*(y5-y3))
            Ay4 = (2*(y5-y4))

            Ax_array = np.array([Ax1, Ax2, Ax3, Ax4])
            Ay_array = np.array([Ay1, Ay2, Ay3, Ay4])
            B_array = np.array([b1, b2, b3, b4])
            A = np.vstack([Ax_array, Ay_array]).T
       
            x0, y0 = np.linalg.lstsq(A,B_array,rcond=None)[0]

            tempIndex = 7
            for value in range(5):
                if(self.receivedRSSI[value] < 0.32):
                    tempIndex = value

            if (tempIndex != 7):
                nodeX = self.receivedXPositions[tempIndex]
                nodeY = self.receivedYPositions[tempIndex]
                deltaX = abs(x0 - nodeX)
                deltaY = abs(y0 - nodeY)
                smallX = deltaX * 0.5 / math.sqrt((x0 - nodeX)**2 + (y0 - nodeY)**2)
                smallY = deltaX * 0.5 / math.sqrt((x0 - nodeX)**2 + (y0 - nodeY)**2)
                x0 = smallX + nodeX
                y0 = smallY + nodeY
            #ADD KNN

            if (x0 < 2):
                x0 = 2
            if (y0 < 4.1):
                y0 = 4.1
            

            if(self.receivedMobileNodeNumber == 1):
                self.mobileNode1x = x0
                self.mobileNode1y = y0
            else:
                self.mobileNode2x = x0
                self.mobileNode2y = y0
        self.lock_rssi_coords.release()
        #self.lock_data.release()


    def test(self):
        print(self.x0 * self.y0)




def update_data(BaseProcessing):
    while True:
        
        if not (BaseProcessing.isConnected):
            try:
                    
                    # setup serial port
                    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
                    BaseProcessing.serial_port = ser
                    # change connection status
                    BaseProcessing.lock_connection_status.acquire()
                    BaseProcessing.isConnected = 1
                    BaseProcessing.lock_connection_status.release()

            except Exception:
                
                pass
        else:
            try:
                
                output = BaseProcessing.serial_port.readline()
                
                output_converted = ((str(output, 'utf-8'))[:-1])
                #FIXME REMOVE WHEN NOT TESTING
                #output_converted = '{"Mobile2, 2:-75, 4:-69, 5:-67, 10:-73,12:-59,}'
                #output_converted = '{"Mobile2, 2:-75, 4:-69, 5:-67, 10:-73,}'
                #output_converted = '{"Mobile2, 2:-75, 4:-69, 5:-60}'
                if len(output_converted) > 1:
                    print(output_converted)
                    
                    
                    outputs = output_converted[1:-2].replace(" ", "").split(",")
                    #print(outputs)
                    #print(len(outputs))
                    # 3 rssi readings
                    
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
                        # assign x and y position of nodes that sent data
                        BaseProcessing.receivedXPositions[0] = BaseProcessing.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = BaseProcessing.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = BaseProcessing.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = BaseProcessing.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = BaseProcessing.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = BaseProcessing.beacon_yvals[nodeNumber3]
                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        #convert rssi db to distance
                        receivedRSSI_actual1 = 10**((-55-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-55-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-55-float(receivedRSSI3))/ (10*2))

                        
                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        BaseProcessing.calculate_positions()
                    # recieved 4 rssi info
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
                        # assign x and y position of nodes that sent data
                        BaseProcessing.receivedXPositions[0] = BaseProcessing.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = BaseProcessing.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = BaseProcessing.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = BaseProcessing.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = BaseProcessing.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = BaseProcessing.beacon_yvals[nodeNumber3]
                        BaseProcessing.receivedXPositions[3] = BaseProcessing.beacon_xvals[nodeNumber4]
                        BaseProcessing.receivedYPositions[3] = BaseProcessing.beacon_yvals[nodeNumber4]

                        # assign rssi info
                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        receivedRSSI4= (nodeInfo4[1])
                        # convert rssi db to distance
                        receivedRSSI_actual1 = 10**((-55-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-55-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-55-float(receivedRSSI3))/ (10*2))
                        receivedRSSI_actual4 = 10**((-55-float(receivedRSSI4))/ (10*2))

                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        BaseProcessing.receivedRSSI[3] = receivedRSSI_actual4
                        BaseProcessing.calculate_positions()

                    # recieved 5 rssi info
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
                        # assign x and y positions of nodes that sent rssi
                        BaseProcessing.receivedXPositions[0] = BaseProcessing.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = BaseProcessing.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = BaseProcessing.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = BaseProcessing.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = BaseProcessing.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = BaseProcessing.beacon_yvals[nodeNumber3]
                        BaseProcessing.receivedXPositions[3] = BaseProcessing.beacon_xvals[nodeNumber4]
                        BaseProcessing.receivedYPositions[3] = BaseProcessing.beacon_yvals[nodeNumber4]
                        BaseProcessing.receivedXPositions[4] = BaseProcessing.beacon_xvals[nodeNumber5]
                        BaseProcessing.receivedYPositions[4] = BaseProcessing.beacon_yvals[nodeNumber5]

                        # assign rssi info
                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        receivedRSSI4= (nodeInfo4[1])
                        receivedRSSI5= (nodeInfo5[1])
                        #convert rssi db to distance
                        receivedRSSI_actual1 = 10**((-55-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-55-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-55-float(receivedRSSI3))/ (10*2))
                        receivedRSSI_actual4 = 10**((-55-float(receivedRSSI4))/ (10*2))
                        receivedRSSI_actual5 = 10**((-55-float(receivedRSSI5))/ (10*2))


                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        BaseProcessing.receivedRSSI[3] = receivedRSSI_actual4

                        BaseProcessing.receivedRSSI[4] = receivedRSSI_actual5



                        BaseProcessing.calculate_positions()
                                                
            except Exception as e:

                
                
                BaseProcessing.serial_port.close()
                BaseProcessing.lock_connection_status.acquire()
                BaseProcessing.isConnected = 0
                BaseProcessing.lock_connection_status.release()
                pass
        time.sleep(0.001)
    
def updateGraph(BaseProcessing):
    pass

def main():
    time.sleep(4)
    baseProgram = BaseProcessing()
    #thread to process messages and update mobile locations
    reading_thr = threading.Thread(target = update_data, args=(baseProgram,))
    reading_thr.setDaemon(True)
    reading_thr.start()
    baseProgram.animate()
    plt.show()

    while True:
        time.sleep(100)
        






if __name__ == '__main__':
    main()