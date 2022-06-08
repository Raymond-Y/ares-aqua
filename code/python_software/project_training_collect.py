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
        # self.x0 = 3
        # self.y0 = 2
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
        
        #self.ax.axis([0, 41, 0, 21])
        self.ax.axis([0, 32, 0, 21])
        self.setup_beacons()
        self.ax.plot(self.beacon_xvals,self.beacon_yvals,'bo', markersize='4')
        self.writeFile = open('resultsStatic.csv','w')
        self.tempcount =0
        self.token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
        self.org1 = "s4589619@student.uq.edu.au"
        self.bucket1 = "TrainingData"
        self.url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"
        self.client = influxdb_client.InfluxDBClient(url=self.url1, token=self.token1, org=self.org1) 
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS) 
        # actual position measured to collect training data in form "x-y"
        self.actual_position = "1-2.5"
        
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
        #  BaseProcessing.mobileNode1x = random.randint(5,15)
        # BaseProcessing.mobileNode1y = random.randint(8,10)
        # print("Coords:%f,%f" % (BaseProcessing.x0, BaseProcessing.y0))
        # print("MobileNode1:%f,%f" % (BaseProcessing.mobileNode1x, BaseProcessing.mobileNode1y))
        self.mobilenode1.set_data(self.mobileNode1x, self.mobileNode1y)
        self.mobilenode2.set_data(self.mobileNode2x, self.mobileNode2y)
        self.fig.suptitle(" MobileNode1:(%f,%f), MobileNode2:(%f,%f)" %(self.mobileNode1x, self.mobileNode1y, self.mobileNode2x, self.mobileNode2y))

        self.lock_rssi_coords.release()
        

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.update,interval=400, blit=False)


    def calculate_positions(self):
        self.lock_rssi_coords.acquire()
        #self.lock_data.acquire()
        #print("ENTER CALCULATE POSITIONS")
        #self.tempcount+= 1
        #print("Self count%i", self.tempcount)
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
                #print("trying to read")
                output = BaseProcessing.serial_port.readline()
                #print(output)
                output_converted = ((str(output, 'utf-8'))[:-1])
                #FIXME REMOVE WHEN NOT TESTING
                #output_converted = '{"Mobile2, 2:-75, 4:-69, 5:-67, 10:-73,12:-59,}'
                #output_converted = '{"Mobile2, 2:-75, 4:-69, 5:-67, 10:-73,}'
                #output_converted = '{"Mobile2, 2:-75, 4:-69, 5:-60}'
                if len(output_converted) > 1:
                    print(output_converted)
                    
                    #output_converted = '{"Node": 52, "rssi":93}'
                    #output_converted = '{"Node": 52, "rssi":93, "ultra":53}'
                    #output_converted = '{"Mobile1, 2:-75, 4:-69, 5:-67, 10:-73,12:-59,'
                    #format = 
                    outputs = output_converted[1:-2].replace(" ", "").split(",")
                    #print(outputs)
                    #print(len(outputs))
                    # 3 rssi readings
                    #BaseProcessing.lock_data.acquire()
                    if(len(outputs) == 4):
                        print("ACUUIRED 3 data")
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
                        BaseProcessing.receivedXPositions[0] = BaseProcessing.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = BaseProcessing.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = BaseProcessing.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = BaseProcessing.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = BaseProcessing.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = BaseProcessing.beacon_yvals[nodeNumber3]
                        receivedRSSI1 = (nodeInfo1[1])
                        receivedRSSI2 = (nodeInfo2[1])
                        receivedRSSI3= (nodeInfo3[1])
                        receivedRSSI_actual1 = 10**((-53-float(receivedRSSI1))/ (10*2))
                        receivedRSSI_actual2 = 10**((-53-float(receivedRSSI2))/ (10*2))
                        receivedRSSI_actual3 = 10**((-53-float(receivedRSSI3))/ (10*2))

                        #BaseProcessing.lock_data.release()
                        BaseProcessing.receivedRSSI[0] = receivedRSSI_actual1
                        BaseProcessing.receivedRSSI[1] = receivedRSSI_actual2
                        BaseProcessing.receivedRSSI[2] = receivedRSSI_actual3
                        print("END ACUUIRED 3 data")
                        BaseProcessing.calculate_positions()
                        
                    if(len(outputs) == 5):
                        print("ACUUIRED 4 data")
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

                        BaseProcessing.receivedXPositions[0] = BaseProcessing.beacon_xvals[nodeNumber1]
                        BaseProcessing.receivedYPositions[0] = BaseProcessing.beacon_yvals[nodeNumber1]
                        BaseProcessing.receivedXPositions[1] = BaseProcessing.beacon_xvals[nodeNumber2]
                        BaseProcessing.receivedYPositions[1] = BaseProcessing.beacon_yvals[nodeNumber2]
                        BaseProcessing.receivedXPositions[2] = BaseProcessing.beacon_xvals[nodeNumber3]
                        BaseProcessing.receivedYPositions[2] = BaseProcessing.beacon_yvals[nodeNumber3]
                        BaseProcessing.receivedXPositions[3] = BaseProcessing.beacon_xvals[nodeNumber4]
                        BaseProcessing.receivedYPositions[3] = BaseProcessing.beacon_yvals[nodeNumber4]


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
                        #BaseProcessing.lock_data.release()
                        BaseProcessing.calculate_positions()
                        print("END ACUUIRED 4 data")

                        
                    if(len(outputs) == 6):
                        print("ACUUIRED 5 data")
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

                        print("END ACUUIRED 5 data")

                        #BaseProcessing.lock_data.release()
                        BaseProcessing.calculate_positions()
                    
                            

               






                    
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
    time.sleep(4)
    baseProgram = BaseProcessing()
    
    reading_thr = threading.Thread(target = update_data, args=(baseProgram,))
    reading_thr.setDaemon(True)
    reading_thr.start()
    baseProgram.animate()
    plt.show()

    while True:
        time.sleep(100)
        
    #     #update_data(baseProgram)





if __name__ == '__main__':
    main()