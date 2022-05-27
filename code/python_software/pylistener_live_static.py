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
        self.beacon_xvals = np.empty(12)
        self.beacon_yvals = np.empty(12)
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
        self.x0 = 0
        self.y0 = 0
        self.fig, self.ax = plt.subplots()
        self.graph, = self.ax.plot([], [],'ro')
        self.ax.axis([0, 4, 0, 4])
        self.setup_beacons()
        self.token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
        self.org1 = "s4589619@student.uq.edu.au"
        self.bucket1 = "staticNode"
        self.url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"
        self.client = influxdb_client.InfluxDBClient(url=self.url1, token=self.token1, org=self.org1) 
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS) 
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
        self.graph.set_data(self.x0,self.y0)
        self.fig.suptitle("At X:%f, Y:%f" %(self.x0, self.y0))
        self.lock_rssi_coords.release()
        

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.update,interval=200, blit=False)

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
        # x0 = random.randint(0,4)
        # y0 = random.randint(0,4)

        BaseProcessing.x0 = x0
        BaseProcessing.y0 = y0
    
        print("Coords:%f,%f" % (BaseProcessing.x0, BaseProcessing.y0))
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

                    #print("OUtputCOnver" + output_converted)
                    outputs = output_converted[1:-1].replace(" ", "").split(",")
                    #print(outputs)
                    if(len(outputs) == 2):
                        #print(outputs)
                        node = outputs[0].split(":")[1]
                        #print("Node:" + node + "!")
                        rssi_db = outputs[1].split(":")[1]
                        # rssi = 10**((-52-float(rssi_db))/ (10*4))
                        rssi = 10**((-53-float(rssi_db))/ (10*2))
                        BaseProcessing.lock_data.acquire()

                        if(node =='0'):
                            #print("RNODE_A")
                            BaseProcessing.rssi_data_samples_nodeA[BaseProcessing.nodeA_index] = rssi
                            BaseProcessing.nodeA_index = (BaseProcessing.nodeA_index + 1) % ARRAY_SIZE
                            #print("INDEXA")
                            #print(BaseProcessing.nodeA_index)
                        elif(node =='1'):
                            BaseProcessing.rssi_data_samples_nodeB[BaseProcessing.nodeB_index] = rssi
                            BaseProcessing.nodeB_index = (BaseProcessing.nodeB_index + 1) % ARRAY_SIZE
                        elif(node =='2'):
                            BaseProcessing.rssi_data_samples_nodeC[BaseProcessing.nodeC_index] = rssi
                            BaseProcessing.nodeC_index = (BaseProcessing.nodeC_index + 1) % ARRAY_SIZE
                        elif(node =='3'):
                            BaseProcessing.rssi_data_samples_nodeD[BaseProcessing.nodeD_index] = rssi
                            BaseProcessing.nodeD_index = (BaseProcessing.nodeD_index + 1) % ARRAY_SIZE
                        elif(node =='4'):
                            BaseProcessing.rssi_data_samples_nodeE[BaseProcessing.nodeE_index] = rssi
                            BaseProcessing.nodeE_index = (BaseProcessing.nodeE_index + 1) % ARRAY_SIZE
                        elif(node =='5'):
                            BaseProcessing.rssi_data_samples_nodeF[BaseProcessing.nodeF_index] = rssi
                            BaseProcessing.nodeF_index = (BaseProcessing.nodeF_index + 1) % ARRAY_SIZE
                        elif(node =='6'):
                            BaseProcessing.rssi_data_samples_nodeG[BaseProcessing.nodeG_index] = rssi
                            BaseProcessing.nodeG_index = (BaseProcessing.nodeG_index + 1) % ARRAY_SIZE
                        elif(node =='7'):
                            BaseProcessing.rssi_data_samples_nodeH[BaseProcessing.nodeH_index] = rssi
                            BaseProcessing.nodeH_index = (BaseProcessing.nodeH_index + 1) % ARRAY_SIZE
                        elif(node =='8'):
                            BaseProcessing.rssi_data_samples_nodeI[BaseProcessing.nodeI_index] = rssi
                            BaseProcessing.nodeI_index = (BaseProcessing.nodeI_index + 1) % ARRAY_SIZE
                        elif(node =='9'):
                            BaseProcessing.rssi_data_samples_nodeJ[BaseProcessing.nodeJ_index] = rssi
                            BaseProcessing.nodeJ_index = (BaseProcessing.nodeJ_index + 1) % ARRAY_SIZE
                        elif(node =='10'):
                            BaseProcessing.rssi_data_samples_nodeK[BaseProcessing.nodeK_index] = rssi
                            BaseProcessing.nodeK_index = (BaseProcessing.nodeK_index + 1) % ARRAY_SIZE
                        elif(node =='11'):
                            BaseProcessing.rssi_data_samples_nodeL[BaseProcessing.nodeL_index] = rssi
                            BaseProcessing.nodeL_index = (BaseProcessing.nodeL_index + 1) % ARRAY_SIZE
                        BaseProcessing.lock_data.release()


                
                    if(len(outputs) == 3):
                        
                       
                        print("UPLOADING")
                        node = outputs[0].split(":")[1]
                        rssi = outputs[1].split(":")[1]
                        ultra = outputs[2].split(":")[1]
                        if(node =='12'):
                            point = Point("StaticNodeA") \
                            .field("RSSI", rssi) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point)
                            point1 = Point("StaticNodeA") \
                            .field("Ultrasound", ultra) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point1)
                        elif(node == '13'):
                            point = Point("StaticNodeB") \
                            .field("RSSI", rssi) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point)
                            point1 = Point("StaticNodeB") \
                            .field("Ultrasound", ultra) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point1)
                        elif(node == '14'):
                            point = Point("StaticNodeC") \
                            .field("RSSI", rssi) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point)
                            point1 = Point("StaticNodeC") \
                            .field("Ultrasound", ultra) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point1)
                        elif(node == '15'):
                            point = Point("StaticNodeD") \
                            .field("RSSI", rssi) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point)
                            point1 = Point("StaticNodeD") \
                            .field("Ultrasound", ultra) \
                            .time(datetime.utcnow(), WritePrecision.NS)
                            BaseProcessing.write_api.write(BaseProcessing.bucket1, BaseProcessing.org1, point1)
                    #print(node)
                    #print(rssi)



            except Exception as e:
                #print(output_converted)
                #print("Hit ERROR2")
                #print(e)
                
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
    lsq_thr.start()
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