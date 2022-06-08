from datetime import datetime
import random
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client import InfluxDBClient, Point, Dialect
#InfluxdB dashboard details
token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
org1 = "s4589619@student.uq.edu.au"
bucket1 = "TrainingData"
url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"
# influxdB client to perform queries
client = influxdb_client.InfluxDBClient(url=url1, token=token1, org=org1)    

query_api = client.query_api()
data_frame = query_api.query_data_frame('from(bucket:"TrainingData") '
                                        '|> range(start: -2d) '
                                        '|> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value") '
                                        '|> keep(columns: ["x_calc", "y_calc","actual_value"])')

#print out webdashboard training data to csv file
data_frame.to_csv('TrainingData.csv')