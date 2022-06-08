#https://docs.influxdata.com/influxdb/cloud/query-data/common-queries/multiple-fields-in-calculations/
from datetime import datetime
import random
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client import InfluxDBClient, Point, Dialect

token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
org1 = "s4589619@student.uq.edu.au"
bucket1 = "Test2"
url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"
#client = influxdb_client.InfluxDBClient(url=url1, token=token1, org=org1)    
client = influxdb_client.InfluxDBClient(url=url1, token=token1, org=org1)    

query_api = client.query_api()
#working for x vals
# query = ' from(bucket:"Test2")\
# |> range(start: -6h)\
# |> filter(fn:(r) => r._measurement == "Point")\
# |> filter(fn:(r) => r._field == "x" )'

# query = ' from(bucket:"Test2")\
# |> range(start: -6h)\
# |> filter(fn:(r) => r._measurement == "Point")\
# |> filter(fn:(r) => r._field == "x" or r._field == "y" or r._field == "actual_value" )'

# query = ' from(bucket:"Test2")\
# |> range(start: -6h)\
# |> filter(fn:(r) => r._measurement == "Point")\
# |> filter(fn:(r) => r._field == "x" or r._field == "y" or r._field == "actual_value" )\
# |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")'

# result = query_api.query(org=org1, query=query)
# results = []
# for table in result:
#     for row in table.records:
#         print((row))
#         # print(f'{row.values["_time"]}: host={row.values["host"]},device={row.values["device"]} '
#         #           f'{row.values["_value"]} °C')
#         #results.append((record.get_field(), record.get_value()))

# print(results)
# print(type(results))

# csv_result = query_api.query_csv(org=org1, query=query)
# val_count = 0
# for row in csv_result:
#     for cell in row:
#         val_count += 1
data_frame = query_api.query_data_frame('from(bucket:"Test2") '
                                        '|> range(start: -6h) '
                                        '|> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value") '
                                        '|> keep(columns: ["x", "y","actual_value"])')

print(data_frame.to_string())
print(type(data_frame))
