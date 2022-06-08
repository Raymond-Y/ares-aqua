#https://docs.influxdata.com/influxdb/cloud/query-data/common-queries/multiple-fields-in-calculations/
from datetime import datetime
import random
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
org1 = "s4589619@student.uq.edu.au"
bucket1 = "Test2"
url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"
#client = influxdb_client.InfluxDBClient(url=url1, token=token1, org=org1)    
client = influxdb_client.InfluxDBClient(url=url1, token=token1, org=org1)    

query_api = client.query_api()
query = ' from(bucket:"Test2")\
|> range(start: -6h)\
|> filter(fn:(r) => r._measurement == "Point")\
|> filter(fn:(r) => r._field == "x" )'
result = query_api.query(org=org1, query=query)
results = []
for table in result:
    for record in table.records:
        results.append((record.get_field(), record.get_value()))

print(results)
print(type(results))