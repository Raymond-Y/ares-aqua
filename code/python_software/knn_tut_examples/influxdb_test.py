from datetime import datetime
import random
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# You can generate an API token from the "API Tokens Tab" in the UI
token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
org1 = "s4589619@student.uq.edu.au"
bucket1 = "Test2"
url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"
client = influxdb_client.InfluxDBClient(url=url1, token=token1, org=org1)    

write_api = client.write_api(write_options=SYNCHRONOUS)
data = "node1,host=host1 used_percent=23.43234543"
#x = random.uniform(0,10,2)
#y = random.uniform(11,20,2)
#actual_val = str(x) + '-' + str(y)

for i in range(10):
    x = random.uniform(0,10)
    y = random.uniform(11,20)
    actual_val = str(x) + '-' + str(y)
    print(actual_val)
    point = Point("Point") \
            .field("x", x) \
            .field("y", y) \
            .field("actual_value", actual_val) \
            .time(datetime.utcnow(), WritePrecision.NS)
                    #.field("LOL", 231.2) \
    write_api.write(bucket1, org1, point)





