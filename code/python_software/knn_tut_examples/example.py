#https://dganais.medium.com/getting-started-with-python-and-influxdb-v2-0-f22e5175aba5
from influxdb_client import InfluxDBClient
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
#org = "my-org"
#bucket = "my-bucket"
#$token = $my-token
token1 = "cBKaA_KGTrY2i6_7NiYqhAnRzSTKK_d7jhfXSuNHrZaYUn3VnjMkrSeHmW_p8aEwOKPy_YACgXJH9vkLxB7MjA=="
org1 = "s4589619@student.uq.edu.au"
bucket1 = "Test2"
url1 = "https://us-east-1-1.aws.cloud2.influxdata.com"


query = 'from(bucket: "Test2")\
|> range(start: -10m)\
|> filter(fn: (r) => r._measurement == "h2o_level")\
|> filter(fn: (r) => r._field == "water_level")\
|> filter(fn: (r) => r.location == "coyote_creek")'#establish a connection
client = InfluxDBClient(url=url1, token=token1, org=org1)#instantiate the WriteAPI and QueryAPI
write_api = client.write_api(write_options=SYNCHRONOUS)
query_api = client.query_api()#create and write the point
p = Point("h2o_level").tag("location", "coyote_creek").field("water_level", 1)
write_api.write(bucket=bucket1,org=org1,record=p)
#return the table and print the result
result = client.query_api().query(org=org1, query=query)
results = []
for table in result:
    for record in table.records:
        results.append((record.get_value(), record.get_field()))
print(results)