# ares-aqua
CSSE4011 ares-aqua project

#software library- python instructions

Libraries required 
- Pandas Library - "pip install pandas"
- InfluxDb Lib - "pip install influxdb"
- InfluxDb client - "pip install influxdb-client"
- Numpy lib - "pip install numpy"
- Pyserial library - "pip install pyserial"

Python files and their functionality
- "newProject.py" - runs the python localisation gui 
- "project_training_collect.py" - Python program uploads training data to influxDB. Must change the "self.actual_position" value on line 66 to correct 
values of position that you are testing. 
- "download_influxdb_training_data.py" - Python program which downloads all training data from influxdB into csv. 

Embedded Files
- base_mesh_node - subsribes to mobile node publication (0xC000 - Mobile A and 0xC002 - Mobile B). Publishes to 0xC003 to send proximity alerts to mobile nodes
- mobile_mesh_nodes - both mobile nodes publish data to base node with group ID's as described above
- static_mesh_relays - relay data over the network from the base and mobile nodes. Also advertise data to mobile nodes with TTL of 1

Build instructions

Mobile

west build -p auto -b thingy52_nrf52832 -- -DMobile_A=1   (replace A with mobile node)

Static

west build -p auto -b particle_argon -- -DX=1   (replace X with particle argon (X, W, Y, Z))

