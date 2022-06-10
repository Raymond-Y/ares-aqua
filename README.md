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
