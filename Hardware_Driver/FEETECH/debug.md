when adding python environment path, we should directly add the folder that contains the lib. For example
```
export PYTHONPATH="${PYTHONPATH}:/home/leonaruic/D-Gripper/D_Gripper/Hardware_Driver/FEETECH/SCServo_Python"
```
if I want to use the scservo_sdk.


If I've installed the pyserial lib (contains serial moudle), but the interpreter still can't find it, maybe there's diferent interpreters in my computer.
I should use the python interpreter associated with the pip command, for example
```
python3 read_write.py
```
instead of 
```
/usr/bin/python3 read_write.py
```

finally, don't forget to give the permission to te port
```
sudo chmod a+rw /dev/ttyUSB0
```