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



我有一份代码在本机运行无误，移植到另一台ubuntu时出现报错，" 'serial' has no attribute 'Serial' "，这是为什么呢我明明已经装了pyserial和serial包

[这个问题可能是由于你安装了名为`serial`的包，而不是你需要的`pyserial`包](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[1](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[2](https://stackoverflow.com/questions/53834584/serial-has-no-attribute-serial-error-not-filename-or-import-method)[3](https://stackabuse.com/bytes/fix-the-attributeerror-module-object-has-no-attribute-serial-error-in-python/)[。这两个包是不同的，`Serial`属性是`pyserial`模块的一部分，用于Python中的串行通信](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[3](https://stackabuse.com/bytes/fix-the-attributeerror-module-object-has-no-attribute-serial-error-in-python/)。

[此外，如果你的本地文件名为`serial.py`，那么在导入`pyserial`模块时，可能会导入你自己的文件，而不是系统级的`pyserial`模块](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[1](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[4](https://bobbyhadz.com/blog/python-attributeerror-module-serial-has-no-attribute-serial)。

[你可以尝试以下步骤来解决这个问题](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[1](https://stackoverflow.com/questions/41199876/attributeerror-module-serial-has-no-attribute-serial)[2](https://stackoverflow.com/questions/53834584/serial-has-no-attribute-serial-error-not-filename-or-import-method)[5](https://blog.csdn.net/irober/article/details/107088237)[3](https://stackabuse.com/bytes/fix-the-attributeerror-module-object-has-no-attribute-serial-error-in-python/)：

1. 卸载`serial`包：`pip uninstall serial`
2. 卸载`pyserial`包：`pip uninstall pyserial`
3. 重新安装`pyserial`包：`pip install pyserial`