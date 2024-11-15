The GPS package contains a node that gets data from a software serial connection to the GNSS that then publishes it over the gps_data topic.

The custom_msgs package contains the custom msg that the topic publishes with.

The .ino file contains the arduino script that gets data from the GNSS and then prints it over the serial port.

The pyserial library is required to be installed for the GPS package to work.
