The GPS package contains a node that gets data from a software serial connection to the GNSS that then publishes it over the gps_data topic.

The custom_msgs package contains the custom msg that the topic publishes with.

The .ino file contains the arduino script that gets data from the GNSS and then prints it over the serial port.

The pyserial library is required to be installed for the GPS package to work.

There is also the custom_hardware package. It contains the basic outline for a hardware interface that can communicate over serial, which is best done with an arduino. Some configuration has to be done as in inputting the correct port that the arduino is on in the .cpp file. Custom configuration for the specific use case will need to be done. The hardware type is system and it is configured for writing to position and reading position.

The HW_interface arduino code is the code that interfaces wqith the custom_hardware package and will also need custom configuration for use. It will need to be combined with the Arm_Control INO file.
