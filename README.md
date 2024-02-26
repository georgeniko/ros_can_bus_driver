This section will briefly describe the core logic of the ”can to ros.cpp” and ”ros to can.cpp”
nodes in a ROS2 environment. This software was developed for the autonomous system of the Formula Student Team Aristurtle of the Aristotle University of Thessaloniki. It demonstrates the communication between the autonomous system and the vehicle's ECU.  

1. can to ros

After including all the needed libraries we should include the message files that we want to
publish. It is worth noting that the name of the included files is not determined by the actual .msg file that we created but by the name it is given after the “colcon build”. 

Afterwards it is useful to define the IDs of each message at the top, so as to be able to
change them quickly. In order to unpack the data we have to take into consideration the type
of the variables that we use. It is important to know each message type, in which byte each
variable starts and ends etc. That is why we need to take a look to dbc file or contact the person
who did the packing of these messages.

For instance if we know that a variable is signed and 16 bit long, the union that we should
use is the following:

![alt text](https://github.com/georgeniko/ros_can_driver/blob/main/evaluation.png?raw=true)
------------------------

Afterwards, we need to actually unpack the data. For this purpose I created some functions,
since the same thing had to be done several times. Of course, we cannot use the same unpacking
function for all the data. The choice depends again on the data type and length. The core
idea, however, is the same.

The return type of these functions is the same as the type of the actual variable. They have
3 parameters:
• the can data matrix, which is the frame that we read,
• the starting byte of the specific variable, since a CAN frame can include up to 64 bits
that can be split in whatever way we want and finally,
• the factor, which is only used for floats. We cannot send float numbers through CAN, but
we can let the developer know the format of the message with the dbc file. For instance
if a sensor had a value of 1.56, it will send a CAN message with data 156 and since we
read the dbc filem we will multiply if with its factor (which in this case shoould be 0.01)
in order to ge the real value.

Moving onto the main function, we start the CAN communication. We also do a few checks to make sure everything worked fine. Then the ROS code begins. We initialize the node and each publisher. Topic names should be inside the
quotation marks and they have to be same as the subscriber ones, so as for them to communicate properly.

Then inside a do-while we read the frames, with some error handling. I decided to use the
blocking function, since it is CPU friendly and the programm basically sleeps until a message
is actually there. If a message is arrived, we do find out its ID with a switch-case, call the
unpacking functions accordingly and publush the message to the correct topic.
Finally, there is a printing funtion, for debugging pursposes only.

2. ros to can

Similarly to the other node, we inlcude the messages that we want to send to the CAN channel
and define a few IDs. Here we will only use one union, but the can message will not be a
matrix. Instead we will handle it as a pointer.

The “Ros2Can” class constructor initializes the node, the subscribers and the CAN mes-
sages. Each message has a unique callback function. Since the publisher can use a timer to
publish data, I decided that it would be the best choice, since using a while(1) or a sleep or
even a CAN broadcast manager were rejected for performance reasons. Therefore, each time a
new message is published, it will be sent as a CAN message as well.
Since the callback function has to be a able to access class variables some basic getter func-
tions were created. In each callback function, we read the message, pack it as CAN frame (with
the reverse procedure as the one described before) and finally send it to the channel. Caution is
required in order to properly handle endianness and negative values (signed variables). Obvi-
ously the structure of the messages can be changed according to the needs of our specific CAN
architecture.
In the main function, we simply start the node and wait for callbacks.

3. Evaluation

We can test our code without real devices using the vcan. First we need to cd in our ros2
workspace and bring everything up to date with a colcon build. If needed install the setup
file as well. We will describe the process of evaluation of the current ros to can and can to ros
nodes.
For the ros to can node, we have to use a dummy node that publishes messages on topics
that are CAN related. In our repository there is a file in the turtle mpc package named
canbus dummy that gets the job done.
In total we will need 3 terminals. In the CAN terminal, we initialize CAN and run candump
to see the traffic on the channel. In the ros dummy one we just run the node (ros2 run turtle mpc
can dummy). Finally in a third terminal we run the ros to can node. If we did everything right
we should see some traffic in the candump, that in closer examination is the communication
from the mpc dummy. The next photo shows this example.

-------------------------------------------

For the can to ros node we will use 3 terminals as well. In the first one we should have the
CAN and vcan0 initialized. In the second one we run the can to ros node and in a third one we
echo a topic in which the node publishes messages from CAN (ros2 topic echo/Accelerometer).
Now we go back to the first terminal and use the cansend command that was described pre-
viously to send a message to the correct ID that is related to the topic we are monitoring. If
we did everything right, we should see that message in the echo. The following photo 
shows this example.




