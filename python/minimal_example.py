import zmq

'''
This is a minimal Python example that talks to our MANUS C++ SDK and prints the data out directly to the terminal.
To run this, first run our MANUS SDK and then this script.

Keep in mind this data runs VERY fast, I would not recommend directly setting robot hands at this rate but instead slow down the data.
'''

left_glove_sn = "b8852964"
right_glove_sn = "6bb82ce1"
SERIAL_NUMBER = "1302917"
DONGLE_ID = "B9A09CF4"

context = zmq.Context()
context_ergo = zmq.Context()
#Socket to talk to Manus SDK
print("Connecting to SDK")
socket = context.socket(zmq.PULL)
socket_ergo = context_ergo.socket(zmq.PULL)
socket.setsockopt(zmq.CONFLATE, True)
socket_ergo.setsockopt(zmq.CONFLATE, True)
socket.connect("tcp://192.168.1.97:8000")
socket_ergo.connect("tcp://192.168.1.97:8001")
print("Connection succeed")

'''
This is ordered from Thumb to pinky, palm out to fingertip. 
The data is x,y,z for position and then quaternion (x,y,z,w) for rotation.  
25 positions * 7 for each pose = 175.  175 + 1 = 176, one datapoint for ID.

I highly recommend you visuzlize this data, it makes it much easier to figure this out.  :)
'''
def parse_full_skeleton(data):
    if data[0] == left_glove_sn:
        print("Left Glove Skeleton Data")
        print(list(map(float,data[1:])))
    elif data[0] == right_glove_sn:
        print("Right Glove Skeleton Data")
        print(list(map(float,data[1:])))
    else:
        print("Serial Number not found: " + str(data[0]))
while True:
    #wait for message
    # print("Inside while loop")
    message = socket.recv()
    #receive the message from the socket
    # print("message received: ", message)
    message = message.decode('utf-8')
    # print("Received reply %s" % (message))
    data = message.split(",")
    # print("data: ", data)  
    # if len(data) == 40:
    #     print("Left Glove Joint-level Ergonomics Data:")
    #     print(list(map(float,data[0:20])))
    #     print("Right Glove Joint-level Ergonomics Data:")
    #     print(list(map(float,data[20:40])))
    if len(data) == 352:
        parse_full_skeleton(data[0:176])
        parse_full_skeleton(data[176:352])
    elif len(data) == 176:
        parse_full_skeleton(data[0:176])


    message_ergo = socket_ergo.recv()
    message_ergo = message_ergo.decode('utf-8')
    # print("Received reply %s" % (message))
    data_ergo = message_ergo.split(",")
    if len(data_ergo) == 40:
        print("Left Glove Joint-level Ergonomics Data:")
        print(list(map(float,data_ergo[0:20])))
        print("Right Glove Joint-level Ergonomics Data:")
        print(list(map(float,data_ergo[20:40])))