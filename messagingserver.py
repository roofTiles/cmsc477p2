import time
import zmq

def CommunicateAtLine():
    print("Starting Server")
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    print("Server Started")

    print("Sending Line info")
    waiting = True
    while waiting:

        message = socket.recv()
        print("Received request: %s" % message)

        if message == b"Is passer at line?":
            print("Communicating that passer is at line")
            socket.send(b"Passer at Line")
            waiting = False


def StartPassingComms(ep_gripper=None):
    print("Starting Server")
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    print("Server Started")
    waiting = True
    while waiting:

        message = socket.recv()
        print("Received request: %s" % message)

        if message == b"Reciever Grabbed":
            print("Passer Releasing")
            ep_gripper.open(power=100)
            socket.send(b"Passer Releasing")
            waiting = False

