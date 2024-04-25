import time
import zmq

context = None
socket = None
waiting = None
message = None

def StartServer():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")


def CommunicateAtLine(ep_gripper=None):
    print("Sending request %s …" % request)
    socket.send(b"Passer at Line")
    print("Communicating that passer is at line")

def StartPassingComms(ep_gripper=None):
    waiting = True
    while waiting:

        message = socket.recv()
        print("Received request: %s" % message)

        if message == b"Reciever Grabbed":
            print("Passer Releasing")
            ep_gripper.open(power=100)
            socket.send(b"Passer Releasing")
            waiting = False

