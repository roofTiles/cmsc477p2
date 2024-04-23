import time
import zmq

def StartPassingComms(ep_gripper=None):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    waiting = True
    while waiting:

        message = socket.recv()
        print("Received request: %s" % message)

        if message == b"Reciever Grabbed":
            print("Passer Releasing")
            ep_gripper.open(power=100)
            socket.send(b"Passer Releasing")
            waiting = False

