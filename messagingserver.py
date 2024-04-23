import time
import zmq
import proj_2.gripping as gripping


def StartPassingComms():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    waiting = True
    while waiting:

        message = socket.recv()
        print("Received request: %s" % message)

        if message == b"Reciever Grabbed":
            gripping.DropLego()
            socket.send(b"Passer Releasing")
            print("Passer Releasing")
            waiting = False

