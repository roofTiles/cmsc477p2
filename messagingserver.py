import time
import zmq
import gripping

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:

    message = socket.recv()
    print("Received request: %s" % message)

    if message == b"Reciever Grabbed":
        gripping.DropLego()
        socket.send(b"Passer Releasing")
