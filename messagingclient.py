import zmq
import time


#  Waiting for giver to reach line
def RecieveReadyMessage(request):
    context = zmq.Context()
    print("Connecting to lego passing server…")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    

    print("Sending request %s …" % request)
    socket.send(b"Is passer at line?")
    print("Asking if passer is at line")

    #  Get the reply.
    message = socket.recv()
    time.sleep(1)
    print("Received reply %s [ %s ]" % (request, message))
    if message == b"Passer at Line":
        print("Beginning to approach line")
        return True
    else:
        return RecieveReadyMessage(request=request+1)

#  Communicate that receiver is grabbing and passer should release
def SendGrabMessage(request):
    context = zmq.Context()
    print("Connecting to lego passing server…")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    
    print("Sending request %s …" % request)
    socket.send(b"Reciever Grabbed")
    print("Reciever Grabbed")

    #  Get the reply.
    message = socket.recv()
    time.sleep(1)
    print("Received reply %s [ %s ]" % (request, message))
    if message == b"Passer Releasing":
        print("Confirmed Release")
        return True
    else:
        return SendGrabMessage(request=request+1)