import zmq
import time


#  Socket to talk to server


#  Do 10 requests, waiting each time for a response
def SendGrabMessage(request):
    context = zmq.Context()
    print("Connecting to lego passing server…")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    
    print("Sending request %s …" % request)
    socket.send(b"Reciever Grabbed")

    #  Get the reply.
    message = socket.recv()
    time.sleep(1)
    print("Received reply %s [ %s ]" % (request, message))
    if message == b"Passer Releasing":
        return True
    else:
        return SendGrabMessage(request=request+1)