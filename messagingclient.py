import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to lego passing server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s …" % request)
    socket.send(b"Reciever Grabbed")

    #  Get the reply.
    message = socket.recv()
    print("Received reply %s [ %s ]" % (request, message))