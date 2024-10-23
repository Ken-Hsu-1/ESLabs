import socket
import matplotlib.pyplot as plt
import numpy as np

def server_program():
    # get the hostname
    host = socket.gethostname()
    port = 8002  # initiate port no above 1024

    server_socket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    server_socket.bind((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    server_socket.listen(2)
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))

    a_x = [0]
    t = [0]
    n = 1
    plt.ion()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    line1, = ax.plot(a_x, t, 'r-')

    data = 'y'
    conn.send(data.encode())

    while True:
        # receive data stream. it won't accept data packet greater than 1024 byte
        data = conn.recv(1024)
        if not data:
            # if data is not received break
            break
        #print("from connected user: " + str(data))
          
        data_x =  int.from_bytes(data, byteorder='little', signed=True)
        print(data_x)
        a_x.append(data_x)
        t.append(n)
        n=n+1
        plt.plot(t,a_x)
        plt.draw()
        plt.pause(0.0001)
        plt.clf()

        #data = input(' -> ')
        data = 'y'
        conn.send(data.encode())  # send data to the client

    conn.close()  # close the connection


if __name__ == '__main__':
    server_program()