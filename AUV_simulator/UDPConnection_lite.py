import socket
import threading
import time
import struct

class CustomUDP:
    """
    Define custom point to point UDP communication
    """

    def __init__(self, host, port, host_d, port_d):
        # host address and port
        self.__HOST_PORT = port
        self.__HOST_IP = host
        # recipient address and port
        self.__DEST_PORT = port_d
        self.__DEST_IP = host_d
        # initializing UDP connection
        self.sock = socket.socket(socket.AF_INET,
                                  socket.SOCK_DGRAM)
        self.sock.bind((self.__HOST_IP, self.__HOST_PORT))
        # connect socket with a specific server
        # self.sock.connect((self.__DEST_IP, self.__DEST_PORT))
        self.sock.settimeout(0)
        # not blocking mode
        self.sock.setblocking(0.01)
        # TODO: see how does select work for errors detection 
        self.__packet_list = []
        self.__to_send_list = b'0'
        self.__die_flag = False
        # period
        self.__DELAY = 0.01
        # self.__exit_flag = threading.Event()

    def run(self):
        data = None
        while True:
            # print(time.ctime())

            totalsent = 0
            if self.__to_send_list != b'0':
                sent = self.sock.sendto(self.__to_send_list,(self.__DEST_IP, self.__DEST_PORT))

            self.__to_send_list = b'0'

            try:
                data, addr = self.sock.recvfrom(1024000)
            except Exception as e:
                # print(e)
                pass

            if data != None:
                # print(data.decode())  
                self.__packet_list.append(struct.unpack(str(int(len(data)))+'B',data))

            if self.__die_flag == True:
                print("UDP Connection closed\n")
                self.sock.close()  
                return
            time.sleep(0.005)

    def pullPacket(self, msg):
        self.__to_send_list = msg

    def start(self):
        threading.Thread(target=self.run).start()
        time.sleep(1)

    def close(self):
        self.__die_flag = True

    # first packet extraction on arrival order
    def getPacket(self):
        if len(self.__packet_list) > 0:
            #print(len(self.__packet_list))
            return self.__packet_list.pop(0)
        return None