import serial
import enum
import threading
import time
import struct

""" packets need to be ordered, at first double then float """
# packets dimension definition
n_double1c = 0      # number of double in packet 1c
n_single1c = 21     # number of float in packet 1c
n_double1d = 3      # number of double in packet 1d 
n_single1d = 3      # number of float in packet 1d

def calculate_checksum(packet):
    checksum = 0
    for i in range(len(packet)):
        checksum = checksum ^ packet[i]
    return checksum

"""
def decode1c(packet):
    decoded_packet = list()
    for i in range(0, 3*3):
        decoded_packet.append(struct.unpack(
            '<f', packet[4+i*4:4+(i+1)*4])[0])  # little endian
    decoded_packet.append(struct.unpack('<f', packet[40:44])[0])
    decoded_packet.append(struct.unpack('<f', packet[44:48])[0])
    return decoded_packet
"""
# TODO: define different kind of packets. 
# We have to define the header of the serial packet as input parameter
        


def decode1c(packet):

    decoded_packet = list()
    for i in range(0,n_double1c):
        decoded_packet.append(struct.unpack('<d', packet[4+i*8:4+(i+1)*8])[0]) #4byte header 8byte per double

    for i in range(0, n_single1c):
        decoded_packet.append(struct.unpack(
            '<f', packet[4+n_double1c*8+i*4:4+n_double1c*8+(i+1)*4])[0])  

    return decoded_packet

def decode1d(packet):

    decoded_packet = list()
    for i in range(0,n_double1d):
        decoded_packet.append(struct.unpack('<d', packet[4+i*8:4+(i+1)*8])[0]) #4byte header 8byte per double

    for i in range(0, n_single1d):
        decoded_packet.append(struct.unpack(
            '<f', packet[4+n_double1d*8+i*4:4+n_double1d*8+(i+1)*4])[0])  

    return decoded_packet


known_packets = {0x1c: ((8*n_double1c)+(4*n_single1c), decode1c), 0x1d: ((8*n_double1d)+(4*n_single1d), decode1d)} 


class SerialConnection:

    class State(enum.IntEnum):
        SYNC1 = 0
        SYNC2 = 1
        WAIT_ID = 2
        WAIT_LENGTH = 3
        WAIT_PAYLOAD = 4
        WAIT_CHECKSUM = 5

    def __init__(self, baud, port):
        self.connect = serial.Serial(
            port=port, baudrate=baud, write_timeout=1, timeout=0.01)
        self.__packet_list = []
        self.__die_flag = False

    def run(self):
        packet = bytearray()
        wait_state = self.State.SYNC1
        T = 0.1

        while True:
            t = time.time()
            rcv = self.connect.read()
            if len(rcv) != 0:
                #while len(rcv) == 0:
                #   time.sleep(0.1)
                #  rcv = self.connect.read()

                byte = int.from_bytes(rcv, byteorder="big")

                if wait_state == self.State.SYNC1:
                    if byte == 0x1a:
                        packet.clear()
                        packet.append(byte)
                        wait_state = self.State.SYNC2

                elif wait_state == self.State.SYNC2:
                    if byte == 0x1b:
                        packet.append(byte)
                        wait_state = self.State.WAIT_ID
                    else:
                        wait_state = self.State.SYNC1

                elif wait_state == self.State.WAIT_ID:
                    if known_packets.get(byte) != None: 
                        msg_id = byte
                        packet.append(byte)
                        wait_state = self.State.WAIT_LENGTH
                    else:
                        wait_state = self.State.SYNC1

                elif wait_state == self.State.WAIT_LENGTH:
                    msg_length = byte
                    if msg_length == known_packets[msg_id][0]: 
                        packet.append(byte)
                        offset = 0
                        wait_state = self.State.WAIT_PAYLOAD
                    else:
                        wait_state = self.State.SYNC1

                elif wait_state == self.State.WAIT_PAYLOAD:
                    packet.append(byte)
                    offset = offset+1
                    if (offset == msg_length):
                        wait_state = self.State.WAIT_CHECKSUM

                elif wait_state == self.State.WAIT_CHECKSUM:
                    if calculate_checksum(packet) == byte:
                        #self.__packet_list.append(
                        #    known_packets.get(msg_id)[1](packet))
                        #self.__packet_list=[0]
                        #self.__packet_list[0]= known_packets.get(msg_id)[1](packet)
                        # due to the slower server speed than the client's, accumulation of packets needs to be avoided
                        # append is commented, I am creating a list with a 0 and I am placing the last received packet on it
                        # => i keep the same structure but i use a bit different code to see if that's the error :
                        ''' IndexError: list assignment index out of range
                        'int' object is not subscriptable
                        'int' object is not subscriptable
                        Exception in thread Thread - 6:
                        '''
                        self.__packet_list = []
                        self.__packet_list.append(known_packets.get(msg_id)[1](packet))

                        #print("Icaro Connection: OK")
                        # print(self.__packet_list[len(self.__packet_list)-1])
                    else:
                        #print("Icaro Connection: checksum error")
                        pass

                    wait_state = self.State.SYNC1
                else:
                    pass

                # print(wait_state)

            if self.__die_flag == True:  
                print("Serial Connection closed\n")
                self.connect.close()
                return

    # first packet extraction on arrival order

    def getPacket(self):
        if len(self.__packet_list) > 0:
            return self.__packet_list.pop(0)
        return None

    # building and sending 1c packet
    def send1c(self, fields):
        msg_id = 0x1C
        length = (8*n_double1c)+(4*n_single1c)   
        packet = bytes.fromhex("1A1B")      
        packet = packet + \
            msg_id.to_bytes(1, "little") + length.to_bytes(1, "little")

        #packet = packet + struct.pack('>9f2i', *fields)
       
        for i in range(0,n_double1c):
            packet = packet + struct.pack('<d', fields[i])
        for i in range(0,n_single1c):
            packet = packet + struct.pack('<f', fields[i+n_double1c])
        
        packet = packet + (calculate_checksum(packet)).to_bytes(1, "little")

        self.connect.reset_output_buffer()
        # print(packet)
        self.connect.write(packet)

    def send1d(self, fields):
        msg_id = 0x1D
        length = (8*n_double1d)+(4*n_single1d) 
        packet = bytes.fromhex("1A1B")   
        packet = packet + \
            msg_id.to_bytes(1, "little") + length.to_bytes(1, "little")

        #packet = packet + struct.pack('>9f2i', *fields)
        
        for i in range(0,n_double1d):
            packet = packet + struct.pack('<d', fields[i])
        for i in range(0,n_single1d):
            packet = packet + struct.pack('<f', fields[i+n_double1d])
        
        packet = packet + (calculate_checksum(packet)).to_bytes(1, "little")

        self.connect.reset_output_buffer()
        #print(packet)
        self.connect.write(packet)

    def start(self):
        threading.Thread(target=self.run).start()

    def close(self):
        self.__die_flag = True



