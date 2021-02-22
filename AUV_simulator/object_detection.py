import numpy as np
import threading
import time
import cv2
from constants_module import TS
from UDPConnection_lite import CustomUDP

class ObjectDetection:
    
    # list of the object classes that the neural network is able to detect
    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                "sofa", "train", "tvmonitor"]

    # list of the bounding box colours (each class is associated with a color)
    COLORS = np.random.uniform(0, 255, size = (len(CLASSES), 3))

    output = None
    output_lock = threading.Lock()


    def __init__(self):
        pass


    def start(self, obj_des, conf_des):
        # loading the neural network "MobileNet SSD"
        print("[INFO] loading model...")
        self.net = cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt.txt', 'MobileNetSSD_deploy.caffemodel')  

        # define UDP connection parameters
        HOST_IP = 'localhost' #'192.168.1.52'
        HOST_PORT_UDP = 27000 #26000
        UDP_IP = 'localhost'
        UDP_PORT = 29000

        # initialize UDP connection with Unity
        try:
            self.comm_socket = CustomUDP(HOST_IP, HOST_PORT_UDP, UDP_IP, UDP_PORT)
            self.comm_socket.start()
            
        except Exception as e:
            print(e)

        # initialize "obj_det_thread"
        self.obj_det_running = True
        self.obj_det_thread = threading.Thread(target = self.process_images, args=(obj_des, conf_des))
        self.obj_det_thread.start()

    def get_output(self):
        self.output_lock.acquire()
        output_copy= self.output
        self.output_lock.release()
        return output_copy

    def process_images(self, obj_des, conf_des):
        start_time = None
        image = None
        while self.obj_det_running:  
            rx_data_temp = None

            while True:
                # check if data to be received is ready 
                rx_data = self.comm_socket.getPacket()
                if rx_data != None:
                    rx_data_temp = rx_data
                else:
                    if rx_data_temp != None:
                        break
                    else:
                        print("Waiting for UDP packets..")

            if rx_data_temp != None:
                # data received, treat it a sa jpg, try to decode it and display
                rx_data_array = np.asarray(rx_data_temp,  dtype = np.uint8)
                # decode jpg
                frame = cv2.imdecode(np.frombuffer(rx_data_array, dtype = 'uint8'), -1)
                # check that data is ok
                if frame.size > 0:
                    image = frame

            (h, w) = image.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

            # pass the blob through the network and obtain the detections and predictions
            # print("[INFO] computing object detections...")
            self.net.setInput(blob)
            detections = self.net.forward()
            
            # loop over the detections
            for i in np.arange(0, detections.shape[2]):         
                # extraction of the confidence level (i.e., probability) associated with the prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the "confidence" is
                # greater than the desired confidence level
                if confidence > conf_des:
                    # extract the index of the class retrieved from "detections" variable 
                    # compute the bounding box coordinates (x, y) of the object 
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype('int')

                    if self.CLASSES[idx] == obj_des:
                        # instant object detection
                        start_time = time.time()
                        # notice detection of the object wanted (higher confidence)
                        self.output_lock.acquire()
                        self.output = [box, 0]      # elapsed_time = 0
                        self.output_lock.release()

                    # print the information related to class and confidence of the detected object
                    label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                    # print("[INFO] {}".format(label))
                    
                    # vertical alignment of label text with respect to bounding box
                    if startY - 15 > 15:
                        y = startY - 15
                    else:
                        y = startY + 15

                    # tracking the bounding box and writing the corresponding label
                    cv2.rectangle(image, (startX, startY), (endX, endY), self.COLORS[idx], 2)
                    cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

                    # image center coordinates
                    x_c = (endX + startX)//2
                    y_c = (endY + startY)//2

                    # lenght of the cross elements to mark the center of the box
                    delta_x = w//32
                    delta_y = h//32

                    # draw cross at the box center
                    cv2.line(image, (x_c - delta_x, y_c), (x_c + delta_x, y_c), self.COLORS[idx], 1)
                    cv2.line(image, (x_c, y_c - delta_y), (x_c, y_c + delta_y), self.COLORS[idx], 1)

            if start_time != None:
                elapsed_time = time.time() - start_time
                self.output_lock.acquire()
                self.output[1] = elapsed_time
                self.output_lock.release()

            # draw viewfinder on the image center
            # colour
            white = [255, 255, 255]

            # center image coordinates
            x_c = w//2
            y_c = h//2
            
            # semi-sides length of the rectangle
            delta_x = w//32
            delta_y = h//32

            cv2.rectangle(image, (x_c - delta_x, y_c - delta_y), (x_c + delta_x, y_c + delta_y), white, 1)
            cv2.line(image, (x_c - 2 * delta_x, y_c), (x_c - delta_x, y_c), white, 1)
            cv2.line(image, (x_c + delta_x, y_c), (x_c + 2 * delta_x, y_c), white, 1)
            cv2.line(image, (x_c, y_c - 2 * delta_y), (x_c, y_c - delta_y), white, 1)
            cv2.line(image, (x_c, y_c + delta_y), (x_c, y_c + 2 * delta_y), white, 1)

            # show on screen the image processed
            cv2.imshow('Camera', image)
            cv2.waitKey(10)

            # if the cam window is closed stop the loop
            if cv2.getWindowProperty('Camera', cv2.WND_PROP_VISIBLE) == 0:      
                break

        time.sleep(TS)
            

    def stop(self):
        # close all windows opened
        cv2.destroyAllWindows()

        # wait for the end of "obj_det_thread"
        self.obj_det_running = False
        self.obj_det_thread.join()

        # close UDP connection with Unity
        self.comm_socket.close()