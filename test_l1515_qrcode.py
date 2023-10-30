import pyrealsense2 as rs
import numpy as np
import time
import cv2
from pyzbar import pyzbar
from pyzbar.pyzbar import decode
# from PIL import Image

import datetime

from collections import OrderedDict


class L515:

    def __init__(self,config):
        #init setting
        self.resolution=[1920,1080]
        self.cameraStatus=False
        self.serialNumber=None
        #income setting
        if 'resolution' in config:
            self.resolution=config['resolution']
        if 'SerialNumber' in config:
            self.serialNumber=config['SerialNumber']
        #camera init
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pc = rs.pointcloud()
        
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        
        #Camera setting
        self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.resolution[0], self.resolution[1], rs.format.rgb8, 30)
        if self.serialNumber is not None:
            self.config.enable_device(self.serialNumber)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.depth_sensor = self.device.first_depth_sensor()
        self.cameraSensor = self.device.query_sensors()[1]
        self.depth_sensor.set_option(rs.option.visual_preset, 3)
        self.depth_sensor.set_option(rs.option.min_distance,0)
        self.depth_sensor.set_option(rs.option.receiver_gain,18)
        self.depth_sensor.set_option(rs.option.laser_power,100)
        


        #self.cameraSensor.set_option(rs.option.exposure,300)

    def openCamera(self):
        try:          
            self.pipeline.start(self.config)
            # Skip 5 first frames to give the Auto-Exposure time to adjust
            for x in range(5):
                self.pipeline.wait_for_frames()
            self.cameraStatus=True
        except:
            print('open camera error')
            raise
    
    def closeCamera(self):
        try:
            self.pipeline.stop()
            self.cameraStatus=False
        except:
            print('close camera error')
            raise
    
    def getData(self):
        while True:
            
            if self.cameraStatus:
                    
                startTime = time.time()
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                colorFrame = aligned_frames.get_color_frame()
                colorImage = np.asanyarray(colorFrame.get_data())
                depthFrame = aligned_frames.get_depth_frame()
                pointCloudData = self.pc.calculate(depthFrame)
                pointCloudData = pointCloudData.get_vertices()
                pointCloudData = np.asanyarray(pointCloudData).view(np.float32).reshape(self.resolution[1],self.resolution[0],3)*1000

                endTime = time.time()-startTime
                yield [colorImage,pointCloudData,endTime]
            else:
                yield []




def decode(image):
    # decodes all barcodes from an image
    
    # barcode.data.decode("utf-8")  utf-8

    decoded_objects = pyzbar.decode(image)
    for obj in decoded_objects:
        # draw the barcode
        # print("detected barcode:", obj)
        
        image = draw_barcode(obj, image)

    return image


def decode2(image):
        decoded_objects = pyzbar.decode(image)
        qrcodedic = {}

        for idx,obj in enumerate(decoded_objects):
            qrcodedic[idx]= obj.rect.left
        sorted_dict_by_value_desc = dict(sorted(qrcodedic.items(), key=lambda item: item[1], reverse=True))
        
        BoxIDList = []
        for k,v in sorted_dict_by_value_desc.items():
            obj = decoded_objects[k]
            image = draw_barcode(obj, image)
                  
            BoxIDList.append(obj.data.decode("utf-8"))

        # return image, self.qrcodedic, angle_list
        return image, BoxIDList


def draw_barcode(decoded, image):
    # n_points = len(decoded.polygon)
    # for i in range(n_points):
    #     image = cv2.line(image, decoded.polygon[i], decoded.polygon[(i+1) % n_points], color=(0, 255, 0), thickness=5)
    # uncomment above and comment below if you want to draw a polygon and not a rectangle
    image = cv2.rectangle(image, (decoded.rect.left, decoded.rect.top), 
                            (decoded.rect.left + decoded.rect.width, decoded.rect.top + decoded.rect.height),
                            color=(0, 255, 0),
                            thickness=5)
    cv2.putText(image, str(decoded.data.decode("utf-8")), (decoded.rect.left, decoded.rect.top) ,
		cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0 ,255), 2)
    return image



camera = L515(dict({'SerialNumber':'f1230465'}))
camera.openCamera()
bg_img = np.full((900,1600,3),0).astype(np.uint8)

#crop = {'xmin' :400, 'xmax':1700,'ymin':50, 'ymax':1080, 'total height':500}


crop = {'xmin' :1500, 'xmax':1700,'ymin':50, 'ymax':1080, 'total height':500}
#crop = {'xmin' :270, 'xmax':1920,'ymin':50, 'ymax':1080, 'total height':790}


for idx_, i in enumerate(camera.getData()):
    start_ = time.time()
    print(i)
    image, pc, time_  = i
    # print(image)
    # print(pc)
    # print(time_)
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    #image = decode(image)
    image,boxlist = decode2(image)    

    cv2.putText(image, f"ID: {boxlist}",(50,50), cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,0,255,1))

    cv2.imshow('Image', image)
    
    k = cv2.waitKey(10)
    if k == ord('s'):

        img_name = f"image_{datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S')}.png"
        cv2.imwrite(img_name, image)
        print(f"Image saved as {img_name}")

        
    if k == ord('q'):
        break

    
    code = cv2.waitKey(1)
    if code == ord('q'):
        break