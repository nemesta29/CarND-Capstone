import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        img_hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        s_channel = img_hls[:,:,2]
        s_bin = np.zeros_like(s_channel)
        s_bin2 = np.zeros_like(s_channel)
        s_bin[(s_channel>220) & (s_channel<=255)] = 1
        s_bin2[(s_channel>100) & (s_channel<=120)] = 1
        mask=np.zeros_like(s_bin)
        mask2 = np.zeros_like(s_bin)
        mask3 = np.zeros_like(s_bin2)
        mask4 = np.zeros_like(s_bin2)
        ignore_mask = 255
        imgshape=image.shape

        vertices1=np.array([[(220,500),(220, 450), (650, 450), (650,500)]], dtype=np.int32)
        vertices2=np.array([[(220,450),(220, 350), (650, 350), (650,450)]], dtype=np.int32)
        vertices3=np.array([[(220,304),(220, 295), (650, 295), (650,304)]], dtype=np.int32)
        vertices4=np.array([[(220,370),(220, 350), (650, 350), (650,370)]], dtype=np.int32)
        cv2.fillPoly(mask,vertices1,ignore_mask)
        cv2.fillPoly(mask2,vertices2,ignore_mask)
        cv2.fillPoly(mask3,vertices3,ignore_mask)
        cv2.fillPoly(mask4,vertices4,ignore_mask)
        green_bin = cv2.bitwise_and(s_bin, mask)
        red_bin = cv2.bitwise_and(s_bin, mask2)

        red = np.nonzero(red_bin)
        green = np.nonzero(green_bin)
        sum_red = 0
        sum_green = 0
        for i in green_bin[green]:
            sum_green+=i
        for i in red_bin[red]:
            sum_red+=i
        if sum_red+sum_green < 25 :
            green_bin = cv2.bitwise_and(s_bin, mask3)
            red_bin = cv2.bitwise_and(s_bin2, mask4)

            red = np.nonzero(red_bin)
            green = np.nonzero(green_bin)
            sum_red = 0
            sum_green = 0
            for i in green_bin[green]:
                sum_green+=i
            for i in red_bin[red]:
                sum_red+=i
                
        if sum_red>sum_green:
            return TrafficLight.RED
        else:
            return TrafficLight.GREEN
        
        return TrafficLight.UNKNOWN
