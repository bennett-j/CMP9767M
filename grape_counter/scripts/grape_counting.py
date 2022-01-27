#!/usr/bin/env python

# Python libs
from cmath import sqrt
import sys, time
from tokenize import String

# OpenCV
import cv2

# numpy
import numpy as np

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String, Int32

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud, transform_to_kdl

# going to use tf2 because tf depreciated 
# useful for what I want to do with transforms and pc2
# https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
import tf2_ros

import PyKDL
import math


class image_projection:
    camera_model = None
    image_depth_ros = None

    
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file - makes sense and numbers verified
    # (84.1/1920) / (70.0/512)
    color2depth_aspect = (84.1/1920) / (70.0/512)

    def __init__(self):    
        
        self.visualisation = True
        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        self.cur_pc2_pub = rospy.Publisher('/thorvald_001/current_grape_point_cloud', PointCloud2, queue_size=10)

        self.cum_pc2_pub = rospy.Publisher('/thorvald_001/cumulative_grape_point_cloud', PointCloud2, queue_size=10)

        self.count_pub = rospy.Publisher('/grape_count', Int32, queue_size=10)

        
        rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
            Image, self.image_color_callback)

        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect",
            Image, self.image_depth_callback)

        rospy.Subscriber("/process_img", String, self.do_image_proc)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.

        self.bunch_centroids = []
                
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        self.colour_img_ros = data
        
    def do_image_proc(self, msg):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv format
        try:
            self.colour_img = self.bridge.imgmsg_to_cv2(self.colour_img_ros, "bgr8")
            self.depth_img = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # checking the stamps of the two images are close - they are.
        # print('Colour header stamp: ', self.colour_img_ros.header.stamp)
        # print('Depth header stamp: ', self.image_depth_ros.header.stamp)
        #print('Time(0): ', rospy.Time().now())

        #image_color = cv2.resize(image_color, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        # detect a red blob in the color image
        #image_mask = cv2.inRange(image_color, (0, 0, 80), (20, 20, 255))

        image_color = self.colour_img
        

        if self.visualisation:
            # base image
            drawing = self.colour_img.copy()
            # drawing = np.zeros((self.colour_img.shape[0], self.colour_img.shape[1], 3), dtype=np.uint8)
            
        
        blur = cv2.GaussianBlur(image_color,(5,5),0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (75, 0, 25), (150, 150, 75))
        kernel_e5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        morphed = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel_e5)

        _, contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        centroid = []
        points = []
        corners = []

        for i, c in enumerate(contours):
            a = cv2.contourArea(c)
            
            # if M["m00"] == 0: this only occurs when area = 0 so won't get erros if we filter out small areas
            if a > 100:
                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroid.append((cx, cy))
                
                colour = (255,0,150) # purple
                

                # bounding box
                x,y,w,h = cv2.boundingRect(c)               
                # corner coords   
                #    (x,y) .  . (x+w,y)
                #  (x,y+h) .  . (x+w,y+h)

                # above we have image coords (just x,y) need to find z then
                # need to go from image > camera > map

                # convert centroid as image pixel to 3D point in camera frame
                centroid_cam = self.pixel_to_camera((cx,cy))                
                if centroid_cam is not None:
                    # if the centroid pixel is valid, do the following

                    # we're sensing front of the bunch so estimate true centroid by 
                    # calculating width, assuming revolved shape and adding half the
                    # width to the z value in camera frame.

                    # convert two corner points to cam frame taking depth value to be same as centroid
                    corner_cam = self.pixel_to_camera((x,y), depth_value=centroid_cam[2])
                    corners.append(corner_cam)
                    corner2_cam = self.pixel_to_camera((x+w,y), depth_value=centroid_cam[2])
                    width = math.sqrt((corner2_cam[0]-corner_cam[0])**2
                                       + (corner2_cam[1]-corner_cam[1])**2
                                       + (corner2_cam[2]-corner_cam[2])**2)
                    
                    # add half of width to z value
                    centroid_cam[2] += width/2

                    # add to a list of detected bunch centroids
                    points.append(centroid_cam)
                                    
                if self.visualisation:
                    # only draw centroid for larger areas
                    drawing = cv2.circle(drawing, (cx, cy), 3, (255,255,0), 2)
                    drawing = cv2.rectangle(drawing,(x,y),(x+w,y+h),(0,255,0),2)
                

                else:
                    # if point not in depth img we won't show it so just continue
                    continue

            else:
                colour = (0,255,255) # yellow
        
        if self.visualisation:
            cv2.drawContours(drawing, contours, i, colour, 2)
            
           
        # get the transform
        try:
            # will data.header.stamp use the time the image arrived?
            transform = self.tfBuffer.lookup_transform('map', "thorvald_001/kinect2_right_rgb_optical_frame", self.colour_img_ros.header.stamp, rospy.Duration(1.0)) #
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
           print(e)
           return
           # TODO figure out what to do here with the exception
        
        # apply the transform

        # transform points to map coords for saving in python
        # using structure of tf2_sensor_msgs.py by Willow Garage
        t_kdl = transform_to_kdl(transform)
        points_map = []
        for p in points:
            p_out = t_kdl * PyKDL.Vector(p[0], p[1], p[2])
            #print(p[3:])
            points_map.append((p_out[0], p_out[1], p_out[2])) #+tuple(p[3:])

        
        for i, point in enumerate(points_map):
            # corner = corners_map[4*i]

            # # approximate size of bunch as radius from centre to one corner
            # size_radius = math.sqrt((point[0] - corner[0])**2
            #                         + (point[1] - corner[1])**2
            #                         + (point[2] - corner[2])**2)
            
            # initialise query append to true (add unless proven not to)
            q_append = True
            # loop through all saved bunches, if current point isn't within distance of an existing one, append to list.
            for bunch in self.bunch_centroids:
                separation = math.sqrt((point[0]-bunch[0])**2
                                       + (point[1]-bunch[1])**2
                                       + (point[2]-bunch[2])**2)
                
                # if bunch is close assume it's the same one and no reason to continue checking so break
                if separation < (0.25):
                    q_append = False
                    break
            
            if q_append:
                self.bunch_centroids.append(point)


        # == REPORT ==
        # set up point clouds
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1)]

        # either do pc2 = PointCloud2() then fill in the fields manually 
        # or use Tim Field's point_cloud2.create_cloud() which returns a PointCloud2 obj
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'    
        
        # create point clouds
        current_bunch_pc2_map = point_cloud2.create_cloud(header, fields, points_map)
        cumulat_bunch_pc2_map = point_cloud2.create_cloud(header, fields, self.bunch_centroids)

        # publish point cloudsso we can see in rviz
        self.cur_pc2_pub.publish(current_bunch_pc2_map)
        self.cum_pc2_pub.publish(cumulat_bunch_pc2_map)

        # publish count
        self.count_pub.publish(len(self.bunch_centroids))

        print("Number of points: ", len(points))
        print("Number of bunches: ", len(self.bunch_centroids))
        
       
        if self.visualisation:
            # show contours and centres 
            cv2.imshow('Contours ext', cv2.resize(drawing, (0,0), fx=0.5, fy=0.5))
            cv2.waitKey(1)

    def pixel_to_camera(self, point, depth_value=None):
        """Convert an x,y point in the image to a 3D point in the camera frame using the depth image.
        
        @param point: a tuple (x,y) of pixel location to be
        @param depth_value: if defined the value will be use, else it will be extracted from the depth camera
        @return: the point (x,y,z) in camera coordinates
        """

        if depth_value is None:
            # Map(ping) from pixel coord in colour img to pixel coord in depth img
            depth_coords = (self.depth_img.shape[0]/2 + (point[1] - self.colour_img.shape[0]/2)*self.color2depth_aspect,    # y
                            self.depth_img.shape[1]/2 + (point[0] - self.colour_img.shape[1]/2)*self.color2depth_aspect)    # x
        
            # 1080 x 1920 scales to 345 x 614 which is wider than the depth image at 424 x 512
            # Therefore only attempt point transform if depth value available
            if depth_coords[1] >= 0 and depth_coords[1] < self.depth_img.shape[1]:
                # Extract depth at the found coordinate
                depth_value = self.depth_img[int(depth_coords[0]), int(depth_coords[1])] 

                # if the pixel has value nan (that's how the camera outputs it) then return
                if math.isnan(depth_value):
                    # TODO: in perfect world this shouldn't happen - are the cameras misaligned - is the transform wrong - are we sensing grape where there isn't?
                    #print("Depth for ", point, " isnan")
                    return
            else:
                return

        # Calculate point in camera coords
        # Project point (x,y) to 3D ray - returns unit vector at camera pinhole in direction of pixel
        camera_coords = self.camera_model.projectPixelTo3dRay((point[0], point[1]))
        # Scale vector so z=1 (metre) then multiply by depth value
        camera_coords = [x/camera_coords[2] for x in camera_coords]
        camera_coords = [x*depth_value for x in camera_coords]

        return camera_coords
    

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    ic = image_projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
