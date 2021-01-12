#!/usr/bin/env python



import roslib
import rospy
import os 
import shutil #for file management, copy file
import rosbag, sys, csv
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix 
import nav_msgs 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from cola2_msgs.msg import NavSts
import datetime
import csv
import subprocess
import shutil

import cv2
import matplotlib
import numpy
import message_filters
import numpy as np

from cv_bridge import CvBridge
from math import pi,tan
import tf
import geometry_msgs.msg
from cola2_lib.utils.ned import NED

from osgeo import gdal
import osr

class image_georeferencer:

    def __init__(self, name):
        """ Init the class """
        # Initialize some parameters

        print("Georeferencer_started !!!!")
        default_dir='~home/bagfiles'

        dir = rospy.get_param('~saving_dir', default=default_dir)

        self.dir_save_bagfiles = dir + 'TIFF_imgs_byte/'

        #crear directori si no existeix
        if not os.path.exists(self.dir_save_bagfiles):
            os.makedirs(self.dir_save_bagfiles)


        self.image_secs=None

        self.counter=0        #counter
        self.skip_n_imgs=1    #it just picks one image in every skips_n_img
        self.image_i=0

        self.rows=0

        self.image_info_dict={}
        self.listener = tf.TransformListener()


        # FOV de la camera extret de excel de framerate i exposicio -> modificar
        # FOV-x water 34,73   deg         # FOV_x_water=34.73
        # FOV-y water 26,84   deg         # FOV_y_water=26.84

        # FOV_x_water=rospy.get_param("FOV_x_water",default=34.73)
        # FOV_y_water=rospy.get_param("FOV_y_water",default=26.84)


        FOV_x_water=rospy.get_param('~FOV_x_water',default=34.73)
        FOV_y_water=rospy.get_param('~FOV_y_water',default=26.84)

        print("node started FOV_X_water, FOV_Y_water: ",FOV_x_water, FOV_y_water)


        #deg to rad
        self.FOV_x=34.73*((2*pi)/360.0)
        self.FOV_y=26.84*((2*pi)/360.0)


        #Subscribers

        #Uncoment this to use ekf info
        # rospy.Subscriber("/turbot/navigator/ekf_map/odometry",
        #                  Odometry,    
        #                  self.update_gps,
        #                  queue_size=1)

        # rospy.Subscriber("/stereo_down/left/camera_info",
        #              CameraInfo,
        #              self.update_camera_info,
        #              queue_size=1)

        rospy.Subscriber("/turbot/navigator/navigation",
                NavSts,
                self.update_gps_info,
                queue_size=1)

        
        rospy.Subscriber("/stereo_down/left/image_raw",
                Image,
                self.update_img_info,
                queue_size=1)

        gps_sub=message_filters.Subscriber("/turbot/navigator/navigation",
                        NavSts,  
                        queue_size=1)

        img_sub=message_filters.Subscriber("/stereo_down/left/image_raw",
                         Image,
                         queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([gps_sub, img_sub], 10, 0.5, allow_headerless=True)
        ts.registerCallback(self.gps_image_update)


    def update_gps_info(self, msg):
        pass
        # print("gps_info!!!!!")
        # self.gps_secs = msg.header.stamp.secs
        # print("gps_info: ",self.gps_secs)


    def update_img_info(self, msg):
        pass
        # print("img_info!!!!!")
        # self.imagetest_secs = msg.header.stamp.secs
        # print("imagetest_info: ",self.imagetest_secs)

    
    #get base_link to camera center tf
    def get_tf_transform(self):
        rot=None
        trans=None
        try:
            (trans,rot) = self.listener.lookupTransform('/turbot/base_link', '/turbot/stereo_down/left_optical', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        print(trans,"     ",rot)
        return trans,rot

    def get_overlapping_val(self,img1_dim,img2_dim,img1_dim,img2_dim):
        #img1_dim=[xleft,xright,ydown,yup]
        #img1_dim=[xleft,xright,ydown,yup]
        #img1 dim= xdim,ydim
        
        #overlap surface: 
        y_dif=abs(img1[3]-img2[3])
        x_dif=abs(img1[0]-img2[0])

        if img1_dim[0]<=img2_dim[0]:
            overlap_x=(img1_dim[1]-img1_dim[0])-x_dif
            overlap_y=(img1_dim[3]-img1_dim[2])-y_dif
            total_surface=img1_dim
        else:
            overlap_x=(img2_dim[1]-img2_dim[0])-x_dif
            overlap_y=(img2_dim[3]-img2_dim[2])-y_dif
            total_surface=img2_dim

        overlap=(overlap_x*overlap_y)/total_surface
        


    #navigation and image callback
    #Gets image and localization info and saves a georeferenced image
    def gps_image_update(self,gps_sub,img_sub):

        if self.counter==0:
            #Primer pic
            self.trans,self.rot=self.get_tf_transform()

            self.ned_origin_lat=gps_sub.origin.latitude
            self.ned_origin_lon=gps_sub.origin.longitude
            self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)

        print("GPS_IMAGE_UPDATE")
        print("-------------------------------------------")
        print("")
        self.position_secs = gps_sub.header.stamp.secs
        self.latitude = gps_sub.global_position.latitude
        self.longitude = gps_sub.global_position.longitude
        self.altitude = gps_sub.altitude
        print("lat,long, alt:",self.latitude,"   ",self.longitude," ",self.altitude)

        self.image_secs = img_sub.header.stamp.secs
        self.image_seq = img_sub.header.seq

        print("GPS_time_stamp: ",self.position_secs)
        print("image_time_stamp: ",self.image_secs)

        # print(self.altitude)
        # print(self.FOV_x)

        if self.counter%self.skip_n_imgs==0:

            print("Counter= ",self.counter," georeferencing image.....")
            #north (x) east (y)
            x_img_pos=gps_sub.position.north + self.trans[0]
            y_img_pos=gps_sub.position.east + self.trans[1]

            #Image dimensions
            #La funcio tan va amb radians        
            self.x_dim_img=2*self.altitude*tan(self.FOV_x/2)#width
            self.y_dim_img=2*self.altitude*tan(self.FOV_y/2)#height

            #test:
            print("IMG_DIM: ",self.x_dim_img,"        ",self.y_dim_img)
            lat_center, long_center, _ = self.ned.ned2geodetic([x_img_pos, y_img_pos, 0.0])

            lat_0, long_0, _ = self.ned.ned2geodetic([gps_sub.position.north, gps_sub.position.east, 0.0])


            print("CENTER_LAT_LONG__0: ",lat_0, long_0)
            print(" ")

            print("CENTER_LAT_LONG__1: ",lat_center, long_center)
            print(" ")

            #Revisar signes!
            #left up corner coordinates
            x_img_corner_up= x_img_pos - self.x_dim_img/2
            y_img_corner_up= y_img_pos + self.y_dim_img/2
            #bottom right corner coordinates
            x_img_corner_down= x_img_pos + self.x_dim_img/2
            y_img_corner_down= y_img_pos - self.y_dim_img/2

            print("x_corner0= ",x_img_corner_up)
            print("y_corner0=" ,y_img_corner_up)
            print("")

            print("x_corner1= ",x_img_corner_down)
            print("y_corner1=" ,y_img_corner_down)
            # Transform to lat lon
            pxl_lat_up, pxl_lon_up, _ = self.ned.ned2geodetic([x_img_corner_up, y_img_corner_up, 0.0])
            pxl_lat_down, pxl_lon_down, _ = self.ned.ned2geodetic([x_img_corner_down, y_img_corner_down, 0.0])


            print("LC_LAT_LONG: ",pxl_lat_up,"  ", pxl_lon_up)
            print(" ")

            print("RC_LAT_LONG: ",pxl_lat_down," ",pxl_lon_down)
            print(" ")
            

            bounds=[pxl_lat_up, pxl_lon_up,pxl_lat_down, pxl_lon_down]

            print("img_msg type: ",type(img_sub))

            #convert to CVimage
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_sub, desired_encoding="rgb8")

            #test conversion
            # filename_suffix='JPEG'
            # filename=os.path.join(self.dir_save_bagfiles, str(self.image_seq) + "." + filename_suffix)
            # # cv2.imwrite(filename,cv_image)
            # print("img saved")

            print("IMG shape")
            print(cv_image.shape[0])

            print(cv_image.shape)

            if(self.position_secs==self.image_secs):
                print("Callback sync works")
                print(self.position_secs)
                print(self.image_secs)
            

            # In a north up image, padfTransform[1] is the pixel width, and padfTransform[5] is the pixel height. 
            # The upper left corner of the upper left pixel is at position (padfTransform[0],padfTransform[3]).


            #Necessit les coordenades del canto superior

            #Geotransform[0]-> bounds[1] =coordenada x del pixel d adalt a l'esquerra de la imatge 
            #Geotransfrom[1]-> yres ->   = pixel width
            #Geotransform[2]->  0
            #Geotransform[3]-> bounds[0] =coordenada y del pixel d adalt a l'esquerra de la imatge 
            #Geotransform[4]->  0
            #Geotransform[5]-> xres ->pixel height


            # adfGeoTransform[0] /* top left x */
            # adfGeoTransform[1] /* w-e pixel resolution */ ->xres?
            # adfGeoTransform[2] /* 0 */
            # adfGeoTransform[3] /* top left y */
            # adfGeoTransform[4] /* 0 */
            # adfGeoTransform[5] /* n-s pixel resolution (negative value) */ ->yres

#             A geotransform consists in a set of 6 coefficients:
            # GT(0) x-coordinate of the upper-left corner of the upper-left pixel.
            # GT(1) w-e pixel resolution / pixel width.
            # GT(2) row rotation (typically zero).
            # GT(3) y-coordinate of the upper-left corner of the upper-left pixel.
            # GT(4) column rotation (typically zero).
            # GT(5) n-s pixel resolution / pixel height (negative value for a north-up image).




            filename=os.path.join(self.dir_save_bagfiles, str(self.image_seq))

            self.geotiff(filename,cv_image,bounds)

        self.counter+=1

    def geotiff(self,filename, image,bounds):
        # nx = image.shape[0]
        # ny = image.shape[1]
        # xres = (bounds[2] - bounds[0]) / float(nx)
        # yres = (bounds[3] - bounds[1]) / float(ny)

        # nx=self.x_dim_img
        # ny=self.y_dim_img
        # geotransform = (bounds[1], yres, 0, bounds[0], 0, xres)

        #????? no hauria de ser alreves???

        print("filename: ",filename)

        ny=image.shape[0] #img height 1440
        nx=image.shape[1] #img width 1920


        xres = (bounds[2] - bounds[0]) / float(nx)
        yres = (bounds[3] - bounds[1]) / float(ny)

        # xres=self.x_dim_img/nx  #pixel width resolution
        # yres=self.y_dim_img/ny  #pixel height resolution

        geotransform = (bounds[1], xres, 0, bounds[0], 0, yres)

        srs = osr.SpatialReference()            # establish encoding
        srs.ImportFromEPSG(4326)                # WGS84 lat/long
        
        if len(image.shape) == 3:
            dst_ds = gdal.GetDriverByName('GTiff').Create(filename, nx, ny, 3, gdal.GDT_Float64)
            # dst_ds = gdal.GetDriverByName('GTiff').Create(filename, ny, nx, 3, gdal.GDT_Float64)
            dst_ds.SetGeoTransform(geotransform)    # specify coords
            srs = osr.SpatialReference()            # establish encoding
            srs.ImportFromEPSG(4326)                # WGS84 lat/long
            dst_ds.SetProjection(srs.ExportToWkt())  # export coords to file

            dst_ds.GetRasterBand(1).WriteArray(image[:, :, 0])   # write r-band to the raster
            dst_ds.GetRasterBand(2).WriteArray(image[:, :, 1])   # write g-band to the raster
            dst_ds.GetRasterBand(3).WriteArray(image[:, :, 2])   # write b-band to the raster
        
        dst_ds.FlushCache()                     # write to disk
        dst_ds = None


        
if __name__ == '__main__':
    try:
        rospy.init_node('image_georeferencer')
        image_georeferencer = image_georeferencer(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
