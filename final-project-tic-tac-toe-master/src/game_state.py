#!/usr/bin/env python

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2 as cv
import numpy as np
import argparse
from cv_bridge import CvBridge, CvBridgeError
import rospy
import intera_interface
import rospkg

class Camera(object):

    def get_game_state(self, img_data, (edge_detection, window_name, stream)):
        """The callback function to show image by using CvBridge and cv
           and converts image to the gamestate
        """
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(img_data, "bgr8")
        except CvBridgeError, err:
            rospy.logerr(err)
            return

        #create a 2d array to hold the gamestate
        gamestate = [0,0,0,0,0,0,0,0,0]

        # Crop top of image
        img_crop = img[0:70, 0:275]
        img_crop_tape = img[5:70, 40:275]


        # Find contours of edges
        edged = cv.Canny(img_crop, 30, 200)

        im, contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)

        # Find largest contour (the green tape)
        large_cnt_index = 0
        largest_peri = 0
        for i in range(len(contours)):
            perimeter = cv.arcLength(contours[i],True)
            if perimeter > largest_peri:
                largest_peri = perimeter
                large_cnt_index = i

        # get coordinates of tile
        x,y,w,h = cv.boundingRect(contours[large_cnt_index])

        # Origin adjustments
        x += 1
        y -= 0
        w += 0
        h += 1

        # measure 1 cm in pixels
        cm = (w+h)/4

        buffer = cm/4

        tape_crop = img[y:y+h, x:x+w]

        # Crop frame for each board space
        top_left = img[y + cm: y + 9*cm - 4*buffer,  x + 2*cm: x + 9*cm - 2*buffer]
        top_mid = img[y+1*cm : y + 8*cm - 2*buffer,  x + 11*cm + buffer: x + 17*cm - buffer]
        top_right = img[y+1*cm: y + 8*cm - 3*buffer,  x + 20*cm: x + 28*cm ]
        mid_left = img[y + 10*cm + 2*buffer: y + 17*cm - buffer,  x + 2*cm: x + 9*cm - buffer]
        mid_mid = img[y+10*cm + buffer: y + 17*cm - 2*buffer,  x + 11*cm + buffer: x + 18*cm - buffer]
        mid_right = img[y+10*cm + buffer: y + 17*cm - 3*buffer,  x + 21*cm: x + 29*cm]
        bot_left = img[y + 19*cm + 2*buffer: y + 29*cm - buffer,  x + 2*cm: x + 9*cm]
        bot_mid = img[y+19*cm + buffer: y + 29*cm - buffer,  x + 11*cm + 2*buffer: x + 19*cm - 3*buffer]
        bot_right = img[y+19*cm: y + 28*cm - buffer,  x + 20*cm + 3*buffer: x + 29*cm]

        board = [top_left, top_mid, top_right, mid_left, mid_mid, mid_right, bot_left, bot_mid, bot_right]
        kernel = np.ones((4,4),np.uint8)

        board_index = -1

        for space in board:

            board_index += 1

            # Find contours of edges
            edge = cv.Canny(space, 30, 100)

            # Dilate edges to fill in gaps
            dilation = cv.dilate(edge,kernel,iterations = 1)

            # Find contours in the board space
            im2, contours, hierarchy = cv.findContours(dilation, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)

            # Find largest contour (the X or O)
            large_cnt_index = 0
            largest_peri = 0
            for i in range(len(contours)):
                perimeter = cv.arcLength(contours[i],True)
                if perimeter > largest_peri:
                    largest_peri = perimeter
                    large_cnt_index = i

            cv.drawContours(space, contours, large_cnt_index, (255,0,0), 2)

            try:
                largest_ct = contours[large_cnt_index]
            except IndexError:
                continue

            # Determine if the space is filled by an X or O


            # display image
            cv.imshow('image', img)
            cv.waitKey(250)
            cv.destroyAllWindows()

            if cv.contourArea(largest_ct) > 1000:
                # Calculate the solitity
                area = cv.contourArea(largest_ct)
                hull = cv.convexHull(largest_ct)
                hull_area = cv.contourArea(hull)
                try:
                    solidity = float(area)/hull_area
                except ZeroDivisionError:
                    solidity = -100

                # Fill the corresponding gamestate space
                if(solidity > 0.6):
                        # Fill in O
                        gamestate[board_index] = 2
                else:
                        # Fill in X
                        gamestate[board_index] = 1

        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('final-project-tic-tac-toe')

        dim = (1024,600)
        resize = cv.resize(img, dim)
        cv.imwrite(pkgpath + '/images/board.jpg', resize)
        self.gamestate = np.reshape(gamestate, (3,3))
        self.once = True
        return

    def __init__(self):
        self.gamestate = np.zeros((3,3))
        self.once = False

        """Camera Display

        Hand Camera Ranges
          - exposure: [0.01-100]
          - gain: [0-255]
        Head Camera Ranges:
          - exposure: [0-100], -1 for auto-exposure
          - gain: [0-79], -1 for auto-gain
        """
        rp = intera_interface.RobotParams()
        valid_cameras = rp.get_camera_names()
        if not valid_cameras:
            rp.log_message(("Cannot detect any camera_config"
              " parameters on this robot. Exiting."), "ERROR")
            return
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt)
        parser.add_argument(
            '-c', '--camera', type=str, default="right_hand_camera",
            choices=valid_cameras, help='Setup Camera Name for Camera Display')
        parser.add_argument(
            '-r', '--raw', action='store_true',
            help='Specify use of the raw image (unrectified) topic')
        parser.add_argument(
            '-e', '--edge', action='store_true',
            help='Streaming the Canny edge detection image')
        parser.add_argument(
            '-g', '--gain', type=int, default=15,
            help='Set gain for camera (-1 = auto)')
        parser.add_argument(
            '-x', '--exposure', type=float, default=5,
            help='Set exposure for camera (-1 = auto)')
        args = parser.parse_args(rospy.myargv()[1:])

        print("Initializing node... ")
        #rospy.init_node('camera_display', anonymous=True)
        cameras = intera_interface.Cameras()
        if not cameras.verify_camera_exists(args.camera):
            rospy.logerr("Could not detect the specified camera, exiting.")
            return
        rospy.loginfo("Opening camera '{0}'...".format(args.camera))
        rectify_image = not args.raw
        use_canny_edge = args.edge

        # optionally set gain and exposure parameters
        if args.gain is not None:
            if cameras.set_gain(args.camera, args.gain):
                rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(args.camera)))

        if args.exposure is not None:
            if cameras.set_exposure(args.camera, args.exposure):
                rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(args.camera)))

        self.cameras = cameras
        self.argsCamera = args.camera

        cameras.set_callback(args.camera, self.get_game_state,
              rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera, cameras))
        cameras.stop_streaming(args.camera)

        return

    def updateBoard(self):
        self.cameras.start_streaming(self.argsCamera)
        self.once = False
        while(not self.once):
            rospy.sleep(1)
        self.cameras.stop_streaming(self.argsCamera)
        return self.gamestate
