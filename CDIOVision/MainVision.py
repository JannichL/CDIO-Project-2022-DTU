# Author: Gustav Fuhr Høyer (Heavily using Evan Juras Code)
# Date: 23/6/22
# Description: Python script to detect and identify playing cards
# from a PiCamera video feed.

import cv2
import numpy as np
import time
import os
import Cards
import VideoStream
import sys
import serial
from time import sleep

## Camera settings
IM_WIDTH = 640
IM_HEIGHT = 480 
FRAME_RATE = 10

## Define font to use
font = cv2.FONT_HERSHEY_DUPLEX

# Initialize camera object and video feed from the camera. The video stream is set up
# as a seperate thread that constantly grabs frames from the camera feed. 
# See VideoStream.py for VideoStream class definition
## USING USB CAMERA INSTEAD OF PICAMERA? CHANGE THE THIRD ARGUMENT FROM 1 TO 2 IN THE FOLLOWING LINE:
## Fourth argument changes the id of the camera.
videostream = VideoStream.VideoStream((IM_WIDTH,IM_HEIGHT),FRAME_RATE,2,2).start()
time.sleep(1)

# Load the train rank and suit images
path = os.path.dirname(os.path.abspath(__file__))
train_ranks = Cards.load_ranks( path + '/GrayPics/')
train_suits = Cards.load_suits( path + '/GrayPics/')


# MAIN LOOP

cam_quit = 0

while cam_quit == 0:

    # Grab frame from video stream
    image = videostream.read()

    # Pre-process camera image
    pre_proc = Cards.preprocess_image(image)
	
    # Find and sort the outlines of all cards in the image (query cards)
    cnts_sort, cnt_is_card = Cards.find_cards(pre_proc)

    # Don't do anything if there are no shapes
    if len(cnts_sort) != 0:

        # Initialize a new "cards" list to assign the card objects.
        # k indexes the newly made array of cards.
        cards = []
        k = 0

        # For each contour detected:
        for i in range(len(cnts_sort)):
            if (cnt_is_card[i] == 1):

                # Create a card object from the contour and append it to the list of cards.
                # preprocess_card function takes the card contour and contour and
                # determines the cards properties.
                cards.append(Cards.preprocess_card(cnts_sort[i],image))

                # Find the best rank and suit match for the card.
                cards[k].best_rank_match,cards[k].best_suit_match,cards[k].rank_diff,cards[k].suit_diff = Cards.match_card(cards[k],train_ranks,train_suits)

                # Make center point and match result on the image.
                image = Cards.draw_results(image, cards[k])
                k = k + 1
                
        # Draw card contours on image (have to do contours all at once or
        # they do not show up properly for some reason)
        if (len(cards) != 0):
            temp_cnts = []
            for i in range(len(cards)):
                temp_cnts.append(cards[i].contour)
            cv2.drawContours(image,temp_cnts, -1, (255,0,0), 2)
        
    #Display image with the identified cards
    cv2.imshow("Card Detector",image)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('v'):
        cam_quit = 1

# Close everything.
cv2.destroyAllWindows()
videostream.stop()

