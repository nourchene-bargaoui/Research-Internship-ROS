#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class LineFollower(object):
	
	def __init__(self):
		self.image_sub= rospy.Subscriber("/camera/image_raw",Image,self.camera_callback)
		self.bridge_object= CvBridge()
		self.speed_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
	
	def camera_callback(self,data):
		try:
			cv_image=self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)
		height,width, _ = cv_image.shape
		speed_cmd=Twist()
		wicentre=100
		limit=5
		descentre =60
		rows_to_watch=90

		crop_crop= cv_image[int((height)/2+descentre):int((height)/2+(descentre+rows_to_watch)), 1:width]
		crop_down= cv_image[int((2*height)/3):height-220, 1:width]
		hsvc= cv2.cvtColor(crop_crop,cv2.COLOR_BGR2HSV)
		hsvd= cv2.cvtColor(crop_down,cv2.COLOR_BGR2HSV)
		
		upper_color=np.array([25,25,25])
		lower_color=np.array([0,0,0])
		maskc=cv2.inRange(hsvc,lower_color,upper_color)
		maskd=cv2.inRange(hsvd,lower_color,upper_color)

		
		edges = cv2.Canny(maskc, 100, 200)
		
		line_segments = cv2.HoughLinesP(edges, 1,np.pi / 180, 10, np.array([]), 5,0)
		
		height1, width1, _ = crop_crop.shape
		left_fit = []
		right_fit = []
		boundary = 1/3
		left_region_boundary = width1 * (1 - boundary)
		right_region_boundary = width1 * boundary 
		lane_lines = []
		if line_segments is None:
			print('No line_segment segments detected')

		else:
			for line_segment in line_segments:
				for x1, y1, x2, y2 in line_segment:
					if (x1 == x2):
						print('skipping vertical line segment (slope=inf): %s' % line_segment)
						continue
				fit = np.polyfit((x1, x2), (y1, y2), 1)
				slope = fit[0]
				intercept = fit[1]
				if slope < 0:
					if x1 < left_region_boundary and x2 < left_region_boundary:
						left_fit.append((slope, intercept))
				else:
					if x1 > right_region_boundary and x2 > right_region_boundary:
						right_fit.append((slope, intercept))

			left_fit_average = np.average(left_fit, axis=0)
			if len(left_fit) > 0:
				slope, intercept = left_fit_average
				y1 = height1  
				y2 = int(y1 * 1 / 2)  
				x1 = max(-width1, min(2 * width1, int((y1 - intercept) / slope)))
				x2 = max(-width1, min(2 * width1, int((y2 - intercept) / slope)))
				lane_lines.append([[x1, y1, x2, y2]])
		
			right_fit_average = np.average(right_fit, axis=0)
			if len(right_fit) > 0:
				slope, intercept = right_fit_average
				y1 = height1  
				y2 = int(y1 * 1 / 2)  
				x1 = max(-width1, min(2 * width1, int((y1 - intercept) / slope)))
				x2 = max(-width1, min(2 * width1, int((y2 - intercept) / slope)))
				lane_lines.append([[x1, y1, x2, y2]])
		
			print('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
		


		"""
		
		if (totalm< 27000):
			speed_cmd.angular.z=0
			speed_cmd.linear.x=0
			
		
		elif len(fazza) == 2:
				err = (fazza[0]+fazza[1])
				if abs(err) > 0.05:
					del fazza[0]
					del fazza[0]
					haja=err*4
					if haja > limit: 
						speed_cmd.angular.z=haja
						speed_cmd.linear.x=(0.2-(err))
					
				else : 
					del fazza[0]
					del fazza[0]
					speed_cmd.linear.x=0.25
					speed_cmd.angular.z=0
		else :
			if fazza[0] > 0.1:
				
				haja=fazza[0]*3
				speed_cmd.angular.z=haja
				speed_cmd.linear.x=(0.2-(fazza[0]/1.5))
				del fazza[0]
			elif fazza[0] < -0.1:
				haja=fazza[0]*3
				speed_cmd.angular.z=-haja
				speed_cmd.linear.x=(0.2-(fazza[0]/1.5))
				del fazza[0]
		print(speed_cmd.linear.x)
		print(speed_cmd.angular.z)"""
		print (totalm)
		
		"""self.speed_pub.publish(speed_cmd)		"""
				
		
		#cv2.imshow("lane lines", line_image)
		#cv2.waitKey()
		#totalr = cv2.countNonZero(maskr)
		#totall = cv2.countNonZero(maskl)
		#totalm = cv2.countNonZero(maskm)
		#speed_cmd.linear.x=0.25
		#pm = totalm / (heightm * widthm)
		#pr = totalr / (heightr * widthr)
		#pl = totall / (heightl * widthl)
		#err = (pr - pl)
		#if  pm > 0.15:
		#	if  (pr < 0.2) & (pl < 0.2) :
		#			speed_cmd.angular.z=0
		#		speed_cmd.linear.x=0
		#		self.speed_pub.publish(speed_cmd)
		#	elif (pr > 0.8) & (pl > 0.8) :
		#		speed_cmd.angular.z=0
		#		speed_cmd.linear.x=0
		#		self.speed_pub.publish(speed_cmd)				
		#if abs(err) > 0.125 :
		#	haja=err/0.5
		#else :
		#	haja=0
		#if haja >limit:
		#	haja=limit
		#elif haja< -limit :
		#	haja=-limit
		#speed_cmd.angular.z=haja
		#print(speed_cmd.linear.x)
		# print(speed_cmd.angular.z)
		#self.speed_pub.publish(speed_cmd)		

def main():
	rospy.init_node('line_following_node',anonymous=True)
	line_follower_object = LineFollower()
	try:
		rospy.spin()
	except KeyboardInterrupt :
		print("shutdown")
while not rospy.is_shutdown() :
	main()
