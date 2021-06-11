#! /usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point32
from std_msgs.msg import Int16
from std_msgs.msg import Bool



class LineProcessing(object):
	def __init__(self):
		self.image_sub= rospy.Subscriber("/camera/image_raw",Image,self.camera_callback)
		self.bridge_object= CvBridge()
		self.pub1=rospy.Publisher("/on_track",Bool,queue_size=1)
		self.pub2=rospy.Publisher("/high_point",PoseArray,queue_size=1)

	def camera_callback(self,data):
		start_time=time.time()	
		beat_length = 1/30
		duration = 1/5
		high_point_array=PoseArray()

		tot_white=Int16()
		tot_white.data=0
		
		while time.time() < start_time + duration:
				
			try:
				cv_image=self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
			except CvBridgeError as e:
				print(e)
			pub1=rospy.Publisher("/on_track",Bool,queue_size=1)
			pub2=rospy.Publisher("/high_point",PoseArray,queue_size=1)

			
			height,width, _ = cv_image.shape
			
			high_p=Pose()

			
			high_p.orientation.x=0
			high_p.orientation.y=0
			high_p.orientation.z=0
			high_p.orientation.w=0

			
			wicentre=100
			limit=5
			descentre =60
			rows_to_watch=90

			crop_crop= cv_image[int((height)/2+descentre):int((height)/2+(descentre+rows_to_watch)), 1:width]
			crop_down= cv_image[int((2*height)/3)-80:height-240, 1:width]
			hsvc= cv2.cvtColor(crop_crop,cv2.COLOR_BGR2HSV)
			hsvd= cv2.cvtColor(crop_down,cv2.COLOR_BGR2HSV)
			
			upper_color=np.array([25,25,25])
			lower_color=np.array([0,0,0])
			maskc=cv2.inRange(hsvc,lower_color,upper_color)
			maskd=cv2.inRange(hsvd,lower_color,upper_color)
			height2,width2, _ = crop_down.shape
			totalm = cv2.countNonZero(maskd)

			if ((totalm/(height2*width2)) > 0.75 ) & ((totalm/(height2*width2)) < 0.89 ) :
				tot_white.data+=1
				
			else:
				tot_white.data+=-1
				
				high_p.position.x=0
				high_p.position.y=0
				high_p.position.z=0
				

				
				high_point_array.poses.append(high_p)

				if time.time() < start_time + beat_length:	
					time.sleep(start_time + beat_length - time.time() )
				beat_length+= 1/30
				
								
			if  ((totalm/(height2*width2)) > 0.75  ) & ((totalm/(height2*width2)) < 0.89 ) :			
				edges = cv2.Canny(maskc, 100, 200)
				line_segments = cv2.HoughLinesP(edges, 1,np.pi / 180, 10, np.array([]), 5,0)
				
				height1, width1, _ = crop_crop.shape
				left_fit = []
				right_fit = []
				boundary = 1/3
				left_region_boundary = width1 * (1 - boundary)
				right_region_boundary = width1 * boundary 
				x_lines = []
				y_lines = []
				
				if line_segments is not None:
				
					for line_segment in line_segments:
						for x1, y1, x2, y2 in line_segment:
							if (x1 == x2):
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
						x_lines.append((x1, x2))
						y_lines.append((y1, y2))
					else:	
						x_lines.append((0, 0))
						y_lines.append((0, 0))
				
					right_fit_average = np.average(right_fit, axis=0)
					if len(right_fit) > 0:
						slope, intercept = right_fit_average
						y1 = height1  
						y2 = int(y1 * 1 / 2)  
						x1 = max(-width1, min(2 * width1, int((y1 - intercept) / slope)))
						x2 = max(-width1, min(2 * width1, int((y2 - intercept) / slope)))
						x_lines.append((x1, x2))
						y_lines.append((y1, y2))
					else:	
						x_lines.append((0, width1))
						y_lines.append((0, 0))
					
				high_p.position.x=int((x_lines[0][1]+x_lines[1][1])/2) 
				high_p.position.y=max(y_lines[0][1],y_lines[1][1])
				high_p.position.z=0
					

				
				high_point_array.poses.append(high_p)
					
				
				if time.time() < start_time + beat_length:	
					time.sleep(start_time + beat_length - time.time() )
				beat_length+= 1/30
				
								
			
		self.real_publish (high_point_array, tot_white)
		
			
	def real_publish(self,a,c):
		
		total=Bool()
		
		if c.data > 0:	
			total.data = True
		else :
			total.data = False
			
		self.pub1.publish(total)
		self.pub2.publish(a)
		print(a)
		

		

		
		
def main():
	rospy.init_node('process',anonymous=True)
	process = LineProcessing()
	
	#process.work()
	try:
		rospy.spin()
	except KeyboardInterrupt :
		print("shutdown")
while not rospy.is_shutdown() :
	main()
