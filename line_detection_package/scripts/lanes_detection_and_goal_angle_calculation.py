#!/usr/bin/env python

from __future__ import division
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Quaternion
from line_detection_package.msg import line_follower as Line
import hls
import gray_filter
import gauss_filter
import edge_filter
import math
import w_f

class LineDetector:
    def __init__(self):
        # Initialize the ROS node and set up necessary components
        rospy.init_node("line_detector", anonymous=False)
        self.rate = rospy.Rate(100)  # 10hz
        rospy.on_shutdown(self.cleanup)

        # Set up the publisher for the line detection results
        self.pub = rospy.Publisher('line_follower/detector', Line, queue_size=10)
        self.bridge = CvBridge()
        self.msg=Line()
        cv_window_name = "line_detector"
        cv2.namedWindow(cv_window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(cv_window_name, 25, 75)

    def cleanup(self):
        # Clean up any resources when the node is shutting down
        print("Shutting down line_follower node.")
        cv2.destroyAllWindows()

    def process_image(self):
        while not rospy.is_shutdown():
            # Wait for an image message from the camera topic
            image_msg = rospy.wait_for_message("/camera/image", Image)
            rate = rospy.Rate(100)  # 10hz
            frame = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.array(frame, dtype=np.uint8)

            # Apply filters to process the image
            whitefilter = w_f.white(frame)
            gauss = gauss_filter.apply_smoothing(whitefilter, 15)
            edges = edge_filter.detect_edges(gauss, 50, 150)

            # Region of interest (ROI) processing of bird eye view
            offset1=edges.shape[1]/2
            height_points_top = 275
            mikostop = 228
            mikosbot = 228
            ip_sim_top=180
            pts1 = np.float32([(offset1-mikostop,height_points_top),(offset1-mikosbot,edges.shape[0]),(offset1+mikosbot,edges.shape[0]),(offset1+mikostop,height_points_top)])
            offset=210
            c=cv2.polylines(frame, np.int32([pts1]),True, (255,0,0), 3)
            pts2 = np.float32([((edges.shape[1]/2)-offset,0),((edges.shape[1]/2)-offset,edges.shape[0]),((edges.shape[1]/2)+offset,edges.shape[0]),((edges.shape[1]/2)+offset,0)])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(edges,M,(edges.shape[1],edges.shape[0]))

            # Perform Hough line detection
            lines= cv2.HoughLinesP(dst, rho=1, theta=np.pi/180, threshold=80, minLineLength=30, maxLineGap=10)

            # Process detected lines
            left_lines    = [] # (slope, intercept)
            left_weights  = [] # (length,)
            right_lines   = [] # (slope, intercept)
            right_weights = [] # (length,
            
            HEIGHT_R = 0.5 #the height of the frame
            line_image = np.zeros_like(frame)

            if lines is not None:
                for line in lines:
                    for x1,y1,x2,y2 in line:
                        K=cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)

                        if x1 == x2 : continue
                        slope = (y2-y1)/(x2-x1)
                        intersectx=x2+(dst.shape[0]-y2)/slope
                        length = np.sqrt((y2-y1)**2+(x2-x1)**2)
                        if intersectx < 0 or intersectx >= dst.shape[1]:
                            continue


                        if intersectx<(dst.shape[1]/2):
                            left_lines.append((slope, intersectx))
                            left_weights.append((length))
                        else:
                            right_lines.append((slope, intersectx))
                            right_weights.append((length))
                left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
                right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None

                if left_lane is not None:
                    if right_lane is not None:
                        slope1, intersectx1 = left_lane
                        slope2,intersectx2=right_lane

                        point0 = (int(intersectx1), dst.shape[0])
                        

                        point1 = (int(frame.shape[0] * (HEIGHT_R - 1) / slope1 + intersectx1), int(dst.shape[0] * HEIGHT_R))
                        '''
                        print(int(frame.shape[0]))
                        print((HEIGHT_R - 1))
                        print(int(frame.shape[0] * (HEIGHT_R - 1)))
                        print(slope1)
                        print(int(frame.shape[0] * (HEIGHT_R - 1) / slope1))
                        print(intersectx1)
                        print((int(frame.shape[0] * (HEIGHT_R - 1) / slope1 + intersectx1)))
                        '''

                        point2 = (int(intersectx2), dst.shape[0])

                        point3 = (int(frame.shape[0] * (HEIGHT_R - 1) / slope2 + intersectx2), int(dst.shape[0]* HEIGHT_R))

                        #print(point0, point1, point2, point3)
                        


                        l,k=point0
                        n,m=point1
                        o,ksi=point2
                        ro,pe=point3

                        eqe1xpoints=[]
                        eqe1ypoints=[]
                        eqe2xpoints=[]
                        eqe2ypoints=[]

                        points_left=abs(n-l)
                        #print('points_left',points_left)
                        

                        points_right=abs(o-ro)
                        #print('points_right',points_right)

                        desired_number_of_points=4
                        step1=int(round(points_left/desired_number_of_points))
                        step2=int(round(points_right/desired_number_of_points))

                        #print('step1',step1)
                        #print('step2',step2)
                        



                        #LINE1 EQUATION
                        b1=(k-l*slope1)
                        b1=int(round(b1))
                        if (step1==0):
                            step1=1
                        if n>l:
                            for every_xe1 in range(l,n,step1):
                                every_ye1=(every_xe1*slope1+b1)
                                every_ye1=int(round(every_ye1))

                                eqe1xpoints.append((every_xe1))
                                eqe1ypoints.append((every_ye1))
                        else:
                            for every_xe1 in range(l,n,-step1):
                                every_ye1=(every_xe1*slope1+b1)
                                every_ye1=int(round(every_ye1))

                                eqe1xpoints.append((every_xe1))
                                eqe1ypoints.append((every_ye1))
                                


                        eqe1xpoints=np.asarray(eqe1xpoints)
                        
                        eqe1ypoints=np.asarray(eqe1ypoints)


                        b2=(ksi-o*slope2)
                        b2=int(round(b2))
                        if (step2==0):
                            step2=1
                        #print('o',(o),'ro',(ro))
                        if (o>ro):
                            for every_xe2 in range(o,ro,-step2):
                                every_ye2=int(every_xe2*slope2+b2)
                                every_ye2=int(round(every_ye2))

                                eqe2xpoints.append((every_xe2))
                                eqe2ypoints.append((every_ye2))
                        else:
                            for every_xe2 in range(o,ro,step2):
                                every_ye2=int(every_xe2*slope2+b2)
                                every_ye2=int(round(every_ye2))

                                eqe2xpoints.append((every_xe2))
                                eqe2ypoints.append((every_ye2))
                                


                        eqe2xpoints=np.asarray(eqe2xpoints)
                        eqe2ypoints=np.asarray(eqe2ypoints)

                        #print('eqe1xpoints_before',eqe1xpoints,'eqe2xpoints_before', eqe2xpoints,'eqe1ypoints_before',eqe1ypoints,'eqe2ypoints_before', eqe2ypoints )

                        if len(eqe1xpoints) < len(eqe2xpoints):
                            diff=len(eqe2xpoints)-len(eqe1xpoints)
                            eqe2xpoints=eqe2xpoints[0:len(eqe2xpoints)-diff]
                            f=len(eqe2xpoints)
                            eqe2ypoints=eqe2ypoints[0:f]

                        if len(eqe1xpoints) > len(eqe2xpoints):
                            diff=len(eqe1xpoints)-len(eqe2xpoints)
                            eqe1xpoints=eqe1xpoints[0:len(eqe1xpoints)-diff]
                            f=len(eqe1xpoints)
                            eqe1ypoints=eqe1ypoints[0:f]

                        f=len(eqe1xpoints)
                        #print('eqe1xpoints',eqe1xpoints,'eqe2xpoints', eqe2xpoints,'eqe1ypoints',eqe1ypoints,'eqe2ypoints', eqe2ypoints )
                        new_xpoints=(eqe1xpoints+eqe2xpoints)/2
                        new_xpoints=new_xpoints.astype(int)
                        #print("new_xpoints",len(new_xpoints),new_xpoints)

                        #print("newxpoints",new_xpoints)
                        new_ypoints=(eqe1ypoints+eqe2ypoints)/2
                        new_ypoints=new_ypoints.astype(int)
                        #print("new_ypoints",len(new_ypoints),new_ypoints)

                        e=cv2.line(frame, (point0),(point1), [255, 0, 240], thickness=20)
                        fe=cv2.line(frame, (point2),(point3), [255, 0, 240], thickness=20)

                        for i in range(f-1):
                            fe=cv2.line(frame, (new_xpoints[i],new_ypoints[i]),(new_xpoints[i+1],new_ypoints[i+1]), [255,0,0], thickness=2)

                        j=cv2.line(frame, ((int(dst.shape[1]/2)),frame.shape[0]),((int(dst.shape[1]/2)),int(dst.shape[0]/4)), [255,0,0], thickness=2)

                        x_1=new_xpoints[0]
                        if x_1 is None:
                            break
                    
                        x_2=int(dst.shape[1]/2)

                        y_1=dst.shape[0]
                        #print("y_1",y_1)
                        
                        if (int(dst.shape[1]/2)-new_xpoints[0])==0:
                            new_xpoints[0]=new_xpoints[0]+1
                            y_2=(np.float64((new_ypoints[f-1]-new_ypoints[0])/(new_xpoints[f-1]-new_xpoints[0]))*(int(dst.shape[1]/2)-new_xpoints[0])+new_ypoints[0])
                            '''
                            print(new_ypoints[f-1])
                            print(new_ypoints[0])
                            print((new_ypoints[f-1]-new_ypoints[0]))
                            print(new_xpoints[f-1])
                            print(new_xpoints[0])
                            print(new_xpoints[f-1]-new_xpoints[0])
                            print(new_xpoints[f-1]-new_xpoints[0])
                            print(np.float64((new_ypoints[f-1]-new_ypoints[0])/(new_xpoints[f-1]-new_xpoints[0])))
                            print(int(dst.shape[1]/2))
                            print(new_xpoints[0])
                            print(new_xpoints[0])
                            print(int(dst.shape[1]/2)-new_xpoints[0])
                            print(new_ypoints[0])
                            '''
                        else:y_2=(np.float64((new_ypoints[f-1]-new_ypoints[0])/(new_xpoints[f-1]-new_xpoints[0]))*(int(dst.shape[1]/2)-new_xpoints[0])+new_ypoints[0])


                        

                    

                    if y_2==np.float("inf"):
                            y_2=np.float("inf")
                    elif y_2==np.float("-inf"):
                        y_2=np.float("-inf")
                    else:
                        y_2=round(y_2)
                    #if y_2>480:
                        #print("aaaaaaaaaa",y_2)

                    #NO ANGLE
                    if ((x_1 == x_2) and (y_1==y_2)):
                        
                        fi_right=0
                        fi_left=0
                        #print("go straight")

                        self.msg.line_detected = True
                        self.msg.time_det = rospy.Time.now()
                        self.msg.orientation = "Left"
                        self.msg.error_deg = fi_left
                        rospy.loginfo(self.msg)
                        self.pub.publish(self.msg)

                        self.msg.line_detected = True
                        self.msg.time_det = rospy.Time.now()
                        self.msg.orientation = "Right"
                        self.msg.error_deg = fi_right
                        rospy.loginfo(self.msg)
                        self.pub.publish(self.msg)




                    else:
                        d1=(math.sqrt(((x_2-x_1)**2)+((y_2-y_1)**2)))
                        d2=abs(round(x_2-x_1))
                        #print("D2",d2)
                        
                        rad=math.atan(d2/d1)
                        #rad = rad * (-1)
                        fi=math.degrees(rad)
                        print("fi",fi)
                        



                    
                    #### AS THE PARAMETER DECREASES, THE SYSTEM BECOMES MORE SENSITIVE. GOOD VALUES ARE BETWWEN 500-1000
                        PARAMETER=900
                
                        
                        ##IM ON THE LEFT SIDE OF THE ROAD AND I HAVE TO TO BE ON THE CENTER(NO TURN AHEAD)
                        if (((y_2>480) or (y_2<0)) and (fi<2) and (fi != 0) and (x_1>x_2) ):
                            distance_x=abs(x_1-x_2)
                            
                            rad=distance_x/PARAMETER
                            fi_right=fi
                            '''
                            print("fi_right",fi_right)
                            print("fi",fi)
                            print("im on the left side")
                            print("y_2",y_2)
                            print("x_1",x_1)
                            print("x_2",x_2)
                            '''

                            self.msg.line_detected = True
                            self.msg.time_det = rospy.Time.now()
                            self.msg.orientation = "Right"
                            self.msg.error_rad = rad
                            #self.msg.error_rad = 2*rad
                            self.msg.error_deg = fi_right
                            rospy.loginfo(self.msg)
                            self.pub.publish(self.msg)

                        
                        ## IM ON THE RIGHT SIDE OF THE ROAD, AND I HAVE TO BE ON THE CENTER(NO TURN AHEAD)
                        if (((y_2>480) or (y_2<0)) and (fi<2) and (fi != 0) and (x_1<x_2) ):
                            distance_x=abs(x_1-x_2)
                            fi_left=fi
                            #rad=math.atan(d1/d2)
                            rad=distance_x/PARAMETER
                            '''
                            print("fi_left",fi_left)
                            print("fi",fi)
                            print("im on the right side")
                            print("y_2",y_2)
                            print("x_1",x_1)
                            print("x_2",x_2) 
                            '''

                            self.msg.line_detected = True
                            self.msg.time_det = rospy.Time.now()
                            self.msg.orientation = "Left"
                            self.msg.error_rad = rad
                            #self.msg.error_rad = 2*rad
                            self.msg.error_deg = fi_left
                            rospy.loginfo(self.msg)
                            self.pub.publish(self.msg)            


                        ##right turn ahead. turn right
                        if (((x_1 > x_2)and(y_1 < y_2)) or ((x_1 < x_2)and(y_1 > y_2))) and (fi>2):
                            fi_right=fi
                            #print("im turning right with angle",fi_right)

                            self.msg.line_detected = True
                            self.msg.time_det = rospy.Time.now()
                            self.msg.orientation = "Right"
                            self.msg.error_rad = rad
                            self.msg.error_deg = fi_right
                            rospy.loginfo(self.msg)
                            self.pub.publish(self.msg)   

                            

                    

                        ##left turn ahead. turn left
                        if (((x_1>x_2)and(y_1>y_2)) or ((x_1<x_2)and(y_1<y_2))) and (fi>2):
                            fi_left=fi
                            #print("im turning left with angle",fi_left)
                    
                            self.msg.line_detected = True
                            self.msg.time_det = rospy.Time.now()
                            self.msg.orientation = "Left"
                            self.msg.error_rad = rad
                            self.msg.error_deg = fi_left
                            rospy.loginfo(self.msg)
                            self.pub.publish(self.msg) 
                else:
                    print('no left or right lanes are detected now')

            else:
                self.msg.line_detected = False
                self.msg.time_det = rospy.Time.now()
                self.msg.orientation = "NO LINES"
                self.msg.error_rad = 0
                self.msg.error_deg = 0
                rospy.loginfo(self.msg)
                self.pub.publish(self.msg)

            line_image = np.zeros_like(frame)
            de = cv2.addWeighted(frame, 1.0, line_image, 0.95, 0.0)


            # Display the image
            
            #cv2.imshow("dst", dst)
            #cv2.imshow("Frrame", de)
            cv2.imshow("frame",frame)
            
            #cv2.imshow("wf",whitefilter)
            



            

            # Process any keyboard commands
            keystroke = cv2.waitKey(5)
            if keystroke != -1:
                cc = chr(keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

            self.rate.sleep()
        


    def run(self):
        
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("/camera/image", Image)
        rospy.loginfo("Ready.")

        
        self.process_image()
        
    
            


def main():
    try:
        line_detector = LineDetector()
        line_detector.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        print("Shutting down line_follower node.")

if __name__ == '__main__':
    main()
