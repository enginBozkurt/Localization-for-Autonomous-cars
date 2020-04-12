/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: velodyne_plugin.cpp
* Author: Mahadevaprabhu N , Vijaymeyapan V
* Version: 1.0
* Date: 10-05-2017
* Operating Environment: Linux -ubuntu 16.04, ROS Kinetic distro.,gazebo8
* Compiler with Version Number: gcc version 4.8.4
* Description: Keyboard control
* List of functions used: 
* Revisers Name:
* Date:
* Customer Bug No./ CMF No. :
* Brief description of the fix/enhancement: Replacement of constant numbers with macros. Deleting commented code.
* Created by Tata Elxsi Ltd., < Automotive Group >
* Copyright <Year>Tata Elxsi Ltd.
* All rights reserved.
This code contains information that is proprietary to Tata Elxsi Ltd.
No part of this document/code may be reproduced or used in whole or
part in any form or by any means - graphic, electronic or mechanical
without the written permission of Tata Elxsi Ltd
**********************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <linux/input.h>   //Needed for taking key events from Linux 
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <std_msgs/Float32.h>
#include <fcntl.h>
#include <termios.h>
#include "simulation/SimJoystickAndKeyboard.h"
#include <cstdlib>
#include <unistd.h>     //Needed for close()
#include <string>
#include <stdlib.h>
#include <cstring>
#include <X11/Xlib.h>   //Needed for finding out the active window. Taking keyinputs only if Rviz window is active.
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/keysym.h>

#define MAXSTR 1000
#define EV_PRESSED 1
#define EV_RELEASED 0
#define EV_REPEAT 2

#define  X_KEY 1   //Make 0 for Linux Key Event and 1 for Xlib method

using namespace std;

Display *display;
unsigned long window;
unsigned char *prop;
int screen;
unsigned int count=0;



float g_fCarAngleIncrement=0.09;   //0.01744 is 1 degree 0.09 is 5 degrees
float g_fCarSpeedIncrement=0.1,g_fCarSpeedDecrement=0.1;
float g_fCarLength=2.6; //This is actually wheelbase from the veloster vehicle model
float g_fCarWidth=1.56;     //This is actually vehicle track from the veloster vehicle model
float g_fRadiusIdeal=5.2;   //This is the turning radius as per the veloster spec sheet
float g_fAccMax=0.3;        //Maximum Value of Acceleration
float g_fRevMax=0.3;        //Maximum Value of Acceleration in Reverse
float g_fDccMax=-1.0;        //Maximum Value of Decceleration
float g_fSpeedLevel=0;      //This is the speed level of the car. +ve denotes acceleration. -ve denotes deceleration.
float g_fMaxSteerInside=0.6,g_fMaxSteer=0;
float g_fRelease=0;
float g_fStraight=0;
float g_fRadiusMax=0,g_fCarWidthBuffer,g_fCarLengthBuffer,g_fComputeRadius,g_fLeftWheelRadius,g_fRightWheelRadius;
bool g_bestop=false;
std_msgs::Float32 g_fMsgRearR,g_fMsgRearL,g_fMsgSteerL,g_fMsgSteerR;
std_msgs::Float32 g_fMsgRear,g_fMsgSteer;
//double g_fMsgRearR,g_fMsgRearL;
char chLetter;
float g_fUpArrow1=0,g_fAckermannData=0,g_fUpArrow=0, g_fSteering=0, g_fUpArrowTemp=0, g_fDownArrow=0, g_fDownArrowTemp=0;
float tr = 0;

void check_status(int status, unsigned long window)
{
    if (status == BadWindow) 
    {
        ROS_INFO("window id # 0x%lx does not exists!", window);
        exit(1);
    }

    if (status != Success) 
    {
        ROS_INFO("XGetWindowProperty failed!");
        exit(2);
    }
}

unsigned char* get_string_property(char* property_name)
{
    Atom actual_type, filter_atom;
    int actual_format, status;
    unsigned long nitems, bytes_after;

    filter_atom = XInternAtom(display, property_name, True);

    status = XGetWindowProperty(display, window, filter_atom, 0, MAXSTR, False, AnyPropertyType,
                                &actual_type, &actual_format, &nitems, &bytes_after, &prop);

    check_status(status, window);

    if(status == 0)
      return prop;
    /*else
      return NULL;*/
}

unsigned long get_long_property(char* property_name)
{
    get_string_property(property_name);
    unsigned long long_property = prop[0] + (prop[1]<<8) + (prop[2]<<16) + (prop[3]<<24);

    return long_property;
}

void keypressaction(XEvent event)
{                     
                      switch(event.xkey.keycode) 
		      {
                        case 111://UP 111
                                //ROS_INFO("KeyCode = %d",event.xkey.keycode);
                                   g_fSpeedLevel+=g_fCarSpeedIncrement;
                                   if(g_fSpeedLevel>=g_fAccMax)
                                   { 	
				      g_fSpeedLevel=g_fAccMax;
                                      g_fUpArrow=g_fSpeedLevel;
                                      g_fDownArrow=g_fRelease;
                                   }
				   else if(g_fSpeedLevel >= 0)
				   {    
                                      g_fUpArrow=g_fSpeedLevel;
                                      g_fDownArrow=g_fRelease;
                                   }
                                   else
                                   {
                                      g_fSpeedLevel=g_fRelease;
                                      g_fUpArrow=g_fRelease;
                                      g_fDownArrow=g_fRelease;
                                   }
				break;

		        case 56:
                                //ROS_INFO("KeyCode = %d",event.xkey.keycode);
                                   g_fSpeedLevel-=g_fCarSpeedIncrement;
                                 
                                   if(g_fSpeedLevel<=g_fDccMax)
                                   { 	
				      g_fSpeedLevel=g_fDccMax;
                                      g_fDownArrow=-1*g_fDccMax;
                                      g_fUpArrow=g_fRelease;
                                   }
				   else if(g_fSpeedLevel <= 0)
				   {    
                                      g_fDownArrow=-1*g_fSpeedLevel;
                                      g_fUpArrow=g_fRelease;
                                   }
                                   else
                                   {
                                      g_fSpeedLevel=g_fRelease;
                                      g_fUpArrow=g_fRelease;
                                      g_fDownArrow=g_fRelease;
                                   }
                                   break;

		        case 114: //RIGHT 114
                                   g_fSteering -= g_fCarAngleIncrement;
				break;

			case 113: //LEFT 113
				g_fSteering += g_fCarAngleIncrement;
	                  	break;

		        case 116: //DOWN 116
                                   g_fSpeedLevel=g_fRelease;
                                   g_fUpArrow = -1.0*g_fRevMax;
                                break;
			case 0x18:  //Q
				system("stty cooked");
                                system("killall -9 gzclient");
                                system("killall -9 gzserver");
                                system("killall -9 rviz");
				exit(0);
				break;

                        default:
                               XUngrabKeyboard(display, CurrentTime);
                        break;
			}
}



/*********************************************************************************************
FunctionName  : main function
Description   : Getting the input data from user,calculate the ackerman steering angle and 
                Publishing acc,de-acc position and angle.
Arguments     : No arguments
ReturnType    : Int datatype
*************************************************/

/**  main function .
*
*\param argc int inline arguement
*\param argv char inline arguement
*
* \return type     int     
*/

int main(int argc, char **argv)
{
        //ROS Initialization
	ros::init(argc,argv,"keyboard_input");
	ros::NodeHandle n;
	simulation::SimJoystickAndKeyboard msg1;
	ros::Rate loop_rate(100);
	ros::Publisher pub_car_control=n.advertise<simulation::SimJoystickAndKeyboard>("/bmw/vel_cmd_key",1);

        sleep(6);

        //Initialization to find active window
        char *display_name = NULL; // could be the value of $DISPLAY

        display = XOpenDisplay(display_name);

        char *str_rviz_window_name="RViz";
        char *str_gazebo_window_name="Gazebo";
        char *str_current_window_name;
        int rviz_resize=0;
      
        if (display == NULL) 
        {
             fprintf (stderr, "%s: unable to open display '%s'\n", argv[0], XDisplayName (display_name));
        }
        XAutoRepeatOn(display);  
        screen = XDefaultScreen(display);

        char* Key_Id;
        Key_Id = getenv("GAZEBO_KEYBOARD_EVENT_ID");

        int fd = 0;
        char *device_start = "/dev/input/"; // This is the keyboard device as identified using both: $cat /proc/bus/input/devices 
// and looking in the var/log/Xorg.0.log searching for "keyboard"
        stringstream ss;
        ss<<device_start<<Key_Id;
        const std::string tmp = ss.str();
        const char* device = tmp.c_str();
        //Initialization to find key events from the keyboard
#if !X_KEY
        fd = open(device, O_RDONLY);  //O_RDWR Indicates Read/Write
        if( fd <= 0 )
        {
             return 0;
        }
#endif   
        ROS_INFO("Initialization Complete! You may proceed to control the vehicle.\n");
        

/*
        FILE *fpr=popen("wmctrl -a \"Gazebo\" | wmctrl -a \"RViz\" ","r");
 	if( fpr == 0 ) 
	{
       	   fprintf( stderr, "Could not execute\n" );
           return 1;
    	}
    	else
	{
    	   pclose( fpr );	
	}
  */           
        
        //struct input_event event;
        	
	//system("wmctrl -a \"RViz\"");

        //Main loop
  	while(ros::ok())
  	{    
             window = RootWindow(display, screen);
             window = get_long_property("_NET_ACTIVE_WINDOW");
#if X_KEY       
             XUngrabKeyboard(display, CurrentTime); 
             XEvent event;
             //XSelectInput(display, window, KeyPressMask | KeyReleaseMask | FocusChangeMask | ExposureMask);
             XSelectInput(display, window, KeyPressMask | KeyReleaseMask);
             //XSelectInput(display, window, ExposureMask);
             //XMapWindow (display, window);
             //XFlush (display);
             //str_current_window_name = (char *) get_string_property("_NET_WM_NAME");
             //ROS_INFO("_NET_WM_NAME: %s\n", get_string_property("_NET_WM_NAME"));

             //ROS_INFO("_NET_WM_PID: %lu\n", get_long_property("_NET_WM_PID"));
             //ROS_INFO("WM_CLASS: %s\n", get_string_property("WM_CLASS"));
             //ROS_INFO("_NET_WM_NAME: %s\n", get_string_property("_NET_WM_NAME"));
             //str_current_window_name = (char *) get_string_property("_NET_WM_NAME");
//	     cout<<"str_current_window_name"<<str_current_window_name<<endl;
             //ROS_INFO("CURRENT_WINDOW_NAME: %s\n", str_current_window_name);
#else
             struct input_event event;

#endif
             unsigned char *winname = get_string_property("_NET_WM_NAME");

             if(winname != NULL)
             {
             /* select kind of events we are interested in */
             if(  strncmp( (char *) (str_gazebo_window_name),
                         ( (char *) get_string_property("_NET_WM_NAME")),6) == 0 || 
                   strncmp( (char *) (str_rviz_window_name),
                         ( (char *) get_string_property("_NET_WM_NAME")),4) == 0 )  
/*                   strncmp( (char *) (str_rviz_window_name),
                         ( (char *) get_string_property("_NET_WM_NAME")),4) == 0 ) */
              {
                // ROS_INFO("Entered window condition");
               

/*
              if(rviz_resize==0)
              {
                 if(strncmp( (char *) (str_rviz_window_name),
                         ( (char *) get_string_property("_NET_WM_NAME")),4) == 0 )
                 {
			FILE *fpr=popen("wmctrl -a \"Gazebo\" | wmctrl -a \"RViz\" | wmctrl -r \"RViz\" -e 0,500,0,800,300 | wmctrl -a \"RViz\" ","r");
 			if ( fpr == 0 ) 
			{
       				fprintf( stderr, "Could not execute\n" );
        			return 1;
    			}
    			else
			{
    				pclose( fpr );	
			}
                     
                 }
                 rviz_resize=1;
              }
*/
#if X_KEY       
                 //if(XPending(display))
                 {

                  XGrabKeyboard(display, window, True, KeyPressMask, GrabModeAsync, CurrentTime);

                  XFlush(display);
                  //while(XPending(display))  
                  XNextEvent(display, &event);
                  
                  if(event.type == KeyPress)
                  {
                     if(event.xkey.keycode == 26) //Letter 'e' for E-STOP
                     {
                       if(g_bestop == false)
                         g_bestop=true;
                       else
                         g_bestop=false;
                     }
                     else
                     keypressaction(event);   
                  }
                  else if(event.type == KeyRelease)
                  {
                          unsigned short is_retriggered = 0;

                          if(XEventsQueued(display, QueuedAfterReading))
                          {
                               XEvent nev;
                               XPeekEvent(display, &nev);

                               if (nev.type == KeyPress && nev.xkey.time == event.xkey.time &&
                                   nev.xkey.keycode == event.xkey.keycode)
                               {
                                  
                                  keypressaction(event);                 
                                  
                                  XNextEvent (display, &event); // delete retriggered KeyPress event
                                  is_retriggered = 1;
                               }
                          }
                          if (!is_retriggered)
                          {
                               //Key Release Action
                             switch(event.xkey.keycode)
                             {
                                case 111://UP 111
                                g_fSpeedLevel=g_fRelease;
                                g_fUpArrow=g_fRelease;
                                g_fDownArrow=g_fRelease;
			        break;

		                case 56:  //BRAKE
                                g_fSpeedLevel=g_fRelease;
                                g_fUpArrow=g_fRelease;
                                g_fDownArrow=g_fRelease;
			        break;
                    
		                case 116: //DOWN 116
                                g_fSpeedLevel=g_fRelease;
                                g_fUpArrow=g_fRelease;
                                g_fDownArrow=g_fRelease;
			        break;

        			case 113: //LEFT 113
                                g_fSteering = g_fStraight;
			        break;
		                
                                case 114: //RIGHT 114
                                g_fSteering = g_fStraight;
			        break;
                             }
                          }
                   }
                  }  

#else
		  //ROS_INFO("Active Window is RViz\n");
                  read(fd, &event, sizeof(struct input_event));
                   
                  if(event.type == EV_KEY) 
                  {

		      switch(event.code) 
		      {
			// For similar macros of KEY_UP refer to /usr/src/linux-headers-4.4.0-81/
                        // include/uapi/linux/input-event-codes.h
                        case KEY_UP:
			        if(event.value == EV_PRESSED || event.value == EV_REPEAT)
                                { 
                                   g_fSpeedLevel+=g_fCarSpeedIncrement;
                                   if(g_fSpeedLevel>=g_fAccMax)
                                   { 	
				      g_fSpeedLevel=g_fAccMax;
                                      g_fUpArrow=g_fSpeedLevel;
                                      g_fDownArrow=g_fRelease;
                                   }
				   else if(g_fSpeedLevel >= 0)
				   {    
                                      g_fUpArrow=g_fSpeedLevel;
                                      g_fDownArrow=g_fRelease;
                                   }
                                   else
                                   {
                                      g_fSpeedLevel=g_fRelease;
                                      g_fUpArrow=g_fRelease;
                                      g_fDownArrow=g_fRelease;
                                   }
                                }
                                else  //EV_RELEASED
                                {
                                   g_fSpeedLevel=g_fRelease;
                                   g_fUpArrow=g_fRelease;
                                   g_fDownArrow=g_fRelease;
                                }
				break;

		        case KEY_B:
			        if(event.value == EV_PRESSED || event.value == EV_REPEAT)
			        {
                                   g_fSpeedLevel-=g_fCarSpeedIncrement;
                                 
                                   if(g_fSpeedLevel<=g_fDccMax)
                                   { 	
				      g_fSpeedLevel=g_fDccMax;
                                      g_fDownArrow=-1*g_fDccMax;
                                      g_fUpArrow=g_fRelease;
                                   }
				   else if(g_fSpeedLevel <= 0)
				   {    
                                      g_fDownArrow=-1*g_fSpeedLevel;
                                      g_fUpArrow=g_fRelease;
                                   }
                                   else
                                   {
                                      g_fSpeedLevel=g_fRelease;
                                      g_fUpArrow=g_fRelease;
                                      g_fDownArrow=g_fRelease;
                                   }
                                }
                                else  //EV_RELEASED
                                {
                                   g_fSpeedLevel=g_fRelease;
                                   g_fUpArrow=g_fRelease;
                                   g_fDownArrow=g_fRelease;
                                }
			        break;

		        case KEY_RIGHT:
			        if(event.value == EV_PRESSED || event.value == EV_REPEAT)
                                   g_fSteering -= g_fCarAngleIncrement;
                                else  //EV_RELEASED
                                   g_fSteering = g_fStraight;
				break;

			case KEY_LEFT:
			        if(event.value == EV_PRESSED || event.value == EV_REPEAT)
				g_fSteering += g_fCarAngleIncrement;
                                else  //EV_RELEASED
                                   g_fSteering = g_fStraight;
	                  	break;

		        case KEY_DOWN:
			        if(event.value == EV_PRESSED || event.value == EV_REPEAT)
                                {  
                                   g_fSpeedLevel=g_fRelease;
                                   g_fUpArrow = -1.0*g_fRevMax;
                                   g_fDownArrow=g_fRelease;
                                }
                                else  //EV_RELEASED
                                {
                                   g_fSpeedLevel=g_fRelease;
                                   g_fUpArrow=g_fRelease;
                                   g_fDownArrow=g_fRelease;
                                }
                                break;
                                
			case KEY_Q:
				system("stty cooked");
				exit(0);
				break;
			}
                     }
#endif
                  }
                  }
			//g_fRadiusMax = g_fCarLength/tan(g_fMaxSteerInside);
			//g_fRadiusIdeal = g_fRadiusMax+(g_fCarWidth/2.0);
                        //g_fRadiusIdeal is defined as per the spec sheet of the veloster model
			g_fMaxSteer=atan2(g_fCarLength,g_fRadiusIdeal);
			g_fUpArrow1 = 1*g_fUpArrow;
			//g_fAckermannData = max(-g_fMaxSteer,min(g_fMaxSteer,g_fSteering));
			if(g_fSteering>=1)
			g_fSteering=1;
			if(g_fSteering<=-1)
			g_fSteering=-1;

			g_fAckermannData =(g_fMaxSteer*g_fSteering);		
			/*finding ackerman steering angle*/
			if (g_fAckermannData != 0)
	 		{	
				//printf("\nin if part\n");
				g_fCarWidthBuffer=g_fCarWidth;
				g_fCarLengthBuffer=g_fCarLength;
				g_fComputeRadius = g_fCarLengthBuffer/fabs(tan(g_fAckermannData));
				tr = sqrt(pow(g_fComputeRadius,2) + pow(g_fCarLengthBuffer,2));
				g_fLeftWheelRadius = sqrt(pow((tr-(g_fCarWidthBuffer/2.0)),2) +pow(g_fCarLengthBuffer,2)) ;
				g_fRightWheelRadius = sqrt(pow((tr+(g_fCarWidthBuffer/2.0)),2) +pow(g_fCarLengthBuffer,2)) ;
				//g_fLeftWheelRadius = g_fComputeRadius-(copysign(1,g_fAckermannData)*(g_fCarWidthBuffer/2.0));
				//g_fRightWheelRadius = g_fComputeRadius+(copysign(1,g_fAckermannData)*(g_fCarWidthBuffer/2.0));
				msg1.mult_fl2 = (g_fLeftWheelRadius/g_fComputeRadius);
				msg1.mult_fr2 = (g_fRightWheelRadius/g_fComputeRadius);
				msg1.up = g_fUpArrow; 
				msg1.down = g_fDownArrow;   
				msg1.steering_left_2= (atan2(g_fCarLengthBuffer,g_fLeftWheelRadius)*copysign(1,g_fAckermannData));
				msg1.steering_right_2 = (atan2(g_fCarLengthBuffer,g_fRightWheelRadius)*copysign(1,g_fAckermannData));
				msg1.key_steering_angle = g_fAckermannData;
                                msg1.estop = g_bestop;

				//ROS_INFO("\nvalue of  up =%f", msg1.up);
				/*ROS_INFO("\nvalue of  g_fMsgRearR =%f", msg1.down);
				ROS_INFO("\nvalue of  g_MsgSteerL =%f", msg1.steering_left_2);
				ROS_INFO("\nvalue of  g_MsgSteerR =%f", msg1.steering_right_2);*/
				pub_car_control.publish(msg1);
			}
			else
	  		{	
				msg1.mult_fl2 = 1;
				msg1.mult_fr2 = 1;
				msg1.up= g_fUpArrow;
				msg1.down=g_fDownArrow;
				msg1.steering_left_2=g_fAckermannData;
				msg1.steering_right_2=g_fAckermannData;
				msg1.key_steering_angle = g_fAckermannData;
                                msg1.estop = g_bestop;
				//ROS_INFO("\nvalue of up =%f", msg1.up);
				/*ROS_INFO("\nvalue of  g_fMsgRear of g_fMsgRearR =%f", msg1.down);
				ROS_INFO("\nvalue of  g_MsgSteerL =%f", msg1.steering_left_2);   
				ROS_INFO("\nvalue of  g_MsgSteerR =%f", msg1.steering_right_2);*/
				pub_car_control.publish(msg1);
			}
	
	}
	ros::spinOnce();
	loop_rate.sleep();
 	return 0;
}
