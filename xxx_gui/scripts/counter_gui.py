#!/usr/bin/env python3
import math
import pygame
from pygame.locals import *
import numpy as np
import rospy
from std_msgs.msg import Bool
from rosserial_arduino.msg import Adc
class Node :
    def __init__(self,node_name):
        self.node_name = node_name
    # initialize node
        rospy.init_node(self.node_name)
    # initialize subscribers
        rospy.Subscriber('/pushed',Bool,self.callback_pushed)
        self.pushed = Bool()
        rospy.Subscriber('/adc',Adc,self.callback_adc)
        self.adc = Adc()
    # initialize publishers
        self.pub_LED = rospy.Publisher('/LED',Bool,queue_size=10)
        self.count = 0
    # assign on_shutdown behavior
        rospy.on_shutdown(self.callback_shutdown)        
    def callback_pushed(self,msg):
        self.pushed = msg
        if msg.data:
            self.count += 1
    def callback_adc(self,msg):
        self.adc = msg
    #def publish(self,event):
        #pass
    def callback_shutdown(self):
        print('\n\n... The node "'+self.node_name+'" has shut down...\n')
class GUI:
    size_screen_initial = [500,500]
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.size_screen_initial,HWSURFACE | DOUBLEBUF | RESIZABLE)
        pygame.display.set_caption('xxx_gui')
class Button:
    def __init__(self,size,text):
        self.text = text
        self.size = size
        self.state = 0
        self.font = pygame.font.Font("freesansbold.ttf",20)
        self.center = ( (size[0]+(size[2]/2)), (size[1]+(size[3]/2)) )
    def draw(self,screen,mouse):
        if isinstance(self.text, str):
            text_list = (self.text,self.text,self.text)
        elif len(self.text)==2:
            text_list = (self.text[0],self.text[0],self.text[1])
        else:
            text_list = self.text
        if self.isInBox(mouse,self.size):
            if self.state == 0:
                color = (0,128,0)
                text = text_list[0]
            else:
                color = (128,0,0)
                text = text_list[1]
        else:
            if self.state == 0:
                color = (0,255,0)
                text = text_list[0]
            else:
                color = (255,0,0)
                text = text_list[1]
            
        pygame.draw.rect(screen,color,self.size)
        text_object = self.font.render(text , True ,(0,0,0))
        screen.blit(text_object, self.center)
    def isMouseHover(self,mouse):
        return self.isInBox(mouse,self.size)
    def isInBox(self,var,size):
        return (size[0]<=var[0]<=size[0]+size[2]) and (size[1]<=var[1]<=size[1]+size[3])


# main
if __name__=="__main__":
    node = Node('gui')
    # create graph from 
    gui = GUI()
    button_LED = Button((100,300,150,70),["Off,On"])
    font = pygame.font.SysFont("Courier",38)

    inGame = True
    while inGame:
        mouse = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == QUIT: 
                inGame = False
            #elif event.type == VIDEORESIZE:
            #    pass
            elif event.type == MOUSEBUTTONDOWN:
                if button_LED.isMouseHover(mouse):
                    msg = Bool()
                    msg.data = True
                    node.pub_LED.publish(msg)
            elif event.type == MOUSEBUTTONUP:
                msg = Bool()
                msg.data = False
                node.pub_LED.publish(msg)

        gui.screen.fill((255, 255, 255))
        button_LED.draw(gui.screen,mouse)
        location_text = (100,100)
        text = font.render("Button Count:"+str(node.count),True,(0,0,0))
        gui.screen.blit(text,location_text)
        location_text = (100,200)
        text = font.render("ADC Value:"+str(node.adc.adc0),True,(0,0,0))
        gui.screen.blit(text,location_text)

        pygame.display.flip()
    rospy.spin()
    pygame.quit()

