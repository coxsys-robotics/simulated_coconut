#!/usr/bin/env python3
import pygame
import rospy 
from std_msgs.msg import UInt8, Bool
def isInBox(var,size):
    return (size[0]<=var[0]<=size[0]+size[2]) and (size[1]<=var[1]<=size[1]+size[3])
class GameParam:
    def __init__(self):
        self.screen_width = 800
        self.screen_height = 600
        self.screen = pygame.display.set_mode([self.screen_width, self.screen_height])
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
        if isInBox(mouse,self.size):
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
            text = text_list[2]
        pygame.draw.rect(screen,color,self.size)
        text_object = self.font.render(text , True ,(0,0,0))
        screen.blit(text_object, self.center)
    def isMouseHover(self,mouse):
        return isInBox(mouse,self.size)

class Node :
    def __init__(self):
        rospy.init_node('gui')
        pygame.init()
        # Set up the drawing window
        rospy.Subscriber('/mode',UInt8,self.callback_mode)
        self.pub_map = rospy.Publisher('/mode_map',Bool,queue_size=50)
        self.pub_sav = rospy.Publisher('/mode_save',Bool,queue_size=50)
        self.pub_can = rospy.Publisher('/mode_cancel',Bool,queue_size=50)
        self.pub_nav = rospy.Publisher('/mode_start_nav',Bool,queue_size=50)
        self.pub_fin = rospy.Publisher('/mode_finish_nav',Bool,queue_size=50)
        self.mode = 0
    def callback_mode(self,msg):
        self.mode = msg.data

if __name__=='__main__':
    node = Node()
    gp = GameParam()
    button_map = Button((100,100,150,70),"map")
    button_nav = Button((500,100,150,70),"nav")
    button_sav = Button((100,100,150,70),"save")
    button_can = Button((500,100,150,70),"can")
    button_fin = Button((100,100,150,70),"fin")
    
    running = True
    while running:
        # Did the user click the window close button?
        mouse = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONUP:
                msg = Bool()
                if node.mode == 0:
                    if button_map.isMouseHover(mouse):
                        node.pub_map.publish(msg)
                    elif button_nav.isMouseHover(mouse):
                        node.pub_nav.publish(msg)
                elif node.mode == 1:
                    if button_sav.isMouseHover(mouse):
                        node.pub_sav.publish(msg)
                    elif button_can.isMouseHover(mouse):
                        node.pub_can.publish(msg)
                elif node.mode == 2:
                    if button_fin.isMouseHover(mouse):
                        node.pub_fin.publish(msg)

        # Fill the background with white
        gp.screen.fill((255, 255, 255))

        if node.mode == 0:
            button_map.draw(gp.screen,mouse)
            button_nav.draw(gp.screen,mouse)
        elif node.mode == 1:
            button_sav.draw(gp.screen,mouse)
            button_can.draw(gp.screen,mouse)
        elif node.mode == 2:
            button_fin.draw(gp.screen,mouse)
        
        # Flip the display
        pygame.display.flip()
    # Done! Time to quit.
    pygame.quit()