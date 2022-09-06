import pygame;

#we hebben ook numpy nodig voor vectoren
#download door in je cmd of cmder deze command : pip install numpy
import numpy as np;
import time;

pygame.init()
width = 1000;
height = 600;
surface = pygame.display.set_mode((width,height)) 
time_per_frame = 60;
a = 0.00001;

#Laten we beginnen met een entiteit
class Entity:
    def __init__(self, mass, startpos):

        #een numpy array met twee elementen wordt blijkbaar gezien als vectoren
        self.pos = startpos; #links boven en positie als vector maakt shit later makkelijker
        self.velo = np.array([0, 0]); #richtingsvector van snelheid

        self.m = mass;

    def update(self):
        self.velo = self.velo + np.array([0, a * time_per_frame]);
        self.pos = self.pos + np.multiply(self.velo, time_per_frame);

#Vormen
class Circle(Entity):
    def __init__(self, mass, startpos, radius):
        Entity.__init__(self, mass, startpos);
        self.r = radius;

    def draw(self):
        pygame.draw.circle(surface, (255,0,0), self.pos, self.r);

class Rect(Entity):
    def __init__(self, mass, startpos, width, height):
        Entity.__init__(self, mass, startpos);
        self.w = width;
        self.h = height;

    def draw(self):
        pygame.draw.rect(surface, (255,0,0), pygame.Rect(self.pos[0], self.pos[1], self.w, self.h));

Loop = True;
Entities = [];

Entities.append(Rect(10, np.array([width / 2, 0]), 20, 20));
clock = pygame.time.Clock()

while Loop:
    for Entity in Entities:
        Entity.draw();
        Entity.update();

    pygame.display.flip();
    surface.fill((0,0,0));
    clock.tick(time_per_frame); # nu is het nog niet een dynamische loop wat we later misschien wel kunnen doen.
    
