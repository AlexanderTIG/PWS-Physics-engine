
##############################################################
#plannen voor later:

#fixed point in een entiteit, meerdere fixed points = immovable

#botsing detecteren -> aparte functie
#met een botsing moet de botsing ongedaan worden -> aparte functie of met boven ???????

#daarna vectoren optellen door de natuurkunde zelf -> aparte functie of niet ??????

##############################################################

#we hebben ook numpy nodig voor vectoren
#download door in je cmd of cmder deze command : pip install numpy

import pygame;
import numpy as np;
import time;
import math;

pygame.init()
width = 1000;
height = 600;
surface = pygame.display.set_mode((width,height)) 
time_per_frame = 60;
g = 100;

def normalize(vector):
    mag = np.linalg.norm(vector);
    return np.multiply(vector, 1 / mag);

#Botsing cirkels detecteren vet makkelijk met vectoren
def bool_circle_circle(circle1, circle2):
    disVec = circle1.pos - circle2.pos;
    return np.linalg.norm(disVec) <= circle1.r + circle2.r; #np.linalg.norm is lengte vector

def mag(vector):
    return np.linalg.norm(vector);

def dist_point_point(point1, point2):
    return mag(point1 - point2);
    
def dist_line_point(line, Point2):
    x0 = line.pos1[0];
    y0 = line.pos1[1];

    x1 = line.pos2[0];
    y1 = line.pos2[1];

    x2 = Point2[0];
    y2 = Point2[1];

    num = abs((x2 - x1)*(y1 - y0) - (x1 - x0)*(y2 - y1));
    den = ((x2 - x1)**2 + (y2 - y1)**2)**0.5;
    return num/den;

def closest_point_line(point, line):
    
    if line.a == 0:
        x = point[0];
        
    elif line.a == float("inf"):
        return np.array([line.pos1[0], point[1]]);
    else: 
        perp_a = -1 / line.a;
        perp_b = point[1] - perp_a * point[0];

        x = (perp_b - line.b) / (line.a - perp_a);

    y = line.a * x + line.b;
    return np.array([x, y]);

def point_on_line(point, line):
    if line.a == float("inf") and point[0] == line.pos1[0] and line.up_y() < point[1] < line.down_y():
        return True;
    
    if line.a != float("inf") and point[1] - line.a * point[0] == line.b and line.left_x() < point[0] < line.right_x():
        return True;

    return False;

def collision(circle, line):
    closest_point = closest_point_line(circle.pos, line);
    dist = dist_point_point(closest_point, circle.pos);
    Collision = False;
    
    if dist <= circle.r and point_on_line(closest_point, line):

        Collision = True;
        if line.a == float("inf"):
            Sx = line.pos1[0];
            Sy = circle.rc_v() * line.pos1[0] + circle.b_v();
        else:
            if circle.velo[0] != 0:
                Sx = (line.b - circle.b_v()) / (circle.rc_v() - line.a);
            else:
                Sx = circle.x();
                
            Sy = line.a * Sx + line.b;
            
        S = np.array([Sx, Sy]);
            
        M1_S = circle.r * mag(line.vector) * mag(circle.velo) / abs(np.cross(line.vector, circle.velo));
            
        S_M2_vector = circle.pos - S;
        M1_S_vector = np.multiply(normalize(circle.velo), M1_S)

        push_vector = S_M2_vector + M1_S_vector;
        circle.pos = circle.pos - push_vector;

        normal = normalize(circle.pos - closest_point);
        dot_prod = np.dot(circle.velo, normal);
        circle.velo = circle.velo - np.multiply(normal, 2 * dot_prod);
        
    else:
        if dist_point_point(line.pos1, circle.pos) < dist_point_point(line.pos2, circle.pos):
            Edge = line.pos1
        else:
            Edge = line.pos2;

        if dist_point_point(Edge, circle.pos) <= circle.r:

            Collision = True;
            if circle.velo[0] != 0:
                a = circle.rc_v();
                b = circle.b_v();
                xe = Edge[0];
                ye = Edge[1];
                
                A = a**2 + 1;
                B = 2*a*b - 2*a*ye - 2*xe;
                C = xe**2 + b**2 - 2*b*ye + ye**2 - circle.r**2;

                D = B**2 - 4*A*C;

                x_right = (-B + D**0.5) / (2*A);
                x_left = (-B - D**0.5) / (2*A);

                if circle.velo[0] > 0:
                    x = x_left;
                else:
                    x = x_right;

                circle.pos[0] = x;
                circle.pos[1] = x * circle.rc_v() + circle.b_v();
            else:
                circle.pos[1] = circle.pos[1] + circle.r - dist_point_point(Edge, circle.pos); #later push_vector aanpak misschien

            refl_dir = normalize(circle.pos - Edge);
            circle.velo = np.multiply(refl_dir, mag(circle.velo));

    if Collision:
        circle.velo = np.multiply(circle.velo, circle.bounce_coef);
            
#Laten we beginnen met een entiteit
class Entity:
    def __init__(self, mass, startpos):
        
        #een numpy array met twee elementen wordt blijkbaar gezien als vectoren
        self.pos = startpos; #links boven en positie als vector maakt shit later makkelijker
        self.velo = np.array([0, 0]); #richtingsvector van snelheid
        self.m = mass;
        self.bounce_coef = 1;
        
    def update(self, elapsedTime):
        
        self.acc = np.array([0, g]) #+ np.multiply(self.velo, -0.99);
        self.velo = self.velo + np.multiply(self.acc, elapsedTime);
        self.pos = self.pos + np.multiply(self.velo, elapsedTime);

        #later in aparte functie
        for checkObject in Objects:
            if checkObject!= self:
                collision(self, checkObject);

#Vormen
class Circle(Entity):
    def __init__(self, mass, startpos, radius):
        Entity.__init__(self, mass, startpos);
        self.r = radius;

    def draw(self):
        pygame.draw.circle(surface, (255,0,0), self.pos, self.r);

    def x(self):
        return self.pos[0];

    def y(self):
        return self.pos[1];

    def rc_v(self):
        return self.velo[1] / self.velo[0];

    def b_v(self):
        return self.pos[1] - self.rc_v() * self.pos[0];

class Line:
    def __init__(self, Pos1, Pos2):
        self.pos1 = Pos1;
        self.pos2 = Pos2;

        if Pos1[0] - Pos2[0] != 0:
            self.a = (Pos1[1] - Pos2[1]) / (Pos1[0] - Pos2[0]);
            self.b = Pos1[1] - self.a*Pos1[0];
        else:
            self.a = float("inf");

        self.vector = Pos2 - Pos1;

    def left_x(self):
        return min(self.pos1[0], self.pos2[0]);

    def right_x(self):
        return max(self.pos1[0], self.pos2[0]);

    def up_y(self):
        return min(self.pos1[1], self.pos2[1]);

    def down_y(self):
        return max(self.pos1[1], self.pos2[1]);
    
    def draw(self):
        pygame.draw.line(surface, (255,0,0), self.pos1, self.pos2, 1);

Loop = True;
Objects = [];

Objects.append(Circle(10, np.array([width /2 + 150, height / 2]), 30));
Objects.append(Line(np.array([0, height]), np.array([width, height]))); #GROND
Objects.append(Line(np.array([0, 0]), np.array([width, 0]))); #PLAFOND
Objects.append(Line(np.array([width, height]), np.array([width, 0]))); #rechter MUUR
Objects.append(Line(np.array([0, 0]), np.array([0, height]))); #linker MUUR

clock = pygame.time.Clock()
elapsedTime = 0;
while Loop:

    start_time = time.time();
    #kleine interactie:
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            pygame.quit();

        if event.type == pygame.MOUSEBUTTONDOWN:
            startPoint = pygame.mouse.get_pos();
            
        if event.type == pygame.MOUSEBUTTONUP:
            endPoint = pygame.mouse.get_pos();
            Objects.append(Line(np.array(startPoint), np.array(endPoint)));

    if pygame.mouse.get_pressed()[0]:
        pygame.draw.line(surface, (255,0,0), startPoint, pygame.mouse.get_pos(), 1);
            
    for Object in Objects:
        if isinstance(Object, Entity):
            Object.update(elapsedTime);
        Object.draw();

    pygame.display.flip();
    surface.fill((0,0,0));
    elapsedTime = time.time() - start_time; 

#Dynamische loop fixt veel hiervoor verloor de bal energie terwijl ik dit niet had ingecodeerd, omdat een niet dynamsche loop / framerate minder accuraat is.
