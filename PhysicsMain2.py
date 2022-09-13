import pygame;
import numpy as np;
import time;

pygame.init()
width = 1000;
height = 600;
surface = pygame.display.set_mode((width,height))

mu = 0.6;
g = 100;

class Ball:
    def __init__(self, startPos, r, m):
        self.pos = startPos;
        self.r = r;
        self.m = m;
        self.velo = np.array([0, 0]);

    def update(self, elapsedTime):

        #TOPDOWN
        #normal = self.m * g;
        #friction = np.multiply(-1 * mu * normal, self.velo);
        #forces = friction;
        
        self.acc = [0, g] #+ np.multiply(forces, 1 / self.m);
        self.velo = self.velo + np.multiply(self.acc, elapsedTime);
        self.pos = self.pos + np.multiply(self.velo, elapsedTime);

        if mag(np.multiply(self.velo, elapsedTime)) > 18:
            self.velo = normalize(self.velo) * 18;
            print('oneee');
            
        #lowerBound = 3;
        #if mag(self.velo) < lowerBound:
            #self.velo = np.array([0, 0]);

    def draw(self):
        pygame.draw.circle(surface, (255,0,0), self.pos, self.r);
        
class Line:
    def __init__(self, Pos1, Pos2, r):
        self.pos1 = Pos1;
        self.pos2 = Pos2;
        self.r = r;

    def draw(self):
        if not (self.pos1 == self.pos2).all():
            pygame.draw.circle(surface, (255,0,0), self.pos1, self.r);
            pygame.draw.circle(surface, (255,0,0), self.pos2, self.r);

            vector = self.pos1 - self.pos2;
            normal = normalize(np.array([vector[1], -vector[0]]));
            Pos1 = self.pos1 + np.multiply(normal, self.r);
            Pos2 = self.pos2 + np.multiply(normal, self.r);
            pygame.draw.line(surface, (255,0,0), Pos1, Pos2, 1);
            
            normal = normalize(np.array([-vector[1], vector[0]]));
            Pos1 = self.pos1 + np.multiply(normal, self.r);
            Pos2 = self.pos2 + np.multiply(normal, self.r);
            pygame.draw.line(surface, (255,0,0), Pos1, Pos2, 1);
        else:
            pygame.draw.circle(surface, (255,0,0), self.pos1, self.r);


def normalize(vector):
    mag = np.linalg.norm(vector);
    return np.multiply(vector, 1 / mag);

def mag(vector):
    return np.linalg.norm(vector);

def dist_squared(point1, point2):
    return (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2;

def dist(point1, point2):
    return dist_squared(point1, point2)**0.5;

def closestPoint(Line, Ball):
    dist_AP = dist(Ball.pos, Line.pos1);
    dist_AB = dist(Line.pos1, Line.pos2);
    vector_AP = Ball.pos - Line.pos1;
    vector_AB = Line.pos2 - Line.pos1;

    dist_AC = np.dot(vector_AP, vector_AB) / dist_AB;
    dist_AC = max(0, min(dist_AC, dist_AB));
    
    normal = normalize(vector_AB);
    return Line.pos1 + np.multiply(normal, dist_AC);
        
def collision_ball_line(cBall, Line):

    closest = closestPoint(Line, cBall);
    if dist_squared(closest, cBall.pos) <= (Line.r + cBall.r)**2:
        
        tempBall = Ball(closest, Line.r, cBall.m);
        tempBall.velo = np.multiply(cBall.velo, -1);
        collision_balls(tempBall, cBall);
    
def collision_balls(Ball_1, Ball_2):

    dist_squared_balls = dist_squared(Ball_1.pos, Ball_2.pos);
    if dist_squared_balls <= (Ball_1.r + Ball_2.r)**2:

        #Niet 100% accuraat maar wel stabieler en met statische botsing
        r1 = Ball_1.r;    r2 = Ball_2.r; 
        v1 = Ball_1.velo; v2 = Ball_2.velo;
        p1 = Ball_1.pos;  p2 = Ball_2.pos;
        m1 = Ball_1.m;    m2 = Ball_2.m;

        dist_fix = (r1 + r2 - dist(p1, p2)) / 2;
        vector_dir = normalize(p1 - p2);
        push_vector = np.multiply(vector_dir, dist_fix);

        Ball_1.pos = Ball_1.pos + push_vector;
        Ball_2.pos = Ball_2.pos - push_vector;

        #100% Accuraat maar minder stabiel zonder statische botsing
        '''
        A = dist_squared(v1, v2);
        B = 2 * np.dot(v2 - v1, p1 - p2);
        C = dist_squared(p1, p2) - (r1 + r2)**2;
        D = B**2 - 4*A*C;
        
        t = (-B + D**0.5) / (2 * A);
        
        push_vector_1 = np.multiply(v1, t);
        push_vector_2 = np.multiply(v2, t);

        Ball_1.pos = Ball_1.pos - push_vector_1;
        Ball_2.pos = Ball_2.pos - push_vector_2; '''
        
        p1 = Ball_1.pos;  p2 = Ball_2.pos;
        distance_squared = dist_squared(p1, p2);
        
        C = 2 * m2 / (m1 + m2) * np.dot(v1 - v2, p1 - p2) / distance_squared;
        Ball_1.velo = v1 - np.multiply(C, p1 - p2);
        
        C = 2 * m1 / (m1 + m2) * np.dot(v2 - v1, p2 - p1) / distance_squared;
        Ball_2.velo = v2 - np.multiply(C, p2 - p1);

    
Loop = True;
Balls = [];
Lines = [];

elapsedTime = 0;

Lines.append(Line(np.array([0, 0]), np.array([0, height]), 20)); #LINKS
#Lines.append(Line(np.array([0, 0]), np.array([width, 0]), 20)); #BOVEN
Lines.append(Line(np.array([0, height]), np.array([width, height]), 20)); #ONDER
Lines.append(Line(np.array([width, 0]), np.array([width, height]), 20)); #RECHTS

Balls.append(Ball(np.array([50, height / 2]), 20, 20));
Balls.append(Ball(np.array([95, height / 2]), 20, 20));
Balls.append(Ball(np.array([140, height / 2]), 20, 20));
Balls.append(Ball(np.array([185, height / 2]), 20, 20));
Balls.append(Ball(np.array([width /2, 0]), 20, 20))

selectedBall = None;

selectedEdge_pos = None;
selectedEdge_id = None;
selectedLine = None;

while Loop:
    start_time = time.time();
    
    #Kleine Interactie
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            Loop = False;

        x, y = pygame.mouse.get_pos();
        mousePos_1 = np.array([x, y]);
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            newSelected = True;
            
            for cBall in Balls:
                if dist(mousePos_1, cBall.pos) <= cBall.r:
                    newSelected = False;
                    selectedBall = cBall;

            for cLine in Lines:
                if dist(mousePos_1, cLine.pos1) <= cLine.r:
                    newSelected = False;
                    selectedLine = cLine;
                    selectedEdge_pos = cLine.pos1;
                    selectedEdge_id = 1;
                if dist(mousePos_1, cLine.pos2) <= cLine.r:
                    newSelected = False;
                    selectedLine = cLine;
                    selectedEdge = cLine.pos2;
                    selectedEdge_id = 2;

            if newSelected:
                newLine = Line(mousePos_1, mousePos_1, 20);
                Lines.append(newLine);
                
                selectedLine = newLine;
                selectedEdge = newLine.pos2;
                selectedEdge_id = 2;
                
        if event.type == pygame.MOUSEBUTTONUP:
            x, y = pygame.mouse.get_pos();
            mousePos_2 = np.array([x, y]);

            if selectedBall != None:
                selectedBall.velo = np.multiply(selectedPos - mousePos_2, 2);
                selectedBall = None;

            selectedEdge_pos = None;
            selectedEdge_id = None;
            selectedLine = None;
            
            
    if pygame.mouse.get_pressed()[0]:
        if selectedBall != None:
            selectedPos = selectedBall.pos;
            pygame.draw.line(surface, (255,0,0), selectedPos, pygame.mouse.get_pos(), 1);
            
        if selectedLine != None:
            x, y = pygame.mouse.get_pos();
            mousePos = np.array([x, y]);
            
            if selectedEdge_id == 1:
                selectedLine.pos1 = mousePos;
            if selectedEdge_id == 2:
                selectedLine.pos2 = mousePos;

    #Tekenen
    for cBall in Balls:
        cBall.draw();

    for cLine in Lines:
        cLine.draw();
        
    #Posities en Snelheden etc... Updaten
    #Collision
    for idBall_1 in range(len(Balls)):
        for idBall_2 in range(idBall_1):
            collision_balls(Balls[idBall_1], Balls[idBall_2]);

    for cBall in Balls:
        for cLine in Lines:
            collision_ball_line(cBall, cLine);

    #Updaten
    for cBall in Balls:
        cBall.update(elapsedTime);

    pygame.display.flip();
    surface.fill((0,0,0));
    elapsedTime = time.time() - start_time; 
    
pygame.quit();
quit();
