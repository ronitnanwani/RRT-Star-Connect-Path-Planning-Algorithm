import sys, random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2
import cv2
import numpy as np

EPSILON = 50
NUMNODES = 3500
RADIUS = 70
def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def chooseParent(nn, newnode, nodes):
    for p in nodes:
        if dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and p.cost + dist([p.x, p.y],[newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
    newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
    newnode.parent = nn
    return newnode, nn


def reWire(nodes, newnode, pygame, screen):
    for i in range(len(nodes)):
        p = nodes[i]
        if p != newnode.parent and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and newnode.cost + dist([p.x, p.y],
                                                                                                             [newnode.x,
                                                                                                              newnode.y]) < p.cost:
            if (isThruObstacle((p.x,p.y),(newnode.x,newnode.y),obstacles)):
                continue
            pygame.draw.line(screen, (255,255,255), [p.x, p.y], [p.parent.x, p.parent.y])
            p.parent = newnode
            p.cost = newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y])
            nodes[i] = p
            pygame.draw.line(screen, (0,0,0), [p.x, p.y], [newnode.x, newnode.y])
    return nodes


def drawSolutionPath(startpos, endpos, pygame, screen):
    # print(len(nodes))
    # print(startpos.x,startpos.y, end=" ")
    # print(endpos.x,endpos.y)
    soln=[]
    pink = 200, 20, 240
    itrnode = endpos
    # i=len(nodes)-1
    while (itrnode.x!= startpos.x or itrnode.y!=startpos.y):
        # print(i,end=" ")
        # try:
        # print(itrnode.x,itrnode.y)
        # except:
        #     continue
        soln.append((itrnode.x,itrnode.y))
        pygame.draw.line(screen, pink, [itrnode.x, itrnode.y], [itrnode.parent.x, itrnode.parent.y], 5)
        itrnode = itrnode.parent
        # i-=1
    soln.append((startpos.x,startpos.y))
    # print("Hello")startpos.x,startpos.y
    return soln

def isInObstacle(vex, obstacles):
    if vex.y < 0 or vex.y > 581:
        return True
    if vex.x < 0 or vex.x > 659:
        return True
    alpha=math.floor(vex.y)
    beta=math.floor(vex.x)
    for i in range (alpha-2,alpha+3):
        for j in range (beta-2,beta+3):
            if(i>581 or j>659):
                continue
            if(obstacles[i][j]==0):
                return True
    return False


def isThruObstacle(p0,p1, obstacles):
    xm = int((p0[0] + p1[0]) / 2)
    ym = int((p0[1] + p1[1]) / 2)
    if ym < 0 or ym > 581:
        return True
    if xm < 0 or xm > 659:
        return True
    if obstacles[ym][xm] == 0:
        return True
    xm1 = int((p0[0] + xm) / 2)
    ym1 = int((p0[1] + ym) / 2)
    if obstacles[ym1][xm1] == 0:
        return True
    xm2 = int((p1[0] + xm) / 2)
    ym2 = int((p1[1] + ym) / 2)
    if obstacles[ym2][xm2] == 0:
        return True
    if((p1[0]-p0[0])!=0):
        m=(p1[1]-p0[1])/(p1[0]-p0[0])
        step=0
        if(p1[0]>p0[0]):
            step=1
        else:
            step=-1
        xcoord=p0[0]
        ycoord=p0[1]
        i=1
        while((xcoord<p1[0] and step>0) or (xcoord>p1[0] and step<0)):
            xcoord+=step
            ycoord+=m
            if(isInObstacle(Node(xcoord,ycoord),obstacles)):
                return True
    else:
        step=0
        if (p1[1]-p0[1]>=0):
            step=1
        else:
            step=-1
        ycoord=p0[1]
        while((ycoord<p1[1] and step>0) or (ycoord>p1[1] and step<0)):
            ycoord+=step
            if(isInObstacle(Node(p0[0],ycoord),obstacles)):
                return True
    return False


def rrt_star_connect(T1,T2,screen,obstacles,cnode,pnode,distancemin):
    rand = Node(random.random() * 659, random.random() * 581)
    nn1 = T1.nodes[0]
    for p in T1.nodes:
        if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn1.x, nn1.y], [rand.x, rand.y]):
            nn1= p
    interpolatedNode1= step_from_to([nn1.x, nn1.y], [rand.x, rand.y])
    newnode1 = Node(interpolatedNode1[0], interpolatedNode1[1])
    
    nn2 = T2.nodes[0]
    for p in T2.nodes:
        if dist([p.x, p.y], [newnode1.x, newnode1.y]) < dist([nn2.x, nn2.y], [newnode1.x, newnode1.y]):
            nn2= p
    interpolatedNode2 = step_from_to([nn2.x, nn2.y], [newnode1.x, newnode1.y])
    newnode2 = Node(interpolatedNode2[0], interpolatedNode2[1])
    flag1=0
    flag2=0
    [newnode1, np1] = chooseParent(nn1, newnode1, T1.nodes)
    [newnode2, np2] = chooseParent(nn2, newnode2, T2.nodes)
    if (not(isInObstacle(newnode1,obstacles))):
        if not (isThruObstacle((newnode1.x,newnode1.y),(np1.x,np1.y),obstacles)) :
            T1.nodes.append(newnode1)
            flag1=1
            pygame.draw.line(screen,(0,0,0), [np1.x, np1.y], [newnode1.x, newnode1.y])
            T1.nodes=reWire(T1.nodes, newnode1, pygame, screen)
            pygame.display.update()
    if (not(isInObstacle(newnode2,obstacles))):
        if not (isThruObstacle((newnode2.x,newnode2.y),(np2.x,np2.y),obstacles)) :
            T2.nodes.append(newnode2)
            flag2=1
            pygame.draw.line(screen,(0,0,0), [np2.x, np2.y], [newnode2.x, newnode2.y])
            T2.nodes=reWire(T2.nodes, newnode2, pygame, screen)
            pygame.display.update()
    # if (cnode.x == pnode.x and cnode.y == pnode.y):
    if(flag1==1 and flag2==1):
        if(newnode1.cost+newnode2.cost+dist([newnode1.x,newnode1.y],[newnode2.x,newnode2.y])<=distancemin):
            if(not(isThruObstacle((newnode1.x,newnode1.y),(newnode2.x,newnode2.y),obstacles))):
                pygame.draw.line(screen,(255,255,255), [cnode.x, cnode.y], [pnode.x, pnode.y])
                pygame.draw.line(screen,(0,0,0), [newnode2.x, newnode2.y], [newnode1.x, newnode1.y])
                distancemin=newnode1.cost+newnode2.cost+dist([newnode1.x,newnode1.y],[newnode2.x,newnode2.y])
                # T1.nodes=reWire(T1.nodes, newnode2, pygame, screen)
                cnode=newnode2
                pnode=newnode1
    return T1,T2,cnode,pnode,distancemin

class Node:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord

class Tree:

    def __init__(self,startpos):
        self.startpos=startpos
        self.nodes=[startpos]
    


def main():
    pygame.init()
    screen = pygame.display.set_mode([659,581])
    screen.fill((255,255,255))
    startpositions=[Node(56.,52.),Node(170.,52.),Node(55.,444.)]
    endpositions=[Node(633.,188.),Node(618.,526.),Node(351.,538.)]
    p1=int(input("Enter a start position\n"))
    p2=int(input("Enter an end position\n"))
    startpos=startpositions[p1]
    endpos = endpositions[p2]
    cnode = startpos
    pnode=startpos
    T1=Tree(startpos)
    T2=Tree(endpos)
    distancemin=1000
    # print(len(T1.nodes))
    # print(len(T2.nodes))
    for i in range(NUMNODES):
        T2,T1,pnode,cnode,distancemin=rrt_star_connect(T1,T2,screen,obstacles,cnode,pnode,distancemin)
        for e in pygame.event.get():
            if e.type == KEYUP and e.key == K_ESCAPE:
                sys.exit()
    # print("Success")
    # print(cnode.x,cnode.y)
    # print(cnode.parent.x,cnode.parent.y)
    # print(cnode.parent.parent.x,cnode.parent.parent.y)
    # print(pnode.x,pnode.y)
    # print(pnode.parent.x,pnode.parent.y)
    # print(pnode.parent.parent.x,pnode.parent.parent.y)
    # print(cnode.parent.x,cnode.parent.y)
    soln=[]
    tempcnode=Node(cnode.x,cnode.y)
    tempcnode.parent=pnode
    tempcnode.cost=pnode.cost+dist((tempcnode.x,tempcnode.y),(pnode.x,pnode.y))
    nodeend=pnode.parent
    for i in range (5):
        if(nodeend==startpos or nodeend==endpos):
            break
        if(nodeend.cost + dist((nodeend.x,nodeend.y),(tempcnode.x,tempcnode.y))<tempcnode.cost):
            if(not(isThruObstacle((nodeend.x,nodeend.y),(tempcnode.x,tempcnode.y),obstacles))):
                tempcnode.cost=nodeend.cost + dist((nodeend.x,nodeend.y),(tempcnode.x,tempcnode.y))
                tempcnode.parent=nodeend
        nodeend=nodeend.parent
    if(T1.startpos==startpos):
        # tempcnode=Node(cnode.x,cnode.y)
        # tempcnode.parent=pnode
        # tempcnode.cost=pnode.cost+dist((tempcnode.x,tempcnode.y),(pnode.x,pnode.y))
        # for nodes in T1.nodes:
        #     if(nodes.cost + dist((nodes.x,nodes.y),(tempcnode.x,tempcnode.y))<tempcnode.cost):
        #         tempcnode.cost=nodes.cost + dist((nodes.x,nodes.y),(tempcnode.x,tempcnode.y))
        #         tempcnode.parent=nodes
        soln1=drawSolutionPath(startpos,tempcnode.parent,pygame, screen)
        soln2=drawSolutionPath(endpos,cnode,pygame, screen)
        pygame.draw.line(screen, (200,20,240), [cnode.x,cnode.y], [tempcnode.parent.x,tempcnode.parent.y],5)
    else:
        # tempcnode=Node(cnode.x,cnode.y)
        # tempcnode.parent=pnode
        # tempcnode.cost=pnode.cost+dist((tempcnode.x,tempcnode.y),(pnode.x,pnode.y))
        # for nodes in T2.nodes:
        #     if(nodes.cost + dist((nodes.x,nodes.y),(tempcnode.x,tempcnode.y))<tempcnode.cost):
        #         tempcnode.cost=nodes.cost + dist((nodes.x,nodes.y),(tempcnode.x,tempcnode.y))
        #         tempcnode.parent=nodes
        soln1=drawSolutionPath(startpos, tempcnode.parent,pygame, screen)
        soln2=drawSolutionPath(endpos,cnode,pygame, screen)
        pygame.draw.line(screen, (200,20,240), [cnode.x,cnode.y], [tempcnode.parent.x,tempcnode.parent.y],5)
    soln1.reverse()
    # soln2.reverse()
    # for i in range (len(soln1)-1):
    #     print(soln1[i][0],soln1[i][1])
    # for i in range (len(soln2)-1):
    #     print(soln2[i][0],soln2[i][1])
        
    # print(soln1[1][0],soln1[1][1])
    # pygame.draw.line(screen, (200,20,240), [soln1[-1][0],soln1[-1][1]], [soln2[0][0],soln2[0][1]],5)
    pygame.display.update()
    for i in range (len(soln1)):
        soln.append(soln1[i])
    for i in soln2:
        soln.append(i)
    for i in range (len(soln)-1):
        cv2.line(img,(int(soln[i][0]),int(soln[i][1])),(int(soln[i+1][0]),int(soln[i+1][1])),(0,255,255),1)
    # for i in range (len(soln2)-1):
    #     cv2.line(img,(int(soln2[i][0]),int(soln2[i][1])),(int(soln2[i+1][0]),int(soln2[i+1][1])),(0,255,255),3)
    # for i in range (len(soln1)-1):
    #     cv2.line(img,(int(soln1[i][0]),int(soln1[i][1])),(int(soln1[i+1][0]),int(soln1[i+1][1])),(0,255,255),3)
    # cv2.line(img,(int(soln2[0][0]),int(soln2[0][1])),(int(soln1[0][0]),int(soln1[0][1])),(0,255,255),3)
    # pygame.display.update()
    print(soln)
    cv2.imshow("Final Path",img)
    cv2.waitKey(0)


if __name__ == '__main__':
    img = cv2.imread('_2.png',1)
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray,155, 255,1, cv2.THRESH_BINARY)

    obstacles = thresh
    main()
    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False