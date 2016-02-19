from graphics import *
import numpy as np
from WorldObjects import *
from helperMath import *


def close(pt):
    win.close()
    
win =  GraphWin("test",800,800)
win.setCoords(0,0,500,500)
win.setMouseHandler(close)
dt = .1

objs = []
for i in range(2):
    pos = np.random.random(2) * 480
    v = np.random.random(2) * 80
    o = Ball(pos, 20)
    o.v = v
    objs.append(o)

o = Ball(vect(305, 150), 10)
o.v = vect(0,-20)
objs.append(o)

pts = np.array([[100,100],[200,100],[200,125],[100,125]])
p = RigidPolygon(pts, density = .01)
p.w = 0
p.v = vect(-10,0)
"""
o = Ball(vect(300,250), 10, v=vect(0,100))
o1 = Ball(vect(200,250), 10, v=vect(0,-100))
o2 = Ball(vect(250,300), 10, v=vect(-100,0))
o3 = Ball(vect(250,200), 10, v=vect(100,0))
objs.append(o)
objs.append(o1)
objs.append(o2)
objs.append(o3)
"""
#objs.append(p)

wall1 = Wall(vect(0,0), vect(200,500))
wall2 = Wall(vect(0,0), vect(0,500))
wall3 = Wall(vect(0,0), vect(500,0))
wall4 = Wall(vect(500,0), vect(500,500))
wall5 = Wall(vect(0,500), vect(500,500))

b_hole = BlackHole(vect(260,250), 20)
objs.append(wall1)
objs.append(wall2)
objs.append(wall3)
objs.append(wall4)
objs.append(wall5)
bomb1 = Bomb(vect(280,250), strength = 200000)
bomb2 = Bomb(vect(380,250), strength = 200000)
bomb3 = Bomb(vect(180,250), strength = 200000)
bomb4 = Bomb(vect(480,250), strength = 200000)

#objs.append(bomb1)
#objs.append(bomb2)
#objs.append(bomb3)
#objs.append(bomb4)

objs.append(b_hole)
w = World(win, objs, dt)
w.gravity = 0
w.damping = 1
w.draw()

##########
while not win.closed:
    w.update()
    time.sleep(.02)
##########