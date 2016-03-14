from graphics import *
import numpy as np
from WorldObjects import *
from helperMath import *


max_X,max_Y = 500, 500

def close(pt):
    win.close()    
win =  GraphWin("test",max_X,max_Y)
win.setCoords(0,0,max_X, max_Y)
win.setMouseHandler(close)
dt = .1
w = World(win, dt)


# add some balls
w.objects.append( Ball(vect(100, 300), r=20., v=vect(10,4)) )
w.objects.append( Ball(vect(150, 400), r=40., v=vect(-10,0)) )
w.objects.append( Ball(vect(100, 400), r=20., v=vect(10,8)) )
w.objects.append( Ball(vect(300, 200), r=50., v=vect(5,0)) )

#Add a polygon
pts = np.array([[100,100],[200,100],[200,165],[100,165]])
p = RigidPolygon(pts, density = .01)
p.w = .1
p.v = vect(3,5)
w.objects.append(p)

# create the walls
for coords in [((0,0), (max_X,0)), ((0,0), (0,max_Y)), 
			   ((0,max_Y), (max_X,max_X)), ((max_X,max_Y), (max_X,0))]:
	w.objects.append( Wall(coords[0],coords[1])    )

w.objects.append( BlackHole(vect(400, 400), 20))



w.gravity = 0
w.damping = 1

##########
w.draw()
while not win.closed:
    w.update()
    time.sleep(.02)
##########