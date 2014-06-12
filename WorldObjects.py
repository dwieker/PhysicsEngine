import HelperMath
import numpy as np

def vect(x,y):
    return np.array([x,y], dtype = float)

class Ball():
    def __init__(self, pos, r=1., v=vect(0,0)):
        self.pos = pos
        self.r = r
        self.mass = np.pi * float(r)**2
        self.v = v
        self.cir = Circle(Point(self.pos[0], self.pos[1]), self.r)
        self.cir.setFill('blue') 
    ####################### 
    # Setters
    #######################
    def set_r(self, r):
        self.r = r
    def set_v(self, v):
        self.v = v
    def set_pos(self, pos):
        self.cir.move(pos[0] - self.pos[0], pos[1] - self.pos[1])
        self.pos = pos        
    ####################### 
    # Math
    #######################
    def velocity_mag(self):
        return np.linalg.norm(self.v)
    def distance_to(self, point):
        """Returns distance from center of ball to the given point"""
        return np.linalg.norm(point - self.pos)  
    ####################### 
    # Movers
    #######################
    def move(self, dr):
        self.pos = self.pos + dr
        self.cir.move(dr[0], dr[1])  
    
    def update(self, dt):
        """Updates position according to current velocity and timestep"""
        self.move(self.v * dt)
    
    def apply_impulse(self, f, dt):
        """Changes velocity according to an applied force"""
        self.v = self.v + (f/self.mass * dt) 
        
    def draw(self, win): 
        self.cir.draw(win)
###############################################################################################      
class RigidPolygon():
    def __init__(self, pts, density = 1, axis = None):
        self.pts = pts
        self.density = 1
        self.m_center = self.center_of_mass()
        self.w = 0
        self.theta = 0
        self.v = vect(0,0)
        self.size = pts.size / 2
        if not axis:
            self.I = self.I(self.m_center)
        else:
            self.I = self.I(axis)
        #Create bounding box. Find point fartheset from center of mass and make box width twice that. bbox = half the side length
        self.bbox =  np.sqrt(((pts[:,0] - self.m_center[0])**2 + (pts[:,1] - self.m_center[1])**2).max())
    
    
    def update(self, dt):
        """Updates position according to current velocity and timestep"""
        self.m_center = self.m_center + self.v*dt
        self.pts = self.pts + self.v*dt
        self.theta = self.theta + self.w*dt
        
    
    def apply_impulse(self, pt, j):
        """Changes velocity according to an applied force"""
        self.v = self.v + j/self.mass
        r = pt - self.m_center
        self.w = self.w - np.cross(j/self.I,r)

    
    def center_of_mass(self):
        x = self.pts[:,0].take(range(0, pts[:,0].size + 1), mode = 'wrap')
        y = self.pts[:,1].take(range(0, pts[:,0].size + 1), mode = 'wrap')      
        A = .5*(x[:-1]*y[1:] - x[1:]*y[:-1]).sum()
        self.mass = A*self.density
        C_x = ((x[:-1] + x[1:])*(x[:-1]*y[1:] - x[1:]*y[:-1])).sum() / (6*A)
        C_y = ((y[:-1] + y[1:])*(x[:-1]*y[1:] - x[1:]*y[:-1])).sum() / (6*A)        
        return vect(C_x, C_y)
    
    def I(self, axis):
        """Calculates moment of inertia"""
        #FIX THIS!!!!!!
        pts = self.pts - axis
        x = pts[:,0].take(range(0, pts[:,0].size + 1), mode = 'wrap')
        y = pts[:,1].take(range(0, pts[:,0].size + 1), mode = 'wrap')
        return self.density*((x[:-1]**2 + y[:-1]**2 + x[:-1]*x[1:] + y[:-1]*y[1:] + x[1:]**2 + y[1:]**2)\
                           *(x[:-1]*y[1:] - x[1:]*y[:-1])).sum() / 12
        
    
    def points(self):
        """Given theta and the original set of points, return the new set of points"""
        #move origin to m_center
        pts = self.pts - self.m_center
        #apply rotation matrix
        R_theta = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                           [np.sin(self.theta), np.cos(self.theta)]])

        return (R_theta.dot(pts.T) + self.m_center.reshape(2,1)).T
    
    def draw(self, win):
        try:
            self.polygon.undraw()
            self.cir.undraw()
            win.delItem(self.polygon)
            win.delItem(self.cir)
        except:
            pass
        
        #### Handle image
        draw_pts = []
        for pt in self.points():
            draw_pts.append(Point(pt[0], pt[1]))
        self.polygon = Polygon(draw_pts)
        self.polygon.setFill('green')
        self.cir = Circle(Point(self.m_center[0], self.m_center[1]), 3)
        self.cir.setFill('black')
        self.polygon.draw(win)
        self.cir.draw(win)
         
###############################################################################################   
class Wall():
    # line has the form ax + by + c = o
    def __init__(self, pt1, pt2):
        self.pt1 = pt1
        self.pt2 = pt2
        self.a, self.b, self.c = pts_to_line(pt1, pt2)      
        self.n = vect(self.a, self.b)/mag(vect(self.a, self.b))
        self.t = vect(pt2[0] - pt1[0], pt2[1] - pt1[1])/mag(vect(pt2[0] - pt1[0], pt2[1] - pt1[1]))
        self.line = Line(Point(pt1[0], pt1[1]), Point(pt2[0], pt2[1]))
    def distance_to(self,pt):
        """Finds perpendicular distance from point to line"""
        return abs(self.a*pt[0] + self.b*pt[1] + self.c) / np.sqrt(self.a**2 + self.b**2)
    def draw(self,win):
        self.line.draw(win)        
###############################################################################################     
class Bomb():
    def __init__(self, pos, strength = 1000):
        self.pos = pos
        self.strength = strength
        self.armed = True
        self.rect = Rectangle(Point(pos[0]-5, pos[1]-5), Point(pos[0] + 5, pos[1] + 5))
        self.rect.setFill('red')
    def explode(self, win):
        self.rect.move(10000, 10000)
        self.armed = False
    def draw(self, win):
        if self.armed:
            self.rect.draw(win)
############################################################################################### 
class BlackHole():
    def __init__(self, pos, r):
        self.pos = pos
        self.r = r
        self.mass = 1000 * r**2
        self.cir = Circle(Point(self.pos[0], self.pos[1]), self.r)
        self.cir.setFill('black')
    def draw(self, win):
        self.cir.draw(win)   
############################################################################################### 
class World():
    def __init__(self, win, objects, dt):
        self.objects = objects
        self.win = win
        self.dt = dt
        self.gravity = 0
        self.damping = 1 #efficienty of bounces
    
    def handle_interactions(self):
        i = 0
        for object in self.objects:
            if object.__class__.__name__ == 'Ball':
                handle_ball(object, self.objects[i+1:], self)
            elif object.__class__.__name__ == 'RigidPolygon':
                handle_polygon(object, self.objects[i+1:], self)
            i += 1
                
    def update(self):
        self.handle_interactions()
        for object in self.objects:
            try:
                object.update(self.dt)
            except:
                pass
            if object.__class__.__name__ == 'RigidPolygon':
                object.draw(self.win)
                                         
    def draw(self):
        for object in self.objects:
            object.draw(self.win)
    
    def remove(self, obj):
        self.objects.remove(obj)
        
    def sort_nearby_objects(self):
        #FIX THIS
        """Splits the world into 9 sections, and returns objects in each section in a list. Objects such as lines are put into
            a separate list."""
        collision_sections= []
        other = []
        for object in self.objects:
            try:
                if object.pos[0] >= 500 or object.pos[1] >= 500:
                    self.objects.remove(object)
                else:
                    xs = int(object.pos[0] / 167)
                    ys = int(object.pos[1] / 167)
                    sections[xs][ys].append(object)
            except:
                other.append(object)
                
            
            return collision_sections, other
            
    

###############################################################################################