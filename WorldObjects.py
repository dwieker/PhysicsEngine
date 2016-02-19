from helperMath import *
from graphics import *

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
            self.I - self.I(axis)
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
        x = self.pts[:,0].take(range(0, self.pts[:,0].size + 1), mode = 'wrap')
        y = self.pts[:,1].take(range(0, self.pts[:,0].size + 1), mode = 'wrap')      
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

####################### 
# Functions that handle each object
#######################
def handle_ball(ball, objects, world):
    """Update the given polygon's velocity and rotation, which is affected potentially by all other objects"""
    
    #Handle gravity
    ball.apply_impulse(vect(0,-world.gravity*ball.mass), world.dt)

    for object in objects:
        if object.__class__.__name__=="Ball":
            handle_bb(ball, object, world)
        elif object.__class__.__name__=="RigidPolygon":
            handle_pb(object, ball, world)
        elif object.__class__.__name__=="Bomb":
            handle_object_bomb(ball,object,world)
        elif object.__class__.__name__=="Wall":
            handle_bw(ball,object,world)
        elif object.__class__.__name__=="BlackHole":
            handle_oBH(ball,object,world)
            

def handle_polygon(polygon, objects, world):
    """Update the given polygon's velocity and rotation, which is affected potentially by all other objects"""
    dt = world.dt
    gravity = world.gravity
    win = world.win
    damping = world.damping
        

    polygon.apply_impulse(polygon.m_center, polygon.mass*gravity*vect(0,-1) * dt)
    
    
    for object in objects:     
        if object.__class__.__name__=="Ball":
            handle_pb(polygon, object, world)
        elif object.__class__.__name__=="Bomb":
            pass
        elif object.__class__.__name__=="Wall":
            handle_pw(polygon, object, world)
        elif object.__class__.__name__=="BlackHole":
            pass

  

def handle_bb(ball, object, world):
    """Given 2 balls, solve and set their final velocities after collision"""
    dt = world.dt
    damping = world.damping
    
    #make sure the balls are approaching eachother
    collisionNorm = norm(object.pos - ball.pos)
    if(ball.v.dot(collisionNorm) + object.v.dot(-collisionNorm)) < 0:
        return
        
    #check if they will contact in next update
    next_b_pos = ball.pos + ball.v * dt
    next_o_pos = object.pos + object.v * dt
     
    if (next_o_pos[0] - next_b_pos[0])**2 + (next_o_pos[1] - next_b_pos[1])**2 <= (ball.r + object.r)**2:
        collisionTangent = norm(vect(-collisionNorm[1], collisionNorm[0]))        
        normVmag = (collisionNorm.dot(ball.v)*(ball.mass - object.mass) + 2*object.mass*object.v.dot(collisionNorm) ) / (ball.mass + object.mass) 
        normV = collisionNorm * normVmag      
        tanV = collisionTangent*collisionTangent.dot(ball.v)         
        ball_v = damping*(normV + tanV)
                      
        ##### Handle other ball         
        collisionNorm = -collisionNorm
        collisionTangent = norm(vect(-collisionNorm[1], collisionNorm[0]))           
        normVmag = (collisionNorm.dot(object.v)*(object.mass - ball.mass) + 2*ball.mass*ball.v.dot(collisionNorm) ) / (ball.mass + object.mass) 
        normV = collisionNorm * normVmag      
        tanV = collisionTangent*collisionTangent.dot(object.v)        
        object_v = damping*(tanV + normV)
                  
        object.v = object_v
        ball.v = ball_v
    
def handle_bw(ball, wall, world):
    dt = world.dt
    next_b_pos = ball.pos + ball.v * dt
    if wall.distance_to(next_b_pos) <= ball.r and wall.distance_to(next_b_pos) < wall.distance_to(ball.pos):
        v_n = ball.v.dot(wall.n) * wall.n
        v_t = ball.v.dot(wall.t) * wall.t
        ball.v = -v_n + v_t
        
def handle_pw(polygon, wall, world):
    dt = world.dt
    #check if line intersects bounding box by plugging opposite corner poitns into line equation. signs should be opposite
    pt1 = polygon.m_center + vect(-polygon.bbox, polygon.bbox) #top left corner
    pt2 = polygon.m_center + vect(polygon.bbox, -polygon.bbox) # top bottom corner
    f = lambda pt: wall.a*pt[0] + wall.b*pt[1] + wall.c
    impulses = []
      
    if f(pt1)*f(pt2) <= 0:  
        pts = polygon.points()
        polygon.update(dt)
        next_pts = polygon.points()
        polygon.update(-dt)
        
       
        for i in range(0,len(pts)):
            if f(pts[i])*f(next_pts[i]) <= 0:
                if f(pts[i]) > 0:
                    n = wall.n
                else: 
                    n = -wall.n 
                    
                r = pts[i] - polygon.m_center
                V = polygon.v + np.cross(np.array([0,0,polygon.w]),r)[:-1]
                j = -(1+world.damping)*V.dot(n) / ((1./polygon.mass) + np.cross(r,n)**2 / polygon.I)
                impulses.append((pts[i], j, n))
        
    for j in impulses:    
        polygon.apply_impulse(j[0], j[1]*j[2] / len(impulses))
          
                            
            

def handle_pb(polygon, ball, world):
    
    #Check if bounding boxes overlap
    poly_max_x, poly_min_x = polygon.m_center[0] + polygon.bbox, polygon.m_center[0] - polygon.bbox  
    poly_max_y, poly_min_y = polygon.m_center[1] + polygon.bbox, polygon.m_center[1] - polygon.bbox  
    ball_max_x, ball_min_x = ball.pos[0] + ball.r, ball.pos[0] - ball.r #bot
    ball_max_y, ball_min_y = ball.pos[1] + ball.r, ball.pos[1] - ball.r #bot
    if(poly_max_x < ball_min_x or poly_min_x > ball_max_x): return  
    if(poly_max_y < ball_min_y or poly_min_y > ball_max_y): return


    #For each face of polygon, generate a line and check ball's distance to that face. If less than radius, there is overalap
    pts = np.vstack((p.points(), p.points()[0])) #end of list must be the same point as begining of list
    for i in range(polygon.size):
        line = pts_to_line(pts[i], pts[i + 1])
        t = norm(vect(pts[i+1,0] - pts[i,0], pts[i+1,1] - pts[i,1])) #vect parallel to line
        r1 = ball.pos - pts[i]
        r2 = ball.pos - pts[i+1]
        
        if distance_to_line(ball.pos, line) < ball.r:
            if t.dot(r1)*t.dot(r2) <= 0:
                tx = pts[i+1,0] - pts[i,0] 
                ty = pts[i+1,1] - pts[i,1]
                x = -(tx*ball.pos[0] + ty*ball.pos[1] + line[2]*ty) / (line[0]*ty - tx)
                y = -line[2] - line[0]*x
            
            #Check for corner contacts
            elif t.dot(r1) > -(np.sqrt(2)/2) * mag(r1) and t.dot(r1) < 0:
                x = pts[i,0]
                y = pts[i,1]
                  
            elif t.dot(r2) < (np.sqrt(2)/2) * mag(r2) and t.dot(r2) > 0: 
                x = pts[i+1,0]
                y = pts[i+1,1]
             
            
            else: continue
                
            r = vect(x,y) - polygon.m_center
            n = norm(vect(x,y) - ball.pos)
            V_a = polygon.v + np.cross(np.array([0,0,polygon.w]),r)[:-1]
            V_b = ball.v
            
            j = -(1+world.damping)*(V_a - V_b).dot(n) / ((1./polygon.mass) + (1./ball.mass) + (np.cross(r,n)**2 / polygon.I))      
            polygon.apply_impulse(vect(x,y), j*n)
            ball.apply_impulse(-j*n)
            return
    
                     
def handle_object_bomb(object, bomb, world):
    """Handles interaction between any object and a bomb. Bomb gives object random impulse according to its strength, 
       and disappears"""
    win = world.win
    dt = world.dt
    if(bomb.armed and mag(object.pos - bomb.pos) < object.r):
        bomb.explode(win)
        object.v = vect(0,0)
        force_v = norm(object.pos - bomb.pos)
        object.apply_impulse(force_v * bomb.strength, dt)

def handle_oBH(object, black_hole, world):
    dt = world.dt
    rr = ((object.pos[0] - black_hole.pos[0])**2 + (object.pos[1] - black_hole.pos[1])**2)
    if rr < object.r**2 + black_hole.r**2:
        force = 0
    else:
        force = black_hole.mass * object.mass / rr
    force_v = norm(black_hole.pos - object.pos) * force
    object.apply_impulse(force_v, dt)