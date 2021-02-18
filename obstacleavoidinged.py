#implementing a robot obstacle avoidance algorithm

#import libraries
import simpleguitk as simplegui
import math
import random
import time
from threading import Thread ,Lock
from scipy.spatial import distance as distNum
from statistics import mean 
#define constants

OBSTACLE_RAD = 30 # how big (radius) are the obstacles
ROBOT_RAD = 10 # how big (radius) is the robot?
SENSOR_FOV = 10.0       #10.0 # FOV of each sensor THIS MUST BE A FLOAT!!!!!! 
SENSOR_MAX_R = 400 # max range that each sensor can report
SENSOR_ALERT_R = 315 #range within which sensor reports are acted upon
TURN_SCALE_FACTOR = 2.5 # how drastic do we want the turns to be
SAFETY_DISTANCE = 30
#helper functions

def rel_brg_fm_offset_sensor(true_hdg, sensor_offset, tgt_brg):
    #given robot's true heading, the sensor offset angle and the
    #true brg of the target, this fn will return the relative brg
    #of the target from the sensor's line of sight
    sensor_look_brg = (true_hdg + sensor_offset)%360
    tgt_rel_fm_sensor = tgt_brg - sensor_look_brg

    # if tgt_rel_fm_sensor < -180:
    #     tgt_rel_fm_sensor += 360
    
    return tgt_rel_fm_sensor

def brg_in_deg(p0, p1):#bearing only in degrees
    [x1, y1] = p0
    [x2, y2] = p1
    a = math.degrees(math.atan((y1 - y2)/(x1 - x2 + 0.000000001)))
    #find and correct the quadrant...
    if  x2 >= x1:
        b = 90 + a
    else:
        b = 270 + a
    return b

def dist(p1, p0):#distance only
    return math.sqrt((p1[0] - p0[0])**2+(p1[1]-p0[1])**2)

def dist_and_brg_in_deg(p0, p1):#bearing and distance in degrees between two points
    [x1, y1] = p0
    [x2, y2] = p1
    r = math.sqrt((x1 - x2)**2 + (y1 - y2)**2) # distance
    a = math.degrees(math.atan((y1 - y2)/(x1 - x2 + 0.000000001)))
    #find and correct the quadrant...
    if  x2 >= x1:
        b = 90 + a
    else:
        b = 270 + a
    return r, b

def angle_to_vector(ang):#resolve angles into vectors
    ang = math.radians(ang)
    return [math.cos(ang), math.sin(ang)]

def relative_brg(b1, b2):
    rb = b2 - b1
    if rb > 180:
        rb = 360 - rb
    if rb < -180:
        rb += 360
        rb *= -1
    return rb

def create_vector(from_pos, length, brg):       
        u_vec = angle_to_vector(brg)
        #print "generating target line..."
        vec0 = from_pos[0] + length * u_vec[1] 
        vec1 = from_pos[1] - length * u_vec[0]
        
        return [vec0, vec1]

#define classes

class Sonar:
    def __init__(self, index, FOV, max_r, robot_co):
        #create a instance of this class
        self.pos = [0,0]
        self.index = index
        self.max_r = max_r
        self.FOV = FOV
        self.offset =  index * FOV #+ FOV/2 # on what relative bearing is this sensor looking?
        print(self.offset)
        self.look_brg = (robot_co + self.offset)%360
        self.vec = [0,0] # just a vector for grpahical ouptut of pings
        self.has_valid_echo = False #indicates if this sonar has a "valid" obstacle in sight
        #print "Creating Sonar:" , index, " offset ", self.offset, "true LOS:", self.look_brg
            
    def ping_actual():#ping for real in a robot and return range observed
        pass
    
    def ping_simulated(self, obstacle_list, robot_co):
        #from robot position and robot_co, run through obs_list
        #return the distance to closest object within 
        #FOV and within self.max_r of THIS SENSOR
        # range all the obstacles in view, find the nearest
        
        range_list = []
        
        for obs in obstacle_list:# find objects within max_r and inside FOV
            #print "pinging for robot_co:", robot_co
            can_observe, d = self.can_observe(robot_co, obs, OBSTACLE_RAD)
            #print can_observe, d
            if can_observe:
                range_list.append(d)
        if len(range_list) > 0:
            self.output = min(range_list)- SAFETY_DISTANCE
        else:
            self.output = SENSOR_MAX_R
        # if self.index == 12:
        #     print("RL: ", range_list, "OP: ",self.output)
        #print "closest point: " , obs , " distance:", self.output
    
    def get_output(self):
        return self.output
    
    def can_observe(self, robot_co, obstacle_pos, obstacle_rad):
        #if obstacle is within within the max_r of the sensor
        #and within FOV, return True and the distance observed
        #else return False, 0
        
        dist, brg = dist_and_brg_in_deg(self.pos, obstacle_pos)
        
        if dist < self.max_r: #if the object is within max_r....
            rel_brg = rel_brg_fm_offset_sensor(robot_co, self.offset, brg)#rel brg of tgt from sensor LOS
            # d_test = abs(dist * math.sin(math.radians(rel_brg) + 0.000000001))
            if dist >= 0.1 and abs(rel_brg) <= self.FOV*1.5:
                self.has_valid_echo = True
                return True, dist # if the object is within min allowed lateral separation
            else:
                self.has_valid_echo = False
                return False, 0 # ignore it
        else:#if the object is outside max_r of this sonar...ignore it
            self.has_valid_echo = False
            return False, 0
      
    def update(self, platform_pos, platform_co, obstacle_list):
              
        #update own parameters
        self.pos = platform_pos
        
        self.look_brg = (platform_co + self.offset)%360
        
        #calculate output of this sensor
        
        self.ping_simulated(obstacle_list, platform_co)
        
        self.vec = create_vector(self.pos, self.output + ROBOT_RAD, self.look_brg)#calculate distance vector for drawing on canvas
        
        #print "sensor index:", self.index, " look brg:", self.look_brg
    def draw(self, canvas): # draw the sensor's output
        #if self.has_valid_echo:
        canvas.draw_line(self.pos,self.vec, 1, 'lime')
        canvas.draw_text(str(self.index),(self.vec[0]+4, self.vec[1]+4), 10, "lime"),
        #print self.index, " VE:" , self.has_valid_echo
        
class Sonar_Array:
    def __init__(self, n_sensors, SENSOR_FOV, SENSOR_MAX_R, robot_co):
        self.sonar_list = []
        self.need_diversion_flag = False
        
        i_pos = [x for x in range(1, int(n_sensor/2) + 1)]
        i_pos.extend([11])
        i_pos.extend([12])
        i_neg = [x for x in range(-int(n_sensor/2) , 0)]
        i_pos.reverse()
        i_neg.reverse()
        i_pos.extend([0])
        i_pos.extend(i_neg)
        i_pos.extend([-11])
        i_pos.extend([-12])
        self.i_pos = i_pos
        self.goal_brg = 0
        
        for i in i_pos:#create a list of individual sonars...
            self.sonar_list.append(Sonar(i, SENSOR_FOV, SENSOR_MAX_R, robot_co))

    def NormaliseVector(self, v):
        '''Return the normalised vector'''
        d = math.sqrt(v[0]*v[0] + v[1]*v[1])
        return [v[0]/d, v[1]/d]

    def update(self, robot_pos, robot_co, obstacle_list, method):
        #update sonar array
        for sonar in self.sonar_list:#update output of each sensor
            sonar.update(robot_pos, robot_co, obstacle_list)
            
        if method == "w_sum":#process data by method of weighted sums
            return self.weighted_sum_method(robot_pos, robot_co)
    
    def weighted_sum_method(self, robot_pos, robot_co):
        #process data by the weighted sum method and 
        #return (1) whether turn is required or not (2) index of recommended sonar LOS to turn to
        sum_d = 0
        sum_wt = 0
        alert = False
        #print "checking all sonars:" 
        g_list = []
        d_list = []
        str_vec = 0.0

        so = [x.output for x in self.sonar_list]

        if mean(so[2:14]) < SENSOR_ALERT_R:#has this sonar found anything in danger zone?
            alert = True

            d_11 = self.sonar_list[0].output
            d_12 = self.sonar_list[1].output
            d11 = self.sonar_list[15].output
            d12 = self.sonar_list[16].output

            d1 = self.sonar_list[2].output
            d2 = self.sonar_list[3].output
            d3 = self.sonar_list[4].output
            d4 = self.sonar_list[5].output
            d5 = self.sonar_list[6].output
            d6 = self.sonar_list[7].output
            d7 = self.sonar_list[8].output
            d7 = min(so[7:9])
            d8 = self.sonar_list[9].output
            d9 = self.sonar_list[10].output
            d10 = self.sonar_list[11].output
            d11 = self.sonar_list[12].output
            d12 = self.sonar_list[13].output
            d13 = self.sonar_list[14].output

            self.goal_brg = brg_in_deg(robot_pos, goal_pos)

            theta =  8500 * (1/min(d6,d8)) * ((1/d8)-(1/d6))
            theta += 7500 * (1/min(d9,d5)) * ((1/d9)-(1/d5))
            theta += 6500 * (1/min(d10,d4)) * ((1/d10)-(1/d4))
            theta += 6000 * (1/min(d11,d3)) * ((1/d11)-(1/d3))
            theta += 5500 * (1/min(d12,d2)) * ((1/d12)-(1/d2))
            theta += 5000 * (1/d7) * ((1/d13)-(1/d1))

            print("Bearing: ", self.goal_brg, "avoid: ", theta, "correction: ", (0.04 * ((self.goal_brg+360) - (robot_co+360))) )
            theta += 0.04 * ((self.goal_brg+360) - (robot_co+360))
            # theta = 50000 * (1/d3) * ((1/d4)-(1/d2)) + (0.02 * (self.goal_brg - robot_co))
            rec_index = round(theta)

            if mean(so[7:9]) < 150:
                print("Dead zone")
                ltheta = theta
                theta += 35000 * (1/d3) * ((1/d12)-(1/d_11))
                theta += 35000 * (1/d3) * ((1/d11)-(1/d_12))
                print("Dead avoidance: ", theta - ltheta)
                rec_index = round(theta)

            if abs(rec_index) > n_sensor/2:
                print("rec index too large")
                # rec_index = n_sensor/2
                if rec_index < 0:
                    rec_index = -n_sensor/2
                else:
                    rec_index = n_sensor/2

        else: #no obstacle in danger zone
            alert = False
            self.goal_brg = brg_in_deg(robot_pos, goal_pos)
            print("Bearing: ", self.goal_brg ,"correction: ", (0.03* ((self.goal_brg+360) - (robot_co+360))), "robot_co: ", robot_co)
            theta = 0.03 * ((self.goal_brg+360) - (robot_co+360))
            rec_index = round(theta)
            if abs(rec_index) > n_sensor/2:
                print("rec index too large")
                # rec_index = n_sensor/2
                if rec_index < 0:
                    rec_index = -n_sensor/2
                else:
                    rec_index = n_sensor/2

        print("Rec index:", rec_index)
        # print("Robot Pos: ", robot_pos)

        offset =  rec_index * SENSOR_FOV #how much is the angular offset
        print("offset: ", offset)
        print("robot_co: ", robot_co)
        return robot_co+offset, True, False, robot_co+offset

    def draw(self, canvas):
        for sonar in self.sonar_list:
            sonar.draw(canvas)

class Robot:
    def __init__(self, pos, co, n_sensor):
        self.pos = pos
        self.history = [pos]
        self.co = co
        self.real_bearing = 0
        self.spd = 5 # robot speed in pixels/ step
        self.s_array = Sonar_Array(n_sensor, SENSOR_FOV, SENSOR_MAX_R, self.co)
        self.goal_brg = 0
        self.obstacles_in_view = []
    
    def get_obstacles_in_view(self):
        return self.obstacles_in_view
    
    def update(self):
        self.obstacles_in_view = [] #delete all the old obstacles in view
        for obs in full_obstacle_list:
            if dist(self.pos, obs) < SENSOR_MAX_R:
                self.obstacles_in_view.append(obs)
                
        #re-calculate direction to goal
        # self.goal_brg = brg_in_deg(self.pos, goal_pos)
        #re-estimate sensor output by weighted sum method
        co1, need_turn, no_alter, self.real_bearing = self.s_array.update(self.pos, self.co, self.obstacles_in_view, "w_sum")
        self.goal_brg = self.real_bearing
        self.co = co1
        # #print "Path Clear:", self.path_is_clear()
        # if self.path_is_clear():#can we reach the goal directly from here?
        #     self.co = brg_in_deg(self.pos, goal_pos)
        #     print("path clear. ignoring recommendation")
        # elif need_turn: #do we need to turn
        #     self.co = co1
        #     print("path not clear. following recommendation")
        # else: # path is not fully clear, but there are no immediate obstacles
        #     pass
            #self.co = brg_in_deg(self.pos, goal_pos)

        #move the robot by one step...
        self.move(1)
        return no_alter

    def path_is_clear(self):#return True if there is a clear path to the goal
        goal_brg = brg_in_deg(self.pos, goal_pos)
        for obs in self.obstacles_in_view:
            if dist(self.pos, goal_pos) > dist(self.pos, obs):
                d_obs, obs_brg = dist_and_brg_in_deg(self.pos, obs)
                rel_brg = abs(relative_brg(goal_brg, obs_brg))
                d_lateral = abs(d_obs * math.sin(math.radians(rel_brg)))
                if d_lateral < OBSTACLE_RAD + ROBOT_RAD: 
                    return False
        return True
    
    def move(self, dT):

        u_vec = angle_to_vector(self.co)
        
        self.pos[0] += self.spd * dT * u_vec[1]
        self.pos[1] -= self.spd * dT * u_vec[0]
        
        self.history.append([self.pos[0], self.pos[1]])
        
    def get_pos(self):
        return self.pos
    
    def set_pos(self, pos):
        self.pos = pos
    
    def set_co(self, co):
        self.co = co
        #print "setting robot co:", self.co
    
    def delete_history(self):
        self.history = []
   
    def draw(self, canvas):
        #Draw the robot
        canvas.draw_circle(self.pos, 4, 3, "yellow")
        canvas.draw_text("R", [self.pos[0] + 10, self.pos[1] +10], 16, "yellow")
        #Draw brg line to goal
        self.goal_vec = create_vector(self.pos, 150, self.goal_brg)
        canvas.draw_line(self.pos, self.goal_vec, 2, "teal")
        #Draw current heading vector
        self.co_vec = create_vector(self.pos, 150, self.co)
        canvas.draw_line(self.pos, self.co_vec, 2, "white")
        #draw the output of the sonar array
        self.s_array.draw(canvas)
        #draw the obstacles in view
        for obs in self.obstacles_in_view:
            canvas.draw_circle(obs,2,1, "red")
            canvas.draw_circle(obs,OBSTACLE_RAD, 1, "green") 
        #draw history
        for point in self.history:
            canvas.draw_circle(point,2,2, "lime")
        
        
#define globals

g_state = "None"

thread = 0 
start_pos = [500,500]
robot_pos = [500, 500]
robot_co = 130
goal_pos = [570,380]
ind = 0
goal_pos_list = [[20,20] , [20,980] , [980,980] , [980,20] , [300,300] , [300, 600] , [600,600] , [600, 300] , [500,500]]
goal_pos = goal_pos_list[ind]
#obstacle_list = [(300, 213), (310, 124), (250, 110), (300, 230)]
full_obstacle_list = []
#full_obstacle_list = [(136, 7), (133, 57), (141, 116), (152, 166), (147, 233), (151, 203), (157, 282), (155, 327), (202, 336), (249, 331), (296, 328), (345, 323), (348, 281), (336, 222), (341, 174), (327, 126), (334, 67), (336, 12), (7, 844), (65, 840), (119, 831), (188, 836), (153, 834), (199, 786), (195, 718), (199, 752), (199, 682), (159, 652)]
full_obstacle_list = [(136, 7), (133, 57), (141, 116), (152, 166), (147, 233), (151, 203), (157, 282), (155, 327), (202, 336), (249, 331), (296, 328), (345, 323), (348, 281), (336, 222), (341, 174), (327, 126), (334, 67), (336, 12), (7, 844), (65, 840), (119, 831), (188, 836), (153, 834), (199, 786), (195, 718), (199, 752), (199, 682), (159, 652), (557, 336), (576, 294), (576, 252), (589, 204), (622, 163), (661, 152), (710, 141), (758, 139), (793, 159), (835, 182), (872, 208), (900, 246), (919, 283), (940, 330)]


#create a robot with 6 sensors

n_sensor =  12 #34

#create a sonar array
#s1 = Sonar_Array(n_sensor, SENSOR_FOV, SENSOR_MAX_R, robot_co)
r1 = Robot(robot_pos, robot_co, n_sensor)

_ = r1.update()
#define event handlers

def click(pos):
    global g_state, start_pos, goal_pos, robot_pos
    if g_state == "Start":
        start_pos = pos
        r1.set_pos(list(pos))
    elif g_state == "Goal":
        goal_pos = pos
        r1.set_co(brg_in_deg(r1.get_pos(), pos))
    elif g_state == "Set Robot":
        r1.set_co(brg_in_deg(r1.get_pos(), pos))
        r1.set_pos(list(pos))
        r1.delete_history()
    elif g_state == "Add Obs":
        full_obstacle_list.append(pos)
        print(full_obstacle_list)
        #update the robot
    # r1.update()
    g_state = "None"

def set_start():
    global g_state
    g_state = "Start"
    
def set_goal():
    global g_state
    g_state = "Goal"

def set_robot_pos():
    global g_state
    g_state = "Set Robot"

def alter_co(text):
    r1.set_co(float(text))
    _ = r1.update()

def go_fun():
    thread = Thread(target=gogo, args=())
    thread.daemon = True
    thread.start()
    
def gogo():
    global goal_pos, ind
    ind = 0
    while True:
        no_alter = r1.update()
        time.sleep(0.08)
        dia = distNum.euclidean( r1.get_pos(), goal_pos_list[ind] )
        # print(" Dia : ", r1.get_pos(), goal_pos_list[ind], dia)
        if dia <= 50.0 or no_alter is True:
            ind += 1
            goal_pos = goal_pos_list[ind]
            r1.set_co(brg_in_deg(r1.get_pos(), goal_pos_list[ind]))

def draw(canvas):
    #draw start 
    canvas.draw_circle(start_pos, 4, 3, "red")
    canvas.draw_text("S", [start_pos[0] + 10, start_pos[1] +10], 16, "red")
    #draw goal
    canvas.draw_circle(goal_pos, 4, 3, "green")
    canvas.draw_text("G", [goal_pos[0] + 10, goal_pos[1] +10], 16, "green")
    #draw the obstacles
    for obs in full_obstacle_list:
        canvas.draw_circle(obs,2,1, "red")
        canvas.draw_circle(obs,OBSTACLE_RAD, 1, "white") 
    
    #draw sonar lines...
    r1.draw(canvas)

def step():
    _ = r1.update()

def add_obs():
    global g_state
    g_state = "Add Obs"    

#create simplegui controls

f1 = simplegui.create_frame("Obs Avoidance", 1000, 1000)
btn_start = f1.add_button("Set Start", set_start, 100)
btn_goal = f1.add_button("Set Goal", set_goal, 100)
btn_robot = f1.add_button("Set Robot", set_robot_pos, 100)
txt_r_co = f1.add_input("Robot Co", alter_co, 100)
btn_step = f1.add_button("Step", step, 100)
btn_add_obs = f1.add_button("Add Obs", add_obs, 100)
btn_go = f1.add_button("GO", go_fun, 100)

f1.set_draw_handler(draw)
f1.set_mouseclick_handler(click)

#start simplegui

f1.start()
