"""
Final TORCS driver used by Pyton Slayers.

The driver is based on live telemetry from TORCS. It uses track sensors,
speed, angle, track position, lateral speed and RPM to control steering,
braking, throttle and gear shifting.

Main ideas:
- detect upcoming turn direction from track sensors
- split corners into driving stages
- brake based on available distance and estimated safe speed
- reduce throttle during risky steering/braking situations
- use recovery logic when the car becomes unstable or leaves the center
"""
import socket
import sys
import getopt
import os
import time
import math
PI= 3.14159265359

data_size = 2**17

ophelp=  'Options:\n'
ophelp+= ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp+= ' --port, -p <port>    TORCS port. [3001]\n'
ophelp+= ' --id, -i <id>        ID for server. [SCR]\n'
ophelp+= ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp+= ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp+= ' --track, -t <track>  Your name for this track. Used for learning. [unknown]\n'
ophelp+= ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp+= ' --debug, -d          Output full telemetry.\n'
ophelp+= ' --help, -h           Show this help.\n'
ophelp+= ' --version, -v        Show current version.'
usage= 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage= usage + ophelp
version= "20130505-2"

def clip(v,lo,hi):
    if v<lo: return lo
    elif v>hi: return hi
    else: return v

def bargraph(x,mn,mx,w,c='X'):
    '''Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.'''
    if not w: return '' # No width!
    if x<mn: x= mn      # Clip to bounds.
    if x>mx: x= mx      # Clip to bounds.
    tx= mx-mn # Total real units possible to show on graph.
    if tx<=0: return 'backwards' # Stupid bounds.
    upw= tx/float(w) # X Units per output char width.
    if upw<=0: return 'what?' # Don't let this happen.
    negpu, pospu, negnonpu, posnonpu= 0,0,0,0
    if mn < 0: # Then there is a negative part to graph.
        if x < 0: # And the plot is on the negative side.
            negpu= -x + min(0,mx)
            negnonpu= -mn + x
        else: # Plot is on pos. Neg side is empty.
            negnonpu= -mn + min(0,mx) # But still show some empty neg.
    if mx > 0: # There is a positive part to the graph
        if x > 0: # And the plot is on the positive side.
            pospu= x - max(0,mn)
            posnonpu= mx - x
        else: # Plot is on neg. Pos side is empty.
            posnonpu= mx - max(0,mn) # But still show some empty pos.
    nnc= int(negnonpu/upw)*'-'
    npc= int(negpu/upw)*c
    ppc= int(pospu/upw)*c
    pnc= int(posnonpu/upw)*'_'
    return '[%s]' % (nnc+npc+ppc+pnc)

class Client():
    def __init__(self,H=None,p=None,i=None,e=None,t=None,s=None,d=None,vision=False):
        self.vision = vision

        self.host= 'localhost'
        self.port= 3001
        self.sid= 'SCR'
        self.maxEpisodes=1 # "Maximum number of learning episodes to perform"
        self.trackname= 'unknown'
        self.stage= 3 # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug= False
        self.maxSteps= 100000  # 50steps/second
        self.parse_the_command_line()
        if H: self.host= H
        if p: self.port= p
        if i: self.sid= i
        if e: self.maxEpisodes= e
        if t: self.trackname= t
        if s: self.stage= s
        if d: self.debug= d
        self.S= ServerState()
        self.R= DriverAction()
        self.setup_connection()

    def setup_connection(self):
        try:
            self.so= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...')
            sys.exit(-1)
        self.so.settimeout(1)

        n_fail = 5
        while True:
            a= "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"

            initmsg='%s(init %s)' % (self.sid,a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata= str()
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print("Waiting for server on %d............" % self.port)
                print("Count Down : " + str(n_fail))
                if n_fail < 0:
                    print("relaunch torcs")
                    os.system('pkill torcs')
                    time.sleep(1.0)
                    if self.vision is False:
                        os.system('torcs -nofuel -nodamage -nolaptime &')
                    else:
                        os.system('torcs -nofuel -nodamage -nolaptime -vision &')

                    time.sleep(1.0)
                    os.system('sh autostart.sh')
                    n_fail = 5
                n_fail -= 1

            identify = '***identified***'
            if identify in sockdata:
                print("Client connected on %d.............." % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                       ['host=','port=','id=','steps=',
                        'episodes=','track=','stage=',
                        'debug','help','version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug= True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host= opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid= opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname= opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage= int(opt[1])
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port= int(opt[1])
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes= int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps= int(opt[1])
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print('Bad parameter \'%s\' for option %s: %s\n%s' % (
                                       opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata= str()

        while True:
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('.', end=' ')
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print((("Server has stopped the race on %d. "+
                        "You were in %d place.") %
                        (self.port,self.S.d['racePos'])))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print("Server has restarted the race on %d." % self.port)
                self.shutdown()
                return
            elif not sockdata: # Empty?
                continue       # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H") # Clear for steady output.
                    print(self.S)
                break # Can now return from this function.

    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1],str(emsg[0])))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps,self.port)))
        self.so.close()
        self.so = None

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])

    def __repr__(self):
        return self.fancyout()
        out= str()
        for k in sorted(self.d):
            strout= str(self.d[k])
            if type(self.d[k]) is list:
                strlist= [str(i) for i in self.d[k]]
                strout= ', '.join(strlist)
            out+= "%s: %s\n" % (k,strout)
        return out

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out= str()
        sensors= [ # Select the ones you want in the order you want them.
        'stucktimer',
        'fuel',
        'distRaced',
        'distFromStart',
        'opponents',
        'wheelSpinVel',
        'z',
        'speedZ',
        'speedY',
        'speedX',
        'targetSpeed',
        'rpm',
        'skid',
        'slip',
        'track',
        'trackPos',
        'angle',
        ]

        for k in sensors:
            if type(self.d.get(k)) is list: # Handle list type data.
                if k == 'track': # Nice display for track sensors.
                    strout= str()
                    raw_tsens= ['%.1f'%x for x in self.d['track']]
                    strout+= ' '.join(raw_tsens[:9])+'_'+raw_tsens[9]+'_'+' '.join(raw_tsens[10:])
                elif k == 'opponents': # Nice display for opponent sensors.
                    strout= str()
                    for osensor in self.d['opponents']:
                        if   osensor >190: oc= '_'
                        elif osensor > 90: oc= '.'
                        elif osensor > 39: oc= chr(int(osensor/2)+97-19)
                        elif osensor > 13: oc= chr(int(osensor)+65-13)
                        elif osensor >  3: oc= chr(int(osensor)+48-3)
                        else: oc= '?'
                        strout+= oc
                    strout= ' -> '+strout[:18] + ' ' + strout[18:]+' <-'
                else:
                    strlist= [str(i) for i in self.d[k]]
                    strout= ', '.join(strlist)
            else: # Not a list type of value.
                if k == 'gear': # This is redundant now since it's part of RPM.
                    gs= '_._._._._._._._._'
                    p= int(self.d['gear']) * 2 + 2  # Position
                    l= '%d'%self.d['gear'] # Label
                    if l=='-1': l= 'R'
                    if l=='0':  l= 'N'
                    strout= gs[:p]+ '(%s)'%l + gs[p+3:]
                elif k == 'damage':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,10000,50,'~'))
                elif k == 'fuel':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,100,50,'f'))
                elif k == 'speedX':
                    cx= 'X'
                    if self.d[k]<0: cx= 'R'
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-30,300,50,cx))
                elif k == 'speedY': # This gets reversed for display to make sense.
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k]*-1,-25,25,50,'Y'))
                elif k == 'speedZ':
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-13,13,50,'Z'))
                elif k == 'z':
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k],.3,.5,50,'z'))
                elif k == 'trackPos': # This gets reversed for display to make sense.
                    cx='<'
                    if self.d[k]<0: cx= '>'
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k]*-1,-1,1,50,cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout= '%3d %s' % (self.d[k], bargraph(self.d[k],0,300,50,"'"))
                    else: strout= 'Not stuck!'
                elif k == 'rpm':
                    g= self.d['gear']
                    if g < 0:
                        g= 'R'
                    else:
                        g= '%1d'% g
                    strout= bargraph(self.d[k],0,10000,50,g)
                elif k == 'angle':
                    asyms= [
                          "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                          "---  ", ".__  ", "-._  ", "'-.  ", "'\.  ", "'|.  ",
                          "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                          "  ---", "  --.", "  -._", "  -..", "  '\.", "  '|."  ]
                    rad= self.d[k]
                    deg= int(rad*180/PI)
                    symno= int(.5+ (rad+PI) / (PI/12) )
                    symno= symno % (len(asyms)-1)
                    strout= '%5.2f %3d (%s)' % (rad,deg,asyms[symno])
                elif k == 'skid': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    skid= 0
                    if frontwheelradpersec:
                        skid= .5555555555*self.d['speedX']/frontwheelradpersec - .66124
                    strout= bargraph(skid,-.05,.4,50,'*')
                elif k == 'slip': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    slip= 0
                    if frontwheelradpersec:
                        slip= ((self.d['wheelSpinVel'][2]+self.d['wheelSpinVel'][3]) -
                              (self.d['wheelSpinVel'][0]+self.d['wheelSpinVel'][1]))
                    strout= bargraph(slip,-5,150,50,'@')
                else:
                    strout= str(self.d[k])
            out+= "%s: %s\n" % (k,strout)
        return out

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0
                    }

    def clip_to_limits(self):
        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to. The "clip" command is still a snakeoil
        utility function, but it should be used only for non standard
        things or non obvious limits (limit the steering to the left,
        for example). For normal limits, simply don't worry about it."""
        self.d['steer']= clip(self.d['steer'], -1, 1)
        self.d['brake']= clip(self.d['brake'], 0, 1)
        self.d['accel']= clip(self.d['accel'], 0, 1)
        self.d['clutch']= clip(self.d['clutch'], 0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear']= 0
        if self.d['meta'] not in [0,1]:
            self.d['meta']= 0
        if type(self.d['focus']) is not list or min(self.d['focus'])<-180 or max(self.d['focus'])>180:
            self.d['focus']= 0

    def __repr__(self):
        self.clip_to_limits()
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) is list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'

    def fancyout(self):
        '''Specialty output for useful monitoring of bot's effectors.'''
        out= str()
        od= self.d.copy()
        od.pop('gear','') # Not interesting.
        od.pop('meta','') # Not interesting.
        od.pop('focus','') # Not interesting. Yet.
        for k in sorted(od):
            if k == 'clutch' or k == 'brake' or k == 'accel':
                strout=''
                strout= '%6.3f %s' % (od[k], bargraph(od[k],0,1,50,k[0].upper()))
            elif k == 'steer': # Reverse the graph to make sense.
                strout= '%6.3f %s' % (od[k], bargraph(od[k]*-1,-1,1,50,'S'))
            else:
                strout= str(od[k])
            out+= "%s: %s\n" % (k,strout)
        return out

def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]

#############################################
# MODULAR DRIVE LOGIC WITH USER PARAMETERS  #
#############################################

# ================= USER CONFIGURABLE PARAMETERS =================
TARGET_SPEED = 245  # Target speed in km/h. Increasing this makes the car go faster but may reduce stability.
CENTERING_GAIN = 0.3  # How strongly the car corrects its position toward the center of the track.
# Traction control was tested, but disabled in the final version because
# it reduced lap performance on our final setup.
ENABLE_TRACTION_CONTROL = False
UPSHIFT_RPM = 19500    # Shift up when RPM is above this
DOWNSHIFT_RPM = 10000  # Shift down when RPM is below this
max_deceleration = 14 # m/s²
active_turn = 'straight'
pending_turn = 'straight'
turn_confirm_count = 0
# ================= HELPER FUNCTIONS =================
def calculate_steering(state, turn, stage):
    """
    Calculates steering from track position, car angle, track sensors and
    current corner stage.
    """

    track = state.get('track', None)

    if not isinstance(track, list) or len(track) < 12:
        return 0.0

    target_position = 0.0
    position_gain = 0.0
    turn_bias = 0.0

    front_distance = max(0.1, min(track[8], track[9], track[10], track[7], track[11]))

    if turn == 'left':
        if stage == 'prepare':
            target_position = -0.9
            position_gain = 0.1
            turn_bias = -0.03
        elif stage == 'entry':
            target_position = 0.3
            position_gain = 0.08
            turn_bias = 0.10
        elif stage == 'apex':
            target_position = 1.05
            position_gain = 0.3
            turn_bias = 0.22
        elif stage == 'exit':
            target_position = -0.05
            position_gain = 0.11
            turn_bias = 0.04

    elif turn == 'right':
        if stage == 'prepare':
            target_position = 0.9
            position_gain = 0.1
            turn_bias = 0.03
        elif stage == 'entry':
            target_position = -0.3
            position_gain = 0.08
            turn_bias = -0.10
        elif stage == 'apex':
            target_position = -1.05
            position_gain = 0.3
            turn_bias = -0.22
        elif stage == 'exit':
            target_position = 0.05
            position_gain = 0.11
            turn_bias = -0.04

    else:
        target_position = 0.0
        position_gain = CENTERING_GAIN
        turn_bias = 0.0

    position_error = (state['trackPos'] - target_position) * position_gain

    angle_error_factor = max(0.5, min(1.0, (front_distance / 200.0) * 0.7))
    angle_error = state['angle'] * (15.0 / math.pi) * angle_error_factor

    combined_error = angle_error - position_error + turn_bias

    steering = 0.95 * combined_error

    if abs(steering) < 0.02:
        steering = 0.0

    return max(-1.0, min(1.0, steering))

def upcoming_turn(state):
    """
    Detects whether the upcoming section is a left turn, right turn or straight.

    The result is confirmed over several simulation steps before changing the
    active turn. This prevents the driver from reacting too quickly to noisy
    sensor readings.
    """
    track = state.get('track', None)
    global active_turn, pending_turn, turn_confirm_count

    if not isinstance(track, list) or len(track) < 15:
        return active_turn

    turn_delta =  (track[8]+ track[7]) - (track[11]+ track[10])
    if turn_delta > 1 and track[9] < 150:
        detected_turn= 'left'
    elif turn_delta < -1  and track[9] < 150:
        detected_turn= 'right'
    else:
        detected_turn= 'straight'

    if detected_turn == pending_turn:
        turn_confirm_count += 1
    else:
        pending_turn = detected_turn
        turn_confirm_count =0
    if turn_confirm_count >= 12:
        active_turn = pending_turn
        turn_confirm_count =0

    return active_turn

def turn_opening(state, turn):
    track = state.get('track', None)

    if not isinstance(track, list) or len(track) < 12:
        return 0.0

    front_distance = track[9]

    if turn == 'left':
        side_distance = track[8]
    elif turn == 'right':
        side_distance = track[10]
    else:
        return 0.0

    opening_amount = (side_distance - front_distance) / 100.0

    if opening_amount < 0.0:
        opening_amount = 0.0
    if opening_amount > 1.0:
        opening_amount = 1.0

    return opening_amount

def turn_stage(state, turn, opening_amount):
    """
    Classifies the current part of the corner.

    The stage is used by the steering logic to decide how the car should
    position itself before, during and after the corner.
    """
    track = state.get('track', None)
    if turn == 'straight':
        return 'straight'

    if not isinstance(track, list) or len(track) < 12:
        return 'straight'

    front_distance = min(track[8],track[9],track[10])
    if opening_amount > 0.08:
        return 'exit'
    elif track[9] > 80:
        return 'prepare'
    elif front_distance > 50:
        return 'entry'
    else:
        return 'apex'

def calculate_throttle(state, action):
    """
    Controls acceleration.

    The driver accelerates strongly on straights, but reduces throttle when
    braking is active or when steering demand is high.
    """
    speed = state.get('speedX', 0.0)
    steering = abs(action.get('steer', 0.0))

    brake_command = action.get('brake', 0.0)

    if brake_command > 0.02:
        return 0.0

    acceleration_command = 1.0

    if steering > 0.3:
        acceleration_command *= 0.18
        acceleration_command *= 0.6
    elif steering > 0.25:
        acceleration_command *= 0.2
        acceleration_command *= 0.6
    elif steering > 0.15:
        acceleration_command *= 0.8

    target_speed = TARGET_SPEED

    distance_from_start = state.get('distFromStart', 0.0)
    if distance_from_start > 3350:
        target_speed = 270
    else:
        target_speed = TARGET_SPEED

    if speed >= target_speed:
        acceleration_command= 0.0

    return max(0.0, min(1.0, acceleration_command))

def apply_brakes(state, action):
    """
    Applies braking when the current speed is too high for the available
    distance ahead.

    The logic compares estimated braking distance with the front track sensors.
    """
    speed = state.get('speedX', 0.0)
    track = state.get('track', None)

    if not isinstance(track, list) or len(track) < 15:
        return 0.0

    front_distance = max(0.1,min(track[8], track[9], track[10], track[7], track[11]))
    front_distance = front_distance*0.9

    speed_meters_per_second = speed / 3.6
    safe_speed = 35 + 0.9*front_distance
    safe_speed_meters_per_second = safe_speed / 3.6

    required_braking_distance = max(0.0,(speed_meters_per_second**2 - safe_speed_meters_per_second**2) / (2 * max_deceleration))

    if speed > 120:
        if (track[7] - track[9]) > (front_distance * 0.05) or (track[11] - track[9]) >(front_distance * 0.05):
            return 0.0
        if (track[8] - track[9]) > (front_distance * 0.07) or (track[10] - track[9]) > (front_distance * 0.07):
            return 0.0

    if required_braking_distance > front_distance:
        brake_intensity = (required_braking_distance - front_distance) / front_distance
        return min(1, 0.15+ 0.85 * brake_intensity)

    return 0.0

def shift_gears(state, action):
    rpm = state.get('rpm', 0.0)
    gear = int(action.get('gear', 1))

    if gear < 1:
        gear = 1
    if gear > 6:
        gear = 6

    if rpm > UPSHIFT_RPM and gear < 6:
        gear += 1

    if rpm < DOWNSHIFT_RPM and gear > 1 and action.get('accel', 0.0) < 0.6:
        gear -= 1

    return gear

def traction_control(state, acceleration_command):
    if ENABLE_TRACTION_CONTROL and state['speedX'] > 50:
        rear_wheel_spin = (state['wheelSpinVel'][2] + state['wheelSpinVel'][3]) / 2
        front_wheel_spin = (state['wheelSpinVel'][0] + state['wheelSpinVel'][1]) / 2

        if front_wheel_spin > 1 and rear_wheel_spin > front_wheel_spin * 1.35:
            acceleration_command *= 0.95

    return max(0.0, min(1.0, acceleration_command))

def anti_spin_recover(state, action):
    """
    Recovery logic for unstable lateral movement.
    """
    lateral_speed = state.get('speedY',0.0)
    if abs(lateral_speed) < 16.0:
        return False
    counter_steer = lateral_speed * 0.02
    action['steer']= max(-0.55, min(0.55,counter_steer))
    action['accel'] = 0.10
    action['brake'] = 0.0
    return True

border_active = False
def border_recovery(state, action):
    """
    Recovery logic used when the car moves too far from the center of the track.
    """
    global border_active
    track_position = state.get('trackPos', 0.0)
    angle = state.get('angle', 0.0)
    if not border_active and abs(track_position) > 1.1:
        border_active = True
    if border_active and abs(track_position) < 0.85:
        border_active =False

    if not border_active:
        return False
    steering = (-track_position * 0.4) + (angle * 0.5)
    action['steer'] = max (-1 , min (1, steering))
    action['accel'] = 0.2
    action['brake'] = 0
    return True

def drive_modular(client):
    """
    Main control pipeline for one simulation step.

    This function reads the current TORCS state, detects the turn and corner
    stage, calculates steering, applies recovery logic when needed, then sets
    braking, throttle and gear commands.
    """
    state, action = client.S.d, client.R.d

    turn = upcoming_turn(state)
    opening_amount = turn_opening(state, turn)
    stage = turn_stage(state, turn, opening_amount)
    action['steer'] = calculate_steering(state, turn, stage)

    if anti_spin_recover(state, action):
        action['gear'] = shift_gears(state, action)
        return
    if border_recovery(state, action):
        action['gear'] = shift_gears(state, action)
        return

    action['brake'] = apply_brakes(state, action)

    action['accel'] = calculate_throttle(state, action)

    if action['brake'] > 0.05:
        action['accel'] = min(action['accel'], 0.05)

    action['accel'] = traction_control(state, action['accel'])

    action['gear'] = shift_gears(state, action)

if __name__ == "__main__":
    client = Client(p=3001)
    for step in range(client.maxSteps, 0, -1):
        client.get_servers_input()
        drive_modular(client)
        client.respond_to_server()
    client.shutdown()