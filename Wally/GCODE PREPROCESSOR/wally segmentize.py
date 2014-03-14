# Cartesian model coordinates to Wally Left Polar  
#Corrected YZ plane calculation
import math,copy,turtle,sys

try:
    import numpy
except:
    print "Download numpy."
    raw_input("PRESS ENTER TO EXIT PROGRAM")
    sys.exit()

try:
    import scipy.optimize
    import scipy.interpolate 
    
except:
    print "Download scipy."
    raw_input("PRESS ENTER TO EXIT PROGRAM")
    sys.exit()

#various machine coordinate sets where the effector barely touches the bed
touch_points=[(820,720,153.9),(820,1370,153.3),(1370,820,153.3),(1200,1200,153.3),(1350,1350,153.15)] 

#CALIBRATION DATA FOR THE BED HEIGHT
z_machine_actual=[(0, 0),
                  (10,8),
                  (20,18.5),
                  (30,28),
                  (40,39),
                  (50,49.2),
                  (60,59.5),
                  (70,70),
                  (80,80),
                  (90,90),
                  (100,100),
                  (110,110),
                  (120,120),
                  (130,129),
                  (140,139),
                  (150,148.6),
                  (155,154)]

#the z height where the bed arms are at 90 degrees
square_z=80.77  # USER INPUT

#LENGTH OF ARMS
l=float(150)
arm_length = l

#DISTANCE BETWEEN SHOULDERS
L=float(250)

#DISTANCE FROM BED ARM ATTACHMENT TO THE CENTER OF THE BED IN THE Y DIRECTION
#y_offset=37.5
wall_to_motor_baseline = 20.0               # motor baseline to wall difference
wall_to_bed_center_at_z_square = 132.0      # USER INPUT: wall to bed difference in Y at z_square
motor_to_bed_center = wall_to_bed_center_at_z_square - wall_to_motor_baseline # motor baseline to bed origin difference in Y at z_square

#Using "G1 X? Y?" to find the machine coordinates that make the arms colinear
straight_forearms=float(1008)    # USER INPUT

machine_z=numpy.array([i for i,j in z_machine_actual])
actual_z=numpy.array([j for i,j in z_machine_actual])
square_z=float(square_z)

#y_offset=float(y_offset)
straight_forearms=float(straight_forearms)
mechanical_advantage=(straight_forearms/200.0*math.pi+math.asin(1-L/2.0/l)+math.asin(L/4.0/l))/(math.pi-math.acos(1-L/2.0/l))




def interpolate2(v,leftLookup=True):
    try:
        x=machine_z
        y=actual_z
        if leftLookup:
            f = scipy.interpolate.interp1d(x, y)#, kind='cubic')
        else:
            f = scipy.interpolate.interp1d(y,x)
        return float(f(v))
    except:
        return 0

    """total_weight=0
    new_v=0
    table.sort()
    for m,a in table:
        if not leftLookup:
            m,a=a,m
        if m!=v:
            weight=1.0/(m-v)**2
        else:
            weight=100.0
        new_v+=weight*a
        total_weight+=weight
    return new_v/total_weight"""
#print interpolate2(0)



def machine2reference((x,y,z)):
    zprime=interpolate2(z)
    def func((i,j)):
        (x2,y2,z2)=reference2machine((i,j,0))
        return (x-x2)**2+(y-y2)**2
    xprime,yprime=scipy.optimize.fmin(func,(100,-100), xtol=0.000001, ftol=0.000001,disp=False)
    return xprime,yprime,zprime
    
def reference2machine((x,y,z)):
    try:
        zprime=interpolate2(z,False)
        initial_angle=math.acos(L/(4*l))
        left_leg=math.sqrt(x*x+y*y)
        right_leg=math.sqrt((L-x)*(L-x)+y*y)
        left_elbow=math.acos((left_leg*left_leg-2*l*l)/(-2*l*l))
        right_elbow=math.acos((right_leg*right_leg-2*l*l)/(-2*l*l))
        left_small_angle=(math.pi-left_elbow)/2
        right_small_angle=(math.pi-right_elbow)/2
        left_virtual=math.atan(-y/x)
        right_virtual=math.atan(-y/(L-x))
        left_drive=left_small_angle+left_virtual-initial_angle
        right_drive=right_small_angle+right_virtual-initial_angle
        left_stepper=-left_drive+(math.pi-left_elbow)*mechanical_advantage
        right_stepper=-right_drive+(math.pi-right_elbow)*mechanical_advantage
        return left_stepper*200/math.pi,right_stepper*200/math.pi,zprime
    except:
        return 0,0,0

def refPlane():
    ref_points=[(machine2reference(p)) for p in touch_points]
    #print ref_points
    def func((a,b,c,d)):
        v=0
        for x,y,z in ref_points:
            v+=(a*x+b*y+c*z+d)**2
        return v
    a,b,c,d=scipy.optimize.fmin(func,(1,1,1,1),disp=False)
    return a,b,c,d

print "Finding bed level from touch points.  This may take a while."
ap,bp,cp,dp=refPlane()
#print ap,bp,cp,dp

#print machine2reference((1000,1000,100))
#print reference2machine((125,125,100))
def actual2reference((gX,gY,gZ)):
    # bed_angle=math.asin((z-interpolate2(square_z))/l)
    # leg_offset=l*math.cos(bed_angle)
    # yprime=y+y_offset-leg_offset

    corrected_z_square = interpolate2(square_z)
    z_above = corrected_z_square - gZ
    z_arm_y_offset = math.sqrt(arm_length**2 - z_above**2)
    y_offset_by_z = arm_length - z_arm_y_offset # in E coords 
    eY = -gY + y_offset_by_z + motor_to_bed_center  # compensate for motor baseline

    eX=gX+L/2
    zero_z=(-dp-ap*eX-bp*eY)/cp
    #print xprime,yprime,zero_z
    eZ=zero_z-gZ
    return eX,eY,eZ
print actual2reference((0,0,0))

def reference2actual((x,y,z)):
    pass

def transform(x,y,z):
    return reference2machine(actual2reference((x,y,z)))
#print transform(0,0,0)

def testcode(x,y,z):
    a,b,c=transform(x,y,z)
    #print transform(x,y,z)
    return "G1 X"+str(a)+" Y"+str(b)+" Z"+str(c)+" F9000"
#print testcode(0,0,0)

def getABC(position1):
    global coord
    if "X" not in position1:
        return position1
    position=copy.deepcopy(position1)
    d=distance(coord,position)
    f=position["F"]
    a1,b1,c1=transform(coord["X"],coord["Y"],coord["Z"])
    a2,b2,c2=transform(position["X"],position["Y"],position["Z"])                                                     
    virtual_d=math.sqrt((a1-a2)**2+(b1-b2)**2+(c1-c2)**2)
    fnew=f*1.0
    if d!=0:
        fnew=f*virtual_d/d

    position['X']=a2
    position['Y']=b2
    position['Z']=c2
    position['F']=fnew
    coord=position1
    return position
#print testcode(0,0,0)


def distance(start, end):
    try:
        x1,y1,z1=start['X'],start['Y'],start['Z']
        x2,y2,z2=end['X'],end['Y'],end['Z']
        return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
    except:
        return 0
#print testcode(0,0,0)


def interpolate(start, end, i, n):
    x1,y1,z1,e1=start['X'],start['Y'],start['Z'],start['E']
    x2,y2,z2,e2=end['X'],end['Y'],end['Z'],end['E']
    middle={}
    for c in end:
        if c in end and c in start and c in "XYZE":
            middle[c]=(i*end[c]+(n-i)*start[c])/float(n)
        else:
            middle[c]=end[c]
    return middle


def segmentize(start,end,maxLength):
    l=distance(start,end)
    if l<=maxLength:
        return [end]
    else:
        output=[]
        n=int(math.ceil(l/maxLength))
        for i in range(1,n+1):
            output.append(interpolate(start,end,i,n))
        return output
            
    

f=file(raw_input("Input File: "))
coord={"X":0,"Y":0,"Z":0, "E":0, "F":0}
prefixes="MGXYZESF"
commands="MG"
f2=file(raw_input("Output File: "),"w")
f2.write("G92 X0 Y0 Z0 E0\n")
program=[]
move_count=0
for line in f:
    line=line.strip()
    chunks=line.split(";")[0].split(" ")
    stuff={}
    for chunk in chunks:
        if len(chunk)>1:
            stuff[chunk[0]]=chunk[1:]
            try:
                stuff[chunk[0]]=int(stuff[chunk[0]])
            except:
                try:
                    stuff[chunk[0]]=float(stuff[chunk[0]])
                except:
                    pass
        if "X" in stuff or "Y" in stuff or "Z" in stuff:
            move_count+=1
            for c in coord:
                if c not in stuff:
                    stuff[c]=coord[c]           
    if move_count<=3 and len(stuff)>0:
        program+=[stuff]
    elif len(stuff)>0:
        segments=segmentize(coord,stuff,1)
        program+=segments
    for c in coord:
        if c in stuff:
            coord[c]=stuff[c]
for i in range(len(program)):
    line=program[i]
    if i%100==0:
        print str(i*100.0/len(program))+"%"
    abcline=getABC(line)
    for letter in prefixes:
        if letter in abcline and letter in commands:
            f2.write(letter+str(abcline[letter])+" ")
        elif letter in abcline:
            f2.write(letter+str(round(abcline[letter],3))+" ")
    f2.write("\n")



f2.close()
print "done"



