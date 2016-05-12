"""
Implements inverse and forward kinematics functions, in addition to other movement calculation functions for the Dobot Arm
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below):
1.
License: MIT


"""





import math




"""

To truly understand the inverse kinematics algorithm,it is strongly advised that one watches these videos.
polar coordinates conversion: https://www.youtube.com/watch?v=L4v98ZZft68
IK 2D, 2DOF, revolute revolute: https://www.youtube.com/watch?v=cvzv3YxuoQE

Also, see the readme on inverse kinematics in the github repository.

IMPORTANT: default working angle units are radians


A person on reddit suggested that I was performing too many float calculations in this file, which was leading to systematic error.
He provided a link to some simpler equations to fix this in his code (open-dobot is the github repository).
I still need to look into this and verify his claim. To the best of my knowledge, the math is correct.


# i think i correctly implemented the forward kinematics, but I don't remember whether or not I thoroughly tested it.
# Need to go back and double check this.

algorithm (my original idea. the actual code is a bit different from this I think. HAVE TO REVISE THIS COMMENT

xy plane is paralell to surface dobot is on. z is perpendicular
1. first get distance, in xy plane, to current point from origin using forward kinematics, known angles, and pythagoreas thm. This is your radius. Your angle can be defined to be theta original. You now have your starting point in polar coordinates.
2. Ignore the desired z position data for now. Get the polar coordinates for the desired point. The radius is not important now. The angle is though. Subtracting the desired angle from the current angle gives you the number of degrees and direction to rotate the base.
3. The radius from step 1 (starting radius) gives you your current horizontal position (imagine it as x or y, doesn't matter). You also already know your current z position (potentially from step 1).
4. The radius from step 2 (desired radius) gives you your desired horizontal position (imagine it as x or y, doesn't matter). Of course, the user has already input the desired z position. This is now a 2D problem with two arms that each rotate (2 degrees of freedom)
5: use IK, see ik 2d video, to find number of degrees and direction to rotate upper and lower arms. Note that there are often two solutions. One (elbow down) is not possible.
6. Check that move is valid (e.g. not out of range, etc...)
7. move
"""
#length and height dimensions in mm
#these are the lengths from the arm angles. Note, I need to look at a diagram posted by the redditor.
#The height from base is not the same has height from surface dobot is sitting on.
lengthUpperArm = 135
lengthLowerArm = 160
heightFromBase = 80

#input:
#cartesian (x,y,z) coordinate
#robot dimensions
#output:
#angles for robot arm base, upper, and lower arms in degree
def convert_cartesian_coordinate_to_arm_angles(x, y, z, upperArmLength, lowerArmLength, baseHeight):

    #do a general check to see if even a maximally stretched arm could reach the point
    # if it can't, return some dummy angle numbers of -999
    #note the base height correction in z
    distanceFromOriginToEndPoint = get_distance_from_origin_to_cartesian_point_3D(x,y,z-baseHeight)
    #print(str(distanceFromOriginToEndPoint))
    if (distanceFromOriginToEndPoint > (upperArmLength + lowerArmLength)):
        return [-999,-999,-999]

    baseAngle = get_polar_coordinate_angle_from_cartesian_x_y_coordinate(x,y)
    radiusToEndPoint = get_polar_coordinate_radius_from_cartesian_x_y_coordinate(x,y)
    #note the correction for base height
    armAngles = get_arm_angles_from_radius_z_coordinate_using_2d_revolute_revolute_inverse_kinematics(radiusToEndPoint, z-baseHeight, upperArmLength, lowerArmLength)
    upperArmAngle = armAngles[0]
    lowerArmAngle = armAngles[1]

    #convert the angles to degrees when you return them
    return [baseAngle * (180/math.pi), upperArmAngle * (180/math.pi), lowerArmAngle * (180/math.pi)]



def get_arm_angles_from_radius_z_coordinate_using_2d_revolute_revolute_inverse_kinematics(r, z, upperArmLength, lowerArmLength):

    #eq is a dummy variable for the equation that it represents
    eq = ( pow(upperArmLength,2) + pow(lowerArmLength,2) - pow(r,2) - pow(z,2) ) / ( 2 * upperArmLength * lowerArmLength )

    #the two angles are due to the two possible angles, elbow up or down. I always want elbow up.
    #Not sure which one is elbow up, guessing it's the positive sqrt equation for now. CHECK THIS!
    #note that pi radians = 180 degrees
    lowerArmAngle = math.pi - math.atan2( math.sqrt(1 - pow(eq,2)), eq )
    #including the angle two
    lowerArmAngleAlternative = math.pi - math.atan2( -1 * math.sqrt(1 - pow(eq,2)), eq )
    lowerArmAngle = lowerArmAngleAlternative

    #note that the upperArmAngle is dependent on the lowerArmAngle. This makes sense.
    # can easily envision that upper arm would be at two different angles if arm is elbow up or down
    upperArmAngle = math.atan2(z,r) - math.atan2( (lowerArmLength * math.sin(lowerArmAngle)) , (upperArmLength + (lowerArmLength * math.cos(lowerArmAngle))) )

    return [upperArmAngle, lowerArmAngle]



#input:
#x,y coordinate
#output
#polar coordinate angle to x,y point in radians
def get_polar_coordinate_angle_from_cartesian_x_y_coordinate(x,y):
    #use cartesian to polar conversion equations to get the angle
    #alternatively, this is just using the toa rule
    angle = math.atan2(y,x)
    return angle

#input:
#x,y coordinate
#output
#length of polar coordinate radius to x,y point
def get_polar_coordinate_radius_from_cartesian_x_y_coordinate(x,y):
    #use cartesian to polar conversion equations to get the angle
    #alternatively, this is just using the circle equation rule
    radius = math.sqrt(pow(x,2) + pow(y,2))
    return radius

def get_distance_from_origin_to_cartesian_point_3D(x,y,z):
    #get distance from origin (0,0,0) to end point in 3D using pythagorean thm in 3D; distance = sqrt(x^2+y^2+z^2)
    distanceToEndPoint = math.sqrt( pow(x,2) + pow(y,2) + pow(z,2) )
    #print(distanceToEndPoint)
    return distanceToEndPoint



#forward kinematics
def get_cartesian_coordinate_from_angles_using_forward_kinematics(baseAngle, upperArmAngle, lowerArmAngle):
    #convert degrees to radians
    baseAngle = (baseAngle/180) * math.pi
    upperArmAngle = (upperArmAngle/180) * math.pi
    lowerArmAngle = (lowerArmAngle/180) * math.pi

    radius = get_radius_in_horizontal_plane_to_end_effector_position(upperArmAngle,lowerArmAngle)
    x = radius * math.cos(baseAngle)
    y = radius * math.sin(baseAngle)
    z = heightFromBase + (math.sin(upperArmAngle) * lengthUpperArm)  + (math.sin(lowerArmAngle) * lengthLowerArm)
    return [x,y,z]



def get_radius_in_horizontal_plane_to_end_effector_position(upperArmAngle, lowerArmAngle):
    print(upperArmAngle)
    print(lowerArmAngle)
    radius = (math.cos(upperArmAngle) * lengthUpperArm) + (math.cos(lowerArmAngle) * lengthLowerArm)
    print(radius)
    return radius





#these functions aren't used/implemented yet. might need them in the future. just leaving them here as a reminder.

def get_upper_arm_angle():
    #return the angle in degrees or radians for the upper arm from the accelerometer data and/or known theoretical angle
    return 45#or radians!

def get_lower_arm_angle():
    #return the angle in degrees or radians for the lower arm from the accelerometer data and/or known theoretical angle
    return 45#or radians!

def get_base_angle():
    #return the angle in degrees or radians for the base from the accelerometer data and/or known theoretical angle
    return 45#or radians!





#an old command line interface function I used to debug the InverseKinematics code. No longer used.

def command_line_test_inverse_kinematics():



    # length and height dimensions in mm
    lengthUpperArm = 135
    lengthLowerArm = 160
    heightFromBase = 80

    commandFlag = True
    while(commandFlag):
        x = input('Enter x: ')
        y = input('Enter y: ')
        z = input('Enter z: ')
        angles = convert_cartesian_coordinate_to_arm_angles(float(x),float(y),float(z),lengthUpperArm,lengthLowerArm,heightFromBase)

        if(angles[0] != -999):
            print('\nFor the point (' + str(x) + ' , ' + str(y) + ' , ' + str(z) + ') , the angles are:' )
            print('Base Angle: ' + str(angles[0]) )
            print('Upper Arm Angle: ' + str(angles[1]) )
            print('Lower Arm Angle: ' + str(angles[2]))
            print('\n')
        else:
            print('Invalid coordinate: Out of arm\'s range.')
            print('\n')

        s = input('quit?')
        if (s == 'y'):
            commandFlag = False
        print('\n')

