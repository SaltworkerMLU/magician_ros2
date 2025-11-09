from math import radians, degrees, cos, sin, atan2, sqrt, acos


def dobot_FK(theta1, theta2, theta3, theta4=0):
    """
    Calculates forward kinematics of the Dobot Magician
    """
    theta = [radians(theta1), radians(theta2), radians(theta3), radians(theta4)]
    a = [0, 0, 135, 147, 0]
    x = cos(theta[0]) * (a[2] * sin(theta[1]) + a[3] * cos(theta[2]))
    y = sin(theta[0]) * (a[2] * sin(theta[1]) + a[3] * cos(theta[2]))
    z = a[2] * cos(theta[1]) - a[3] * sin(theta[2])
    r = theta[0] + theta[3]

    return [x, y, z, r]

def dobot_IK(x, y, z, r=0):
    """
    Calculates inverse kinematics of the Dobot Magician
    """
    try:
        theta = [0.0, 0.0, 0.0, 0.0]
        a = [0, 0, 135, 147, 0]

        theta[0] = atan2(y,x)

        theta[3] = radians( r - degrees(theta[0]) ) # r is presumed to be in degrees

        xy = sqrt( pow(x,2) + pow(y,2) )
        xyz = sqrt( pow(x,2) + pow(y,2) + pow(z,2))

        phi_1 = acos( xy / xyz )
        phi_2 = acos( ( pow(xyz,2) + pow(a[2],2) - pow(a[3],2) ) / ( 2 * xyz * a[2] ) )

        theta[1] = radians(90) - phi_1 - phi_2

        phi_3 = acos( (-pow(xyz,2) + pow(a[2],2) + pow(a[3],2) ) / ( 2 * a[2] * a[3] ) )

        theta[2] = radians(90) - phi_3 + theta[1]

        return [ degrees(theta[0]), degrees(theta[1]), degrees(theta[2]), degrees(theta[3])] # in Dobot convention
    except:
        return False

def execution_time(position, target, v, a):
    """
    Calculates the time for either a MoveJ or MoveL to complete
    """
    tb = v / a
    tf_v = tb + max(target - position) / v
    tf_a = sqrt(abs(4 * (target - position) / a))

    if 2 * tb > tf_a:
        tf = tf_a
    else:
        tf = tf_v

    return tf