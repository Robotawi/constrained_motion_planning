import numpy as np
import utiltools.robotmath as rm

def ctchecker(ctangle, robot, armname, jnts, objcmlist):

    """
    Check the violation of the constraint
    :param ctangle: constraint angle
    :param robot: robot object
    :param armname: robot arm to move in forward and inverse kinematics
    :param jnts: joints at the current sample to define the end effector pose
    :param objcmlist: object collision model for definition of the constraint
    :return: True if violated, False if not
    """
    ''' Check if the constrained is verified'''

    # move the robot to get the end effector pose
    initjnts = robot.initrgtjnts
    if armname == 'lft':
        initjnts = robot.initlftjnts
    robot.movearmfk(jnts, armname)

    eepos = robot.lftarm[-1]['linkend']
    eerot = robot.lftarm[-1]['rotmat']
    robot.movearmfk(initjnts, armname)

    # the normal to the grasp axis
    ee_zaxis = eerot[:, 1]

    # if the object is to be used, future
    objcm = objcmlist[0]
    objmat = objcm.getMat()

    # check for the constraint satisfaction
    angle = abs(rm.degree_betweenvector(ee_zaxis, np.array([0, 0, 1])))
    if not (angle >= 90 - ctangle and angle <= 90 + ctangle):
        # print("The angle between eeY and worldZ is out of range")
        # print(angle)
        return True
    else:
        # print("The angle between eeY and worldZ is safe range between", 90 - ctangle, " and", 90 + ctangle)
        #print(angle)
        return False
