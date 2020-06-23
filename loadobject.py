import numpy as np
import utiltools.robotmath as rm
from environment import collisionmodel as cm
import copy
import pandaplotutils.pandageom as pg

def rot_transform(rot_mat, incline_angle):
    """
    Set the inclination angle of the object according
    :param rot_mat: initial rotation matrix of the robot arm
    :param incline_angle: inclination angle about the end effector Z axis
    :return: rotation matrix of the required inclination
    """
    tmp_rot1 = rm.rodrigues(rot_mat[:, 2], 90)
    tmp_rot1 = np.dot(tmp_rot1, rot_mat)
    tmp_rot2 = rm.rodrigues(tmp_rot1[:, 2], incline_angle)
    return np.dot(tmp_rot2, tmp_rot1)

class Objsim(object):
    def __init__(self, envbase, name):
        """
        Define an object, set its collision model, and add to the simulation environment
        :param envbase: the simulation environment
        :param name: the object name used define its loading path
        """

        self.base = envbase
        self.objpath = "./objects/board_" + name + ".stl"

        if self.objpath == "./objects/board_big.stl":
            self.length = 587
            self.width = 295
            self.height = 10
            self.m = 1.8
            self.torque = 9.81 * self.m * self.length / 2 / 1000  # Nm
        elif self.objpath == "./objects/board_small_10mm.stl":
            self.length = 390
            self.width = 288
            self.height = 10
            self.m = 0.8
            self.torque = 9.81 * self.m * self.length / 2 / 1000  # Nm
        elif self.objpath == "./objects/board_small_3mm.stl":
            self.length = 397
            self.width = 280
            self.height = 3
            self.m = 0.22
            self.torque = 9.81 * self.m * self.length / 2 / 1000  # Nm
        else:
            pass

        print ("self.torque is:", self.torque)

    def gencm(self, startposobj, startrotobj, startposrbt, startrotrbt, isrotated = False, iscornered = False, isdrooped = False, isinclined = False):
        """
        make the collision model of the object, set its pose according to the robot end effector pose
        :param startposobj: object position
        :param startrotobj: object orientation
        :param startposrbt: robot position
        :param startrotrbt: robot orientation
        :param isrotated: is the object to be rotated 90 degrees about the vertical axis of the world frame
        :param iscornered:is the object grasped from its corner
        :param isdrooped: is the object drooped due to orientation
        :param isinclined: is the object at an inclined pose
        :return: collision model of the object with set pose
        """

        #load the object and set its pose related to the robot pose
        startposobj = startposobj
        startrotobj = startrotobj
        startpos = startposrbt
        startrot = startrotrbt
        #create the collision model
        self.objcm = cm.CollisionModel(objinit=self.objpath)

        # if required to flip the object and the robot pose
        # startrot = rot_transform(startrot, 180)

        # set the object pose in hand
        # easy to be automated for planning
        if isrotated:
            self.torque = 9.81 * self.m * self.width / 2 / 1000  # Nm
            print("rotated torque is set!")
            print("self.torque is:", self.torque)

            rotz = rm.rodrigues([0, 0, 1], -90)
            tmp_rot = np.dot(rotz, startrotobj)
            startrotobj = np.dot(tmp_rot, startrotobj)
            startposobj[0] -= (self.length/2-self.width/2)

        if isinclined:
            incline_angle = 10
            tmp_incline_rot = rm.rodrigues(startrot[:,2],incline_angle)
            startrot = np.dot(tmp_incline_rot,startrot)
            startrotobj = np.dot(tmp_incline_rot, startrotobj)
            objcmcopy = copy.deepcopy(self.objcm)
            objcmcopy.setMat(pg.npToMat4(startrotobj,startposobj))

            ##for visualization of the limit
            # for incangle in range(0,20,10):
            #     print (incangle)
            #     tmp_incline_rot = rm.rodrigues(startrot[:, 2], incangle)
            #     startrotobjinc = np.dot(tmp_incline_rot, startrotobj)
            #     objcmcopy = copy.deepcopy(objcm)
            #     startrotrbt = np.dot(tmp_incline_rot, startrot)
            #     rbtangles = rbt.numik(startpos, startrotrbt, "lft")
            #     rbt.movearmfk(rbtangles, "lft")
            #     rbtmg.genmnp(rbt, jawwidthlft=objheight,toggleendcoord=False).reparentTo(base.render)
            #     objcmcopy.setMat(pg.npToMat4(startrotobjinc, startposobj))
            #     if incangle == 10:
            #         objcmcopy.setColor(.8, .0, .0, .5)
            #     objcmcopy.reparentTo(base.render)
            # base.run()

        if iscornered:
            corner_angle = 45
            rotz = rm.rodrigues([0,0,1],corner_angle)
            tmp_rot = np.dot(rotz,startrotobj)
            startrotobj = np.dot(tmp_rot,startrotobj)
            grasppnt = [-self.length/2, self.width/2,0,1]
            obj_w_t = np.eye(4)
            obj_w_t[:3,:3] = startrotobj
            obj_w_t[:3,3] = startposobj
            print("obj_w_t",obj_w_t)
            grasppnt_w = np.dot(obj_w_t,grasppnt)
            print(grasppnt_w)
            print(startpos)
            pos_diff = [a-b for a,b in zip(grasppnt_w,startpos)]
            print ("Position difference is: ", pos_diff)
            print("Position difference is: ", [grasppnt_w[0],grasppnt_w[1],grasppnt_w[2]]-startpos)
            startposobj-=pos_diff

        if isdrooped == True:
            droop_angle = -20
            tmp_rot_ee_x = rm.rodrigues(startrot[:,0],droop_angle)
            startrotobj = np.dot(tmp_rot_ee_x,startrotobj)
            # grasppnt = [-objlength/2*np.cos(droop_angle), objwidth/2*np.sin(droop_angle), 0, 1]
            # obj_w_t = np.eye(4)
            # obj_w_t[:3, :3] = startrotobj
            # obj_w_t[:3, 3] = startposobj
            # grasppnt_w = np.dot(obj_w_t, grasppnt)
            # print(grasppnt_w)
            # print(startpos)
            # pos_diff = [a - b for a, b in zip(grasppnt_w, startpos)]
            # print("Position difference is: ", pos_diff)
            # print("Position difference is: ", [grasppnt_w[0], grasppnt_w[1], grasppnt_w[2]] - startpos)
            # startposobj += pos_diff

        self.objcm.setMat(pg.npToMat4(startrotobj,startposobj))
        return self.objcm