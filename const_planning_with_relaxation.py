import numpy as np
import pickle
import time
#math and graphics utils
import utiltools.robotmath as rm
import pandaplotutils.pandageom as pg
from pandaplotutils import pandactrl as pandactrl
#the gripper and the robot
import manipulation.grip.robotiq85.robotiq85 as rtq85
import robotsim.ur3dual.ur3dual as robotsim
import robotsim.ur3dual.ur3dualmesh as robotmesh
#the environment, the object, and the constraints
from environment import bunrisettingfree as bsf
import loadobject as loadobj
from constraintchecker import ctchecker
#motion planning components
from motionplanning import collisioncheckerball as cdck
from motionplanning import ctcallback as ctcb
from motionplanning import smoother as sm
from motionplanning.rrt import rrtconnect as rrtc
from robotsim.ur3dual import ur3dualball as ur3dualball
#for the animation of the returned path
from direct.task.TaskManagerGlobal import taskMgr



def rot_transform(rot_mat, incline_angle):
    """
    Set the inclination angle of the robot end effector
    :param rot_mat: initial rotation matrix of the robot arm
    :param incline_angle: inclination angle about the end effector Z axis
    :return: rotation matrix of the required inclination
    """
    tmp_rot1 = base.pg.rm.rodrigues(rot_mat[:, 2], 90)
    tmp_rot1 = np.dot(tmp_rot1, rot_mat)
    tmp_rot2 = base.pg.rm.rodrigues(tmp_rot1[:, 2], incline_angle)
    return np.dot(tmp_rot2, tmp_rot1)

#constraint to consider if no relaxation
ctangle = 60

#max torque limit
friction_torque = 0.9 #Nm

if __name__ == "__main__":
    base = pandactrl.World(camp=[2700,300,2700], lookatp=[0,0,1000])
    #set the env and its collision models
    env = bsf.Env()
    env.reparentTo(base.render)
    obscmlist = env.getstationaryobslist()

    #set the hand and robot
    hndfa = rtq85.Robotiq85Factory()
    rgthnd = hndfa.genHand()
    lfthnd = hndfa.genHand()
    rbt = robotsim.Ur3DualRobot(rgthnd, lfthnd)
    rbtmg = robotmesh.Ur3DualMesh()
    #robot initial start and tranformed start for the task
    startpos = np.array([600, 400, 1550])
    startrot = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    startrot = rot_transform(startrot, 180)

    #to visualize the starting pose
    jnts = rbt.numik(startpos,startrot,armname="lft")
    rbt.movearmfk(jnts,armname="lft")
    # rbtmg.genmnp(rbt).reparentTo(base.render)

    #load the object and set its pose
    #objects are "small_3mm", "small_10mm", "big"
    obj = loadobj.Objsim(base, name= "small_10mm")
    startposobj = np.array([600 + obj.length / 2, 400, 1550])  # 1350 #500
    startrotobj = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    objcm = obj.gencm(startposobj=startposobj,startrotobj=startrotobj, startposrbt=startpos, startrotrbt=startrot, isrotated=True)
    objcm.reparentTo(base.render)
    #visualize the set object pose
    #base.run()

    #Set the relaxation on the constraints
    relax_ct = True
    if relax_ct:
        print("Constraint relaxation considered")
        incline_angs = []

        for i in range(90, -10, -1):
            incline_angs.append(i)
        # print(incline_angs)
        for angle in incline_angs:
            gravity_torque = obj.torque * np.sin(angle * np.pi / 180.0)
            print("Gravity torque at ", angle, " is ", gravity_torque)
            if gravity_torque < friction_torque:
                #the new torque-based constraint
                ctangle = angle

                print("constraint angle is: ", ctangle)
                break
            else:
                print(ctangle, " is not safe constraint limit")

    else:
        print("No constraint relaxation, constraint angle is ",ctangle)

    #start and goal joints and move the robot
    goalpos1 = np.array([600, 400, 1550])  # 1350 #500
    goalrot1 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    goalrot1 = rot_transform(goalrot1, 0)
    start_armjnts = rbt.numik(startpos, startrot, "lft")
    goal1_armjnts = rbt.numik(goalpos1, goalrot1, "lft")
    rbt.movearmfk(start_armjnts,"lft")
    rbt.movearmfk(rbt.initrgtjnts,"rgt")

    # prepare the planner
    rbtball = ur3dualball.Ur3DualBall()
    cdchecker = cdck.CollisionCheckerBall(rbtball)
    ctcallback = ctcb.CtCallback(base, rbt, cdchecker=cdchecker, ctchecker= ctchecker, ctangle=ctangle,armname="lft")
    planner = rrtc.RRTConnect(start=start_armjnts,
                              goal=goal1_armjnts,
                              ctcallback=ctcallback, starttreesamplerate=30.0, endtreesamplerate=30.0, expanddis=10,
                              maxiter=2000.0, maxtime=100.0)
    smoother = sm.Smoother()

    #calculate the object pose relative to the end effector pose
    rbtmesh = rbtmg.genmnp(rbt,jawwidthlft=obj.height)
    leepos = rbt.lftarm[-1]['linkend']
    leerot = rbt.lftarm[-1]['rotmat']
    relpos1, relrot1 = rm.relpose(leepos, leerot, startposobj, startrotobj)
    #build the tranformation of the object pose in the end effector frame
    relpos_t = np.eye(4)
    #the rotation part
    relpos_t[:3,:3] = relrot1
    relpos_t[:3,3] = relpos1
    #the translation part
    obj_ee_t = relpos_t

    #plan
    tic = time.time()
    [path, samples] = planner.planninghold([objcm],[[relpos1,relrot1]],obscmlist)
    toc = time.time()
    print("Planning time is ", toc - tic)
    tic = time.time()
    path = smoother.pathsmoothinghold(path,planner,[objcm],[[relpos1,relrot1]],100)
    toc = time.time()
    print("Smoothing time is ", toc - tic)

    #pickle the returned path
    pickle_list_in = open("angles_list.pickle", "rb")
    pickled_angles = pickle.load(pickle_list_in)
    pickled_angles.append(path)
    pickle_list_out = open("angles_list.pickle", "wb")
    pickle.dump(pickled_angles, pickle_list_out)
    pickle_list_out.close()
    print("Pickled angles length is ", len(pickled_angles))

    #animate the returned path
    ee_w_t = np.eye(4)
    obj_w_t = np.eye(4)

    def update(rbtmnp, motioncounter, rbt, path, armname, rbtmg, objcm,task):
        global obj_ee_t
        if motioncounter[0] < len(path):
            if rbtmnp[0] is not None:
                rbtmnp[0].detachNode()
            pose = path[motioncounter[0]]
            rbt.movearmfk(pose, armname)
            rbtmnp[0] = rbtmg.genmnp(rbt,0,obj.height)

            pos = rbt.lftarm[-1]['linkend']
            rot = rbt.lftarm[-1]['rotmat']
            ee_w_t[:3,:3] = rot
            ee_w_t[:3,3] = pos
            obj_w_t = np.dot(ee_w_t,obj_ee_t)
            objcm.setMat(pg.npToMat4(obj_w_t[:3,:3]))
            objcm.setPos(obj_w_t[0,3],obj_w_t[1,3],obj_w_t[2,3])
            objcm.reparentTo(base.render)
            rbtmnp[0].reparentTo(base.render)
            motioncounter[0] += 1
        else:
            motioncounter[0] = 0
        return task.again

    rbtmnp = [None]
    motioncounter = [0]
    taskMgr.doMethodLater(0.1,update,"udpdate",extraArgs=[rbtmnp, motioncounter, rbt, path, "lft",rbtmg, objcm],appendTask=True)


    base.run()
