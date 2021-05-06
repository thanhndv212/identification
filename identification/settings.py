from pinocchio.robot_wrapper import RobotWrapper
from os.path import dirname, join, abspath
from sys import argv
import numpy as np

def loadModels(robotname, robot_urdf_file, isFext=False):
    """This function create a robot model and its data by inputting a URDF file that describes the robot.
            Input:  robotname: directory containing model of robot
                    robot_urdf_file: URDF file of robot
            Output: robot: robot model and data created by Pinocchio"""

    pinocchio_model_dir = join(
        dirname(dirname(str(abspath(__file__)))), "models")
    model_path = join(pinocchio_model_dir, "others/robots")
    mesh_dir = model_path
    urdf_filename = robot_urdf_file
    urdf_dir = robotname + "/urdf"
    urdf_model_path = join(join(model_path, urdf_dir), urdf_filename)
    if not isFext:
        robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)
    else:
        robot = RobotWrapper.BuildFromURDF(
            urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())
    return robot

isFext = False
isActuator_int = True
isFrictionincld = True
isOffset = True
isCoupling = True
if len(argv) > 1:
    if argv[1] == '-f':
        isFrictionincld = True

#example TX40
fv = np.array([8.05e0, 5.53e0, 1.97e0, 1.11e0, 1.86e0, 6.5e-1])
fs = np.array([7.14e0, 8.26e0, 6.34e0, 2.48e0, 3.03e0, 2.82e-1])
Ia = np.array([3.62e-1, 3.62e-1, 9.88e-2, 3.13e-2, 4.68e-2, 1.05e-2])
off = np.array([3.92e-1, 1.37e0, 3.26e-1, -1.02e-1, -2.88e-2, 1.27e-1])
Iam6 = 9.64e-3
fvm6 = 6.16e-1
fsm6 = 1.95e0
#reduction ratio
N1 = 32
N2 = 32
N3 = 45
N4 = -48
N5 = 45
N6 = 32
#velocity limits 
qd_lim = 0.01 * np.array([287, 287, 430, 410, 320, 700]) * np.pi / 180
# load robot
robot = loadModels("staubli_tx40_description", "tx40_mdh_modified.urdf")
# robot = loadModels("2DOF_description", "2DOF_description.urdf")
# robot = loadModels("SC_3DOF", "3DOF.urdf")

#essential ratio
ratio_essential = 30