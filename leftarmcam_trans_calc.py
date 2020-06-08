import numpy as np
import manipulation.grip.robotiq85.rtq85nm as rtq85nm
import robotsim.ur3dual.ur3dual as ur3dualsim
import robotsim.ur3dual.ur3dualmesh as ur3dualsimmesh
import robotcon.ur3dual as ur3urx
import cv2
import cv2.aruco as aruco
if __name__=='__main__':

    #load the left arm camera parameters
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv_file = cv2.FileStorage("aruco/calib_images/left_arm_camera/left_arm_camera_calib__.yaml", cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode("camera_matrix").mat()
    dist = cv_file.getNode("dist_coeff").mat()

    #The robot simulation model
    robot = ur3dualsim.Ur3DualRobot()
    rgthnd = rtq85nm.newHand(hndid = "rgt")
    lfthnd = rtq85nm.newHand(hndid = "lft")
    robotmesh = ur3dualsimmesh.Ur3DualMesh(rgthand = rgthnd, lfthand = lfthnd)

    #move the robot simulation model to its initial pose
    robot.goinitpose()
    #the joints of the left arm calibration pose
    jntlft_calib = [29.359462279367676, -163.2380687300504, -50.661251584499425, 175.0638136354098, -147.0334995657085, 192.5160534554379]
    #move the robot simulation model left arm to the calibration pose
    robot.movearmfk(jntlft_calib, "lft")

    #the real robot
    ur3u = ur3urx.Ur3DualUrx()
    # move the real robot right arm to its initial pose
    ur3u.movejntssgl(robot.initrgtjnts, armid="rgt")
    #move the real robot left arm to the clibration pose
    ur3u.movejntssgl(jntlft_calib, armid='lft')


    #the marker is held in the robot arm right end effector (because a known pose is required for the calculation of the required transformation)
    #the transformation for the right arm (calculated from from the forward kinematics) defines the T_MW transformation from the marker to the World frame
    R_MW = robot.rgtarm[-1]['rotmat']
    P_MW = robot.rgtarm[-1]['linkend']
    #build the trnasformation matrix for the right arm
    T_MW = np.eye(4)
    T_MW[:3,:3] = R_MW
    T_MW[:3,3] = P_MW

    print ("T_MW from the right arm is: {}".format(repr(T_MW)))

    #the transformation for the left arm T_LFTW is the transformation from the left EE to the world frame
    R_LFTW = robot.lftarm[-1]['rotmat']
    P_LFTW = robot.lftarm[-1]['linkend']
    #build the tranformation matrix for the left arm
    T_LFTW = np.eye(4)
    T_LFTW[:3,:3] = R_LFTW
    T_LFTW[:3,3] = P_LFTW

    print("T_LFTW from the left arm is: {}".format(repr(T_LFTW)))


    #prepare for the pose detection
    cap = cv2.VideoCapture(0)

    #to save the detected pose
    det_T_MC = np.eye(4)

    while (True):
        detected = False

        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        #detect markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        #to save the returned result of the detection
        rvec = []
        tvec = []

        if np.all(ids != None):
            #pose estimation
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.025, mtx, dist) #the second value is the marker size in meters

            for i in range(ids.shape[0]):
                aruco.drawAxis(frame, mtx, dist, rvec[i, 0], tvec[i, 0], 0.1)  # Draw Axis

                if ids[i]== 5:
                    #detected marker orientation in axis-angle rotation representation
                    det_rvec = rvec[i]
                    #*important* convert the length units meter (from opencv) to mm (for the planner world frame)
                    det_tvec = 1000*tvec[i]
                    #for visualization
                    aruco.drawDetectedMarkers(frame, corners, ids)  # Draw A square around the markers
                    cv2.putText(frame, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    # convert rotation from axis-angle to rotation matrix
                    det_rmat, _ = cv2.Rodrigues(det_rvec)
                    # build the transformation from the detected pose to the camera T_CM transformation from the marker to the camera
                    det_T_MC[:3, :3] = det_rmat
                    det_T_MC[:3, 3] = det_tvec[0]
                    print ("Detected T_MC = {}".format(repr(det_T_MC)))
                    detected = True
                    #detection is done, break for and while loops and calculate the result
                    break
            if detected:
                break
        ##uncomment to visualize the detection, and comment breaks
        # cv2.imshow('frame', frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    #transformation from the camera frame to the world frame
    T_CW = np.dot(T_MW, np.linalg.inv(det_T_MC))
    #transformation from the left camera frame to the left EE frame
    T_LFTCEE = np.dot(np.linalg.inv(T_LFTW), T_CW)

    print ("Transformation from left camera to EE T_CLFTEE = ", repr(T_LFTCEE))
