import numpy as np
import cv2
import cv2.aruco as aruco
import utils.robotmath as rm

def getposearmcam(eepos, eerotmat, armname = "rgt", markerid = 6):
    """
    Calcluate the pose of the object when detected from the end effector camera
    :param eepos: right end effector position
    :param eerotmat: right end effector orientation
    :return: the transformation matrix of the detected object in the world frame
    """
    #build the tranformation matrix of the end effector pose in the world frame
    T_EEW = np.eye(4)
    T_EEW[:3, :3] = eerotmat
    T_EEW[:3, 3] = eepos


    if armname == "rgt":
        #set the calculated transformation between the right arm camera and the right end effector
        T_CEE = np.array([[-9.99442764e-01, -2.18179570e-02, 2.52614411e-02,
                              -4.21361377e+00],
                             [2.07905605e-02, -9.98973601e-01, -4.02428632e-02,
                              -2.62700248e+01],
                             [2.61135184e-02, -3.96952281e-02, 9.98870549e-01,
                              -8.44215037e+01],
                             [0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                              1.00000000e+00]])
        #load the right arm camera parameters
        cv_file = cv2.FileStorage("aruco/calib_images/right_arm_camera/right_arm_camera_calib__.yaml",
                                  cv2.FILE_STORAGE_READ)
        mtx = cv_file.getNode("camera_matrix").mat()
        dist = cv_file.getNode("dist_coeff").mat()
    else:
        #set the calculated transformation between the left arm camera and the left end effector
        T_CEE = np.array([[9.98987167e-01, 3.75007165e-02, -2.48664049e-02,
                              1.09009752e+01],
                             [-3.71338501e-02, 9.99196892e-01, 1.50548575e-02,
                              3.98462236e+01],
                             [2.54110028e-02, -1.41162236e-02, 9.99577417e-01,
                              -8.19180008e+01],
                             [0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                              1.00000000e+00]])

        #load the left arm camera parameters
        cv_file = cv2.FileStorage("aruco/calib_images/left_arm_camera/left_arm_camera_calib__.yaml",
                                  cv2.FILE_STORAGE_READ)
        mtx = cv_file.getNode("camera_matrix").mat()
        dist = cv_file.getNode("dist_coeff").mat()


    #calculate the transformation from the camera to the world frame
    T_CW_a = np.dot(T_EEW, T_CEE)
    #to store the transformation from the object to the camera frame
    det_T_MC = np.eye(4)
    #connect to the camera
    cap = cv2.VideoCapture(0)
    marker_id = markerid

    while (True):
        detected = False
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        #make detection
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if np.all(ids != None):
            #make the pose estimation
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.025, mtx, dist)
            for i in range(ids.shape[0]):
                if ids[i] == marker_id:
                    det_rvec = rvec[i]
                    #*important* convert the m (from opencv) to mm (for the planner world frame)
                    det_tvec = 1000 * tvec[i]
                    #convert rotation from axis-angle to rotation matrix
                    det_rmat, _ = cv2.Rodrigues(det_rvec)
                    #build the transformation from the detected marker pose to the camera
                    det_T_MC[:3, :3] = det_rmat
                    det_T_MC[:3, 3] = det_tvec[0]
                    #calculate the marker pose in the world frame
                    T_MW_a = np.dot(T_CW_a, det_T_MC)
                    detected = True
                    break
            if detected:
                break

            final_T = T_MW_a

            #uncoment for cube detection
            #T_MO means the transformation from the marker to the object, and different ids are for different markers on different faces of a cube
            #all the following transformation map to the same thing to be able to detect the pose of the cube (center) in the world  from any side of it
            # if markerid == 3:
            #     T_MO = np.eye(4)
            #     T_MO[:3,:3] = np.dot(rm.rodrigues([1.0,0.0,0.0],-90),rm.rodrigues([0.0,0.0,1.0],90))
            #     #calculate the rotation part
            #     final_T = np.dot(T_MW_a,T_MO)
            #     #then, adust the translation part
            #     final_T[:3,3] = T_MW_a[:3,3] - T_MW_a[:3,1] * 30
            #
            # elif markerid == 4:
            #     T_MO = np.eye(4)
            #     T_MO[:3, :3] = rm.rodrigues([1.0, 0.0, 0.0], -270)
            #     final_T = np.dot(T_MW_a,np.linalg.inv(T_MO))
            #     final_T[:3,3] =  T_MW_a[:3, 3] - T_MW_a[:3,2] *30 - T_MW_a[:3,1] *25
            #
            # elif markerid == 5:
            #     T_MO = np.eye(4)
            #     T_MO[:3, :3] = rm.rodrigues([0.0, 0.0, 1.0], 90.0)
            #     temp = np.ones((3,))
            #     temp[:] = T_MW_a[:3, 3]
            #     T_MW_a[:3, 3] = np.zeros((3,))
            #     final_T = np.dot(T_MW_a, np.linalg.inv(T_MO))
            #     final_T[:3, 3] = temp - T_MW_a[:3, 2] * 50.0
            #
            # elif markerid == 6:
            #     T_MO = np.eye(4)
            #     T_MO[:3,:3] = np.dot(rm.rodrigues([0.0,0.0,1.0],90), rm.rodrigues([1.0,0.0,0.0],180))
            #     final_T = np.dot(T_MW_a,np.linalg.inv(T_MO))
            #
            # else:
            #     #just return the marker transformation without any further transformations
            #     final_T = T_MW_a

    return final_T
