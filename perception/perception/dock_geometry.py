"""Calibrated tag36h11 pose observations from real RGB pixels; no oracle landmarks."""
import cv2
import numpy as np


def detect_tags(gray,k,distortion,size,max_reprojection):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters=cv2.aruco.DetectorParameters_create();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary,parameters=parameters)
    if ids is None:return []
    half=size/2;object_points=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
    observations=[]
    for ident,box in zip(ids.ravel(),corners):
        if ident not in range(8):continue
        ok,rvec,tvec=cv2.solvePnP(object_points,box.reshape(4,2),k,distortion,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or tvec[2,0]<=0:continue
        projected,_=cv2.projectPoints(object_points,rvec,tvec,k,distortion)
        error=float(np.sqrt(np.mean((projected.reshape(4,2)-box.reshape(4,2))**2)))
        if error>max_reprojection:continue
        observations.append((int(ident),tvec.ravel(),cv2.Rodrigues(rvec)[0],error))
    return observations
