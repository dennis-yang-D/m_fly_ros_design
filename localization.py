import numpy as np
import cv2



def triangulate_two_rays(pt1, pt2, K1, K2, R1=np.eye(3), t1=np.zeros((3,1)), R2=np.eye(3), t2=np.zeros((3,1)), dist1=None, dist2=None):
    """
    Backproject a pixel px=(u,v) to a 3D point on a horizontal plane (z=plane_z) in world coordinates.
    
    Arguments:
      px: (u,v) pixel coordinates (tuple or 1x2 array)
      K: 3x3 camera intrinsic matrix
      dist: distortion coeffs (None if none)
      plane_z: z coordinate of the plane in WORLD frame (meters)
      cam_pose_R, cam_pose_t: camera-to-world rotation (3x3) and translation (3x1).
                           (i.e., world_point = R_cam_to_world @ cam_point + t_cam_to_world)

    Returns:
      final (3,) 3D point in world coordinates lying on plane_z (if intersection exists)
    """
    RT1=np.hstack([R1,t1]) #Combine matrices sideways
    RT2=np.hstack([R2,t2]) 
    P1= K1 @ RT1 #@ symbol is matrix multiplication
    P2= K2 @ RT2 #P is the projection matrix

    # Format points as 2×N arrays for triangulatePoints
    pts1 = np.array(pt1, dtype=np.float64).reshape(2, 1)
    pts2 = np.array(pt2, dtype=np.float64).reshape(2, 1)

    #triangulation returns homogenous points for some reason
    h = cv2.triangulatePoints(P1,P2,pts1, pts2)
    final = h[:3] / h[3] #:3 selects the first 3 elements and divides it by the 4th [3] element to unhomogenous the point
    return final