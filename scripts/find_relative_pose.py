import rosbag
import re
import cv2
import pyopengv
import numpy as np
from scipy.spatial.transform import Rotation
from skimage.util import img_as_float
from skimage.color import rgb2gray
from skimage.feature import (match_descriptors, corner_harris, corner_peaks, ORB, plot_matches)
import matplotlib.pyplot as plt

rotate_images = True

def main():
    # camera matrices, counters, etc defined at bottom of file
    global num_left_im
    global num_right_im
    global recent_left_im
    global recent_right_im
    global left_cam_mat
    global right_cam_mat
    for topic, msg, t in rosbag.Bag('tressider-2019-04-26_0.bag').read_messages():
	# This also replaces tf timestamps under the assumption
	# that all transforms in the message share the same timestamp
	if re.search("ros_indigosdk_node/image0/compressed", topic) != None:
	    recent_left_im = format_left_image(msg)
	    num_left_im = num_left_im + 1
	    if(num_right_im == num_left_im):
		compute_pose_and_print()
	elif re.search("ros_indigosdk_node/image1/compressed", topic) != None: 
	    recent_right_im = format_right_image(msg)
	    num_right_im = num_right_im + 1
	    if(num_right_im == num_left_im):
		compute_pose_and_print()

def format_left_image(img_msg):
    return format_image_for_orb(img_msg, left_cam_mat, left_dist_coeffs)
def format_right_image(img_msg):
    return format_image_for_orb(img_msg, right_cam_mat, right_dist_coeffs)
def format_image_for_orb(img_msg, cam_mat, dist_coeffs):
    np_arr = np.fromstring(img_msg.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv_img_undistorted = cv2.undistort(cv_img, cam_mat, dist_coeffs)
    if(rotate_images):
	cv_img_undistorted = cv2.rotate(cv_img_undistorted, cv2.ROTATE_90_COUNTERCLOCKWISE)
    sk_img = img_as_float(cv_img_undistorted)
    grey_sk_img = rgb2gray(sk_img)
    return grey_sk_img

def compute_pose_and_print():
 #   if(num_right_im%100==1):
#	R, t = find_relative_pose(recent_left_im, recent_right_im, True)
 #   else:
#	return
    R, t = find_relative_pose(recent_left_im, recent_right_im, False)
    if(num_right_im == 1):
	print "x, y, z, qx, qy, qz, qw"
    print_r_t_csv(R, t)

def find_relative_pose(imL,imR,show_matches=True):
    keypoints1, keypoints2, matches12 = generate_keypoint_matches(imL, imR)  
    pts1=keypoints1[matches12[:,0],:]
    pts2=keypoints2[matches12[:,1],:]
    if show_matches:
        fig, ax = plt.subplots(nrows=1, ncols=1)
        plot_matches(ax, imL, imR, keypoints1, keypoints2, matches12)    
	plt.show()   
    # R, t, mask = find_relative_pose_cv_pixel(pts1, pts2)
    R, t, mask = find_relative_pose_cv_proj(pts1, pts2)
    # R, t, mask = find_relative_pose_gv_proj(pts1, pts2)
    if show_matches:
	inlier_E_matches = matches12[np.append(mask, mask, 1)==1].reshape(np.count_nonzero(mask), 2)
        fig, ax = plt.subplots(nrows=1, ncols=1)
        plot_matches(ax, imL, imR, keypoints1, keypoints2, inlier_E_matches)    
	plt.show()   
    return R, t

def generate_keypoint_matches(imL, imR):
    descriptor_extractor = ORB(n_keypoints=2000)    
    descriptor_extractor.detect_and_extract(imL)
    keypoints1 = descriptor_extractor.keypoints
    descriptors1 = descriptor_extractor.descriptors        
    descriptor_extractor.detect_and_extract(imR)
    keypoints2 = descriptor_extractor.keypoints
    descriptors2 = descriptor_extractor.descriptors            
    matches12 = match_descriptors(descriptors1, descriptors2,metric='hamming', cross_check=True) 
    return keypoints1, keypoints2, matches12

def find_relative_pose_cv_pixel(pts1, pts2):
    if(not rotate_images):
	focal = average_focal
	pp = average_pp
    else:
	focal = average_focal_rot
	pp = average_pp_rot
    E, mask = cv2.findEssentialMat(pts1, pts2, focal=focal, pp=pp, method=cv2.RANSAC, prob=0.999, threshold=1)
    num_E_inliers = np.count_nonzero(mask)
    mask_2d = np.append(mask, mask, 1)
    inlier_E_pts1 = pts1[mask_2d==1].reshape(num_E_inliers, 2)
    inlier_E_pts2 = pts2[mask_2d==1].reshape(num_E_inliers, 2)
    points, R, t, pose_mask = cv2.recoverPose(E, inlier_E_pts1, inlier_E_pts2)
    return R, t, mask

def find_relative_pose_cv_proj(pts1, pts2):
    projection_pts1, projection_pts2 = to_projection_pts(pts1, pts2)
    E, mask = cv2.findEssentialMat(projection_pts1, projection_pts2, focal=1, pp=(0,0), method=cv2.RANSAC, prob=0.999, threshold=0.002)
    num_E_inliers = np.count_nonzero(mask)
    mask_2d = np.append(mask, mask, 1)
    inlier_E_pts1 = pts1[mask_2d==1].reshape(num_E_inliers, 2)
    inlier_E_pts2 = pts2[mask_2d==1].reshape(num_E_inliers, 2)
    points, R, t, pose_mask = cv2.recoverPose(E, inlier_E_pts1, inlier_E_pts2)
    return R, t, mask
    
def find_relative_pose_gv_proj(pts1, pts2):
    projection_pts1, projection_pts2 = to_projection_pts(pts1, pts2)
    bearing_vecs1, bearing_vecs2 = to_bearing_vectors(projection_pts1, projection_pts2)
    ransac_transformation = pyopengv.relative_pose_ransac(
        bearing_vecs1, bearing_vecs2, "NISTER", 0.005, 1000, 0.999)
    R = ransac_transformation[:, :3]
    t = ransac_transformation[:, 3]
    mask = None
    return R, t, mask

def to_projection_pts(left_pxl_points, right_pxl_points):
    homog_pts1 = np.ones([left_pxl_points.shape[0], 3])
    homog_pts1[:,:-1] = left_pxl_points
    homog_pts2 = np.ones([right_pxl_points.shape[0], 3])
    homog_pts2[:,:-1] = right_pxl_points
    if(not rotate_images):
	inv_left_cam_mat = np.linalg.inv(left_cam_mat)
	inv_right_cam_mat = np.linalg.inv(right_cam_mat)
    else:
	inv_left_cam_mat = np.linalg.inv(left_rot_cam_mat)
	inv_right_cam_mat = np.linalg.inv(right_rot_cam_mat)
    projection_homog_pts1 = np.transpose(np.matmul(inv_left_cam_mat, np.transpose(homog_pts1)))
    projection_homog_pts2 = np.transpose(np.matmul(inv_right_cam_mat, np.transpose(homog_pts2)))
    projection_homog_pts1 = homogenize_homog_pts(projection_homog_pts1)
    projection_homog_pts2 = homogenize_homog_pts(projection_homog_pts2)
    projection_pts1 = projection_homog_pts1[:,:-1]
    projection_pts2 = projection_homog_pts2[:,:-1] 
    return projection_pts1, projection_pts2

def to_match_ptpairs(matches):
    match_ptpairs = matches.copy()
    match_ptpairs[:,0] = range(match_ptpairs.shape[0])
    match_ptpairs = match_ptpairs[np.argsort(match_ptpairs[:, 1])]
    match_ptpairs[:,1] = range(match_ptpairs.shape[0])
    match_ptpairs = match_ptpairs[np.argsort(match_ptpairs[:, 0])]
    return match_ptpairs

def homogenize_homog_pts(homog_pts):
    # print "to homogenize: {}".format(homog_pts)
    homogenizer = np.reciprocal(homog_pts[:,2])
    # print "homogenizer: {}".format(homogenizer)
    homogenized = (homog_pts.T * homogenizer).T
    # print "homogenized: {}".format(homogenized)
    return homogenized

def to_bearing_vectors(left_proj_points, right_proj_points):
    homog_pts1 = np.ones([left_proj_points.shape[0], 3])
    homog_pts1[:,:-1] = left_proj_points
    homog_pts2 = np.ones([right_proj_points.shape[0], 3])
    homog_pts2[:,:-1] = right_proj_points
    bearing_vecs1 = normalize_homog_pts(homog_pts1)
    bearing_vecs2 = normalize_homog_pts(homog_pts2)
    return bearing_vecs1, bearing_vecs2

def normalize_homog_pts(homog_pts):
    # print "to normalize: {}".format(homog_pts)
    x_sq = np.multiply(homog_pts[:,0], homog_pts[:,0])
    y_sq = np.multiply(homog_pts[:,1], homog_pts[:,1])
    z_sq = np.multiply(homog_pts[:,2], homog_pts[:,2])
    normalizer = np.reciprocal(np.sqrt(np.add(x_sq, np.add(y_sq, z_sq))))
    # print "normalizer: {}".format(normalizer)
    normalized = (homog_pts.T * normalizer).T
    # print "normalized: {}".format(normalized)
    return normalized
    

def print_r_t_pretty(R,t):
    print "R: {}".format(Rotation.from_dcm(R).as_quat())
    print "t: {}, {}, {}".format(t[0][0], t[1][0], t[2][0])
def print_quat_angle_mag(R):
    quat = Rotation.from_dcm(R).as_quat()
    w = quat[3]
    angle = np.degrees(2*np.arccos(w))
    print angle
def print_r_t_csv(R, t):
    quat = Rotation.from_dcm(R).as_quat()
    print "{}, {}, {}, {}, {}, {}, {}".format(t[0], t[1], t[2], quat[0], quat[1], quat[2], quat[3])
def print_t_csv(t):
    print "{}, {}, {}".format(t[0][0], t[1][0], t[2][0])
def print_r_quat_csv(R):
    quat = Rotation.from_dcm(R).as_quat()
    print "{}, {}, {}, {}".format(quat[0], quat[1], quat[2], quat[3])
    


# Begin global

num_left_im = 0
num_right_im = 0
recent_left_im = None
recent_right_im = None
left_cam_mat = np.array([[476.7099914550781, 0.0, 350.73760986328125], [0.0, 479.5046081542969, 209.53211975097656], [0.0, 0.0, 1.0]])
left_dist_coeffs = np.array([-0.33659109473228455, 0.1597423255443573, 0.00012696981139015406, -7.225573062896729e-05, -0.04619533196091652])
right_cam_mat = np.array([[483.2540283203125, 0.0, 365.3299560546875], [0.0, 485.7801513671875, 210.9530792236328], [0.0, 0.0, 1.0]])
right_dist_coeffs = np.array([-0.3350730240345001, 0.15195880830287933, -0.000232060527196154, 0.0003201397485099733, -0.03968252241611481])
cam_width = 720
left_rot_cam_mat = np.array([[left_cam_mat[1,1], 0.0, left_cam_mat[1,2]], [0.0, left_cam_mat[0,0], cam_width-left_cam_mat[0,2]], [0.0, 0.0, 1.0]])
right_rot_cam_mat = np.array([[right_cam_mat[1,1], 0.0, right_cam_mat[1,2]], [0.0, right_cam_mat[0,0], cam_width-right_cam_mat[0,2]], [0.0, 0.0, 1.0]])
average_focal = (left_cam_mat[0,0] + left_cam_mat[1,1] + right_cam_mat[0,0] + right_cam_mat[1,1])/4
average_pp = ((left_cam_mat[0, 2]+right_cam_mat[0, 2])/2, (left_cam_mat[1, 2]+right_cam_mat[1, 2])/2)
average_focal_rot = (left_rot_cam_mat[0,0] + left_rot_cam_mat[1,1] + right_rot_cam_mat[0,0] + right_rot_cam_mat[1,1])/4
average_pp_rot = ((left_rot_cam_mat[0, 2]+right_rot_cam_mat[0, 2])/2, (left_rot_cam_mat[1, 2]+right_rot_cam_mat[1, 2])/2)

main()




