import numpy as np
from scipy.spatial.transform import Rotation

def righthtobase_from_leftvtorightv(leftv_to_rightv_rot):
    print "input rightv to leftv rotation:\n{}".format(rightv_to_leftv_rot.as_dcm())
    rightv_to_leftv_rot = leftv_to_rightv_rot.inv()
    righth_to_base_rot = leftv_to_base_rot * rightv_to_leftv_rot * h_to_v_rot
    print "output righth to base rotation:\n{}".format(righth_to_base_rot.as_dcm())
    return righth_to_base_rot

def leftvtorightv_from_righthtobase(righth_to_base_rot):
    print "input righth to base rotation:\n{}".format(righth_to_base_rot.as_dcm())
    rightv_to_leftv_rot = leftv_to_base_rot.inv() * righth_to_base_rot * h_to_v_rot.inv()
    leftv_to_rightv_rot = rightv_to_leftv_rot.inv()
    print "output leftv to rightv rotation:\n{}".format(leftv_to_rightv_rot.as_dcm())
    return leftv_to_rightv_rot

def leftvtorightv_from_two_rel_rots(leftv_to_base_rot, rightv_to_base_rot):
    print "input leftv_to_base:\n{}\ninput rightv_to_base:\n{}".format(leftv_to_base_rot.as_dcm(), rightv_to_base_rot.as_dcm())
    leftv_to_rightv_rot = rightv_to_base_rot.inv() * leftv_to_base_rot
    print "output leftv to rightv rotation:\n{}".format(leftv_to_rightv_rot.as_dcm())
    return leftv_to_rightv_rot

# Definitions
h_to_v_mat_arr = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
h_to_v_rot = Rotation.from_dcm(h_to_v_mat_arr)
leftv_to_base_mat_arr = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
leftv_to_base_rot = Rotation.from_dcm(leftv_to_base_mat_arr)

# Inputs
leftv_to_rightv_quat_array = [-0.00150226, -0.03612349, 0.01413419, 0.99924625]
rightv_to_leftv_rot = Rotation.from_quat(leftv_to_rightv_quat_array).inv()

successful_righth_to_base_mat_arr = [[-4.82722098e-02,  2.00100946e-02,  9.98633762e-01], [1.08046955e-03,  9.99799771e-01, -1.99812305e-02], [-9.98833633e-01,  1.14455222e-04, -4.82841647e-02]]
successful_righth_to_base_rot = Rotation.from_dcm(successful_righth_to_base_mat_arr)

successful_leftv_to_rightv_rot = leftvtorightv_from_righthtobase(successful_righth_to_base_rot)
print "good leftv_to_rightv quat: {}".format(successful_leftv_to_rightv_rot.as_quat())

calib_leftv_to_base_rot = Rotation.from_dcm([[0.999994, 0.000654539, 0.00340293], [-0.000654519, 1, -6.81963e-06], [-0.00340293, 4.59231e-06, 0.999994]])
calib_rightv_to_base_rot = Rotation.from_dcm([[0.999995, 0.00282205, 0.00163291], [-0.00282345, 0.999996, 0.000852931], [-0.00163049, -0.000857537, 0.999998]])
calib_leftv_to_rightv_rot = leftvtorightv_from_two_rel_rots(calib_leftv_to_base_rot, calib_rightv_to_base_rot)
print "questionable leftv_to_rightv quat: {}".format(calib_leftv_to_rightv_rot.as_quat())


exit()

imu_to_base_quat_arr = [9.38186291074e-07, 0.707107250279, 0.707106312093, 9.38185046294e-07]
base_to_imu_rot = Rotation.from_quat(imu_to_base_quat_arr).inv()
print "base_to_imu rotation:\n{}".format(base_to_imu_rot.as_dcm())

left_to_base_mat_arr = [ [0, 0, 1], [0, 1, 0], [-1, 0, 0]]
left_to_base_rot = Rotation.from_dcm(left_to_base_mat_arr)
left_to_right_quat_arr = [-0.000298833293867, -0.024147343869568, 0.010001248333751, 0.999658337390568]
right_to_left_rot = Rotation.from_quat(left_to_right_quat_arr).inv()

right_to_base_rot = left_to_base_rot * right_to_left_rot 
right_to_base_mat = right_to_base_rot.as_dcm()
print "old right_to_base rotation:\n{}".format(right_to_base_mat)

left_to_imu_rot = base_to_imu_rot * left_to_base_rot
left_to_imu_mat = left_to_imu_rot.as_dcm()
print "left_to_imu rotation:\n{}".format(left_to_imu_mat)

right_to_imu_rot = left_to_imu_rot * right_to_left_rot
right_to_imu_mat = right_to_imu_rot.as_dcm()
print "right_to_imu rotation:\n{}".format(right_to_imu_mat)
