from ur_ikfast import ur_kinematics


def ik_fast(joint_angles):
    ur5_arm = ur_kinematics.URKinematics('ur5')
    
    pose_quat = ur5_arm.forward(joint_angles)
    pose_matrix = ur5_arm.forward(joint_angles, 'matrix')
    # print("forward() quaternion \n", pose_quat)
    # print("forward() matrix \n", pose_matrix)
    
    # print("inverse() one from quat", ur5_arm.inverse(pose_quat, False, q_guess=joint_angles))

    # print("inverse() one from matrix", ur5_arm.inverse(pose_matrix, False, q_guess=joint_angles))
    
    joints_ik = ur5_arm.inverse(pose_matrix, False, q_guess=joint_angles)
    
    return joints_ik