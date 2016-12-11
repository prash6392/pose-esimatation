def read_camera_params_file(file_name):
    camera_params = []
    with open(file_name, 'rb') as fp:
        camera_params.append(fp.readlines())
    camera_params = np.asarray(camera_params[0], dtype='float32')
    return camera_params
    
camera_file = './testout.txt'
camera_parameters = read_camera_params_file(camera_file)
print('Estimated camera parameters:')
print(camera_parameters)

# Transforming camera matrix
rel_trans = [1, 3, 3]

new_camera_params = transform_camera_params(camera_parameters, rel_trans, [2])
print('Modified camera:')
print(new_camera_params)
print(camera_parameters)

def params_to_camera_matrix(params):
    rotation_matrix = np.zeros((3, 4), dtype='float32')
    translation_matrix = np.zeros((4, 4), dtype='float32')
    intrinsic_matrix = np.zeros((3, 3), dtype='float32')
    # Set diagonal elements to one
    translation_matrix[0][0] = 1
    translation_matrix[1][1] = 1
    translation_matrix[2][2] = 1
    translation_matrix[3][3] = 1
    # Set translation matrix values to corresponding values from params
    translation_matrix[0][3] = params[9]
    translation_matrix[1][3] = params[10]
    translation_matrix[2][3] = params[11]
    
    # Fill rotation matrix
    for i in range(3):
        for j in range(3):
            rotation_matrix[i][j] = params[3*i+j]
    rotation_matrix[2][3] = 1.0
    
    # Fill intrinsic_matrix
    intrinsic_matrix[0][0] = params[12]
    intrinsic_matrix[1][1] = params[12]
    intrinsic_matrix[2][2] = 1.0
    intrinsic_matrix[0][2] = params[13]
    intrinsic_matrix[1][2] = params[14]
    
    return rotation_matrix, translation_matrix, intrinsic_matrix


# convert params to matrices
rot_mat, t_mat, i_mat = params_to_camera_matrix(camera_parameters)
print(rot_mat)
print(t_mat)
print(i_mat)
print(np.dot(rot_mat, t_mat))


def homo_3d_to_image_cordinates(homo_3d_cor):
    image_cordinates = np.zeros((2, 1))
    # Invert sign because of camera direction
    image_cordinates[0, 0] = -(homo_3d_cor[0]/homo_3d_cor[2])
    image_cordinates[1, 0] = -(homo_3d_cor[1]/homo_3d_cor[2])
    return image_cordinates

def camera_transform(point, rot_mat, trans_mat, int_mat):
    rp = np.dot(rot_mat, point)
#     rp = np.dot(trans_mat, rp)
    rp[0] += trans_mat[0][3]
    rp[1] += trans_mat[1][3]
    rp[2] += trans_mat[2][3]

#     rp = homo_3d_to_image_cordinates(rp)
#     rp[0, 0] = rp[0, 0]*int_mat[0, 0]
#     rp[1, 0] = rp[1, 0]*int_mat[1, 1]
    rp = np.dot(int_mat, rp)
    return rp

points_3d_file = '../data/3D-models/68-landmark-points'
points_3d = read_3d_points(points_3d_file)
homogenous_4d = np.ones((4, 1))
ro_dup = np.zeros((3, 4), dtype='float32')
ro_dup[0][0] = 1.0
ro_dup[1][1] = 1.0
ro_dup[2][2] = 1.0
ro_dup[2][3] = 1.0
image_xy = []
for i in range(len(points_3d)):
    homogenous_4d[0] = points_3d[i][0]
    homogenous_4d[1] = points_3d[i][1]
    homogenous_4d[2] = points_3d[i][2]
#     print(homogenous_4d)
    result = camera_transform(homogenous_4d, rot_mat, t_mat, i_mat)
#     print(result)
    image_xy.append(homo_3d_to_image_cordinates(result))

image_xy = np.array(image_xy)
image_xy = np.reshape(image_xy, (-1, 2))
# print(image_xy.shape)
print(image_xy)

