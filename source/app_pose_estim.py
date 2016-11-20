from pose_estimation_utils import *
import sys


def main():
	if len(sys.argv) < 2:
		raise ValueError('app_pose_estim file_name')
	else:
		file_name_2d = sys.argv[1]
		out_file_2d = sys.argv[2]
		file_name_3d = sys.argv[3]
		out_file_3d = sys.argv[4]
		
		save_2d_gt_to_file(file_name_2d, out_file_2d)
		save_3d_points_file(read_3d_points(file_name_3d), out_file_3d)
		

if __name__ == '__main__':
	main()	
