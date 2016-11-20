import os
import dlib
import sys
from skimage import io
import xml.etree.ElementTree as ET

# function to retrieve 68 landmarks from an input image
def get_facial_landmarks(file_name, predictor_path):
	detector = dlib.get_frontal_face_detector()
	predictor = dlib.shape_predictor(predictor_path)
	img = io.imread(file_name)
	
	# get bounding boxes
	dets = detector(img, 1)
	
	if len(dets) > 0:
		# Use first face only
		first_face = dets[0]
	else:
		raise ValueError('No faces detected in the image!')
	
	landmarks = predictor(img, first_face)
	
	landmarks_in_2d = []
	for i in range(landmarks.num_parts):
		landmarks_in_2d.append(str(landmarks.part(i)))

	return landmarks_in_2d
	
# Function to save landmarks to an output file
def save_landmarks_to_file(landmarks, output_file_name):
	with open(output_file_name, 'wt') as fp:
		for elem in landmarks:
			fp.write(elem)
			fp.write('\n')
			
# function to read saved 3d points from an xml out
def read_3d_points(file_name):
	try:
		tree = ET.parse(file_name)
	except:
		raise ValueError('something bad happened when opening the file.')
	root = tree.getroot()
	
	points3d = []
	for point in root.iter('point'):
		x = point.get('x')
		y = point.get('y')
		z = point.get('z')
	
		points3d.append([x, y, z])

	return points3d

# save extracted 3d points to file
def save_3d_points_file(points3d, outfile):
	with open(outfile, 'wb') as fp:
		for point in points3d:
			fp.write(point[0])
			fp.write('\n')
			fp.write(point[1])
			fp.write('\n')
			fp.write(point[2])
			fp.write('\n')


def save_2d_gt_to_file(in_file, out_file):
	with open(in_file, 'rb') as fp:
		version = fp.readline()
		num_points = fp.readline()
		temp = fp.readline()
		correspondences_2d = []
		for i in range(68):
			corres = fp.readline()
			correspondences_2d.append(corres.split('\n')[0])
		with open(out_file, 'wt') as op:
			for corres in correspondences_2d:
				op.write(corres)
				op.write('\n')

		
	

