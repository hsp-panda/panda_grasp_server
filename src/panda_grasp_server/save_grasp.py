from geometry_msgs.msg import Pose
import xml.etree.ElementTree as ET
import pyquaternion as pq
import numpy as np
import os
import xml.dom.minidom

class GRASPAResult(object):

    def __init__(self):

        self._board_pose = Pose()
        self._board_pose.orientation.x = 1.0

        self._grasp_pose = Pose()
        self._grasp_pose.orientation.x = 1.0

        self._savepath = "."
        self._object_name = "none"

        self._stability_score = 0.0
        self._grasped_score = False

    def set_poses(self, board_pose, grasp_pose):

        self._board_pose = board_pose
        self._grasp_pose = grasp_pose

    def set_stability_score(self, stability_score):

        self._stability_score = float(stability_score)

    def set_grasped_score(self, grasped_score):

        self._grasped_score = bool(grasped_score)

    def set_savepath(self, savepath):

        self._savepath = str(savepath)

    def set_obj_name(self, obj_name):

        self._object_name = str(obj_name)

    def save_result(self):

        board_rotation = pq.Quaternion(self._board_pose.orientation.w,
                                       self._board_pose.orientation.x,
                                       self._board_pose.orientation.y,
                                       self._board_pose.orientation.z)

        board_position = np.array([self._board_pose.position.x,
                                   self._board_pose.position.y,
                                   self._board_pose.position.z])

        board_T_matrix = board_rotation.transformation_matrix
        board_T_matrix[:3, 3] = board_position

        grasp_rotation = pq.Quaternion(self._grasp_pose.orientation.w,
                                      self._grasp_pose.orientation.x,
                                      self._grasp_pose.orientation.y,
                                      self._grasp_pose.orientation.z)

        grasp_position = np.array([self._grasp_pose.position.x,
                                   self._grasp_pose.position.y,
                                   self._grasp_pose.position.z])

        grasp_T_matrix = grasp_rotation.transformation_matrix
        grasp_T_matrix[:3, 3] = grasp_position

        # Compute the grasping pose in the board ref frame

        grasp_T_board = np.dot(np.linalg.inv(board_T_matrix), grasp_T_matrix)

        # Create the file structure

        grasp_data = ET.Element('grasp_data')

        object_field = ET.SubElement(grasp_data, 'ManipulationObject')
        object_field.set('name', self._object_name)

        matrix = ET.SubElement(object_field, 'Matrix4x4')
        row1 = ET.SubElement(matrix, 'row1')
        row2 = ET.SubElement(matrix, 'row2')
        row3 = ET.SubElement(matrix, 'row3')
        row4 = ET.SubElement(matrix, 'row4')

        for col_idx in range(4):
            row1.set('c'+str(col_idx+1), str(grasp_T_board[0,col_idx]))
            row2.set('c'+str(col_idx+1), str(grasp_T_board[1,col_idx]))
            row3.set('c'+str(col_idx+1), str(grasp_T_board[2,col_idx]))
            row4.set('c'+str(col_idx+1), str(grasp_T_board[3,col_idx]))

        grasped_field = ET.SubElement(grasp_data, 'Grasped')
        grasped_field_entry = ET.SubElement(grasped_field, 'Grasp')
        grasped_field_entry.set('name', 'Grasp 0')
        grasped_field_entry.set('quality', str(1) if self._grasped_score else str(0))

        stability_field = ET.SubElement(grasp_data, 'GraspStability')
        stability_field_entry = ET.SubElement(stability_field, 'Grasp')
        stability_field_entry.set('name', 'Grasp 0')
        stability_field_entry.set('quality', str(self._stability_score))

        # Build filename

        filename = "."
        for grasp_idx in range(1000):
            filename = os.path.join(os.path.abspath(self._savepath), "grasp_" + self._object_name + "_{:03d}.xml".format(grasp_idx))
            if not os.path.isfile(filename): break

        domstring = xml.dom.minidom.parseString(ET.tostring(grasp_data))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())

gr = GRASPAResult()
gr._board_pose.orientation.x = 1.0
gr._grasp_pose.orientation.x = 1.0
gr.save_result()



