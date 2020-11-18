from geometry_msgs.msg import Pose
import xml.etree.ElementTree as ET
import pyquaternion as pq
import numpy as np
import os
from xml.dom import minidom
from xml.dom.minidom import Node

def remove_blanks(node):
    for x in node.childNodes:
        if x.nodeType == Node.TEXT_NODE:
            if x.nodeValue:
                x.nodeValue = x.nodeValue.strip()
        elif x.nodeType == Node.ELEMENT_NODE:
            remove_blanks(x)

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

        domstring = minidom.parseString(ET.tostring(grasp_data))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())

    def save_edit_result(self):

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

        # Look for existing file
        # If file exists, parse the tree
        # If it doesn't, create the tree
        # XML structure:
        #   grasp_data
        #       ManipulationObject
        #           GraspSet
        #               Grasp
        #                   Transform
        #                       Matrix4x4
        #                           rows
        #       Graspable
        #       Grasped
        #           Grasp
        #       GraspStability
        #           Grasp
        #       ObstacleAvoidance
        #           Grasp

        filename = os.path.join(os.path.abspath(self._savepath), "grasp_" + self._object_name + ".xml")

        current_grasp_idx = 0
        tree = None
        grasp_data = None
        manip_object_field = None
        graspset_field = None
        grasped_field = None
        stability_field = None
        avoidance_field = None

        if os.path.isfile(filename):

            tree = ET.parse(filename)
            grasp_data = tree.getroot()
            manip_object_field = grasp_data.find('ManipulationObject')
            graspset_field = manip_object_field.find('GraspSet')
            current_grasp_idx = len(list(graspset_field))

            # grasp_field = ET.SubElement(graspset_field, 'Grasp')
            # grasp_field.set('name', 'Grasp '+ str(current_grasp_idx))
            # grasp_field.set('quality', '0')
            # grasp_field.set('Creation', 'auto')
            # grasp_field.set('Preshape', 'Grasp Preshape')

            # transform_field = ET.SubElement(grasp_field, 'Transform')
            # matrix = ET.SubElement(transform_field, 'Matrix4x4')
            # row1 = ET.SubElement(matrix, 'row1')
            # row2 = ET.SubElement(matrix, 'row2')
            # row3 = ET.SubElement(matrix, 'row3')
            # row4 = ET.SubElement(matrix, 'row4')

            # for col_idx in range(4):
            #     row1.set('c'+str(col_idx+1), str(grasp_T_board[0,col_idx]))
            #     row2.set('c'+str(col_idx+1), str(grasp_T_board[1,col_idx]))
            #     row3.set('c'+str(col_idx+1), str(grasp_T_board[2,col_idx]))
            #     row4.set('c'+str(col_idx+1), str(grasp_T_board[3,col_idx]))

            # # graspable_field = ET.SubElement(grasp_data, 'Graspable')
            # # graspable_field.set('quality', str(1))

            grasped_field = grasp_data.find('Grasped')
            # grasped_field_entry = ET.SubElement(grasped_field, 'Grasp')
            # grasped_field_entry.set('name', 'Grasp ' + str(current_grasp_idx))
            # grasped_field_entry.set('quality', str(1) if self._grasped_score else str(0))

            stability_field = grasp_data.find('GraspStability')
            # stability_field_entry = ET.SubElement(stability_field, 'Grasp')
            # stability_field_entry.set('name', 'Grasp '+ str(current_grasp_idx))
            # stability_field_entry.set('quality', str(self._stability_score))

            avoidance_field = grasp_data.find('ObstacleAvoidance')
            # avoidance_field_entry = ET.SubElement(avoidance_field, 'Grasp')
            # avoidance_field_entry.set('name', 'Grasp '+ str(current_grasp_idx))
            # avoidance_field_entry.set('quality', str(0))

        else:

            grasp_data = ET.Element('grasp_data')

            manip_object_field = ET.SubElement(grasp_data, 'ManipulationObject')
            manip_object_field.set('name', self._object_name)

            visu_field = ET.SubElement(manip_object_field, 'Visualization')
            visu_filename_field = ET.SubElement(visu_field, 'File')
            visu_filename_field.set('type', 'inventor')
            visu_filename_field.text = os.path.join('../../../GRASPA-benchmark/data/objects/YCB', self._object_name, 'nontextured.stl')

            collision_field = ET.SubElement(manip_object_field, 'CollisionModel')
            collision_filename_field = ET.SubElement(collision_field, 'File')
            collision_filename_field.set('type', 'inventor')
            collision_filename_field.text = os.path.join('../../../GRASPA-benchmark/data/objects/YCB', self._object_name, 'nontextured.stl')

            graspset_field = ET.SubElement(manip_object_field, 'GraspSet')
            graspset_field.set('name', 'Benchmark_Layout_X')
            graspset_field.set('RobotType', 'Panda')
            graspset_field.set('EndEffector', 'panda_hand')

            # grasp_field = ET.SubElement(graspset_field, 'Grasp')
            # grasp_field.set('name', 'Grasp '+ str(0))
            # grasp_field.set('quality', '0')
            # grasp_field.set('Creation', 'auto')
            # grasp_field.set('Preshape', 'Grasp Preshape')

            # transform_field = ET.SubElement(grasp_field, 'Transform')
            # matrix = ET.SubElement(transform_field, 'Matrix4x4')
            # row1 = ET.SubElement(matrix, 'row1')
            # row2 = ET.SubElement(matrix, 'row2')
            # row3 = ET.SubElement(matrix, 'row3')
            # row4 = ET.SubElement(matrix, 'row4')

            # for col_idx in range(4):
            #     row1.set('c'+str(col_idx+1), str(grasp_T_board[0,col_idx]))
            #     row2.set('c'+str(col_idx+1), str(grasp_T_board[1,col_idx]))
            #     row3.set('c'+str(col_idx+1), str(grasp_T_board[2,col_idx]))
            #     row4.set('c'+str(col_idx+1), str(grasp_T_board[3,col_idx]))

            graspable_field = ET.SubElement(grasp_data, 'Graspable')
            graspable_field.set('quality', str(1))

            grasped_field = ET.SubElement(grasp_data, 'Grasped')
            # grasped_field_entry = ET.SubElement(grasped_field, 'Grasp')
            # grasped_field_entry.set('name', 'Grasp ' + str(current_grasp_idx))
            # grasped_field_entry.set('quality', str(1) if self._grasped_score else str(0))

            stability_field = ET.SubElement(grasp_data, 'GraspStability')
            # stability_field_entry = ET.SubElement(stability_field, 'Grasp')
            # stability_field_entry.set('name', 'Grasp '+ str(current_grasp_idx))
            # stability_field_entry.set('quality', str(self._stability_score))

            avoidance_field = ET.SubElement(grasp_data, 'ObstacleAvoidance')
            # avoidance_field_entry = ET.SubElement(avoidance_field, 'Grasp')
            # avoidance_field_entry.set('name', 'Grasp '+ str(current_grasp_idx))
            # avoidance_field_entry.set('quality', str(0))

        grasp_field = ET.SubElement(graspset_field, 'Grasp')
        grasp_field.set('name', 'Grasp '+ str(current_grasp_idx))
        grasp_field.set('quality', '0')
        grasp_field.set('Creation', 'auto')
        grasp_field.set('Preshape', 'Grasp Preshape')

        transform_field = ET.SubElement(grasp_field, 'Transform')
        matrix = ET.SubElement(transform_field, 'Matrix4x4')
        row1 = ET.SubElement(matrix, 'row1')
        row2 = ET.SubElement(matrix, 'row2')
        row3 = ET.SubElement(matrix, 'row3')
        row4 = ET.SubElement(matrix, 'row4')

        # Multiplication for 1K because Simox uses millimiters as length units...

        for col_idx in range(4):
            row1.set('c'+str(col_idx+1), str(grasp_T_board[0,col_idx] * 1000.0))
            row2.set('c'+str(col_idx+1), str(grasp_T_board[1,col_idx] * 1000.0))
            row3.set('c'+str(col_idx+1), str(grasp_T_board[2,col_idx] * 1000.0))
            row4.set('c'+str(col_idx+1), str(grasp_T_board[3,col_idx]))

        grasped_field = grasp_data.find('Grasped')
        grasped_field_entry = ET.SubElement(grasped_field, 'Grasp')
        grasped_field_entry.set('name', 'Grasp ' + str(current_grasp_idx))
        grasped_field_entry.set('quality', str(1) if self._grasped_score else str(0))

        stability_field = grasp_data.find('GraspStability')
        stability_field_entry = ET.SubElement(stability_field, 'Grasp')
        stability_field_entry.set('name', 'Grasp '+ str(current_grasp_idx))
        stability_field_entry.set('quality', str(self._stability_score))

        avoidance_field = grasp_data.find('ObstacleAvoidance')
        avoidance_field_entry = ET.SubElement(avoidance_field, 'Grasp')
        avoidance_field_entry.set('name', 'Grasp '+ str(current_grasp_idx))
        avoidance_field_entry.set('quality', str(0))

        domstring = minidom.parseString(ET.tostring(grasp_data))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())

        # Remove whitespaces

        xml = minidom.parse(filename)
        remove_blanks(xml)
        xml.normalize()
        with file(filename, 'w') as handle:
            handle.write(xml.toprettyxml(indent = '  '))




