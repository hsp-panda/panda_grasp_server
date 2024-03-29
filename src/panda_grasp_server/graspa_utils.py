from geometry_msgs.msg import Pose, PoseStamped
import xml.etree.ElementTree as ET
import pyquaternion as pq
import numpy as np
import os
from xml.dom import minidom
from xml.dom.minidom import Node
from rospy_message_converter import message_converter
import rospy
import tf

def remove_blanks(node):
    """Recursive function to explore the XML tree and purge whitespaces in nodes.

    Parameters
    ----------
    node : xml.dom.minidom.Document or xml.dom.minidom.Node
        The DOM tree to parse or a terminal node
    """

    for x in node.childNodes:
        if x.nodeType == Node.TEXT_NODE:
            if x.nodeValue:
                x.nodeValue = x.nodeValue.strip()
        elif x.nodeType == Node.ELEMENT_NODE:
            remove_blanks(x)


def get_GRASPA_board_pose(tf_listener, base_frame = 'panda_link0', board_frame = 'graspa_board'):

    # Get the GRASPA board pose and return it as a PoseStamped

    # Check if the board and base tf frames are available

    if not (tf_listener.frameExists(base_frame) and tf_listener.frameExists(board_frame)):
        rospy.logerr("Either tf transform {} or {} do not exist".format(base_frame, board_frame))
        return None

    tf_listener.waitForTransform(base_frame, board_frame, rospy.Time.now(), rospy.Duration(5.0))
    last_heard_time = tf_listener.getLatestCommonTime(base_frame, board_frame)

    # Get the GRASPA board reference frame pose
    pose_board = PoseStamped()
    pose_board.header.frame_id = board_frame
    # pose_board.header.stamp = last_heard_time
    pose_board.pose.orientation.w = 1.0
    pose_board = tf_listener.transformPose(base_frame, pose_board)

    rospy.loginfo("Acquired GRASPA board pose:")
    rospy.loginfo(pose_board)

    return pose_board


def save_GRASPA_grasp(grasp_save_path, grasp_pose, graspa_board_pose=None):

        # save the pose according to the graspa standard

        grasp_result = GRASPAResult()

        # Get object name, layout, and algorithm
        current_object = rospy.get_param("~ops_params/current_obj", 'any')
        current_layout = rospy.get_param("~ops_params/current_layout", 'any')
        current_alg    = rospy.get_param("~ops_params/current_algorithm", 'any')

        if (raw_input("Current layout: {}. Change layout? [y/N]".format(current_layout)).lower() == 'y') or current_layout == 'any':
            current_layout = raw_input("Enter layout number: ")
            rospy.set_param("~ops_params/current_layout", current_layout)

        if (raw_input("Current algorithm: {}. Change algorithm? [y/N]".format(current_alg)).lower() == 'y') or current_alg == 'any':
            current_alg = raw_input("Enter algorithm name: ")
            rospy.set_param("~ops_params/current_algorithm", current_alg)

        if (raw_input("Current object: {}. Change object? [y/N]".format(current_object)).lower() == 'y') or current_object == 'any':
            current_object = raw_input("Enter object name: ")
            rospy.set_param("~ops_params/current_obj", current_object)

        # Check if path exists, otherwise create it
        full_graspa_save_path = os.path.join(grasp_save_path, current_alg, "grasps_data", "layout_" + str(current_layout))
        if not (os.path.isdir(full_graspa_save_path)):
            rospy.loginfo("Creating directory {}".format(full_graspa_save_path))
            os.makedirs(full_graspa_save_path)
        else:
            rospy.loginfo("Directory {} already exists, hence I'll use that".format(full_graspa_save_path))

        # Get scores
        grasped_score = -1.0
        while (grasped_score < 0.0) or (grasped_score > 1.0):
            try:
                grasped_score = float(raw_input("Object grasped? [0.0, 1.0]"))
            except ValueError:
                grasped_score = -1.0
        stab_score = -1.0
        while (stab_score < 0.0) or (stab_score > 1.0):
            try:
                stab_score = float(raw_input("Enter stability score [0.0, 1.0]"))
            except ValueError:
                stab_score = -1.0

        # Get the board pose
        # We should have it already though
        if not graspa_board_pose:
            graspa_board_pose = get_GRASPA_board_pose()

        # Fill in results and save
        grasp_result.set_savepath(full_graspa_save_path)
        grasp_result.set_poses(graspa_board_pose.pose, grasp_pose.pose if isinstance(grasp_pose, PoseStamped) else grasp_pose)
        grasp_result.set_obj_name(current_object)
        grasp_result.set_layout_name(current_layout)
        grasp_result.set_algorithm_name(current_alg)
        grasp_result.set_stability_score(stab_score)
        grasp_result.set_grasped_score(grasped_score)

        try:
            grasp_result.save_result()
            rospy.loginfo("Grasp info saved!")
            return True

        except Exception as exc:
            rospy.logerr("Error saving grasp results: {}".format(exc.message))
            return False


class GRASPAResult(object):
    """Class to create and save GRASPA-compatible XML files
    """

    def __init__(self):

        self._board_pose = Pose()
        self._board_pose.orientation.x = 1.0

        self._grasp_pose = Pose()
        self._grasp_pose.orientation.x = 1.0

        self._savepath = "."
        self._object_name = None
        self._layout = None
        self._algorithm = None

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

    def set_layout_name(self, layout_name):

        self._layout = str(layout_name)

    def set_algorithm_name(self, alg_name):

        self._algorithm = str(alg_name)

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

        # Filenames in GRASPA are something like YcbObjectName_grasp.xml

        name_graspa_savefile = "Ycb" + "".join([string.capitalize() for string in self._object_name.split('_')])
        filename = os.path.join(os.path.abspath(self._savepath), name_graspa_savefile + "_grasp.xml")

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

            grasped_field = grasp_data.find('Grasped')

            stability_field = grasp_data.find('GraspStability')

            avoidance_field = grasp_data.find('ObstacleAvoidance')

        else:

            grasp_data = ET.Element('grasp_data')

            manip_object_field = ET.SubElement(grasp_data, 'ManipulationObject')
            manip_object_field.set('name', self._object_name)

            visu_field = ET.SubElement(manip_object_field, 'Visualization')
            visu_filename_field = ET.SubElement(visu_field, 'File')
            visu_filename_field.set('type', 'inventor')
            visu_filename_field.text = os.path.join('data/objects/YCB', self._object_name, 'nontextured.stl')

            collision_field = ET.SubElement(manip_object_field, 'CollisionModel')
            collision_filename_field = ET.SubElement(collision_field, 'File')
            collision_filename_field.set('type', 'inventor')
            collision_filename_field.text = os.path.join('data/objects/YCB', self._object_name, 'nontextured.stl')

            graspset_field = ET.SubElement(manip_object_field, 'GraspSet')
            graspset_field.set('name', 'Benchmark_Layout_' + str(self._layout))
            graspset_field.set('RobotType', 'Panda')
            graspset_field.set('EndEffector', 'Panda Hand')

            graspable_field = ET.SubElement(grasp_data, 'Graspable')
            graspable_field.set('quality', str(1))

            grasped_field = ET.SubElement(grasp_data, 'Grasped')

            stability_field = ET.SubElement(grasp_data, 'GraspStability')

            avoidance_field = ET.SubElement(grasp_data, 'ObstacleAvoidance')

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

        # Multiplication for 1K only in column 3 because Simox uses millimiters as length units...

        for col_idx in range(4):
            if col_idx == 3:
                row1.set('c'+str(col_idx+1), str(grasp_T_board[0,col_idx] * 1000.0))
                row2.set('c'+str(col_idx+1), str(grasp_T_board[1,col_idx] * 1000.0))
                row3.set('c'+str(col_idx+1), str(grasp_T_board[2,col_idx] * 1000.0))
            else:
                row1.set('c'+str(col_idx+1), str(grasp_T_board[0,col_idx]))
                row2.set('c'+str(col_idx+1), str(grasp_T_board[1,col_idx]))
                row3.set('c'+str(col_idx+1), str(grasp_T_board[2,col_idx]))
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

        # Save dictionary in ros-friendly language

        filename_json = os.path.join(os.path.abspath(self._savepath), "grasp_" + self._object_name + "_{:03d}.json".format(current_grasp_idx))

        grasp_dict = {
                    'object' : self._object_name,
                    'GRASPA_board_pose' : message_converter.convert_ros_message_to_dictionary(self._board_pose),
                    'grasp_pose' : message_converter.convert_ros_message_to_dictionary(self._grasp_pose),
                    'grasped_score' : str(1) if self._grasped_score else str(0),
                    'stability_score' : self._stability_score
        }

        import json
        with open(filename_json, 'w') as handle:
            json_string = json.dumps(grasp_dict)
            handle.write(json_string)
