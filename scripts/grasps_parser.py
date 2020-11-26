import xml.etree.ElementTree as et
import os
import argparse

def parse_object_file(filename):

    if not os.path.isfile(filename):

        print("File {} does not exist".format(filename))
        return -1

    tree = et.parse(filename)
    grasp_data = tree.getroot()
    manip_object_field = grasp_data.find('ManipulationObject')
    # graspset_field = manip_object_field.find('GraspSet')
    # current_grasp_idx = len(list(graspset_field))

    grasped_field = grasp_data.find('Grasped')

    stability_field = grasp_data.find('GraspStability')

    # avoidance_field = grasp_data.find('ObstacleAvoidance')

    print("\tObject: {}".format(manip_object_field.attrib['name']))

    overall_success = 0
    overall_stability = 0.0

    for success, stability in zip(grasped_field.getchildren(), stability_field.getchildren()):

        print("{} : success {} | stability {}".format(success.get('name'), success.get('quality'), stability.get('quality')))

        overall_success += int(success.get('quality'))
        overall_stability += float(stability.get('quality'))

    print('Overall: \n\tsuccess {}/{}\n\tstability {}/{}.\n'.format(overall_success, len(grasped_field.getchildren()), overall_stability, float(len(grasped_field.getchildren()))))

    return 0


if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser()

    # Add an argument for layout and one for planner
    arg_parser.add_argument('-p', dest='planner', default="")
    arg_parser.add_argument('-l', dest='layout', default="")
    arg_parser.add_argument('-o', dest='manip_object', default="")

    args = arg_parser.parse_args()

    cwd = os.path.abspath(os.getcwd())
    planners = [d for d in os.listdir(cwd) if (os.path.isdir(d) and '.' not in d)]

    if args.planner:
        planners = [args.planner]

    # Decide what planner and layout to use

    for p in planners:

        print("========================")
        print("\nPlanner {}\n".format(p))
        print("========================")

        planner_dir = os.path.join(cwd, p)

        if args.layout:
            layouts = [args.layout]
        else:
            layouts = [l for l in os.listdir(planner_dir) if os.path.isdir(os.path.join(planner_dir, l))]

        for l in layouts:

            print("========================")
            print("Parsing layout {}\n".format(l))

            layout_dir = os.path.join(planner_dir, l)

            if args.manip_object:
                objects = ['grasp_' + args.manip_object + '.xml']
            else:
                objects = [o for o in os.listdir(layout_dir) if '.xml' in o]

            for o in objects:

                object_filename = os.path.join(layout_dir, o)

                print ("Parsing {}".format(object_filename))
                parse_object_file(object_filename)

