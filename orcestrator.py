from loco_planning.scripts.utils.communication_utils import checkRosMaster, launchFileNode
import rospy

def main():
    checkRosMaster()
    additional_args = ["start_controller:=false"]
    # launchFileNode(package="loco_planning", launch_file="multiple_robots.launch", additional_args=additional_args)
    launchFileNode(package="map_pkg", launch_file="spawn_map.launch", additional_args=additional_args)
    rospy.sleep(60.)

if __name__ == '__main__':
    main()
