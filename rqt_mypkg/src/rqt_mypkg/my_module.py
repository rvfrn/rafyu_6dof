import os
import rospy
import rospkg
import math
import csv
import numpy as np
import moveit_commander.robot as roboarm
import moveit_commander.planning_scene_interface as ScenePlanning
import moveit_commander
import moveit_commander.move_group as MoveCommander
import moveit_msgs.msg
import geometry_msgs.msg

import time

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QSlider, QLabel
from python_qt_binding.QtGui import QIcon, QPixmap
from rafyu6dof_moveit.msg import ArmJointState
from progressbar import ProgressBar
from moveit_commander.conversions import pose_to_list
from random import randint

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)

        #os.system('roslaunch rafyu6dof_moveit_config demo.launch')

        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()

        robot = roboarm.RobotCommander()
        scene = ScenePlanning.PlanningSceneInterface()
        group_name = "rafyu6dof_arm"
        group = MoveCommander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print("============ Reference frame: %s")

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s")

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print ("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print ("============ Printing robot state")
        print (robot.get_current_state())
        print ("")
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, self.joint_states_callback)

        self.username = os.path.expanduser("~")


        target = "logo_su.png"
        initial_dir = self.username+'/catkin_ws/src'

        path_file = ''
        for root, _, files in os.walk(initial_dir):
            if target in files:
               path_file = os.path.join(root, target)
               break
        #To search for the username
        img = QPixmap(path_file)

        self._widget.LabelImageUao.setPixmap(img)

        self._widget.PlayButton.setIcon(QIcon.fromTheme('media-record'))
        self._widget.PlayButton.clicked[bool].connect(self._Send_joints_teleoperation)

        self._widget.HomeButton.setIcon(QIcon.fromTheme('go-home'))
        self._widget.HomeButton.clicked[bool].connect(self._Center_joints_teleoperation)

        self._widget.RandomizeButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.RandomizeButton.clicked[bool].connect(self._Randomize_joints_teleoperation)

        self._widget.GripperButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.GripperButton.clicked[bool].connect(self._fcn_gripper)

        self._widget.SavePoseButton.setIcon(QIcon.fromTheme('document-save'))
        self._widget.SavePoseButton.clicked[bool].connect(self._save_pose)

        self._widget.DeletePoseButton.setIcon(QIcon.fromTheme('edit-clear'))
        self._widget.DeletePoseButton.clicked[bool].connect(self._delete_pose)

        self._widget.ExecutePathButton.setIcon(QIcon.fromTheme('media-playback-start'))
        self._widget.ExecutePathButton.clicked[bool].connect(self._execute_path)

        self._widget.SaveTrajectoryButton.setIcon(QIcon.fromTheme('document-send'))
        self._widget.SaveTrajectoryButton.clicked[bool].connect(self._write_csv)

        self._widget.ImportTrajectoryButton.setIcon(QIcon.fromTheme('document-open'))
        self._widget.ImportTrajectoryButton.clicked[bool].connect(self._read_csv)

        self._widget.PreviewButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.PreviewButton.clicked[bool].connect(self._Preview_pose_sliders)

        self.goal = ArmJointState()

        self.arr_sl = [self._widget.SlJoint1,self._widget.SlJoint2,self._widget.SlJoint3,self._widget.SlJoint4,self._widget.SlJoint5,self._widget.SlJoint6]
        self.arr_ShowSl = [self._widget.ShowJoint1,self._widget.ShowJoint2,self._widget.ShowJoint3,self._widget.ShowJoint4,self._widget.ShowJoint5,self._widget.ShowJoint6]
        

        self.pub2 = rospy.Publisher('joint_steps', ArmJointState, queue_size=50)
        rate = rospy.Rate(60) # 20hz

        self.savePose = []
        self.trajectory = []
        self.count_save_pose = 0
        self.joint_visualizer = []

        lista_arq = ''
    
        if os.path.exists(self.username+"/trajectories_rafyu6dof"):
            for i in os.listdir(self.username+"/trajectories_rafyu6dof"): lista_arq+=(i+'\n')
            print(lista_arq)
        else:
            os.makedirs(self.username+"/trajectories_rafyu6dof")

        #for i in lista_arq:
        self._widget.ShowText.setText("Saved paths: \n" + lista_arq )

        for i in range(0,6):
            self.arr_sl[i].setEnabled(True)
            self.arr_sl[i].setMaximum(90)
            self.arr_sl[i].setMinimum(-90)
            self.arr_sl[i].setValue(0)
            self.arr_sl[i].valueChanged.connect(self.joints_changes)
            self.arr_ShowSl[i].setEnabled(True)
            self.arr_ShowSl[i].setText(str(self.arr_sl[i].value()))

        self._widget.SlJoint2.setMinimum(0)

        self._widget.spinBoxRepeat.setMaximum(20)
        self._widget.spinBoxRepeat.setMinimum(-20)
        self._widget.spinBoxRepeat.setValue(0)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)    

        self.grip = 0
        self.count_save_pose = 0
        self.activate = 0

    def joint_states_callback(self, joint_state):
        pub = rospy.Publisher('trajectory', ArmJointState, queue_size=20)
        rate = rospy.Rate(60) # 20hz
        goal = ArmJointState()
        goal.position1 = np.int16(joint_state.position[0]*(180/np.pi))
        goal.position2 = np.int16(joint_state.position[1]*(180/np.pi))
        goal.position3 = np.int16(joint_state.position[2]*(180/np.pi))
        goal.position4 = np.int16(joint_state.position[3]*(180/np.pi))
        goal.position5 = np.int16(joint_state.position[4]*(180/np.pi))
        goal.position6 = np.int16(joint_state.position[5]*(180/np.pi))
        goal.position7 = self.goal.position7
        pub.publish(goal)
        
    def joints_changes(self):

        #group = self.group 
        #joint_goal = group.get_current_joint_values()

        for i in range(0,6):
            self.arr_ShowSl[i].setText(str(self.arr_sl[i].value()))     
            #joint_goal[i] = (self.arr_sl[i].value()*np.pi)/180

        #group.go(joint_goal, wait=True)
        #group.stop()

    def _Send_joints_teleoperation(self):
        self._widget.ShowText.setText("Moving Joints with Sliders")

        group = self.group
        joint_goal = group.get_current_joint_values()

        for i in range(0,6):
            joint_goal[i] = (self.arr_sl[i].value()*np.pi)/180
        
        self.goal.position1 = np.int16(((self.arr_sl[0].value()*np.pi)/180)*(16000/(2*np.pi)))
        self.goal.position2 = np.int16(((self.arr_sl[1].value()*np.pi)/180)*(25600/(2*np.pi)))
        self.goal.position3 = np.int16(((self.arr_sl[2].value()*np.pi)/180)*(16000/(2*np.pi)))
        self.goal.position4 = np.int16(((self.arr_sl[3].value()*np.pi)/180)*(3200/(2*np.pi)))
        self.goal.position5 = np.int16(((self.arr_sl[4].value()*np.pi)/180)*(600/(2*np.pi)))
        self.goal.position6 = np.int16(((self.arr_sl[5].value()*np.pi)/180)*(200/(2*np.pi)))

        self.pub2.publish(self.goal)

        group.go(joint_goal, wait=True)
        group.stop()

        current_joints = self.group.get_current_joint_values()
        
    def _Center_joints_teleoperation(self):
        group = self.group 
        joint_goal = group.get_current_joint_values()

        for i in range(0,6):      
            self.arr_sl[i].setValue(0)
            joint_goal[i] = (self.arr_sl[i].value()*np.pi)/180

        self.goal.position1 = np.int16(((self.arr_sl[0].value()*np.pi)/180)*(16000/(2*np.pi)))
        self.goal.position2 = np.int16(((self.arr_sl[1].value()*np.pi)/180)*(25600/(2*np.pi)))
        self.goal.position3 = np.int16(((self.arr_sl[2].value()*np.pi)/180)*(16000/(2*np.pi)))
        self.goal.position4 = np.int16(((self.arr_sl[3].value()*np.pi)/180)*(3200/(2*np.pi)))
        self.goal.position5 = np.int16(((self.arr_sl[4].value()*np.pi)/180)*(600/(2*np.pi)))
        self.goal.position6 = np.int16(((self.arr_sl[5].value()*np.pi)/180)*(200/(2*np.pi)))

        self.pub2.publish(self.goal)

        group.go(joint_goal, wait=True)
        group.stop()

    def _Randomize_joints_teleoperation(self):
        result = []
        for i in range(0,6):
            x = randint(0,90)
            y = randint(0,90)
            result.append(x-y)
            self.arr_sl[i].setValue(result[i])

    def _fcn_gripper(self):
        self.grip = self.grip + 1

        if self.grip == 1:
            self.goal.position7 = 110
            self.pub2.publish(self.goal)
            self._widget.ShowText.setText("Gripper ON")
        else:
            self.goal.position7 = 0
            self.pub2.publish(self.goal)
            self._widget.ShowText.setText("Gripper OFF")
            self.grip = 0
        

    def _save_pose(self):
        self.GoalPosition = [self.goal.position1, self.goal.position2, self.goal.position3, self.goal.position4, self.goal.position5, self.goal.position6,self.goal.position7]
        self.count_save_pose = self.count_save_pose + 1
        if self.count_save_pose == 1:
            self.trajectory = []
            for i in range(0,7):
                self.savePose.append((self.GoalPosition[i]))
                self.joint_visualizer.append((self.GoalPosition[i]))
            self.activate == 0
            print("Enter the first")

        elif self.activate == 1:
            self.trajectory = [[0,0,0],[0,0,0]]
            self.trajectory = []
            self.savePose = []
            self.joint_visualizer = []
            for i in range(0,7):
                self.savePose.append((self.GoalPosition[i]))
                self.joint_visualizer.append((self.GoalPosition[i]))
            self.activate = 0
            print("Enter the second")
        else:
            self.savePose = []
            self.joint_visualizer = []
            for i in range(0,7):
                self.savePose.append((self.GoalPosition[i]))
                self.joint_visualizer.append((self.GoalPosition[i]))
            print("Enter the third")
        self.trajectory.append(self.savePose)

        self._widget.ShowText.setText(
            "Successfully saved position:"
            "\nJoint1:   "+ str(round((self.joint_visualizer[0]*360)/16000))+" degrees"+
            "\nJoint2:   "+ str(round((self.joint_visualizer[1]*360)/25600))+" degrees"+         
            "\nJoint3:   "+ str(round((self.joint_visualizer[2]*360)/16000))+" degrees"+ 
            "\nJoint4:   "+ str(round((self.joint_visualizer[3]*360)/3200))+" degrees"+ 
            "\nJoint5:   "+ str(round((self.joint_visualizer[4]*360)/600))+" degrees"+ 
            "\nJoint6:   "+ str(round((self.joint_visualizer[5]*360)/200))+" degrees"+ 
            "\nGripper:  "+ str(self.joint_visualizer[6])+" degrees"
            )

        print(self.trajectory)

    def _delete_pose(self, scale=1):

        self.trajectory.pop(len(self.trajectory) - 1)
        print(self.trajectory)
        self._widget.ShowText.setText("Previous pose delete successfully")


    def _execute_path(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        timeRepeat = self._widget.spinBoxRepeat.value()

        if timeRepeat != 0 :
            for i in range(timeRepeat):

                for num_array_pose in self.trajectory:
                    goal = ArmJointState()
                    goal.position1 = np.int16(num_array_pose[0])
                    joint_goal[0] = (float(num_array_pose[0])*(2*np.pi))/16000
                    goal.position2 = np.int16(num_array_pose[1])
                    joint_goal[1] = (float(num_array_pose[1])*(2*np.pi))/25600
                    goal.position3 = np.int16(num_array_pose[2])
                    joint_goal[2] = (float(num_array_pose[2])*(2*np.pi))/16000
                    goal.position4 = np.int16(num_array_pose[3])
                    joint_goal[3] = (float(num_array_pose[3])*(2*np.pi))/3200
                    goal.position5 = np.int16(num_array_pose[4])
                    joint_goal[4] = (float(num_array_pose[4])*(2*np.pi))/600
                    goal.position6 = np.int16(num_array_pose[5])
                    joint_goal[5] = (float(num_array_pose[5])*(2*np.pi))/200
                    goal.position7 = np.int16(num_array_pose[6])


                    self.pub2.publish(goal)
                    group.go(joint_goal, wait=True)

                    #time.sleep(2)
                    rospy.sleep(5)#Antes 5
                print(i)
            group.stop()
            self._widget.spinBoxRepeat.setValue(0)  

                
        else : 
            for num_array_pose in self.trajectory:
                goal = ArmJointState()
                goal.position1 = np.int16(num_array_pose[0])
                joint_goal[0] = (float(num_array_pose[0])*(2*np.pi))/16000
                goal.position2 = np.int16(num_array_pose[1])
                joint_goal[1] = (float(num_array_pose[1])*(2*np.pi))/25600
                goal.position3 = np.int16(num_array_pose[2])
                joint_goal[2] = (float(num_array_pose[2])*(2*np.pi))/16000
                goal.position4 = np.int16(num_array_pose[3])
                joint_goal[3] = (float(num_array_pose[3])*(2*np.pi))/3200
                goal.position5 = np.int16(num_array_pose[4])
                joint_goal[4] = (float(num_array_pose[4])*(2*np.pi))/600
                goal.position6 = np.int16(num_array_pose[5])
                joint_goal[5] = (float(num_array_pose[5])*(2*np.pi))/200
                goal.position7 = np.int16(num_array_pose[6])


                self.pub2.publish(goal)
                group.go(joint_goal, wait=True)
                group.stop()
                #time.sleep(2)
                rospy.sleep(5)#Antes 5


    def _write_csv(self):
        name = str(self._widget.NameFileTextEdit.toPlainText())
        locationFile = open(self.username+'/trajectories_rafyu6dof/'+name+'.csv','w')
        file = csv.writer(locationFile)
        file.writerows(self.trajectory)
        self._widget.ShowText.setText("Successfully saved file "+name+" (CSV)")
        os.system('espeak "(Saved file)"')

    def _read_csv(self):
        self.activate = 1
        self.count_save_pose = 1
        name = str(self._widget.NameFileTextEdit.toPlainText())
        self.trajectory = []
        locationFile = open(self.username+'/trajectories_rafyu6dof/'+name+'.csv','r')
        file = csv.reader(locationFile, delimiter=',')
        for num_array_pose in file:
            for num_pose in range(0,7):
                np.asarray(num_array_pose[num_pose])
            self.trajectory.append(num_array_pose)
        self._widget.ShowText.setText("Successfully import file "+name+" (CSV)")
        os.system('espeak "(Imported file)"')
        print(self.trajectory)

    def _Preview_pose_sliders(self):

        group = self.group 
        joint_goal = group.get_current_joint_values()

        for i in range(0,6):   
            joint_goal[i] = (self.arr_sl[i].value()*np.pi)/180

        group.go(joint_goal, wait=True)
        group.stop()
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
