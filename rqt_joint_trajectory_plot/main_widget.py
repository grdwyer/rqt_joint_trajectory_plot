#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QTreeWidgetItem
import rclpy
import rclpy.node
from rclpy.duration import Duration
from rqt_py_common import topic_helpers
from rqt_py_common.message_helpers import get_message_class
from trajectory_msgs.msg import JointTrajectory
# from control_msgs.msg import FollowJointTrajectoryActionGoal
from moveit_msgs.msg import DisplayTrajectory
import numpy as np
from .plot_widget import PlotWidget


class MainWidget(QWidget):
    draw_curves = Signal(object, object)

    def __init__(self, node: rclpy.node.Node):
        super(MainWidget, self).__init__()
        self.setObjectName('MainWidget')
        self.node = node
        ui_file = os.path.join(get_package_share_directory('rqt_joint_trajectory_plot'), 'resource',
                               'JointTrajectoryPlot.ui')
        loadUi(ui_file, self)

        self.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))

        self.handler = None
        self.joint_names = []
        self.topic_name_class_map = {}
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.plot_widget = PlotWidget(self)
        self.plot_layout.addWidget(self.plot_widget)
        self.draw_curves.connect(self.plot_widget.draw_curves)

        self.time = None
        (self.dis, self.vel, self.acc, self.eff) = ({}, {}, {}, {})

        # refresh topics list in the combobox
        self.refresh_topics()
        self.change_topic()

        self.refresh_button.clicked.connect(self.refresh_topics)
        self.topic_combox.currentIndexChanged.connect(self.change_topic)
        self.select_tree.itemChanged.connect(self.update_checkbox)

    def refresh_topics(self):
        '''
        Refresh topic list in the combobox
        '''
        topic_list = self.node.get_topic_names_and_types()
        if topic_list is None:
            return
        self.topic_combox.clear()
        self.topic_name_class_map = {}
        for (name, type) in topic_list:
            self.node.get_logger().info("Found topic: {} with type: {}".format(name, type))
            if type[0] in [ 'trajectory_msgs/msg/JointTrajectory',
                         # 'control_msgs/FollowJointTrajectoryActionGoal',
                         'moveit_msgs/msg/DisplayTrajectory']:
                self.node.get_logger().info("\nadding topic: {} with type: {}".format(name, type[0]))
                self.topic_name_class_map[name] = get_message_class(type[0])
                self.topic_combox.addItem(name)

    def change_topic(self):
        topic_name = self.topic_combox.currentText()
        if not topic_name:
            return
        if self.handler:
            self.node.destroy_subscription(self.handler)
        self.joint_names = []
        self.handler = self.node.create_subscription(
            self.topic_name_class_map[topic_name], topic_name, self.callback, 10)

    def close(self):
        if self.handler:
            self.node.destroy_subscription(self.handler)
            self.handler = None

    def refresh_tree(self):
        self.node.get_logger().info("Refreshing the tree for {}".format(self.joint_names))
        self.select_tree.itemChanged.disconnect()
        self.select_tree.clear()
        for joint_name in self.joint_names:
            item = QTreeWidgetItem(self.select_tree)
            item.setText(0, joint_name)
            item.setCheckState(0, Qt.Unchecked)
            item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            for traj_name in ['position', 'velocity', 'acceleration', 'effort']:
                sub_item = QTreeWidgetItem(item)
                sub_item.setText(0, traj_name)
                sub_item.setCheckState(0, Qt.Unchecked)
                sub_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        self.select_tree.itemChanged.connect(self.update_checkbox)

    def callback(self, anymsg):
        if self.pause_button.isChecked():
            return

        msg_class = self.topic_name_class_map[self.handler.topic_name]
        self.node.get_logger().info("Received trajectory from {} with type {}".format(self.handler.topic_name, msg_class))
        # In case of control_msgs/FollowJointTrajectoryActionGoal
        # set trajectory_msgs/JointTrajectory to 'msg'
        # Convert AnyMsg to trajectory_msgs/JointTrajectory

        if isinstance(anymsg, JointTrajectory):
            msg = anymsg
        # elif msg_class == FollowJointTrajectoryActionGoal:
        #     msg = FollowJointTrajectoryActionGoal().deserialize(anymsg._buff).goal.trajectory
        elif isinstance(anymsg, DisplayTrajectory):
            if anymsg.trajectory.__len__() > 0:
                msg = anymsg.trajectory.pop().joint_trajectory
            else:
                self.node.get_logger().warn("Received planned trajectory has no waypoints in it. Nothing to plot!")
                return
        else:
            self.node.get_logger().error('Wrong message type {}'.format(msg_class))
            return

        self.node.get_logger().info("Got joint trajectory with {} points and a duration of {}".format(len(msg.points),
                                                                                             Duration.from_msg(msg.points[-1].time_from_start).nanoseconds/1e9))

        self.time = np.array([0.0] * len(msg.points))
        (self.dis, self.vel, self.acc, self.eff) = ({}, {}, {}, {})
        for joint_name in msg.joint_names:
            self.dis[joint_name] = np.array([0.0] * len(msg.points))
            self.vel[joint_name] = np.array([0.0] * len(msg.points))
            self.acc[joint_name] = np.array([0.0] * len(msg.points))
            self.eff[joint_name] = np.array([0.0] * len(msg.points))
        for i in range(len(msg.points)):
            point = msg.points[i]
            self.time[i] = Duration.from_msg(point.time_from_start).nanoseconds/1e9
            for j in range(len(msg.joint_names)):
                joint_name = msg.joint_names[j]
                if point.positions:
                    self.dis[joint_name][i] = point.positions[j]
                if point.velocities:
                    self.vel[joint_name][i] = point.velocities[j]
                if point.accelerations:
                    self.acc[joint_name][i] = point.accelerations[j]
                if point.effort:
                    self.eff[joint_name][i] = point.effort[j]
        if self.joint_names != msg.joint_names:
            self.joint_names = msg.joint_names
            self.refresh_tree()
        self.joint_names = msg.joint_names
        self.plot_graph()

    def plot_graph(self):
        '''
        Emit changed signal to call plot_widet.draw_curves()
        '''
        curve_names = []
        data = {}
        data_list = [self.dis, self.vel, self.acc, self.eff]
        traj_names = ['position', 'velocity', 'acceleration', 'effort']
        # Create curve name and data from checked items
        for i in range(self.select_tree.topLevelItemCount()):
            joint_item = self.select_tree.topLevelItem(i)
            for n in range(len(traj_names)):
                item = joint_item.child(n)
                if item.checkState(0):
                    joint_name = joint_item.text(0)
                    curve_name = joint_name + ' ' + traj_names[n]
                    curve_names.append(curve_name)
                    data[curve_name] = (self.time, data_list[n][joint_name])
        self.draw_curves.emit(curve_names, data)

    def update_checkbox(self, item, column):
        self.recursive_check(item)
        self.plot_graph()

    def recursive_check(self, item):
        check_state = item.checkState(0)
        for i in range(item.childCount()):
            item.child(i).setCheckState(0, check_state)
            self.recursive_check(item.child(i))
