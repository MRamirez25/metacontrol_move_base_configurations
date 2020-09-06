#!/usr/bin/env python

import os

########################################3
# Configuration
############################
planners = ['dwa','teb']

base_local_planner = dict()
base_local_planner['dwa'] = 'dwa_local_planner/DWAPlannerROS'
base_local_planner['teb'] = 'teb_local_planner/TebLocalPlannerROS'

# Dictionary with all the parameters
# TODO: explain syntax
rosparams_var = dict()
rosparams_const = dict()

# All
rosparams_var['all'] = { 'max_vel_x': 		['1', '0.8'] }
rosparams_const['all']={'max_vel_theta': 	'1',
						'acc_lim_x': 		'1',
						'acc_lim_theta':	'1' }
# DWA
rosparams_var['dwa'] =  {'sim_time': 		['1.7', '1.0'],
					'scaling_speed': 	['1', '0.25'] }
rosparams_const['dwa'] ={'vx_samples':		'5',
						'vth_samples':		'10' }
# TEB
rosparams_var['teb'] =  {'min_obstacle_dis':['0.5', '0.8'],
						'weight_obstacle':	['50', '80'] }
rosparams_const['teb'] ={'footprint_model/type':		'line',
						'footprint_model/line_start':	'[-0.1, 0]',
						'footprint_model/line_end':		'[0.4, 0]',
						'inflation_dist':				'0',
						'enable_homotopy_classes': 		'false',
						'max_number_classes': 			'2' }



###########################################
# Code for the generation of the packages
#############################################

def create_package_xml(name):
	f = open(name + '/' + "package.xml","w+")

	f.write("<package format='2'> \n")
	f.write("  <name>" + name + "</name> \n")
	f.write("  <version>0.0.1</version> \n")
	f.write("  <description>This package provides launch file for operating " + name + "</description> \n")
	f.write("  <license>Apache 2.0</license> \n")
	f.write("  <url type='website'>http://wiki.ros.org/</url> \n")
	f.write("  <maintainer email='jane.doe@example.com'>Jane Doe</maintainer> \n")
	f.write("  <author email='jane.doe@example.com'>Jane Doe</author> \n")
	f.write("  <buildtool_depend>catkin</buildtool_depend> \n")
	f.write("  <exec_depend>move_base</exec_depend> \n")
	f.write("  <test_depend>roslaunch</test_depend> \n")
	f.write("</package> \n")

	f.close() 

def create_CMakeLists_txt(name):
	f = open(name + '/' + "CMakeLists.txt","w+")

	f.write("cmake_minimum_required(VERSION 2.8.3) \n")
	f.write("project(" + name + ") \n")
	f.write("find_package(catkin REQUIRED) \n")
	f.write("catkin_package() \n")
	f.write("### INSTALL ### \n")
	f.write("install(DIRECTORY launch \n")
	f.write("  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} \n")
	f.write(") \n")

	f.close()

for planner in planners:
	for idv in [0, 1]:
		for ida in [0, 1]:
			for idb in [0, 1]:
				name = planner + '_v' + str(idv) + '_a' + str(ida) + '_b' + str(idb)
				print("Writing files for " + name)

				if not os.path.exists(name):
					os.makedirs(name)
				if not os.path.exists(name + "/launch"):
					os.makedirs(name + "/launch")

				# Package files
				create_package_xml(name)
				create_CMakeLists_txt(name)

				# Launch files
				f= open(name + '/launch/' + name + ".launch","w+")
				f.write("<launch> \n")
				# Slam
				f.write("<node pkg='gmapping' type='slam_gmapping' name='slam_gmapping' output='log'>\n")
				f.write("	<remap from='scan' to='front/scan'/>\n")
				f.write("	<param name='map_update_interval' value='1'/>\n")
				f.write("</node>\n")

				# move_base
				f.write("<node pkg='move_base' type='move_base' respawn='false' name='move_base' output='log'>\n")
				f.write("	<param name='base_local_planner' value=" + base_local_planner[planner] + "/>\n")
				f.write("	<remap from='odom' to='boxer_velocity_controller/odom'/>\n")
				f.write("	<rosparam file='$(find simulation_tests)/config/odom_nav_params/local_costmap_params.yaml' command='load'/>\n")
				f.write("	<rosparam file='$(find simulation_tests)/config/odom_nav_params/global_costmap_params.yaml' command='load'/>\n")
				f.write("	<rosparam file='$(find simulation_tests)/config/move_base_params.yaml' command='load'/>\n")

				# Rosparams const
				for key in ['all',planner]:
					for rosparam_const in rosparams_const[key].keys():
						value = rosparams_const[key][rosparam_const]
						f.write("	<rosparam>")
						f.write(rosparam_const + ': ' + value)
						f.write("<rosparam>\n")

				# Rosparams var
				# all
				idx = 0
				for rosparam_var in rosparams_var['all'].keys():
					if idx == 0:
						value = rosparams_var['all'][rosparam_var][ida]
					elif idx == 1:
						value = rosparams_var['all'][rosparam_var][idb]
					f.write("		<rosparam>" + rosparam_var + ': ' + value + "<rosparam>\n")
					idx += 1

				# planner
				idx = 0
				for rosparam_var in rosparams_var[planner].keys():
					if idx == 0:
						value = rosparams_var[planner][rosparam_var][ida]
					elif idx == 1:
						value = rosparams_var[planner][rosparam_var][idb]
					f.write("		<rosparam>" + rosparam_var + ': ' + value + "<rosparam>\n")
					idx += 1

				f.write("</node>\n")

				f.write("</launch>")
				f.close() 