#!/usr/bin/env python

planners = ['dwa','teb']

base_local_planner = dict()
base_local_planner['dwa'] = 'dwa_local_planner/DWAPlannerROS'
base_local_planner['teb'] = 'teb_local_planner/TebLocalPlannerROS'

# Dictionary with all the parameters
# TODO: explain syntax
rosparams = dict()
rosparams['all'] = { 'max_vel_x': 		['1', '0.8'],
					'max_vel_theta': 	'1',
					'acc_lim_x': 		'1',
					'acc_lim_theta':	'1' }

rosparams['dwa'] =  {'sim_time': 		['1.7', '1.0'],
					'scaling_speed': 	['1', '0.25'],
					'vx_samples':		'5',
					'vth_samples':		'10' }

rosparams['teb'] =  {'min_obstacle_dis':['0.5', '0.8'],
					'weight_obstacle':	['50', '80'],
					'footprint_model':	'line',
					'line_start':		['-0.1', '0'],
					'line_end':			['0.4', '0'],
					'inflation_dist':	'0',
					'enable_homotopy_classes': 'false',
					'max_number_classes': '2' }


for planner in planners:
	for idx in [0, 1]:
		print(planner)
		name = planner + '_v' + str(idx) + '_a' + str(ida) + 'b_' + str(idb)
		f= open(name + ".launch","w+")
		f.write("<launch> \n")
		# Slam
		f.write("""
			<node pkg='gmapping' type='slam_gmapping' name='slam_gmapping' output='log'>
				<remap from='scan' to='front/scan'/>
				<param name='map_update_interval' value='1'/>
			</node>
		""")
		# move_base
		f.write("""
			<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="log">
				<remap from="odom" to="boxer_velocity_controller/odom"/>

				<rosparam file="$(find simulation_tests)/config/costmap_common_params.yaml" command="load" ns="global_costmap"/>
				<rosparam file="$(find simulation_tests)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>

				<rosparam file="$(find simulation_tests)/config/odom_nav_params/local_costmap_params.yaml" command="load"/>
				<rosparam file="$(find simulation_tests)/config/odom_nav_params/global_costmap_params.yaml" command="load"/>

				<rosparam file="$(find simulation_tests)/config/move_base_params.yaml" command="load"/>
		""")
		# Params
		f.write("<param name='base_local_planner' value=" + base_local_planner[planner] + "/>")

		# Rosparams
		for key in ['all',planner]:
			for rosparam in rosparams[key].keys():
				
				if isinstance(rosparams[key][rosparam], list):
					value = rosparams[key][rosparam][idx]
				else:
					value = rosparams[key][rosparam]

				f.write("		<rosparam>")
				f.write(rosparam + ': ' + value)
				f.write("<rosparam>\n")

		f.write("	</node>\n")

		f.write("</launch>")
		f.close() 