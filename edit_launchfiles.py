#!/usr/bin/env python

import os
import shutil

########################################3
# Configuration
############################
planners = ['dwa','teb']

for planner in planners:
    for idv in [0, 1, 2, 3]:
        for ida in [0, 1]:
            for idb in [0, 1]:
                name = planner + '_v' + str(idv) + '_a' + str(ida) + '_b' + str(idb)
                print("editing launch files for " + name)

                # Launch files
                f = open(name + '/launch/' + name + ".launch", "r")
                contents = f.readlines()
                f.close()

                for i in range(len(contents)):
                    contents[i] = contents[i].replace("boxer_velocity_controller", "mobile_base_controller")

                contents.insert(4, "\t<remap from='cmd_vel' to='nav_vel'/>\n")
                f = open(name + '/launch/' + name + ".launch", "w+")
                contents = "".join(contents)
                f.write(contents)
                f.close()