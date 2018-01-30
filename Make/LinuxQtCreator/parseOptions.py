#!/usr/bin/env python

import subprocess
import fileinput

userFile = file("../../CMakeLists.txt.user")
for line in userFile:
    if "ProjectExplorer.ProjectConfiguration.DefaultDisplayName" in line:
        kitName = line
    elif "ProjectExplorer.ProjectConfiguration.DisplayName" in line:
        kitDisplayName = line
    elif "ProjectExplorer.ProjectConfiguration.Id" in line:
        kitID = line
        break

done = False

for line in fileinput.FileInput("../../CMakeLists.txt.user.tmp", inplace=1):

    if not done:
        if "ProjectExplorer.ProjectConfiguration.DefaultDisplayName" in line:
            print kitName
        elif "ProjectExplorer.ProjectConfiguration.DisplayName" in line:
            print kitDisplayName
        elif "ProjectExplorer.ProjectConfiguration.Id" in line:
            print kitID
            done = True
        else:
            print line
    else:
        print line

subprocess.Popen(['cp', '../../CMakeLists.txt.user.tmp', '../../CMakeLists.txt.user'])
subprocess.Popen(['rm', '../../CMakeLists.txt.user.tmp'])