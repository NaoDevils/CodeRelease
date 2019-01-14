#!/usr/bin/env python

#
# This is a little script that formats mof files
# Just run it in terminal with specified mof file als input argument
#
# Example:
# $ python mof_formatter.py anymotion.mof
#
# Note that this scripts need linux 'cp' and 'rm'
#

import sys
import subprocess

if (len(sys.argv) < 2):
    print 'Give mof filename as first argument'
    sys.exit(0)

f = open(sys.argv[1], 'r')
ftmp = open(sys.argv[1]+'.tmp', 'w+')

for line in f:
    line = line.strip()

    words = line.split() # splits all whitespace
    words = filter(None, words)
    
    # find lines intended for | as visual linker
    if (len(words)>0 and words[0]==('"|')):
        ftmp.write('"|        |')
        for i in range(0, 27):
            ftmp.write('      |')
            

    # find joints line
    elif ((len(words) > 0 and words[0]==('"HY'))
       or (len(words) > 0 and words[0]==('stiffness'))
       or (len(words) > 1 and words[0]=='"' and words[1]=='HY')):
        for word in words:
            if (word=='"HY'):
                ftmp.write('"         HY     ')
            elif (word=='"'):
                ftmp.write('"         ')
            elif (word=='stiffness'):
                ftmp.write('stiffness ')
            else:
                ftmp.write(word + ' ')
                n = len(word)
                while n < 6:
                    ftmp.write(' ')
                    n += 1

    # find numbers or dashes or asteriks
    elif (len(words) > 0 and (words[0][0]==('-') or words[0][0].isdigit() or words[0][0]==('*'))):
        ftmp.write('          ')
        for word in words:
            ftmp.write(word + ' ')
            n = len(word)
            while n < 6:
                ftmp.write(' ')
                n += 1
    
    else: # just print line as is
        ftmp.write(line)

    # finally end line
    ftmp.write('\n')
    ftmp.flush()
    continue

f.close()
ftmp.close()

subprocess.Popen(['cp', sys.argv[1]+'.tmp', sys.argv[1]])
subprocess.Popen(['rm', sys.argv[1]+'.tmp'])

print("File " + sys.argv[1] + " was formatted!")


