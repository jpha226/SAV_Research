#! /usr/bin/env python

import math
import subprocess
import random
from subprocess import CalledProcessError, check_output

numTrips = []
numCars = []
totTravel = []
totWaitTime = []
unoccDist = []
unserved = []
reassigned = []
w5 = []
w10 = []
w15 = []
w20 = []
w25 = []
w30 = []
time = []

result = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
avg = []
sd = []

searchStrings = ["Number of trips", "Number of cars", "Unoccupied travel", "Total Travel", "Total wait time", "Unserved trips", "Reassigned trips", "Wait 5", "Wait 10", "Wait 15", "Wait 20", "Wait 25", "Wait 30", "Completion time"]

N = 5

for i in range(N):
	
	seed = random.randint(0,10000)
	print "Run {0}, Seed {1}".format(i + 1, seed)
	success = True
	try:
		output1 = subprocess.check_output(['./simulator', str(seed)])
	except CalledProcessError as e:
		print e.returncode
		success = False
	if (success):
		for j in range(len(searchStrings)):
			a = output1.find(searchStrings[j])
			b = output1.find('\n',a)
			offset = 2
			if j > 6:
				offset = 1 
			if j == 13:
				offset = 2
			number = output1[a+len(searchStrings[j])+offset:b]
			print searchStrings[j] + ": " + number
			if j == 4:
				print "Average wait: {0}".format(float(number) / result[0][i])
			result[j].append(float(number))

		print "\n"

for i in range(N):
	result[14].append(result[4][i] / result[0][i])

for i in range (len(result)):

	s = 0.0

	for j in range(len(result[i])):
		s += result[i][j]

	s = s / len(result[i])
	avg.append(s)
	
	sigma = 0.0
	for j in range(len(result[i])):	
		sigma += (s - result[i][j]) ** 2
	sigma = sigma / N
	sigma = math.sqrt(sigma)
	sd.append(sigma)
	
for i in range (len(searchStrings)):
	print searchStrings[i] + " mean = {0} and sigma = {1}".format(avg[i],sd[i])
	if i == 4:
		print "Average WaitTime mean = {0} and sigma = {1}".format(avg[14],sd[14])

