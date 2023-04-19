#!/usr/bin/env python3
import numpy as np

print('inserire nome file')
file = input()

with open(file) as f:
    array = np.array([[x for x in line.split()] for line in f])


# print(array) 

array = array[array[:, 0].astype(int).argsort()]
print(f'sorted is \n \
      {array}')

groups = np.split(array[:, 0:], np.cumsum(np.unique(array[:, 0].astype(int), return_counts=True)[1])[:-1])
group_stats = np.array([(g[0, 0], len(g), np.mean(g[:, 1].astype(float)).round(2), np.std(g[:, 1].astype(float)).round(2)) for g in groups])
stat_data = np.array2string(group_stats, suppress_small = True)
stat_data = stat_data.replace('[','').replace(']','').replace('\'','')   

stat = open("groups.txt", "w+")
# stat_data = np.array2string(group_stats, suppress_small = True)
stat.write(stat_data)
stat.close()

with open('groups.txt') as s:
    stat_array = np.array([[x for x in line.split()] for line in s])
    
print(stat_array)

variables = np.zeros((len(group_stats),2))
n = 0

for i in range(len(group_stats)):
    if group_stats[i,2].astype(float) > 0.30 and group_stats[i,3].astype(float) < 0.25: 
        variables[n,0] = group_stats[i,0]
        variables[n,1] = group_stats[i,1]
        # print(variables[n])
        n += 1

variables = variables.astype(int)     
# print(variables)
variables = variables[variables[:,1].argsort()[::-1]]
# print(variables)
choosen = variables[:3]
choosen = np.array2string(choosen)
choosen = choosen.replace('[','').replace(']','').replace('\'','')   
print(choosen)

ch_file = open("choosen.txt", "w+")
ch_file.write(str(choosen))
ch_file.close