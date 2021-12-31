#!/usr/bin/env python3

import numpy as np

fr_t1_param = {'lr0':0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.025, 
                    'l2': 0.018, 'l3': 0.028, 'l4': 0.015,'theta3':np.pi/10}

l1 = fr_t1_param['l1']
l2 = fr_t1_param['l2']
rp = fr_t1_param['rp']
lr0 = fr_t1_param['lr0']

q2_crit_n = (-np.sqrt(l1**2+l2**2-2*l1*l2)-lr0)/rp
q2_crit_p = (np.sqrt(l1**2+l2**2-2*l1*l2)-lr0)/rp

print(q2_crit_n)
print(q2_crit_p)

q2_crit_n = (lr0 - np.sqrt(l1**2+l2**2-2*l1*l2))/rp
q2_crit_p = (lr0 + np.sqrt(l1**2+l2**2-2*l1*l2))/rp
print(q2_crit_n)
print(q2_crit_p)

a = np.array([[1],
              [2],
              [3]])
print(a.shape)
print(a[0].shape)
b = np.tile(a,4)
print(b)
c = np.tile(a,3)
d = np.dot(c,b)
print(d)
print(d.transpose())
e = d[1:,0]
print(e[1])

c = np.array([1,2,3,4])
print(c.sum())


q = np.concatenate((np.concatenate((np.array([1,1]),np.array([1,1]))),np.array([c[0]])))

print(q)

a = 0.987
b = 1.0

print(round(a%b,3))

gait_regions = np.array([[-0.1, 0.0, 1.1],
                             [-0.2, 0.0, 1.2],
                             [-0.3, 0.0, 1.3],
                             [-0.4, 0.0, 1.4]])

leg_cycle_offset = np.array([0,0.5,0.5,0])
end = (leg_cycle_offset + 0.6)%1.0
ranger = np.stack((leg_cycle_offset,end), axis=-1)

comparison = (ranger[:,0] > ranger[:,1]).astype(int) 
ranger.sort(axis=1)
print(ranger)

for i in range(4):
    # print(i)
    # print(ranger[i])
    # print(comparison[i])
    gait_regions[i,comparison[i]:(comparison[i]+2)] = ranger[i]
    print(gait_regions)
    # gait_regions[i,comparison[i]:(comparison[i]+1)] = ranger[i]
gait_regions = gait_regions + 1
print(comparison)
# gait_regions[comparison,:] = range[:,0]
print(gait_regions)