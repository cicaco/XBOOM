# -*- coding: utf-8 -*-
"""
Created on Fri May 27 10:52:39 2022

@author: ciuti
"""

import numpy as np
import scipy.io
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx],idx
mat = scipy.io.loadmat('C:\\Users\\ciuti\\Documents\\GitHub\\XBoom\\Matlab_Code\\TraiectorySimo.mat')

Time=mat['Time']
Theta=mat['Theta']
Phi=mat['Phi']
Psi=mat['Psi']
x=mat['x']
y=mat['y']
z=mat['z']
Time_fin=float(Time[-1])
F_sec=60
F_tot= round(F_sec*Time_fin)
Index=np.ones(F_tot)
cont=1
for k in range(F_tot):
    t_eff=Time_fin/F_tot*cont
    Val,idx=find_nearest(Time, t_eff)
    Index[k]=idx
    cont=cont+1
for i in range(np.size(Time)):
    x[i]
    
    
