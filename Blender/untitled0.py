# -*- coding: utf-8 -*-
"""
Created on Fri Jun 10 10:20:09 2022

@author: ciuti
"""
import numpy as np
import scipy.io

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx],idx
mat = scipy.io.loadmat('C:\\Users\\ciuti\\Documents\\GitHub\\XBoom\\Blender\\T.mat')

Time=mat['Time']
Theta=mat['Theta']
Phi=mat['Phi']
Psi=mat['Psi']
x=mat['x']
y=mat['y']
z=mat['z']
