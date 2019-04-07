#!/usr/bin/env python 

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    data_komodo = pd.read_csv('komodo.csv',sep=',')
    data_armadillo = pd.read_csv('armadillo.csv',sep=',')
    
    data_visualization(data_komodo)
    data_visualization(data_armadillo)

def data_visualization(data):    
    x = data['1']
    ref = np.asarray(data['0'])
    err = data['2']
    
    x_temp = []
    ref_temp = []
    err_temp = []
    for ii in range(len(ref)):
        x_temp.append(np.fromstring( x[ii][1:-1], dtype=np.float,count=3, sep=' '))
        ref_temp.append(np.fromstring( ref[ii][1:-1], dtype=np.float,count=2, sep='  '))
        err_temp.append(np.fromstring(err[ii], dtype=np.float))
        
    x = np.array(x_temp)   
    ref = np.array(ref_temp)  
    err = np.array(err_temp) 
    
    plt.plot(x[:,0],x[:,1])
    plt.plot(ref[:,0],ref[:,1])
    plt.show()
    
    plt.plot(err)
    plt.show()

if __name__ == "__main__":
    main()