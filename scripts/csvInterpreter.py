#!/usr/bin/env python3
from datetime import date

from numpy import double
import rospy
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import os
import math
import numpy as np

from datetime import datetime

#argv[1] = dir with csv files


#files = csv file names to open
#files = [ 'UWB', 'deadReckoning', 'leastSquares']
#files = ['UWB', 'deadReckoning', 'deadReckoning1']
#files = [ 'UWB', 'deadReckoning', 'leastSquares', 'deadReckoning1']
#files = [ 'UWB', 'deadReckoning', 'leastSquares', 'deadReckoning1', 'deadReckoning2']
files = [ 'UWB', 'deadReckoning', 'leastSquares', 'leastSquaresMA', 'deadReckoning1', 'deadReckoning2']
#files = [ 'UWB', 'leastSquares' ]
legend = []
averages = []

deadReckoning2 = 0
deadReckoning1 = 0
leastSquares = 1
uwb = 1

#synchPoints = [ [0.6,-1.4], [-0.7, -10.7], [-9.9, -9.3], [-8.6, -0.1]] #simulation
#synchPointX = [ 0.6, -0.7, -9.9, -8.6]
#synchPointY = [ -1.4, -10.7, -9.3, -0.1]
synchPoints = [ [ 0, -1.5], [ 0, -3.5], [ -4, -3.25], [ -4, -1.75]] #real
synchPointX = [0, 0, -4, -4]
synchPointY = [-1.5, -3.5, -3.25, -1.75]


def error ():
    i = 0
    j = 0
    aux = 1
    global fig

    while aux < len (files):
        path = home_dir + "/" + files[aux] + "/error.csv"
        f = open(path, 'w')
        writer = csv.writer(f)
        error_x = []
        error_y = []
        error_distance = []
        while ( i < len ( averages[0] ) ) and ( j < len ( averages[aux] ) ):
            if averages[0][i][0] > averages[aux][j][0]:
                j += 1
            elif averages[0][i][0] < averages[aux][j][0]:
                i += 1
            else:
                #row = [averages[0][i][0], averages[0][i][1] - averages[aux][j][1], averages[0][i][2] - averages[aux][j][2]]
                row = [averages[0][i][0], math.sqrt ( (averages[0][i][1] - averages[aux][j][1])**2 +  (averages[0][i][2] - averages[aux][j][2])**2 )]
                writer.writerow ( row )
        #        error_x.append( row[1] )
        #        error_y.append( row[2] )
                error_distance.append ( row[1] )
                i += 1
                j += 1

        plt.figure( fig )
        #plt.plot ( range ( len ( error_x ) ), error_x )
        #plt.plot ( range ( len ( error_y ) ), error_y )
        plt.plot ( range ( len ( error_distance ) ), error_distance )
        plt.xlabel ("time (s)")
        plt.ylabel ("error (m)")
        #plt.legend ( [ "erro em x", "erro em y" ] )
        plt.savefig ( home_dir + "/" + files[aux] + "/error.png" )

        fig += 1
        
        f.close()
        i = 0
        j = 0
        aux += 1

if __name__ == '__main__':
    home_dir = sys.argv[1]
    
    fig = 1

    cur = 0
    while cur < len ( files ):
        path = home_dir + "/" + files[ cur ] +"/"

        if not os.path.isdir(path):
            os.mkdir(path)
    
        file = home_dir + "/" + files[cur] + ".csv"

        df = pd.read_csv( file, sep=',')
        print ( df )
    
        head = df.head ( 1 )
        
        max = df.max ( axis = 0, numeric_only = True )
    
        min = df.min ( axis = 0, numeric_only = True )
    
        mean = df.mean( axis = 0, numeric_only=True )
    
        median = df.median ( axis = 0, numeric_only=True )
    
        tail = df.tail ( 1 )
    
        col_time = df.iloc[:,0]
    
        col_x = df.iloc[:,-3]
    
        col_y = df.iloc[:,-2]
    
        col_z = df.iloc[:,-1]
    
        while col_x.count() != col_y.count():
            if col_x.count () > col_y.count():
                col_x.pop()
            if col_y.count() > col_x.count():
                col_y.pop()
    
        average = [ ]
        data = ""
        x = 0
        y = 0
        nItems = 0
        i = 0
        while i < col_x.count():
            if abs ( col_x[i] ) > 2000000 or abs ( col_y[i] > 2000000 ):
                col_time.remove( i )
                col_x.remove( i )
                col_y.remove( i )
                col_z.remove( i )
            else:
                if col_time[i][ :col_time[i].rfind(".") ] == data:
                    x += double ( col_x[i] )
                    y += double ( col_y[i] )
                    nItems += 1
                else:
                    if nItems != 0:
                        average.append( [ data, x / nItems, y / nItems, x, y, nItems ] )
                       # print ( average [ -1 ] )
                    data = col_time[i][ :col_time[i].rfind(".") ]
                    x = double ( col_x[i] )
                    y = double ( col_y[i] )
                    nItems = 1
                i += 1 
    
        averages.append( average )

        plt.figure(fig)
        plt.plot( range( col_x.count() ), col_x )
    
        #plt.show()
    
        plt.plot( range( col_y.count() ), col_y )
    
        #plt.show()
    
        plt.plot( range( col_z.count() ), col_z )
    
        plt.xlabel("output order number")
    
        plt.ylabel("position (m)")
        
        plt.legend ( [ "x", "y", "z" ] )
    
        plt.savefig ( path + "positions.png" )
        fig += 1
        #plt.show()

        if files[cur] != "deadReckoning2" and files[cur] != "leastSquares" and files[cur] != "UWB" and files[cur] != "deadReckoning1":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif deadReckoning2 == 1 and files[cur] == "deadReckoning2":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif leastSquares == 1 and files[cur] == "leastSquares":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif uwb == 1 and files[cur] == "UWB":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif deadReckoning1 == 1 and files[cur] == "deadReckoning1":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
    
        plt.figure(fig)
    
        plt.plot ( col_x , col_y )
    
        plt.xlabel ("x (m)")
    
        plt.ylabel ("y (m)")
    
        plt.savefig ( path + "xy.png" )
############# with SynchPoints
        plt.scatter( synchPointX, synchPointY, color="black" )
        plt.savefig ( path + "xySP.png" )

        fig += 1
    
        id_max = [ col_x.idxmax(), col_y.idxmax(), col_z.idxmax() ]
    
        id_min = [ col_x.idxmin(), col_y.idxmin(), col_z.idxmin() ]
    
        f = open( path + "summary.txt", "w")
        f.write ( "start:\n" + str ( head ) )
        f.write ( "\n\n\nend:\n" + str ( tail ) )
        f.write ( "\n\n\nmax:\n" )
        f.write ( str ( max ) + "\n" )
        for id in id_max:
            f.write ( "[" + str( id ) + "] " + str( col_time[id] ) + " = [" + str ( col_x[id] ) + ", " + str ( col_y[id] ) + ", " + str ( col_z[id] ) + "]\n" )
        f.write ( "\n\n\nmin\n" )
        f.write ( str ( min ) + "\n" )
        for id in id_min:
            f.write ( "[" + str( id ) + "] " + str ( col_time[id] ) + " = [" + str ( col_x[id] ) + ", " + str ( col_y[id] ) + ", " + str ( col_z[id] ) + "]\n" )
        f.write ( "\n\n\nmean\n" + str ( mean ) )
        f.write ( "\n\n\nmedian\n" + str ( median ) )
        f.close ()

        cur += 1
    
    error ()


    plt.figure(0)    
    plt.xlabel ("x (m)")
    
    plt.ylabel ("y (m)")

    plt.legend ( legend )
    plt.scatter( synchPointX, synchPointY, color="black" )
    plt.savefig ( home_dir + "/AllXY.png" )

    file = home_dir + "/" + "synchPointError.csv"
    df = pd.read_csv( file, sep=',')

    legend = []

    col_realx = df.iloc[:,21]
    col_realy = df.iloc[:,22]
    col_realz = df.iloc[:,23]

    error = []

    leastSquares = 1
    deadReckoning1 = 0
    

    if leastSquares ==1:
        col_LSx = df.iloc[:,3]
        col_LSy = df.iloc[:,4]
        col_LSz = df.iloc[:,5]
        legend.append ("leastSquares")
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_LSx[i])**2 +  (col_realy[i] - col_LSy[i])**2 ) )
            i += 1
        error.append(e)


    col_LSMAx = df.iloc[:,6]
    col_LSMAy = df.iloc[:,7]
    col_LSMAz = df.iloc[:,8]
    legend.append( "leastSquaresMA")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_LSMAx[i])**2 +  (col_realy[i] - col_LSMAy[i])**2 ) )
        i += 1
    error.append(e)

    col_dRx = df.iloc[:,9]
    col_dRy = df.iloc[:,10]
    col_dRz = df.iloc[:,11]
    legend.append( "deadReckoning")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_dRx[i])**2 +  (col_realy[i] - col_dRy[i])**2 ) )
        i += 1
    error.append(e)

    if deadReckoning1 == 1:
        col_dR1x = df.iloc[:,12]
        col_dR1y = df.iloc[:,13]
        col_dR1z = df.iloc[:,14]
        legend.append( "deadReckoning1")
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_dR1x[i])**2 +  (col_realy[i] - col_dR1y[i])**2 ) )
            i += 1
        error.append(e)

    if deadReckoning2 == 1:
        col_dR2x = df.iloc[:,15]
        col_dR2y = df.iloc[:,16]
        col_dR2z = df.iloc[:,17]
        legend.append ( "deadReckoning2" )
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_dR2x[i])**2 +  (col_realy[i] - col_dR2y[i])**2 ) )
            i += 1
        error.append(e)

    if uwb == 1:
        col_UWBx = df.iloc[:,18]
        col_UWBy = df.iloc[:,19]
        col_UWBz = df.iloc[:,20]
        legend.append ( "uwb" )
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_UWBx[i])**2 +  (col_realy[i] - col_UWBy[i])**2 ) )
            i += 1
        error.append(e)

    plt.figure(fig)  
    barWidth = 0.15
 
    # Set position of bar on X axis
    br = []
    print (error)

    br = np.arange(len(error[0][:21]))
    i = 0
    color = ['red', 'blue', 'green', 'yellow', 'black', 'brown', 'orange']
    while i < len (error):
        br1 = br
        plt.bar(br1, error[i][:21], width = barWidth, color=color[i], edgecolor ='grey', label = legend[i])
        br = [x + barWidth for x in br1]
        i += 1
    #plt.bar(len(error[0]), error[0], width=barWidth, label=legend[0])

    plt.xlabel('Synchronization Point number', fontweight ='bold', fontsize = 15)
    plt.ylabel('error (m)', fontweight ='bold', fontsize = 15)
    plt.xticks([r + barWidth for r in range(len(error[0][:21]))], range(1,len(error[0][:21])))
 
    plt.legend(legend)
    plt.savefig ( home_dir + "/synchPointError.png" )




