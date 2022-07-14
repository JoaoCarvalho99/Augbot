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

from datetime import datetime

files = [ 'localization', 'deadReckoning', 'least_squares']
files = [ 'localization', 'deadReckoning', 'least_squares', 'deadReckoning1']
#files = [ 'localization', 'least_squares' ]
averages = []


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
        while ( i < len ( averages[0] ) ) or ( j < len ( averages[aux] ) ):
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
        plt.xlabel ("segundo decorrido (s)")
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
    
    fig = 0

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
    
        plt.xlabel("numero da recolha")
    
        plt.ylabel("position (m)")
        
        plt.legend ( [ "x", "y", "z" ] )
    
        plt.savefig ( path + "positions.png" )
        fig += 1
        #plt.show()
    
        plt.figure(fig)
    
        plt.plot ( col_x , col_y )
    
        plt.xlabel ("position in x-axis (m)")
    
        plt.ylabel ("position in y-axis (m")
    
        plt.savefig ( path + "xy.png" )
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