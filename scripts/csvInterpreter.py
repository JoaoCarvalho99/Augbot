#!/usr/bin/env python
import rospy
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt


if __name__ == '__main__':
    file = sys.argv[1]
    path = file[ :file.rfind("/") ] + "/" 

    df = pd.read_csv( file, sep=',')
    print ( df )

    head = df.head ( 1 )
    
    max = df.max ( axis = 0, numeric_only = True )

    min = df.min ( axis = 0, numeric_only = True )

    mean = df.mean( axis = 0, numeric_only=True )

    median = df.median ( axis = 0, numeric_only=True )

    tail = df.tail ( 1 )

    col_time = df.iloc[:,0]

    col_x = df.iloc[:,1]
    plt.plot( range( col_x.count() ), col_x )

    col_y = df.iloc[:,2]
    plt.plot( range( col_y.count() ), col_y )

    col_z = df.iloc[:,3]
    plt.plot( range( col_z.count() ), col_z )
    
    plt.legend ( [ "x", "y", "z" ] )
    plt.savefig ( path + "positions.png" )
    #plt.show()

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