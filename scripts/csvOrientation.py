#!/usr/bin/env python3
from datetime import date

from numpy import double
import rospy
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import os

from datetime import datetime

#argv[1] = dir with csv files


#files = csv file names to open

#files = [ 'pipico', 'microbit']
files = ['microbit']
averages = []




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

        col_timestamp = df.iloc[:,1]
    
        col_roll = df.iloc[:,2]
    
        col_pitch = df.iloc[:,3]
    
        col_yaw = df.iloc[:,4]

        col_accel_x = df.iloc[:,5]
        col_accel_y = df.iloc[:,6]
        col_accel_z = df.iloc[:,7]        

        average = [ ]
        data = ""
        roll = 0
        pitch = 0
        yaw = 0
        accel_x = 0
        accel_y = 0
        accel_z = 0        
        nItems = 0
        i = 0
        while i < col_roll.count():
            if col_time[i][ :col_time[i].rfind(".") ] == data:
                roll += double ( col_roll[i] )
                pitch += double ( col_pitch[i] )
                yaw += double ( col_yaw[i] )
                accel_x += double ( col_accel_x [i] )
                accel_y += double ( col_accel_y [i] )
                accel_z += double ( col_accel_z [i] )                
                nItems += 1
            else:
                if nItems != 0:
                    average.append( [ timestamp, roll / nItems, pitch / nItems, yaw / nItems, accel_x / nItems, accel_y / nItems, accel_z, nItems, roll, pitch, yaw, accel_x, accel_y, accel_z, nItems ] )
                    # print ( average [ -1 ] )
                data = col_time[i][ :col_time[i].rfind(".") ]
                roll = double ( col_roll[i] )
                pitch = double ( col_pitch[i] )
                yaw = double ( col_yaw[i] )
                accel_x = double ( col_accel_x [i] )
                accel_y = double ( col_accel_y [i] )
                accel_z = double ( col_accel_z [i] )  
                timestamp = str(col_timestamp[i])[ :str(col_timestamp[i]).rfind(".") ]
                nItems = 1
            i += 1 
        

        averages.append( average )

        plt.figure(fig)
        plt.plot( range( col_roll.count() ), col_roll )
    
        #plt.show()
    
        plt.plot( range( col_pitch.count() ), col_pitch )
    
        #plt.show()
    
        plt.plot( range( col_yaw.count() ), col_yaw )
    
        plt.xlabel("numero da recolha")
    
        plt.ylabel("Degreeº")
        
        plt.legend ( [ "roll", "pitch", "yaw" ] )
    
        plt.savefig ( path + "VALOR.png" )
        fig += 1
        #plt.show()

    
#        id_max = [ col_heading.idxmax(), col_yaw.idxmax()]
    
#        id_min = [ col_heading.idxmin(), col_yaw.idxmin()]
    
        f = open( path + "summary.txt", "w")
        f.write ( "start:\n" + str ( head ) )
        f.write ( "\n\n\nend:\n" + str ( tail ) )
        f.write ( "\n\n\nmax:\n" )
        f.write ( str ( max ) + "\n" )
#        for id in id_max:
#            f.write ( "[" + str( id ) + "] " + str( col_time[id] ) + " = [" + str ( col_roll[id] ) + ", " + str ( col_pitch[id] ) + ", " + str ( col_yaw[id]) +", " + str( col_heading[id] ) + "]\n" )
#        f.write ( "\n\n\nmin\n" )
#        f.write ( str ( min ) + "\n" )
#        for id in id_min:
#            f.write ( "[" + str( id ) + "] " + str ( col_time[id] ) + " = [" + str ( col_roll[id] ) + ", " + str ( col_pitch[id] ) + ", " + str ( col_yaw[id]) +", " + str( col_heading[id] ) + "]\n" )
        f.write ( "\n\n\nmean\n" + str ( mean ) )
        f.write ( "\n\n\nmedian\n" + str ( median ) )
        f.close ()

        cur += 1

    i = 0
    j = 0
    path = home_dir + "/"
    f = open(path + "/combined.csv", 'w')
    writer = csv.writer(f)
    roll = []
    pitch = []
    yaw = []
    heading = []
    pipico_x = []
    pipico_y = []
    pipico_z = []    
    microbit_x = []
    microbit_y = []
    microbit_z = []    
    row = [ "timestamp", "roll", "pitch", "yaw(pipico)", "yaw(microbit)",
    "accel_x(pipico)", "accel_x(microbit)", "accel_y(pipico)", "accel_y(microbit)", "accel_z(pipico)", "accel_z(microbit)"]
    #average.append( [ 0 timestamp, 1 roll / nItems, 2 pitch / nItems, 3 yaw / nItems, 
    # 4 accel_x / nItems, 5 accel_y / nItems, 6 accel_z, nItems, 7 roll, 8 pitch, 9 yaw, accel_x, accel_y, accel_z, nItems ] )
    writer.writerow ( row )

    if len ( files ) == 2:
        while ( i < len ( averages[0] ) ) or ( j < len ( averages[1] ) ): #so microbit
            if i >= len ( averages[0] ):
                row = [averages[1][j][0], 0, 0, 0, averages[1][j][3], 0, averages[1][j][4], 0, averages[1][j][5], 0, averages[1][j][6] ]
                j += 1
            elif j >= len ( averages[1] ): #so pipico
                row = [averages[0][i][0], averages[0][i][1], averages[0][i][2], averages[0][i][3], 0,
                averages[0][i][4], 0, averages[0][i][5], 0, averages[0][i][6], 0]
                i += 1
            elif averages[0][i][0] > averages[1][j][0]: #so microbit
                row = [averages[1][j][0], 0, 0, 0, averages[1][j][3], 0, averages[1][j][4], 0, averages[1][j][5], 0, averages[1][j][6] ]
                j += 1
            elif averages[0][i][0] < averages[1][j][0]: #so pipico
                row = [averages[0][i][0], averages[0][i][1], averages[0][i][2], averages[0][i][3], 0,
                 averages[0][i][4], 0, averages[0][i][5], 0, averages[0][i][6], 0]
                i += 1
            else: #ambos
                row = [averages[0][i][0], averages[0][i][1], averages[0][i][2], averages[0][i][3], averages[1][j][3], 
                averages[0][i][4], averages[1][j][4], averages[0][i][5], averages[1][j][5], averages[0][i][6], averages[1][j][6],]
                i += 1
                j += 1
            writer.writerow ( row )
            roll.append( row[1] )
            pitch.append( row[2] )
            yaw.append ( row[3] )
            heading.append ( row[4] )
            pipico_x.append( row[5] )
            pipico_y.append ( row[7] )
            pipico_z.append( row[9] )
            microbit_x.append( row[6] )
            microbit_y.append( row[8] )
            microbit_z.append( row[9] )
            print ( row )
    else:
        while ( i < len ( averages[0] ) ): #so microbit
            row = [averages[0][i][0], 0, 0, 0, averages[0][i][3], 0, averages[0][i][4], 0, averages[0][i][5], 0, averages[0][i][6] ]
            i += 1
            writer.writerow ( row )
            roll.append( row[1] )
            pitch.append( row[2] )
            yaw.append ( row[3] )
            heading.append ( row[4] )
            pipico_x.append( row[5] )
            pipico_y.append ( row[7] )
            pipico_z.append( row[9] )
            microbit_x.append( row[6] )
            microbit_y.append( row[8] )
            microbit_z.append( row[9] )
            print ( row )
        

    plt.figure(fig)
    plt.plot( range( len ( roll ) ), roll )

    plt.plot( range( len ( pitch ) ), pitch )

    plt.plot( range( len ( yaw ) ), yaw )

    plt.plot( range( len ( heading ) ), heading )

    plt.xlabel("segundo")

    plt.ylabel("ângulo médio por segundo em grausº")
    
    plt.legend ( [ "roll", "pitch", "yaw(pipico)", "yaw(microbit)" ] )

    plt.savefig ( path + "average.png" )
    fig += 1 

    plt.figure(fig)
    plt.plot( range( len ( pipico_x ) ), pipico_x )

    plt.plot( range( len ( pipico_y ) ), pipico_y )

    plt.plot( range( len ( microbit_x ) ), microbit_x )

    plt.plot( range( len ( microbit_y ) ), microbit_y )

    plt.xlabel("segundo")

    plt.ylabel("Acceleração média por segundo (m/s²)")
    
    plt.legend ( [ "accel_x (pipico)", "accel_y (pipico)", "accel_x (microbit)", "accel_y (microbit)" ] )

    plt.savefig ( path + "averageACC.png" )
    fig += 1 

    plt.figure(fig)

    plt.plot( range( len ( yaw ) ), yaw )

    plt.plot( range( len ( heading ) ), heading )

    plt.xlabel("segundo")

    plt.ylabel("ângulo médio por segundo em grausº")
    
    plt.legend ( [ "yaw(pipico)", "yaw(microbit)" ] )

    plt.savefig ( path + "averageYaw.png" )
    fig += 1 

    plt.figure(fig)

    plt.plot( range( len ( yaw ) ), yaw )

    plt.xlabel("segundo")

    plt.ylabel("ângulo médio por segundo em grausº")
    
    plt.legend ( [ "yaw(pipico)" ] )

    plt.savefig ( path + "PipicoAverageYaw.png" )
    fig += 1 

    plt.figure(fig)

    #heading = [h if h < 180 else h - 360 for h in heading]

    plt.plot( range( len ( heading ) ), heading )

    plt.xlabel("segundo")

    plt.ylabel("ângulo médio por segundo em grausº")
    
    plt.legend ( [ "yaw(microbit)" ] )

    plt.savefig ( path + "MicrobitAverageYaw.png" )
    fig += 1 