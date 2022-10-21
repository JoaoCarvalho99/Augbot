#!/usr/bin/env python3
from datetime import date

from numpy import double
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import os
import math
import numpy as np

from datetime import datetime

#argv[1] = dir with csv files


#creates plots from csv files

#files = csv file names to open
#files = [ 'UWB', 'deadReckoningSP', 'leastSquares']
#files = ['UWB', 'deadReckoningSP', 'deadReckoning']
#files = [ 'UWB', 'deadReckoningSP', 'leastSquares', 'deadReckoning']
#files = [ 'UWB', 'deadReckoningSP', 'leastSquares', 'deadReckoning', 'deadReckoningACC']
files = [ 'UWB', 'deadReckoningSP', 'leastSquares', 'leastSquaresMA', 'deadReckoning', 'deadReckoningACC']
#files = [ 'UWB', 'leastSquares' ]
legend = []
averages = []

FONTSIZE = 15

deadReckoningACC = 0
deadReckoning = 0
leastSquares = 0
uwb = 1

sim = 0

if sim == 1:
    synchPoints = [ [0.6,-1.4], [-0.7, -10.7], [-9.9, -9.3], [-8.6, -0.1]] #simulation
    synchPointX = [ 0.6, -0.7, -9.9, -8.6]
    synchPointY = [ -1.4, -10.7, -9.3, -0.1]
    Anchors= [    [-1, 2, 2],    [2, -6, 2],    [-2, -12, 2],    [-12, -12, 2],    [-12, -2, 2]]
    anchorsX = [-1,2,-2,-12,-12]
    anchorsY = [2,-6,-12,-12,-2]
else:
    synchPoints = [ [ 0, -1.5], [ 0, -3.5], [ -4, -3.25], [ -4, -1.75]] #real
    synchPointX = [0, 0, -4, -4]
    synchPointY = [-1.5, -3.5, -3.25, -1.75]
    Anchors= [[0, 0, 2],[-4.80, -5.80, 1.70],[-7.60, -1.60, 2.40],[-1.70, -5.80, 1.80],[-4.60, 0, 1.80]]
    anchorsX = [0,-4.8,-7.6,-1.7,-4.6]
    anchorsY = [0,-5.8,-1.6,-5.8,-0]



def toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, scatterSP, scatterAnchor, xticks):
    plt.figure( nFig )
    if type ( y[0] ) == list or type( y[0]) == pd.core.series.Series:
        for y0 in y:
            plt.plot ( x, y0)
    else: 
        plt.plot (x,y)
    plt.xlabel ( xlabel, fontsize = FONTSIZE )
    plt.ylabel( ylabel, fontsize = FONTSIZE )
    if xticks != -1:
        plt.xticks ( x, xticks )
    if plotLegend != -1:
        plt.legend(plotLegend, bbox_to_anchor = (0.5,1.15), loc="upper center", fancybox = True, ncol=3)
    if scatterSP == 1:
        plt.scatter( synchPointX, synchPointY, color="black",zorder=98 )
    if scatterAnchor == 1:
        plt.scatter( anchorsX, anchorsY, color = "purple", zorder = 99, marker="v",s=100)
    plt.xticks(fontsize = FONTSIZE - 5 )
    plt.yticks(fontsize = FONTSIZE - 4 )
    plt.savefig ( savePath )

def makeXticks ( size ):
    xticks = []
    letter = ['A','B','C','D']
    i = 0
    turn = 0
    while i < size:
        j = 0
        while j < 4 and i < size:
            xticks.append ( letter[j]+str(turn) )
            j += 1
            i += 1
        turn += 1
    return xticks

def synchPointError():
    global deadReckoningACC, fig, uwb
    leastSquares = 0
    deadReckoning = 0

    file = home_dir + "/" + "synchPointError.csv"
    df = pd.read_csv( file, sep=',')

    legend = []

    col_realx = df.iloc[:,21]
    col_realy = df.iloc[:,22]
    col_realz = df.iloc[:,23]

    error = []
    
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
    legend.append( "deadReckoningSP")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_dRx[i])**2 +  (col_realy[i] - col_dRy[i])**2 ) )
        i += 1
    error.append(e)

    if deadReckoning == 1:
        col_dR1x = df.iloc[:,12]
        col_dR1y = df.iloc[:,13]
        col_dR1z = df.iloc[:,14]
        legend.append( "deadReckoning")
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_dR1x[i])**2 +  (col_realy[i] - col_dR1y[i])**2 ) )
            i += 1
        error.append(e)

    if deadReckoningACC == 1:
        col_dR2x = df.iloc[:,15]
        col_dR2y = df.iloc[:,16]
        col_dR2z = df.iloc[:,17]
        legend.append ( "deadReckoningACC" )
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
        legend.append ( "Decawave" )
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_UWBx[i])**2 +  (col_realy[i] - col_UWBy[i])**2 ) )
            i += 1
        error.append(e)

    print (error)
    i = 0
    aux = []
    while i < len (error):
        aux.append(error[i][:21])
        i += 1

    #toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, scatterSP, scatterAnchor, xticks)
    toPlot(fig, range(len(error[0][:21])),aux,'Control Point','error (m)', home_dir + "/synchPointError.pdf",
     legend, -1, -1, makeXticks (len(error[0][:21])))
    fig += 1
    error = []
    legend = []

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

    i = 0
    aux = []
    while i < len (error):
        aux.append(error[i][:21])
        i += 1

    toPlot(fig, range(len(error[0][:21])),aux,'Control Point','error (m)', home_dir + "/synchPointLSError.pdf",
     legend, -1, -1,  makeXticks (len(error[0][:21])))
    fig += 1

    error = []
    legend = []
    e = []

    col_dR1y = df.iloc[:,13]
    col_dR1z = df.iloc[:,14]
    col_dR1x = df.iloc[:,12]
    legend.append( "deadReckoning")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_dR1x[i])**2 +  (col_realy[i] - col_dR1y[i])**2 ) )
        i += 1
    error.append(e)

    i = 0
    aux = []
    while i < len (error):
        aux.append(error[i][:21])
        i += 1

    toPlot(fig, range(len(error[0][:21])),aux,'Control Point','error (m)', home_dir + "/synchPointDRError.pdf",
     legend, -1, -1,  makeXticks (len(error[0][:21])))
    fig += 1




def error ():
    i = 0
    j = 0
    aux = 1
    global fig
    legendError = []
    legendErrorLS = []

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

        #toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, scatterSP, scatterAnchor, xticks)
        toPlot(fig, range ( len ( error_distance ) ), error_distance, "time (s)","error (m)", home_dir + "/" + files[aux] + "/error.pdf"
        , -1, -1, -1, -1)
        fig += 1
        
        f.close()
        i = 0
        j = 0

        if files[aux] == "leastSquaresMA" or files[aux] =="deadReckoning" or files[aux] == "deadReckoningSP":
            legendError.append ( files[aux])
            plt.figure(1)
            plt.plot (range ( len ( error_distance ) ), error_distance)

        if files[aux] == "leastSquaresMA" or files[aux] =="leastSquares":
            legendErrorLS.append ( files[aux])
            plt.figure(2)
            plt.plot (range ( len ( error_distance ) ), error_distance)

        aux += 1


    
    plt.figure(1)
    plt.xlabel ("time (s)", fontsize = FONTSIZE )
    plt.ylabel ("error(m)", fontsize = FONTSIZE )
    plt.legend ( legendError, bbox_to_anchor = (0.5,1.15), loc="upper center", fancybox = True, ncol=3 )
    plt.xticks(fontsize = FONTSIZE - 4 )
    plt.yticks(fontsize = FONTSIZE - 4 )
    plt.savefig ( home_dir + "/realError.pdf" )

    plt.figure(2)
    plt.xlabel ("time (s)", fontsize = FONTSIZE )
    plt.ylabel ("error(m)", fontsize = FONTSIZE )
    plt.legend ( legendError, bbox_to_anchor = (0.5,1.15), loc="upper center", fancybox = True, ncol=3 )
    plt.xticks(fontsize = FONTSIZE - 4 )
    plt.yticks(fontsize = FONTSIZE - 4 )
    plt.savefig ( home_dir + "/realLSError.pdf" )


if __name__ == '__main__':
    home_dir = sys.argv[1]
    
    fig = 3

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

        #toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, scatterSP, scatterAnchor, xticks)
        aux = []
        aux.append (col_x)
        aux.append (col_y)
        aux.append (col_z)
        
        toPlot(fig, range ( col_x.count()), aux, "output number","position (m)", path + "positions.pdf",
         [ "x", "y", "z" ], -1, -1, -1)
        fig += 1
        #plt.show()

        if files[cur] != "deadReckoningACC" and files[cur] != "leastSquares" and files[cur] != "UWB" and files[cur] != "deadReckoning":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif deadReckoningACC == 1 and files[cur] == "deadReckoningACC":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif leastSquares == 1 and files[cur] == "leastSquares":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif uwb == 1 and files[cur] == "UWB":
            legend.append ("Decawave")
            plt.figure (0)
            plt.plot ( col_x , col_y )
        elif deadReckoning == 1 and files[cur] == "deadReckoning":
            legend.append (files[cur])
            plt.figure (0)
            plt.plot ( col_x , col_y )
    
    

        #toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, scatterSP, scatterAnchor, xticks)
        toPlot(fig, col_x, col_y, "x (m)","y (m)", path + "xy.pdf" , -1, -1, -1, -1)
        fig += 1
############# with SynchPoints
        toPlot(fig, col_x, col_y, "x (m)","y (m)", path + "xySP.pdf" , -1, 1, -1, -1)
        fig += 1
############# with SynchPoints + Anchors
        toPlot(fig, col_x, col_y, "x (m)","y (m)", path + "xySPAnchor.pdf" , -1, 1, 1, -1)
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
    plt.xlabel ("x (m)", fontsize = FONTSIZE )   
    plt.ylabel ("y (m)", fontsize = FONTSIZE )
    plt.legend ( legend, bbox_to_anchor = (0.5,1.15), loc="upper center", fancybox = True, ncol=3 )
    plt.scatter( synchPointX, synchPointY, color="black",zorder=99 )
    plt.xticks(fontsize = FONTSIZE - 4 )
    plt.yticks(fontsize = FONTSIZE - 4 )
    plt.savefig ( home_dir + "/AllXY.pdf" )

    synchPointError()
