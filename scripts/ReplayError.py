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


legend = []
legend1 = []

FONTSIZE = 12


deadReckoningACC = 0

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


def toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, xticks):
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
    plt.xticks(fontsize = FONTSIZE - 4 )
    plt.yticks(fontsize = FONTSIZE - 4 )
    plt.savefig ( savePath )

def ReplaySynchPointError():
    global deadReckoningACC, fig, uwb

    file = home_dir + "/" + "synchPointError.csv"
    Real_df = pd.read_csv( file, sep=',')
    file = replay_dir + "/" + "synchPointError.csv"
    Replay_df = pd.read_csv( file, sep=',')

    leastSquares = 0

    legend = []

    error = []
    
    if leastSquares == 1:
        col_LSx = Real_df.iloc[:,3]
        col_LSy = Real_df.iloc[:,4]
        col_LSz = Real_df.iloc[:,5]
        col_realx = Replay_df.iloc[:,3]
        col_realy = Replay_df.iloc[:,4]
        col_realz = Replay_df.iloc[:,5]
        legend.append ("leastSquares")
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_LSx[i])**2 +  (col_realy[i] - col_LSy[i])**2 ) )
            i += 1
        error.append(e)
        #toPlot(fig, range(len(e[:21])),e[:21],'Control Point','difference (m)', replay_dir + "/LSERROR.pdf",
        # -1, makeXticks (len(error[0][:21])))
        toPlot(fig, range(len(e)),e,'Control Point','difference (m)', replay_dir + "/LSERROR.pdf",
         -1, makeXticks (len(error[0])))
        fig += 1

    col_LSMAx = Real_df.iloc[:,6]
    col_LSMAy = Real_df.iloc[:,7]
    col_LSMAz = Real_df.iloc[:,8]
    col_realx = Replay_df.iloc[:,6]
    col_realy = Replay_df.iloc[:,7]
    col_realz = Replay_df.iloc[:,8]
    legend.append( "leastSquaresMA")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_LSMAx[i])**2 +  (col_realy[i] - col_LSMAy[i])**2 ) )
        i += 1
    error.append(e)
    #toPlot(fig, range(len(e[:21])),e[:21],'Control Point','difference  (m)', replay_dir + "/LSmaERROR.pdf",
    # -1, makeXticks (len(error[0][:21])))
    toPlot(fig, range(len(e)),e,'Control Point','difference  (m)', replay_dir + "/LSmaERROR.pdf",
     -1, makeXticks (len(error[0])))
    fig += 1

    col_dRx = Real_df.iloc[:,9]
    col_dRy = Real_df.iloc[:,10]
    col_dRz = Real_df.iloc[:,11]

    col_realx = Replay_df.iloc[:,9]
    col_realy = Replay_df.iloc[:,10]
    col_realz = Replay_df.iloc[:,11]
    legend.append( "deadReckoning")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_dRx[i])**2 +  (col_realy[i] - col_dRy[i])**2 ) )
        i += 1
    error.append(e)
    #toPlot(fig, range(len(e[:21])),e[:21],'Control Point','difference  (m)', replay_dir + "/DRspERROR.pdf",
    # -1, makeXticks (len(error[0][:21])))
    toPlot(fig, range(len(e)),e,'Control Point','difference  (m)', replay_dir + "/DRspERROR.pdf",
     -1, makeXticks (len(error[0])))
    fig += 1

#    if deadReckoning == 1:
    col_dR1x = Real_df.iloc[:,12]
    col_dR1y = Real_df.iloc[:,13]
    col_dR1z = Real_df.iloc[:,14]

    col_realx = Replay_df.iloc[:,12]
    col_realy = Replay_df.iloc[:,13]
    col_realz = Replay_df.iloc[:,14]
    legend.append( "deadReckoningSP")
    i = 0
    e = []
    while i < len( col_realz):
        e.append( math.sqrt ( (col_realx[i] - col_dR1x[i])**2 +  (col_realy[i] - col_dR1y[i])**2 ) )
        i += 1
    error.append(e)
    #toPlot(fig, range(len(e[:21])),e[:21],'Control Point','difference  (m)', replay_dir + "/DRERROR.pdf",
    # -1, makeXticks (len(error[0][:21])))
    toPlot(fig, range(len(e)),e,'Control Point','difference  (m)', replay_dir + "/DRERROR.pdf",
     -1, makeXticks (len(error[0])))
    fig += 1

    if deadReckoningACC == 1:
        col_dR2x = Real_df.iloc[:,15]
        col_dR2y = Real_df.iloc[:,16]
        col_dR2z = Real_df.iloc[:,17]


        col_realx = Replay_df.iloc[:,15]
        col_realy = Replay_df.iloc[:,16]
        col_realz = Replay_df.iloc[:,17]
        legend.append ( "deadReckoningACC" )
        i = 0
        e = []
        while i < len( col_realz):
            e.append( math.sqrt ( (col_realx[i] - col_dR2x[i])**2 +  (col_realy[i] - col_dR2y[i])**2 ) )
            i += 1
        error.append(e)

    print (error)
    i = 0
    aux = []
    while i < len (error):
        #aux.append(error[i][:21])
        aux.append(error[i])
        i += 1

    #toPlot(nFig, x, y, xlabel,ylabel, savePath, plotLegend, scatterSP, scatterAnchor, xticks)
    #toPlot(0, range(len(error[0][:21])),aux,'Control Point','difference  (m)', replay_dir + "/synchPointError.pdf",
    # legend, makeXticks (len(error[0][:21])))
    toPlot(0, range(len(error[0])),aux,'Control Point','difference  (m)', replay_dir + "/synchPointError.pdf",
     legend, makeXticks (len(error[0])))
    fig += 1



if __name__ == '__main__':
    home_dir = sys.argv[1]
    replay_dir = sys.argv[2]
    
    fig = 3

    cur = 0

    ReplaySynchPointError()





