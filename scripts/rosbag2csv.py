#!/usr/bin/env python3
import sys
import rosbag
import rospy
from optparse import OptionParser
from datetime import datetime
import os

def message_to_csv( stream, msg ):
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv( stream, val )
    except:
        if type (msg ) == list:
            for a in msg:
                message_to_csv ( stream, a)
        else:
            msg_str = str(msg)
            if msg_str.find(",") != -1:
                msg_str = "\"" + msg_str + "\""
            stream.write("," + msg_str)

def message_type_to_csv(stream, msg, parent_content_name=""):
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name,s]))
    except:
        if type ( msg ) == list:
            for a in msg:
                message_type_to_csv(stream, a, parent_content_name)
        else:
            stream.write("," + parent_content_name)
 
def bag_to_csv(options, fname):
    try:
        bag = rosbag.Bag(fname)
        streamdict= dict()
        stime = None
        if options.start_time:
            stime = rospy.Time(options.start_time)
        etime = None
        if options.end_time:
            etime = rospy.Time(options.end_time)
    except Exception as e:
        rospy.logfatal("failed to load bag file: %s", e)
        exit(1)
    finally:
        rospy.loginfo(f"loaded bag file: {fname}")

    try:
        for topic, msg, time in bag.read_messages(topics=options.topic_names,
                                                  start_time=stime,
                                                  end_time=etime):
            if topic in streamdict:
                stream = streamdict[topic]
            else:
                print (file)
                parent = file[ :file.rfind("/") ] + "/"
                child = file[ file.rfind( "/" ):-4 ].replace('/','')
                path = os.path.join( parent, child )
                if not os.path.isdir(path):
                    os.mkdir(path)
                    print ( "path created:"+ path)
                stream = open( path + "/" + topic + ".csv" ,'w')
                print ( "new file: " + stream.name )
                streamdict[topic] = stream
                stream.write("time")
                message_type_to_csv(stream, msg)
                stream.write('\n')

            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(stream, msg )
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()


def GetTopicList(path):
    bag = rosbag.Bag(path)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    types=[]
    for dict_values in list(bag.get_type_and_topic_info()[1].values()):
        types.append(dict_values[0])

    results=[]    
    for to,ty in zip(topics,types):
        results.append(to)

    return results

def main( options ):

    if len(file)<1:
        print("Error:Please select a bag file")
        sys.exit()
    topics = GetTopicList(file)

    options.topic_names=[]
    for k in topics:
        options.topic_names.append(k)

    if len(options.topic_names)==0:
        print("Error:Please select topics")
        sys.exit()

    print("Converting....")
    bag_to_csv(options,file)


if __name__ == '__main__':
    print("rosbag_to_csv start!!")
    file = sys.argv[1]

    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-a", "--all", dest="all_topics",
            action="store_true",
            help="exports all topics", default=False)
    parser.add_option("-t", "--topic", dest="topic_names",
            action="append",
                      help="white list topic names", metavar="TOPIC_NAME")
    parser.add_option("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type="float")
    parser.add_option("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type="float")
    parser.add_option("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header / flatten array value")
    (options, args) = parser.parse_args()


    main( options)
    
    