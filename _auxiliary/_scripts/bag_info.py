### Ewerton Lopes


import  os, sys, random, shutil
import subprocess, yaml, json
import argparse
from collections import defaultdict, Counter
import logging

try:
    import colorlog
    have_colorlog = True
except ImportError:
    have_colorlog = False
    
def printHumanReadableSize(nbytes):
    """ Prints the number of bytes in a readable way.
    parameter
        nbytes : number of bytes
    return
        the size with respect to B, KB, MB, GB, TB and PB.
    """
    suffixes = ['B', 'KB', 'MB', 'GB', 'TB', 'PB']
    if nbytes == 0: return '0 B'
    i = 0
    while nbytes >= 1024 and i < len(suffixes)-1:
        nbytes /= 1024.
        i += 1
    f = ('%.2f' % nbytes).rstrip('0').rstrip('.')
    return '%s %s' % (f, suffixes[i])
    
def readArgs():
    """ Deals with the argments"""
    parser = argparse.ArgumentParser(description='Process some integers.') 
    
    parser.add_argument('outputName', metavar='output_name',
                        action='store', default=os.path.dirname(os.path.abspath(__file__)) + "/result.txt",
                        help='the name for the summary output file.')
    parser.add_argument('--path', dest='src_path', action='store',
                        default=os.path.dirname(os.path.abspath(__file__)),
                        help='the folder containing the bag files. (default: the script current path)')
    parser.add_argument('--folder', dest='dst', action='store',
                        default=os.path.dirname(os.path.abspath(__file__)) + "/result",
                        help='the folder name for storing the summary file')

    return parser.parse_args()

def parseBags(outputName, bagsPath, outputPath):
    """Loops through the bag files and save info to outputName file.
    parameters:
        outputName - the name of the file where the info is saved to.
        bagsPath   - the path to the folder containing bag files.
        outputPath - the folder location where to save the outputName.
    return:
        void
    """
    
    logger.info("Analysing current path, searching for files...")
    mypath = bagsPath
    bagfiles = [f for f in os.listdir(mypath) if os.path.isfile(os.path.join(mypath, f)) and f.endswith('.bag')]

    nfiles = len(bagfiles)

    if nfiles == 0:
        logger.error("No '.bag' files found!")
        sys.exit(0)

    logger.info('A number of ' + str(nfiles) + " '.bag' files were found")

    file = open(outputPath + '/' + outputName, 'w')
    file.write(80*'*')
    file.write('\n')
    file.write(30*' ')
    file.write('SUMMARY OF BAG FILES')
    file.write(30*' ')
    file.write('\n')
    file.write(80*'*')
    file.write('\n\n')
    file.write('>> NUMBER OF FILES:   ' + str(nfiles) +'\n')
    file.write('>> LIST OF FILES:\n')
    for f in bagfiles:
            file.write('\t\t\t\t' + f +'\n')
    file.write('\n')
    totalDuration = 0
    totalMessages = 0
    totalSize = 0
    topicNames = []
    avgTopicFreq = defaultdict(list)
    avgTopicMessage = defaultdict(list)
    info_dicts = []
    
    for f in bagfiles:
        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info',
                     '--yaml', mypath + '/' + f],
                     stdout=subprocess.PIPE).communicate()[0])
        info_dict['_fileName'] = f
        
        totalDuration += int(info_dict['duration'])
        totalMessages += int(info_dict['messages'])
        totalSize     += int(info_dict['size'])
        
        for t in info_dict['topics']:
            if t['topic'] not in topicNames:
                topicNames.append(t['topic'])
  
            avgTopicFreq[t['topic']].append(t['messages'] / info_dict['duration'])
            avgTopicMessage[t['topic']].append(t['messages'])
            
        
        logger.info("Processing "+f + '...')
        info_dicts.append(info_dict)

    file.write('>> TOPIC NAMES: \n')
    for n in topicNames:
        file.write('\t\t\t\t' + n +'\n')
    file.write('\n')
    file.write('>> TOTAL DURATION (secs): ' + str(totalDuration) + '\n')
    file.write('>> AVG DURATION (secs): ' + str(totalDuration/nfiles) + '\n')
    file.write('>> TOTAL MESSAGES: ' + str(totalMessages) + '\n')
    file.write('>> FREQUENCE DETAILS: ' +'\n\n')
    for k in avgTopicFreq.keys():
        file.write('\t' + k + ':\n\t\t\t\tAvg sample rate: ' + str(sum(avgTopicFreq[k])/len(avgTopicFreq[k])) + ' Hz\n\t\t\t\tavg # messages: ' + str(sum(avgTopicMessage[k])/len(avgTopicMessage[k])) + '\n' + 10*'\t' + '# of files: '+ str(len(avgTopicFreq[k])) + '/' + str(nfiles) +'\n\n')
    file.write('>> TOTAL SIZE: ' + printHumanReadableSize(totalSize) + '\n\n')
    file.write(80*'*')
    file.write('\n')
    file.write(35*' ')
    file.write('FILE DETAILS')
    file.write(35*' ')
    file.write('\n')
    file.write(80*'*')
    file.write('\n\n')
    
    for d in range(len(info_dicts)):
        if not (d == (len(info_dicts)-1)):
            file.write(39*'-'+'#'+ str(d) + 39*'-' + '\n\n')
        file.write(json.dumps(info_dicts[d], sort_keys=True, indent=4, separators=(',', ': ')) +'\n\n')
        
    
    file.close()
    
    logger.info('DONE!')
    

if __name__=="__main__":
    
    args = readArgs()
    # create logger
    logger = logging.getLogger('bag_logger')
    logger.setLevel(logging.DEBUG)

    # create console handler and set level to debug
    ch = logging.StreamHandler( sys.__stdout__ ) # Add this
    ch.setLevel(logging.DEBUG)

    # create formatter
    #formatter = logging.Formatter('%(asctime)s [%(levelname)s] -- %(message)s')

    format      = '%(asctime)s - %(levelname)-8s - %(message)s'
    date_format = '%Y-%m-%d %H:%M:%S'
    if have_colorlog and os.isatty(2):
        cformat   = '%(log_color)s' + format
        formatter = colorlog.ColoredFormatter(cformat, date_format,
              log_colors = { 'DEBUG'   : 'reset',       'INFO' : 'reset',
                             'WARNING' : 'bold_yellow', 'ERROR': 'bold_red',
                             'CRITICAL': 'bold_red' })
    else:
        formatter = logging.Formatter(format, date_format)
        
    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)

    ### creating folder to store log file ###
    try:
        logger.info("Attempting to create folder for saving log file...")
        os.mkdir(args.dst)
        logger.info("Folder " + args.dst + " created.")
    except OSError, err:
        if err.errno == os.errno.EEXIST:
            logger.error(str(err))
            logger.warn("Folder '" + args.dst + "' already exists! Skipping...")
    except Exception, e:
        logger.error(str(e))
        sys.exit(-1)
    
    ### Dispatcher ###
    if (not args.outputName.endswith(".txt")):
        logger.warn("Appending '.txt' extension to the output file...")
        parseBags(args.outputName+".txt",args.src_path, args.dst)
    else:
        parseBags(args.outputName,args.src_path,args.dst)
