# -*- coding: utf-8 -*-
import  os, sys, random, shutil
import subprocess, yaml, json
import argparse
from collections import defaultdict, Counter
import logging
import tarfile

try:
    import colorlog
    have_colorlog = True
except ImportError:
    have_colorlog = False
    
def readArgs():
    """ Deals with the argments"""
    parser = argparse.ArgumentParser(description='Process some tar files.') 
    
    parser.add_argument('--path', dest='src_path', action='store',
                        default=os.path.dirname(os.path.abspath(__file__)),
                        help='the folder containing the bag files. (default: the script current path)')
    parser.add_argument('--folder', dest='dst', action='store',
                        default=os.path.dirname(os.path.abspath(__file__)) + "/result",
                        help='the folder name for storing the summary file')
    return parser.parse_args()
                        
def untar(fname):
    if (fname.endswith("tar.gz")):
        tar = tarfile.open(fname)
        tar.extractall()
        tar.close()
        logger.info("Extracted in Current Directory")
    else:
        logger.error("Not a tar.gz file: '" + fname + "'")

def parseTars(tarsPath, outputPath):
    """Loops through the bag files and save info to outputName file.
    parameters:
        outputName - the name of the file where the info is saved to.
        bagsPath   - the path to the folder containing bag files.
        outputPath - the folder location where to save the outputName.
    return:
        void
    """
    
    logger.info("Analysing current path, searching for files...")
    mypath = tarsPath
    tarfiles = [f for f in os.listdir(mypath) if os.path.isfile(os.path.join(mypath, f)) and f.endswith('.tar.gz')]

    nfiles = len(tarfiles)

    if nfiles == 0:
        logger.error("No '.tar.gz' files found!")
        sys.exit(0)

    logger.info('A number of ' + str(nfiles) + " '.tar.gz' files were found")
    
    for tar in tarfiles:
        logger.info("Processing file: " + f)
        untar(tar)
        bagfiles = [f for f in os.listdir(mypath) if os.path.isfile(os.path.join(mypath, f)) and f.endswith('.bag')]
        for bag in bagfiles:
            try:
                new_bag_name = tar.strip(".tar.gz")  + "_" + bag
                logger.info("Renaming '" + bag + "' file...")
                os.rename(mypath + "/" + bag, mypath + "/" + new_bag_name)
                logger.info("'"+bag + "' file renamed to '" + new_bag_name + "'") 
                logger.info("moving '" + new_bag_name + "' to " +outputPath)
                shutil.move(mypath + "/" + new_bag_name, outputPath)
                logger.info("done...")
            except Exception, e:
                logger.error(str(e))
                sys.exit(-1)

 
if __name__ == '__main__':
    
    args = readArgs()
    
    # create logger
    logger = logging.getLogger('tar_files')
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
    
    parseTars(args.src_path,args.dst)
    
