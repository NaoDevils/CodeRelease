#!/usr/bin/env python3

import argparse
from utility_functions.loader import loadImages
import pickle
import numpy as np


parser = argparse.ArgumentParser(description='Generate the image database for training etc. '
                                              'using a folder with 0, 1 etc. subfolders with png images.')

parser.add_argument('-b', '--database-path', dest='imgdb_path',
                    help='Path to the image database to write. '
                         'Default is img.db in current folder.')
parser.add_argument('-i', '--image-folder', dest='img_path',
                    help='Path to the folder containing 0, 1 etc. folders with png images. '
                         'Default is current folder.')
parser.add_argument('-r', '--resolution', dest='res',
                    help='Images will be resized to this resolution.')
parser.add_argument('-f', '--flip', dest='flip', action='store_true',
                    help='Rotate images in class 1.') 
parser.add_argument('-t', '--rotate', dest='rotate', action='store_true',
                    help='Flip images in class 1.')
parser.add_argument('-g', '--gain', dest='gain', action='store_true',
                    help='Adjust gain for images in class 1.')    
parser.add_argument('-c', '--color', dest='color', action='store_true',
                    help='Load color images.')                  

args = parser.parse_args()

imgdb_path = "img.db"
img_path = "."
res = {"x": 16, "y": 16}

if args.imgdb_path is not None:
    imgdb_path = args.imgdb_path

if args.img_path is not None:
    img_path = args.img_path

if args.res is not None:
    res = {"x": int(args.res), "y": int(args.res)}
    
with open(imgdb_path, "wb") as f:
    x, y, mean, p = loadImages(img_path, res, 2, args.rotate, args.flip, args.gain, args.color)
    pickle.dump(mean, f)
    pickle.dump(y, f)
    pickle.dump(p, f)

with open(imgdb_path+".x", "wb") as f:
    np.save(f, x)