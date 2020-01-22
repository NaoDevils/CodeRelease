#!/usr/bin/env python3

import argparse
from utility_functions.ocg import keras_compile
from utility_functions.loader import load_imdb
import time

parser = argparse.ArgumentParser(description='Train the network given ')

parser.add_argument('-b', '--database-path', dest='imgdb_path',
                    help='Path to the image database to use for training. '
                         'Default is img.db in current folder.')
parser.add_argument('-m', '--model-path', dest='model_path',
                    help='Store the trained model using this path. Default is model.h5.')
parser.add_argument('-c', '--code-path', dest='code_path',
                    help='Path where the file is to be stored. Default is current directory')
parser.add_argument('-i', '--identifier', dest='identifier',
                    help='A unique identifier for C function name and file name. Default is date and time.')

args = parser.parse_args()

imgdb_path = "img.db"
model_path = "model.h5"
code_path  = "."
identifier = time.strftime("%H%M%S_%d%m%Y")

if args.imgdb_path is not None:
    imgdb_path = args.imgdb_path

if args.model_path is not None:
    model_path = args.model_path

if args.code_path is not None:
    code_path = args.code_path

if args.identifier is not None:
    identifier = args.identifier

images = load_imdb(imgdb_path)
keras_compile(images, model_path, code_path, identifier, unroll_level=2, arch="ssse3", quantize=True)