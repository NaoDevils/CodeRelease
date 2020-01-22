import random
from glob import glob
import cv2
import numpy as np
from keras.utils.np_utils import to_categorical
import imutils
import os
import pickle


def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


def loadImage(path, res):
    img = cv2.imread(path, 0) / 255
    return cv2.resize(img, (res["x"], res["y"]))


def getDBPath(path, clss, filetype):
    return glob(path + "/**/" + str(clss) + "/*." + filetype) + glob(path + "/" + str(clss) + "/*." + filetype)

def load_imdb(imgdb_path):
    print("Loading imdb...")
    images = {}
    with open(imgdb_path, "rb") as f:
        images["mean"] = pickle.load(f)
        images["y"] = pickle.load(f)
        images["p"] = pickle.load(f)
    with open(imgdb_path + '.x', "rb") as f:
        images["images"] = np.load(f)
    return images

def loadImages(path, res, num_classes, rotate=True, flip=True, gain=True, color=False):
    db = []
    print("Loading imgs...")
    num = np.zeros(num_classes)
    img_dir = os.path.join(path)
    for clss in range(num_classes):
        for root, dirs, files in os.walk(img_dir):
            if (os.path.basename(root).endswith('0') and clss != 0):
                continue
            if (os.path.basename(root).endswith('1') and clss != 1):
                continue
            if (not os.path.basename(root).endswith('0') and not os.path.basename(root).endswith('1')):
                continue
            for file in files:
                if (file.endswith('.png') or file.endswith('.jpg')):
                    num[clss] += 1
                    print("\033[1K\rLoading " + root + "... (" + str(len(db)) + ")", end="")
                    p = os.path.join(root, file)
                    if color:
                      img = cv2.imread(p)
                    else:
                      img = cv2.imread(p, 0)
                    try:
                        img = cv2.resize(img, (res["x"], res["y"]))
                    except:
                        print("Error loading image: " + file + " in " + root)
                    db.append((img / 255, to_categorical(clss, num_classes), p))
                    if clss == 1:  # Sorry, hard coded for now
                        if rotate:
                            for a in (90, 180, 270):
                                db.append((imutils.rotate(img / 255, a), to_categorical(clss, num_classes), p))
                        if flip:
                            db.append((cv2.flip(img, 1) / 255, to_categorical(clss, num_classes), p))
                        if gain:
                            for g in (0.4, 1.3):
                                db.append((adjust_gamma(img, g) / 255, to_categorical(clss, num_classes), p))
    if len(db) == 0:
        print("No images loaded, exiting.")
        exit(1)
    random.shuffle(db)
    x, y, p = list(map(np.array, list(zip(*db))))
    mean = np.mean(x)
    x -= mean
    if color:
        x = x.reshape(*x.shape)
    else:
        x = x.reshape(*x.shape, 1)
    print("Loading finished")
    print("Ball images: " + str(num[1]))
    print("Other images: " + str(num[0]))
    return x, y, mean, p