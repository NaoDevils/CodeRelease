import argparse
from keras.models import load_model
from utility_functions.loader import load_imdb
import numpy as np
from utility_functions.stats import save_false_positives

imgdb_path = "img.db"
model_path = "model.h5"

parser = argparse.ArgumentParser(description='Eval false positives etc.')
parser.add_argument('-b', '--database-path', dest='imgdb_path',
                    help='Path to the image database to use for training. '
                         'Default is img.db in current folder.')
parser.add_argument('-m', '--model-path', dest='model_path',
                    help='Store the trained model using this path. Default is model.h5.')

args = parser.parse_args()

if args.imgdb_path is not None:
    imgdb_path = args.imgdb_path

if args.model_path is not None:
    model_path = args.model_path

images = load_imdb(imgdb_path)
model = load_model(model_path, compile=False)

pred = model.predict(images["images"])
thresh = 0.5
y = images["y"]
x = images["images"]

pred_ball = np.array(pred[:, 1] >= thresh)
gt_ball = np.array(y[:, 1] == 1)
gt_noball = np.array(y[:, 1] == 0)
accuracy = list(np.equal(pred_ball, gt_ball)).count(True) / y.shape[0]

gt_ball_idx = list(np.where(y[:, 1] == 1)[0])
gt_noball_idx = list(np.where(y[:, 1] == 0)[0])
TruePositive = list(pred_ball[gt_ball_idx]).count(True)
FalsePositive = list(pred_ball[gt_noball_idx]).count(True)
TrueNegative = list(pred_ball[gt_noball_idx]).count(False)
FalseNegative = list(pred_ball[gt_ball_idx]).count(False)

print(TruePositive, FalsePositive, TrueNegative, FalseNegative)

save_false_positives(pred, x, y, images["p"], images["mean"])