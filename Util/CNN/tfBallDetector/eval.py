import numpy as np
from keras.models import load_model
import pickle
import cv2

imgs = [
# path to imgs to be predicted
]

with open( "img.db", "rb" ) as f:
    mean = pickle.load(f)

for img in imgs:
    im = cv2.imread(img)
    im = cv2.resize(im, (64, 64))
    im = im /  255
    im -= mean
    if 'x' in locals():
        x = np.append(x, im.reshape(1, 64, 64, 3), axis=0)
    else:
        x = im.reshape(1, 64, 64, 3)
model = load_model('modell.h5')
print(model.predict(x))