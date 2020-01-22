#!/usr/bin/env python3

from keras.layers import Flatten, MaxPooling2D, Convolution2D, Dropout, LeakyReLU, BatchNormalization
from keras.models import Sequential
from utility_functions.ocg import keras_compile
import numpy as np

input_shape = (160, 120, 3)


def create_random_weights(shape):
    r = np.random.random_sample()
    return np.ones(shape, dtype=np.float32) * r


def create_batch_normalization_weights(shape):
    return [create_random_weights(shape), create_random_weights(shape), create_random_weights(shape),
            create_random_weights(shape)]


model = Sequential()
model.add(Convolution2D(12, (3, 3), input_shape=input_shape, padding='same'))
model.add(BatchNormalization(weights=create_batch_normalization_weights(model.layers[-1].filters)))
model.add(LeakyReLU(alpha=0.0))  # alpha unknown, so default

model.add(Convolution2D(16, (3, 3), padding='same'))
model.add(BatchNormalization(weights=create_batch_normalization_weights(model.layers[-1].filters)))
model.add(LeakyReLU())
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Convolution2D(24, (3, 3), padding='same'))
model.add(BatchNormalization(weights=create_batch_normalization_weights(model.layers[-1].filters)))
model.add(LeakyReLU())
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Convolution2D(32, (3, 3), padding='same'))
model.add(BatchNormalization(weights=create_batch_normalization_weights(model.layers[-1].filters)))
model.add(LeakyReLU(alpha=0.0))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Convolution2D(32, (3, 3), padding='same'))
model.add(BatchNormalization(weights=create_batch_normalization_weights(model.layers[-1].filters)))
model.add(LeakyReLU())

model.add(Convolution2D(6, (1, 1), padding='same'))

model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

model.summary()
#model.fit(x, y, batch_size=1000, epochs=1, verbose=1, validation_split=0.05)
model.save("yolo.h5")

np.random.seed(42)
array = np.random.randint(0, 255+1, size=input_shape).astype(np.float32)

test_img_db = {'images': [array], 'mean': 0.0}  # Replace this by real images
# keras_compile(test_img_db, "yolo.h5", "yolo.c", unroll_level=2, arch="sse3")
keras_compile(test_img_db, "yolo.h5", "yolo.c", unroll_level=2, arch="sse3")
#keras_compile(test_img_db, "yolo.h5", "yolo_mode_1.c", unroll_level=2, arch="sse3", conv_mode=1)
#keras_compile(test_img_db, "yolo.h5", "yolo_mode_2.c", unroll_level=2, arch="sse3", conv_mode=2)