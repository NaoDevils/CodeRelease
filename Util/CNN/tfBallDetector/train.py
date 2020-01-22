#!/usr/bin/env python3

import argparse
import pickle
from keras.layers import Flatten, MaxPooling2D, Convolution2D, Dropout, LeakyReLU
from keras.models import Sequential
from keras.backend import get_session
import tensorflow as tf
import numpy as np
from utility_functions.keras_weighted_categorical_crossentropy import weighted_categorical_crossentropy
from keras.utils import multi_gpu_model

parser = argparse.ArgumentParser(description='Train the network given ')

parser.add_argument('-b', '--database-path', dest='imgdb_path',
                    help='Path to the image database to use for training. '
                         'Default is img.db in current folder.')
parser.add_argument('-m', '--model-path', dest='model_path',
                    help='Store the trained model using this path. Default is model.h5.')
parser.add_argument('-t', '--tfcompile', dest='tfcompile', action='store_true',
                    help='Export trained model to be use by tfcompile.')

args = parser.parse_args()

imgdb_path = "img.db"
model_path = "model.h5"

if args.imgdb_path is not None:
    imgdb_path = args.imgdb_path

if args.model_path is not None:
    model_path = args.model_path

with open(imgdb_path, "rb") as f:
    pickle.load(f)  # Ignore image mean
    y = pickle.load(f)

with open(imgdb_path + '.x', "rb") as f:
     x = np.load(f)

# The keras network. Adapt to your needs.

usual_model = Sequential()
usual_model.add(Convolution2D(4, (3, 3), input_shape=(x.shape[1], x.shape[2], 1),
                        activation='relu', padding='same'))
usual_model.add(MaxPooling2D(pool_size=(2, 2)))
usual_model.add(Convolution2D(8, (3, 3), padding='same', activation='relu'))
usual_model.add(MaxPooling2D(pool_size=(2, 2)))
usual_model.add(Convolution2D(16, (3, 3), padding='same', activation='relu',))
usual_model.add(MaxPooling2D(pool_size=(2, 2)))
usual_model.add(Dropout(0.4))
usual_model.add(Convolution2D(2, (2, 2), activation='softmax'))
usual_model.add(Flatten())

quantize_model = Sequential()
quantize_model.add(Convolution2D(16, (3, 3), input_shape=(x.shape[1], x.shape[2], 1),
                                 activation='relu', padding='same'))
quantize_model.add(MaxPooling2D(pool_size=(2, 2)))
quantize_model.add(Convolution2D(32, (3, 3), padding='same', activation='relu'))
quantize_model.add(MaxPooling2D(pool_size=(2, 2)))
quantize_model.add(Dropout(0.2))
quantize_model.add(Convolution2D(2, (4, 4), activation='softmax'))
quantize_model.add(Flatten())

# Select the current model here
model = quantize_model

# Do this if you want to train on multiple GPUs
#model = multi_gpu_model(qtest_model, gpus=4)

# Select your loss function here
#current_loss = 'categorical_crossentropy' # This one is the classic one
current_loss = weighted_categorical_crossentropy((0.999, 0.001)) # This one produces near zero false positives

model.compile(loss=current_loss, optimizer='adam', metrics=['accuracy'])
print(model.summary())
model.fit(x, y, batch_size=1000, epochs=200, verbose=1, validation_split=0.05)
model.save(model_path)

if args.tfcompile:
    # Freezing and exporting graph as described in:
    # https://gist.github.com/carlthome/6ae8a570e21069c60708017e3f96c9fd
    # We then can use tfcompile
    model.input.set_shape([1] + list(model.input.shape)[1:])
    session = get_session()
    output_node_names = [node.op.name for node in model.outputs]
    graphdef = tf.graph_util.convert_variables_to_constants(session, session.graph.as_graph_def(), output_node_names)
    tf.train.write_graph(session.graph.as_graph_def(), ".",
                         "graph.pbtxt", as_text=True)
    tf.train.write_graph(graphdef, '.', 'graph.pb', as_text=False)