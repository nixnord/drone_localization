import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import tensorflow as tf
import numpy as np
import time

gpus = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpus[0], True)

from tensorflow.keras.models import load_model

bce = tf.keras.losses.BinaryCrossentropy()

def dice_coef(y_true, y_pred, smooth=1e-7):
    y_true_f = tf.reshape(y_true, [-1])
    y_pred_f = tf.reshape(y_pred, [-1])
    intersection = tf.reduce_sum(y_true_f * y_pred_f)
    return (2. * intersection + smooth) / (tf.reduce_sum(y_true_f) + tf.reduce_sum(y_pred_f) + smooth)

def bce_dice_loss(y_true, y_pred):
    return bce(y_true, y_pred) + (1.0 - dice_coef(y_true, y_pred))

model = load_model(
    "Gan_generator_model.keras",
    custom_objects={"bce_dice_loss": bce_dice_loss, "dice_coef": dice_coef},
    compile=False
)
model.trainable = False
dummy = np.random.rand(1, 32, 32, 12).astype(np.float32) * 2 - 1

for _ in range(3):
    output = model(dummy, training=False).numpy()[0]

start = time.perf_counter()
for _ in range(50):
    model(dummy, training=False).numpy()[0]
elapsed = (time.perf_counter() - start) / 50

print(f"Avg inference time : {elapsed * 1000:.1f} ms")
print(f"Max sustainable rate: {1 / elapsed:.1f} Hz")