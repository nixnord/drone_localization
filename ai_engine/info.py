import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import tensorflow as tf
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

print("=== INPUT ===")
print("Shape      :", model.input_shape)
print("Names      :", [inp.name for inp in model.inputs])

print("\n=== OUTPUT ===")
print("Shape      :", model.output_shape)
print("Names      :", [out.name for out in model.outputs])

print("\n=== FIRST LAYER ===")
first = model.layers[0]
print("Type       :", type(first).__name__)
print("Config     :", first.get_config())

print("\n=== ALL LAYER NAMES ===")
for i, layer in enumerate(model.layers):
    try:
        in_shape  = str(layer.input_shape)
    except AttributeError:
        in_shape  = "N/A"
    try:
        out_shape = str(layer.output_shape)
    except AttributeError:
        out_shape = "N/A"
    print(f"  [{i:03d}] {type(layer).__name__:<25} in={in_shape:<30} out={out_shape:<30} name={layer.name}")