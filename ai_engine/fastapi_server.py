import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0"

import uuid
import base64
import numpy as np
import cv2
from collections import deque
from fastapi import FastAPI
from pydantic import BaseModel
import tensorflow as tf
from tensorflow.keras.models import load_model

gpus = tf.config.list_physical_devices('GPU')
if gpus:
    tf.config.experimental.set_memory_growth(gpus[0], True)
    print(f"GPU enabled: {gpus[0].name}")
else:
    print("WARNING: No GPU found, running on CPU")

IMG_SIZE        = 32          
N_FRAMES        = 4           
FIRE_THRESHOLD  = 0.5         
MIN_FIRE_PIXELS = 10          
OUTPUT_DIR      = "outputs/masks"
os.makedirs(OUTPUT_DIR, exist_ok=True)

bce = tf.keras.losses.BinaryCrossentropy()

def dice_coef(y_true, y_pred, smooth=1e-7):
    y_true_f = tf.reshape(y_true, [-1])
    y_pred_f = tf.reshape(y_pred, [-1])
    intersection = tf.reduce_sum(y_true_f * y_pred_f)
    return (2. * intersection + smooth) / (
        tf.reduce_sum(y_true_f) + tf.reduce_sum(y_pred_f) + smooth
    )

def bce_dice_loss(y_true, y_pred):
    return bce(y_true, y_pred) + (1.0 - dice_coef(y_true, y_pred))

model = load_model(
    "Gan_generator_model.keras",
    custom_objects={
        "bce_dice_loss": bce_dice_loss,
        "dice_coef":     dice_coef
    },
    compile=False
)
model.trainable = False
print(f"Model loaded — input: {model.input_shape}, output: {model.output_shape}")

frame_buffer: deque = deque(maxlen=N_FRAMES)

app = FastAPI(title="Fire Detection Inference Server")

class ImageInput(BaseModel):
    image: str   

@app.post("/infer")
def infer(data: ImageInput):
    try:
        img_bytes = base64.b64decode(data.image)
        np_arr    = np.frombuffer(img_bytes, np.uint8)
        frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except Exception as e:
        return {"status": "error", "message": f"Image decode failed: {e}"}

    if frame_bgr is None:
        return {"status": "error", "message": "cv2.imdecode returned None — invalid image bytes"}

 
    frame_resized = cv2.resize(frame_bgr, (IMG_SIZE, IMG_SIZE))

 
    frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

 
    frame_norm = frame_rgb.astype(np.float32) / 127.5 - 1.0   # (32, 32, 3)
    frame_buffer.append(frame_norm)

    if len(frame_buffer) < N_FRAMES:
        return {
            "status":        "buffering",
            "message":       f"Collecting frames: {len(frame_buffer)}/{N_FRAMES}",
            "fire_detected": False
        }
    stacked = np.concatenate(list(frame_buffer), axis=-1)
    model_input = np.expand_dims(stacked, axis=0)

    raw_output = model(
        {"input_layer_3": model_input},
        training=False
    ).numpy()[0]   # (32, 32, 1)

    mask = (raw_output[:, :, 0] + 1.0) / 2.0  if raw_output.min() < 0 \
           else raw_output[:, :, 0]            # (32, 32) float [0, 1]

    binary_mask = (mask > FIRE_THRESHOLD).astype(np.uint8)   # (32, 32) uint8

    # Fire detection decision
    fire_pixel_count = int(np.sum(binary_mask))
    fire_detected    = fire_pixel_count >= MIN_FIRE_PIXELS

    # Fire coverage as percentage of frame
    fire_coverage_pct = round(float(fire_pixel_count) / (IMG_SIZE * IMG_SIZE) * 100, 2)

    # Average confidence over fire pixels
    if fire_pixel_count > 0:
        confidence = float(np.mean(mask[binary_mask == 1]))
    else:
        confidence = 0.0

    # Save binary mask as PNG for debugging (optional)
    filename    = f"{uuid.uuid4().hex}.png"
    filepath    = os.path.join(OUTPUT_DIR, filename)
    mask_visual = (binary_mask * 255).astype(np.uint8)
    cv2.imwrite(filepath, mask_visual)

    return {
        "status":           "success",
        "fire_detected":    fire_detected,
        "fire_pixel_count": fire_pixel_count,
        "fire_coverage_pct": fire_coverage_pct,
        "confidence":       round(confidence, 4),
        "mask_file":        filename
    }


@app.get("/status")
def status():
    return {
        "model_input":    str(model.input_shape),
        "model_output":   str(model.output_shape),
        "buffer_size":    f"{len(frame_buffer)}/{N_FRAMES}",
        "gpu_available":  len(gpus) > 0
    }
