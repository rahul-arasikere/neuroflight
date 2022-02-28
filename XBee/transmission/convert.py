import tensorflow as tf

with open("actor.tflite", "wb") as f:
    f.write(tf.lite.TFLiteConverter.from_saved_model("actor").convert())
