/*
 * Copyright (C) 2021 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.robotcore.internal.tfod;

import android.graphics.Rect;
import android.graphics.RectF;
import android.util.Log;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.tensorflow.lite.Interpreter;

/**
 * Subclass of RecognizeImageRunnable that uses {@link Interpreter}.
 *
 * @author Vasu Agrawal
 * @author Liz Looney
 */
class RecognizeImageRunnable1 extends RecognizeImageRunnable {

  // Constants for float model, used to normalize float images to [-1, 1]
  private static final float IMAGE_MEAN = 128.0f;
  private static final float IMAGE_STD = 128.0f;

  private final Interpreter interpreter;

  // Where to store the output from the network. These arrays get combined into a single object
  // and fed to the interpreter, which fills them with model inference output.
  /**
   * For each recognition, store an array holding the [top, left, bottom, right] normalized values.
   * Values are normalized to be between [0, 1] for each value. Note that this is a different order
   * than what gets passed into the RectF constructor.
   *
   * <p>Size: [1][MAX_NUM_DETECTIONS][4]
   */
  private final float[][][] outputLocations;

  /**
   * For each recognition, store a single number corresponding to the output class id. As mentioned
   * above, the label offset is necessary to convert between the values in this array and the
   * indexes for the list of class labels. Typically, the values here will be between [0,
   * NUM_CLASSES), but the model will occasionally yield values outside this range, and thus these
   * values must be bounds checked.
   *
   * <p>Size: [1][MAX_NUM_DETECTIONS]
   */
  private final float[][] outputClasses;

  /**
   * For each recognition, output a score. The scores can be roughly interpreted as a confidence, in
   * that the network assigns a higher value here to a recognition it is more confident in, but
   * these scores should not be interpreted as probabilities as they are not constrained to the
   * range [0, 1] and can (and do) exceed 1.
   *
   * <p>Size: [1][MAX_NUM_DETECTIONS]
   */
  private final float[][] outputScores;

  /**
   * For each set of recognitions, output the total number of recognitions made (at any confidence).
   * This number seems to always be 10.0, which is the number of detections programmed into the
   * network during the TOCO (TensorFlow Optimizing COnverter) conversion step. With custom
   * detection models this field may hold more useful values.
   *
   * <p>Size: [1]
   */
  private final float[] numDetections;

  RecognizeImageRunnable1(
      AnnotatedYuvRgbFrame annotatedFrame,
      CameraInformation cameraInformation,
      Interpreter interpreter,
      TfodParameters params,
      double zoomMagnification,
      double zoomAspectRatio,
      List<String> labels,
      float[][][] outputLocations,
      float[][] outputClasses,
      float[][] outputScores,
      float[] numDetections,
      AnnotatedFrameCallback callback) {
    super(annotatedFrame, cameraInformation, params, zoomMagnification, zoomAspectRatio, labels,
        callback);
    this.interpreter = interpreter;
    this.outputLocations = outputLocations;
    this.outputClasses = outputClasses;
    this.outputScores = outputScores;
    this.numDetections = numDetections;
  }

  @Override
  protected void processFrame() {
    Timer timer = new Timer(annotatedFrame.getTag());
    timer.start("RecognizeImageRunnable1.preprocessFrame");
    ByteBuffer imgData = preprocessFrame();
    timer.end();

    // Generate the input array and output map to feed the TensorFlow Lite interpreter
    timer.start("Interpreter.runForMultipleInputsOutputs");
    Object[] inputArray = {imgData};
    Map<Integer, Object> outputMap = new HashMap<>();
    outputMap.put(0, outputLocations);
    outputMap.put(1, outputClasses);
    outputMap.put(2, outputScores);
    outputMap.put(3, numDetections);
    interpreter.runForMultipleInputsOutputs(inputArray, outputMap);
    timer.end();

    // Postprocess detections
    timer.start("RecognizeImageRunnable1.postprocessDetections");
    postprocessDetections();
    timer.end();
  }

  private ByteBuffer preprocessFrame() {
    int[] rgbArray = annotatedFrame.getFrame().getArgb8888Array(
        params.inputSize, zoomMagnification, zoomAspectRatio);

    // Allocate the ByteBuffer to be passed as the input to the network
    int numBytesPerChannel = params.isModelQuantized ? 1 /* Quantized */ : 4 /* Floating Point */;
    // capacity = (Width) * (Height) * (Channels) * (Bytes Per Channel)
    int capacity = params.inputSize * params.inputSize * 3 * numBytesPerChannel;
    ByteBuffer imgData = ByteBuffer.allocateDirect(capacity);
    imgData.order(ByteOrder.nativeOrder());
    imgData.rewind();

    // Copy the data into the ByteBuffer
    for (int i = 0; i < params.inputSize; ++i) {
      for (int j = 0; j < params.inputSize; ++j) {
        int pixelValue = rgbArray[i * params.inputSize + j];
        if (params.isModelQuantized) { // Quantized model
          // Copy as-is
          imgData.put((byte) ((pixelValue >> 16) & 0xFF));
          imgData.put((byte) ((pixelValue >> 8) & 0xFF));
          imgData.put((byte) (pixelValue & 0xFF));
        } else { // Float model
          // Copy with normalization
          imgData.putFloat((((pixelValue >> 16) & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
          imgData.putFloat((((pixelValue >> 8) & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
          imgData.putFloat(((pixelValue & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
        }
      }
    }

    return imgData;
  }

  private void postprocessDetections() {

    //Log.i(annotatedFrame.getTag(), "RecognizeImageRunnable1.postprocessDetections - received " + numDetections[0] + " detections!");

    for (int i = 0; i < params.maxNumDetections; i++) {

      // The network will always generate MAX_NUM_DETECTIONS results.
      final float detectionScore = outputScores[0][i];
      if (detectionScore < params.minResultConfidence) {
        continue;
      }

      // First, determine the label, and make sure it is within bounds.
      int detectedClass = (int) outputClasses[0][i];

      // We've observed that the detections can some times be out of bounds (potentially when the
      // network isn't confident enough in any positive results) so this keeps the labels in bounds.
      if (detectedClass >= labels.size() || detectedClass < 0) {
        Log.w(annotatedFrame.getTag(), "RecognizeImageRunnable1.postprocessDetections - got a recognition with an invalid / background label");
        continue;
      }

      RectF detectionBox = convertOutputLocationToDetectionBox(outputLocations[0][i]);

      annotatedFrame.getRecognitions().add(
          new RecognitionImpl(cameraInformation, labels.get(detectedClass), detectionScore, detectionBox));
    }
  }

  private RectF convertOutputLocationToDetectionBox(float[] outputLocation) {
    // Return the rectangles in the coordinates of the bitmap we were originally given.
    // Note that this conversion switches the indices from what is returned by the model to
    // what is expected by RectF. The model gives [top, left, bottom, right], while RectF
    // expects [left, top, right, bottom].
    int frameWidth = annotatedFrame.getFrame().getWidth();
    int frameHeight = annotatedFrame.getFrame().getHeight();

    if (Zoom.isZoomed(zoomMagnification)) {
      Rect zoomArea = Zoom.getZoomArea(zoomMagnification, zoomAspectRatio, frameWidth, frameHeight);
      return new RectF(
          outputLocation[1] * zoomArea.width() + zoomArea.left,
          outputLocation[0] * zoomArea.height() + zoomArea.top,
          outputLocation[3] * zoomArea.width() + zoomArea.left,
          outputLocation[2] * zoomArea.height() + zoomArea.top);
    }

    return new RectF(
        outputLocation[1] * frameWidth,
        outputLocation[0] * frameHeight,
        outputLocation[3] * frameWidth,
        outputLocation[2] * frameHeight);
  }
}
