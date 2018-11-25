package ca.mcgill.ecse211.mountev3rest.sensor;

import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;

/**
 * Uses the robot's front light sensor to determine the color of a given object. The color detection
 * algorithm is limited to a subset of colors that correspond to the rings provided for the ring
 * collection task.
 * <p>
 * The {@code ColorDetector} class uses the following encoding for colors:
 * <ul>
 *  <li>1 = Blue</li>
 *  <li>2 = Green</li>
 *  <li>3 = Yellow</li>
 *  <li>4 = Orange</li>
 *  <li>5 = None</li>
 * </ul>
 * 
 * <p>
 * In addition to simple color classification, the {@code ColorDetection} class provides a set of
 * methods for particular modes of operation. Some of these methods are color detection demo mode
 * and raw red value printing.
 * 
 * @author angelortiz
 *
 */
public class ColorDetector {

  // Constants
  private static final int COLOR_DETECTION_PERIOD = 50;
  private static final int MINIMUM_DETECTIONS = 5;

  // Color mean RGB references
  private static final double[] ORANGE_MEANS = {0.9578, 0.2786, 0.0696};
  private static final double[] BLUE_MEANS = {0.1461, 0.6783, 0.7200};
  private static final double[] YELLOW_MEANS = {0.8221, 0.5516, 0.1406};
  private static final double[] GREEN_MEANS = {0.4180, 0.8995, 0.1266};

  // Attributes
  private TextLCD lcd;
  private LightPoller lightPoller;

  /**
   * Creates a color detector that can display its results into the LCD display of the robot.
   * 
   * @param lcd         {@code TextLCD} representing the display of the robot.
   * @param lightPoller {@code LightPoller} object containing the light sensors of the robot.
   * 
   * @throws PollerException If the light poller has not been instantiated.
   * 
   * @see   LightPoller 
   * @see   TextLCD
   */
  public ColorDetector(TextLCD lcd) throws PollerException {
    this.lcd = lcd;
    this.lightPoller = LightPoller.getLightPoller();
  }

  /**
   * Uses the light sensor to detect the color of an object placed in front of the central light
   * sensor.
   * 
   * @return A number in the range [1, 5] representing the color detected.
   */
  public int getColor() {
    long correctionStart, correctionEnd;

    int[] counters = new int[5];
    int colorDetected = -1;

    while (true) {
      correctionStart = System.currentTimeMillis();
      
      lightPoller.poll();

      // Compute the distances of the readings with respect to the reference means
      boolean blue = computeDistance(BLUE_MEANS, lightPoller.front) < 0.15;
      boolean green = computeDistance(GREEN_MEANS, lightPoller.front) < 0.15;
      boolean yellow = computeDistance(YELLOW_MEANS, lightPoller.front) < 0.15;
      boolean orange = computeDistance(ORANGE_MEANS, lightPoller.front) < 0.15;

      // Determine which color is detected if any
      if (blue) {
        colorDetected = 0;
      } else if (green) {
        colorDetected = 1;
      } else if (yellow) {
        colorDetected = 2;
      } else if (orange) {
        colorDetected = 3;
      } else {
        colorDetected = 4;
      }

      // Adjust the color counts
      for (int i = 0; i < counters.length; i++) {
        if (i == colorDetected) {
          counters[i] = counters[i] > 0 ? counters[i] + 1 : 1;

          // If a color count reaches the MINIMUM_DETECTION constant, that color is returned
          if (counters[i] == MINIMUM_DETECTIONS)
            return i + 1;

        } else {
          counters[i] = 0;
        }
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < COLOR_DETECTION_PERIOD) {
        try {
          Thread.sleep(COLOR_DETECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }

  /**
   * Runs color detection indefinitely. The result of the detection is displayed on the robot's LCD,
   * if no color is detected the screen is set to blank.
   */
  public void demoDetection() {
    long correctionStart, correctionEnd;

    int colorDetected = 5;

    while (true) {
      correctionStart = System.currentTimeMillis();

      colorDetected = getColor();

      lcd.clear();

      switch (colorDetected) {
        case 1:
          lcd.drawString("Object Detected", 0, 0);
          lcd.drawString("Blue", 0, 1);
          break;
        case 2:
          lcd.drawString("Object Detected", 0, 0);
          lcd.drawString("Green", 0, 1);
          break;
        case 3:
          lcd.drawString("Object Detected", 0, 0);
          lcd.drawString("Yellow", 0, 1);
          break;
        case 4:
          lcd.drawString("Object Detected", 0, 0);
          lcd.drawString("Orange", 0, 1);
          break;
        case 5:
          break;
      }

      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < COLOR_DETECTION_PERIOD) {
        try {
          Thread.sleep(COLOR_DETECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
        }
      }
    }
  }

  /**
   * Prints the red value readings from the left and right light sensors into the console on an
   * infinite loop. This method is useful for tuning of the line detection algorithm.
   */
  public void printRed() {
    long correctionStart, correctionEnd;
    
    lightPoller.poll();

    while (true) {
      correctionStart = System.currentTimeMillis();

      double L = lightPoller.leftMean[0];
      double R = lightPoller.rightMean[0];

      System.out.println(String.format("L: %f | R: %f", L, R));

      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < COLOR_DETECTION_PERIOD) {
        try {
          Thread.sleep(COLOR_DETECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
        }
      }
      lightPoller.poll();
    }
  }

  /*
   * Computes the Euclidean distance of the normalized color reading with respect to the stored mean
   * references.
   */
  private double computeDistance(double[] reference, float[] reading) {
    double[] normReading = normalize(reading);
    double ret = 0;

    for (int i = 0; i < reference.length; i++)
      ret += Math.pow((reference[i] - normReading[i]), 2);

    return Math.sqrt(ret);
  }

  /**
   * Normalizes a set of RGB values with respect to each other.
   * 
   * @param  values List containing RGB values.
   * @return        List of normalized RGB values.
   */
  private double[] normalize(float[] values) {
    double[] ret = new double[values.length];
    double geometricMean = 0;

    for (double v : values)
      geometricMean += Math.pow(v, 2);
    geometricMean = Math.sqrt(geometricMean);

    for (int i = 0; i < ret.length; i++)
      ret[i] = values[i] / geometricMean;

    return ret;
  }

}
