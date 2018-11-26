package ca.mcgill.ecse211.mountev3rest.sensor;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;


/**
 * Provides an interface for accessing the light readings of three light sensors on request.
 * <p>
 * The class is implemented as a singleton to facilitate access to the sensors to different parts of
 * the code. Additionally, the {@code LightPoller} class also provides line detection for the left
 * and right sensors.
 * 
 * @author angelortiz
 *
 */
public class LightPoller {

  // Constants
  private static final double LINE_COLOR_VALUE_LEFT = 0.29; // Minimum value required to treat a sensor
                                                       // reading as a line
  private static final double LINE_COLOR_VALUE_RIGHT = 0.26;
  
  private static final int MEAN_SIZE = 1;

  // Attributes
  // Singleton instance
  private static LightPoller lightPoller = null;

  // SampleProvider and MeanFilter objects
  private SampleProvider frontProvider;
  private SampleProvider leftProvider;
  private SampleProvider rightProvider;
  private MeanFilter frontFilter;
  private MeanFilter leftFilter;
  private MeanFilter rightFilter;

  // Public attributes used to access the sensors' readings
  
  /**
   * Front light sensor reading. Since this sensor is in {@code RGB} mode this is a three element array with
   * the corresponding values.
   */
  public float[] front;
  /**
   * Left light sensor reading. This is a one element sample containing a {@code Red} value only.
   */
  public float[] left;
  /**
   * Right light sensor reading. This is a one element sample containing a {@code Red} value only.
   */
  public float[] right;
  /**
   * Mean of the last three readings of the front light sensor.
   */
  public float[] frontMean;
  /**
   * Mean of the last three readings of the left light sensor.
   */
  public float[] leftMean;
  /**
   * Mean of the last three readings of the right light sensor.
   */
  public float[] rightMean;

  // Public attributes to access line detection results for line and right sensors
  
  /**
   * Indicates whether the left light sensor is currently seeing a line.
   */
  public boolean leftInLine;
  /**
   * Indicates whether the right light sensor is currently seeing a line.
   */
  public boolean rightInLine;

  /**
   * Creates a light poller that uses the provided sensors for polling.
   * 
   * @param frontSensor Light sensor placed on the robot arm, set to {@code RGB} mode.
   * @param leftSensor Light sensor placed to the left of the robot's center, set to {@code Red}
   *        mode.
   * @param rightSensor Light sensor placed to the right of the robot's center, set to {@code Red}
   *        mode.
   */
  private LightPoller(EV3ColorSensor frontSensor, EV3ColorSensor leftSensor,
      EV3ColorSensor rightSensor) {
    // Initialize front sensor values
    frontProvider = frontSensor.getMode("RGB");
    frontFilter = new MeanFilter(frontProvider, MEAN_SIZE);
    front = new float[frontProvider.sampleSize()];
    frontMean = new float[frontFilter.sampleSize()];

    // Initialize left sensor values
    leftProvider = leftSensor.getMode("Red");
    leftFilter = new MeanFilter(leftProvider, MEAN_SIZE);
    left = new float[leftProvider.sampleSize()];
    leftMean = new float[leftFilter.sampleSize()];

    // Initialize right sensor values
    rightProvider = rightSensor.getMode("Red");
    rightFilter = new MeanFilter(rightProvider, MEAN_SIZE);
    right = new float[rightProvider.sampleSize()];
    rightMean = new float[rightFilter.sampleSize()];
  }

  /**
   * Creates a {@code LightPoller} instance if the class has not been instantiated, otherwise
   * returns the existing instance. This method is the factory method for the singleton
   * {@code LightPoller} class.
   * 
   * @param frontSensor Light sensor placed on the robot arm, set to {@code RGB} mode.
   * @param leftSensor Light sensor placed to the left of the robot's center, set to {@code Red}
   *        mode.
   * @param rightSensor Light sensor placed to the right of the robot's center, set to {@code Red}
   *        mode.
   * 
   * @return New or existing instance of the {@code LightPoller} object.
   */
  public static LightPoller getLightPoller(EV3ColorSensor frontSensor, EV3ColorSensor leftSensor,
      EV3ColorSensor rightSensor) {
    if (lightPoller == null) {
      lightPoller = new LightPoller(frontSensor, leftSensor, rightSensor);
      return lightPoller;
    } else {
      return lightPoller;
    }
  }

  /**
   * Factory method for existing instance of the singleton {@code LightPoller} class.
   * 
   * @return Existing {@code LightPoller} object.
   * @throws PollerException If the {@code LightPoller} class has not been instantiated.
   */
  public static LightPoller getLightPoller() throws PollerException {
    if (lightPoller == null)
      throw new PollerException("The singleton LightPoller class has not been instantiated yet."
          + "Three EV3ColorSensor instances must be provided to the factory method.");
    else
      return lightPoller;
  }

  /*
   * Updates the light sensor readings and checks for any line detections.
   */
  public void poll() {
    // Front sensor
    frontProvider.fetchSample(front, 0);
    frontFilter.fetchSample(frontMean, 0);

    // Left sensor
    leftProvider.fetchSample(left, 0);
    leftFilter.fetchSample(leftMean, 0);

    // Right sensor
    rightProvider.fetchSample(right, 0);
    rightFilter.fetchSample(rightMean, 0);

    // Update line detection values
    lineDetection();
  }

  /**
   * Updates the line detection values for the line sensor.
   */
  private void lineDetection() {
    if (leftMean[0] < LINE_COLOR_VALUE_LEFT) {
      if (!leftInLine)
      leftInLine = true;
    } else {
      leftInLine = false;
    }

    if (rightMean[0] < LINE_COLOR_VALUE_RIGHT) {
      rightInLine = true;
    } else {
      rightInLine = false;
    }
  }

}
