package ca.mcgill.ecse211.mountev3rest.sensor;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;


/**
 * Provides an interface for accessing the light readings of two light sensors while running on a
 * single thread.
 * <p>
 * The class is implemented as a singleton to facilitate access to the sensors to different parts of
 * the code. Additionally, the {@code LightPoller} class also provides line detection for the line
 * and right sensors.
 * 
 * @author angelortiz
 *
 */
public class LightPoller {

  // Constants
  private static final double LINE_COLOR_VALUE = 0.30; // Minimum value required to treat a sensor
                                                       // reading as a line

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
  public float[] front;
  public float[] left;
  public float[] right;
  public float[] frontMean;
  public float[] leftMean;
  public float[] rightMean;

  // Public attributes to access line detection results for line and right sensors
  public boolean leftInLine;
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
    frontFilter = new MeanFilter(frontProvider, 4);
    front = new float[frontProvider.sampleSize()];
    frontMean = new float[frontFilter.sampleSize()];

    // Initialize left sensor values
    leftProvider = leftSensor.getMode("Red");
    leftFilter = new MeanFilter(leftProvider, 3);
    left = new float[leftProvider.sampleSize()];
    leftMean = new float[leftFilter.sampleSize()];

    // Initialize right sensor values
    rightProvider = rightSensor.getMode("Red");
    rightFilter = new MeanFilter(rightProvider, 3);
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
    if (leftMean[0] < LINE_COLOR_VALUE) {
      leftInLine = true;
    } else {
      leftInLine = false;
    }

    if (rightMean[0] < LINE_COLOR_VALUE) {
      rightInLine = true;
    } else {
      rightInLine = false;
    }
  }

}
