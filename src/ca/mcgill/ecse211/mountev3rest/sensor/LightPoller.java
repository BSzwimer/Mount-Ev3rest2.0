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
public class LightPoller extends Thread {

  // Constants
  private static final double LINE_COLOR_VALUE = 0.30; // Minimum value required to treat a sensor
                                                       // reading as a line

  // Attributes
  // Singleton instance
  private static LightPoller lightPoller = null;

  // SampleProvider and MeanFilter objects
  private SampleProvider frontProvider;
  private SampleProvider lineProvider;
  private MeanFilter frontFilter;
  private MeanFilter lineFilter;

  // Public attributes used to access the sensors' readings
  public float[] front;
  public float[] line;
  public float[] frontMean;
  public float[] lineMean;

  // Public attributes to access line detection results for line and right sensors
  public boolean inLine;
  public int lineDetections;

  /**
   * Creates a light poller that uses the provided sensors for polling.
   * 
   * @param frontSensor
   * @param lineSensor
   */
  private LightPoller(EV3ColorSensor frontSensor, EV3ColorSensor lineSensor) {
    // Initialize front sensor values
    frontProvider = frontSensor.getMode("RGB");
    frontFilter = new MeanFilter(frontProvider, 4);
    front = new float[frontProvider.sampleSize()];
    frontMean = new float[frontFilter.sampleSize()];

    // Initialize line sensor values
    lineProvider = lineSensor.getMode("Red");
    lineFilter = new MeanFilter(lineProvider, 3);
    line = new float[lineProvider.sampleSize()];
    lineMean = new float[lineFilter.sampleSize()];
  }

  /**
   * Creates a {@code LightPoller} instance if the class has not been instantiated, otherwise
   * returns the existing instance. This method is the factory method for the singleton
   * {@code LightPoller} class.
   * 
   * @param frontSensor Light sensor placed on the robot arm, set to {@code RGB} mode.
   * @param lineSensor Light sensor placed to the left of the robot's center, set to {@code Red}
   *        mode.
   * 
   * @return New or existing instance of the {@code LightPoller} object.
   */
  public static LightPoller getLightPoller(EV3ColorSensor frontSensor, EV3ColorSensor lineSensor) {
    if (lightPoller == null) {
      lightPoller = new LightPoller(frontSensor, lineSensor);
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
  public void run() {
    while (true) {

      // Front sensor
      frontProvider.fetchSample(front, 0);
      frontFilter.fetchSample(frontMean, 0);

      // Line sensor
      lineProvider.fetchSample(line, 0);
      lineFilter.fetchSample(lineMean, 0);

      // Update line detection values
      lineDetection();

      try {
        Thread.sleep(10);
      } catch (Exception e) {
      }
    }
  }

  /**
   * Updates the line detection values for the line sensor.
   */
  private void lineDetection() {
    if (lineMean[0] < LINE_COLOR_VALUE) {
      if (!inLine) {
        inLine = true;
        lineDetections++;
      }
    } else {
      inLine = false;
    }
  }

}
