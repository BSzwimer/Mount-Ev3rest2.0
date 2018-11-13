package ca.mcgill.ecse211.mountev3rest.sensor;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Provides an interface to obtain distance measurements from the ultrasonic sensor in centimeters.
 * <p>
 * The {@code UltrasonicPoller} class is implemented as a singleton to facilitate access to the
 * sensor to different parts of the code.
 * 
 * @author angelortiz
 *
 */
public class UltrasonicPoller {

  // Attributes
  // Singleton instance
  private static UltrasonicPoller usPoller = null;

  private SampleProvider sp;
  private MeanFilter mf;
  private float[] buffer;

  // Constants
  private final static int MEAN_SIZE = 3;

  /**
   * Creates a poller that used the provided ultrasonic sensor to get samples.
   * 
   * @param usSensor
   */
  private UltrasonicPoller(EV3UltrasonicSensor usSensor) {
    sp = usSensor.getMode("Distance");
    mf = new MeanFilter(sp, MEAN_SIZE);

    buffer = new float[sp.sampleSize()];
  }

  /**
   * Creates an {@code UltrasonicPoller} instance if the class has not been instantiated, otherwise
   * returns the existing instance. This method is the factory method for the singleton
   * {@code LightPoller} class.
   * 
   * @param usSensor Ultrasonic sensor that will be used for polling.
   * @return New or existing instance of the {@code UltrasonicPoller} object.
   */
  public static UltrasonicPoller getUltrasonicPoller(EV3UltrasonicSensor usSensor) {
    if (usPoller == null) {
      usPoller = new UltrasonicPoller(usSensor);
      return usPoller;
    } else {
      return usPoller;
    }
  }

  /**
   * Factory method for existing instance of the singleton {@code UltrasonicPoller} class.
   * 
   * @return Existing {@code UltrasonicPoller} object.
   * @throws PollerException If the {@code UltrasonicPoller} class has not been instantiated.
   */
  public static UltrasonicPoller getUltrasonicPoller() throws PollerException {
    if (usPoller == null)
      throw new PollerException(
          "The singleton UltrasonicPoller class has not been instantiated yet."
              + "Three EV3ColorSensor instances must be provided to the factory method.");
    else
      return usPoller;
  }

  /**
   * Gets an updated reading of the ultrasonic sensor in centimeters.
   * 
   * @return Updated raw reading in centimeters.
   */
  public int poll() {
    sp.fetchSample(buffer, 0);
    return (int) (buffer[0] * 100);
  }

  /**
   * Gets an updated mean of the most recent ultrasonic sensor readings in centimeters.
   * 
   * @return Updated mean reading in centimeters.
   */
  public int pollMean() {
    // Poll MEAN_SIZE times to get an updated mean
    for (int i = 0; i < MEAN_SIZE; i++) {
      mf.fetchSample(buffer, 0);
    }
    return (int) (buffer[0] * 100);
  }

}
