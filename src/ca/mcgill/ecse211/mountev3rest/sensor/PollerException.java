package ca.mcgill.ecse211.mountev3rest.sensor;

/**
 * Exception class used to handle errors related to the singleton pattern used for the
 * {@code LightPoller} and {@code UltrasonicPoller} classes.
 *
 */
public class PollerException extends Exception {

  /**
   * Creates a {@code PollerException} object.
   * 
   * @param Error Error message associated to the exception.
   */
  public PollerException(String Error) {
    super(Error);
  }

}
