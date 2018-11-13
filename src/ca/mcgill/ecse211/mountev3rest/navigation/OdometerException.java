package ca.mcgill.ecse211.mountev3rest.navigation;

/**
 * Exception class used to handle errors regarding the singleton pattern used for the
 * {@code Odometer} and {@code OdometerData} classes.
 *
 */
@SuppressWarnings("serial")
public class OdometerException extends Exception {

  /**
   * Creates an {@code OdometerException} object.
   * 
   * @param Error Error message associated to the exception.
   */
  public OdometerException(String Error) {
    super(Error);
  }

}
