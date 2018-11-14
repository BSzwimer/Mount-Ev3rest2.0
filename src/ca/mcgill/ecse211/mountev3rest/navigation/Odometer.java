package ca.mcgill.ecse211.mountev3rest.navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Uses the measurements of the wheel radius, tacho meter readings and gyro sensor angle readings to
 * provide a real time estimation of the cart in a 2 dimensional plane.
 * 
 * @author angelortiz
 *
 */
public class Odometer extends OdometerData implements Runnable {

  // Constants
  private final double TRACK;
  private final double WHEEL_RAD;
  private final double MOTOR_OFFSET;
  private static final long ODOMETER_PERIOD = 25;

  // Attributes
  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private int prevLeftMotorTachoCount = 0;
  private int prevRightMotorTachoCount = 0;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param gyroSensor
   * @param TRACK
   * @param WHEEL_RAD
   * @param MOTOR_OFFSET
   * 
   * @throws OdometerException
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD, final double MOTOR_OFFSET)
      throws OdometerException {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    this.MOTOR_OFFSET = MOTOR_OFFSET;
  }

  /**
   * {@code Odometer} factory method. Creates and instance of the {@code Odometer} class if it has
   * not been instantiated yet, otherwise it returns the existing instance.
   * 
   * @param leftMotor Left motor of the robot.
   * @param rightMotor Right motor of the robot.
   * @param gyroSensor Gyro sensor object to use for odometry.
   * @param WHEEL_RADIUS Wheel radius measurement of the robot.
   * @param MOTOR_OFFSET Ratio between the speed of the left and right motors.
   * 
   * @return New or existing {@code Odometer} object.
   * @throws OdometerException If there is a problem while instantiating the new {@code Odometer}
   *         object.
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RADIUS,
      double MOTOR_OFFSET) throws OdometerException {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS, MOTOR_OFFSET);
      return odo;
    }
  }

  /**
   * Returns the existing instance of the singleton {@code Odometer} class.
   * 
   * @return Existing instance of the {@code Odometer} class.
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   */
  public synchronized static Odometer getOdometer() throws OdometerException {

    if (odo == null) {
      throw new OdometerException("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * Uses the wheel radius measurement, tacho meter measurements from the motors and gyro sensor
   * angle readings to update the X and Y values as well the angle Theta of the cart's current
   * position.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      double[] position = odo.getXYT();

      leftMotorTachoCount = (int) (leftMotor.getTachoCount() / MOTOR_OFFSET);
      rightMotorTachoCount = rightMotor.getTachoCount();

      // Calculate new robot position based on tachometer counts
      double distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - prevLeftMotorTachoCount) / 180;
      double distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - prevRightMotorTachoCount) / 180;
      double deltaD = 0.5 * (distL + distR);
      double deltaT = Math.toDegrees((distL - distR) / TRACK);

      // Get the current odometer values
      double deltaX = deltaD * Math.sin(Math.toRadians(position[2] + deltaT));
      double deltaY = deltaD * Math.cos(Math.toRadians(position[2] + deltaT));

      // Update odometer values with new calculated values
      odo.update(deltaX, deltaY, deltaT);

      // Set current values to be the old values
      prevLeftMotorTachoCount = leftMotorTachoCount;
      prevRightMotorTachoCount = rightMotorTachoCount;

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
