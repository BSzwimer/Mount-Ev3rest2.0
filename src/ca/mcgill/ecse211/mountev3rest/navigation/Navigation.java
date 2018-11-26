package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides an interface to move the robot to an arbitrary point on the grid.
 * <p>
 * The {@code Navigation} enables the robot to go to arbitrary locations on the grid by computing
 * the required distance and angle to reach it from the odometer's readings. Additionally, the class
 * runs on its own thread to enable the caller to change direction at any point if an event occurs.
 * <p>
 * Additionally, the {@code Navigation} class invokes the {@code OdometryCorrector} class while
 * navigating to reduce the error introduced by the motors and other external factors.
 * 
 * @author angelortiz
 *
 */
public class Navigation implements Runnable {

  // Class constants
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATE_SPEED = 80;
  private static final int NAVIGATION_PERIOD = 50;
  private static final int WAIT_PERIOD = 120;
  private static final double TILE_SIZE = 30.48;
  private static final double MIN_TRAVEL_DISTANCE = 0.5;
  private static final int MIN_STATIC_INTERVAL = 400;
  public final double MOTOR_OFFSET;
  public final double WHEEL_RADIUS;
  public final double TRACK;

  // Class attributes
  // Motors
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Information about the robot and target
  private Odometer odometer;
  private OdometryCorrector odometryCorrector;
  private double[] target;
  public double targetAngle;

  // State machine flags
  private boolean directionChanged;
  private boolean isNavigating;

  /**
   * Creates a navigator that will operate using the specified track and wheel radius values.
   * 
   * @param leftMotor Left motor of the robot.
   * @param rightMotor Right motor of the robot.
   * @param odometryCorrector Object used by the class to reduce the error while navigating.
   * @param WHEEL_RADIUS Wheel radius of the robot't wheels measured in centimeters.
   * @param TRACK Measurement of the robot's track in centimeter.
   * @param MOTOR_OFFSET Ratio between the speeds of the left and right motors. This value allows
   *        the system to compensate for differences in the motors' performances.
   * 
   * @throws OdometerException If the singleton {@code Odometer} class has not been instantiated.
   * @throws PollerException If the {@code LightPoller} has not been instantiated.
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      OdometryCorrector odometryCorrector, final double WHEEL_RADIUS, final double TRACK,
      final double MOTOR_OFFSET) throws OdometerException, PollerException {

    // Get navigation related objects
    this.odometer = Odometer.getOdometer();
    this.odometryCorrector = odometryCorrector;

    // Set the motors
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Populate the constants
    this.WHEEL_RADIUS = WHEEL_RADIUS;
    this.TRACK = TRACK;
    this.MOTOR_OFFSET = MOTOR_OFFSET;

    // Instantiate the target
    target = new double[2];
    target[0] = -1;
    target[1] = -1;

    // Initiate the state machine variables
    isNavigating = false;
    directionChanged = false;
  }


  /* ---STATE MACHINE LOGIC--- */

  /**
   * Runs the state machine main logic.
   * <p>
   * This involves setting the direction of the robot to a new target if required and optionally
   * applying trajectory correction.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;
    boolean corrected = false;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Main navigator state machine flow

      // If the direction has changed recompute the trajectory of the robot
      if (directionChanged) {
        goToTarget();
        isNavigating = true;
        directionChanged = false;
      }

      // Set this flag to let other threads know that the robot is currently reaching a waypoint
      if (!leftMotor.isMoving() && !rightMotor.isMoving())
        isNavigating = false;

      // Correct the trajectory if necessary
      if (isNavigating && odometryCorrector.isEnabled()) {
        corrected = odometryCorrector.applyCorrection();
        if (corrected) {
          directionChanged = true;
          isNavigating = true;
        }
      }

      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < NAVIGATION_PERIOD) {
        try {
          Thread.sleep(NAVIGATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }


  /* ---ROBOT DISPLACEMENT INTERFACE--- */

  /**
   * Sets current target and indicates the state machine to retrace the trajectory.
   * 
   * @param x New target X position.
   * @param y New target Y position.
   */
  public void travelTo(double x, double y) {
    target[0] = x;
    target[1] = y;

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * Sets current target and indicates the state machine to retrace the trajectory. This method will
   * only move the robot in the ±X direction.
   * 
   * @param x New target X position.
   */
  public void travelToX(double x) {
    target[0] = x;
    target[1] = -1;

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * Sets current target and indicates the state machine to retrace the trajectory. This method will
   * only move the robot in the ±Y direction.
   * 
   * @param y New target Y position.
   */
  public void travelToY(double y) {
    target[0] = -1;
    target[1] = y;

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * Turns to an absolute angle with respect to the grid ensuring minimal rotation. Positive angles
   * are defined as counter-clockwise rotation and vice-versa. The accuracy of this method heavily
   * relies on the accuracy of the track measurement provided during the instantiation of the class.
   * 
   * @param theta Desired angle of rotation.
   */
  public void turnTo(double theta) {
    double currTheta = odometer.getXYT()[2];
    double targetRotation = 0;
    int direction = 1; // 1 for right turn, -1 for left turn

    // Ensure that the minimal turn is taken
    if (theta < currTheta) {
      targetRotation = currTheta - theta;
      if (targetRotation < 180)
        direction = -1;
      else
        targetRotation = 360 - targetRotation;
    } else {
      targetRotation = theta - currTheta;
      if (targetRotation > 180) {
        targetRotation = 360 - targetRotation;
        direction = -1;
      }
    }

    boolean wasEnabled = odometryCorrector.isEnabled();
    odometryCorrector.disable();

    leftMotor.setSpeed((int) (ROTATE_SPEED * MOTOR_OFFSET));
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(
        (int) (convertAngle(WHEEL_RADIUS, TRACK, targetRotation) * MOTOR_OFFSET) * direction, true);
    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, targetRotation) * direction, false);

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * Turns to an relative angle with respect to the current position ensuring minimal rotation.
   * Positive angles are defined as counter-clockwise rotation and vice-versa.
   * 
   * @param theta Desired angle of rotation.
   */
  public void turnToRelative(double theta) {
    leftMotor.setSpeed((int) (ROTATE_SPEED * MOTOR_OFFSET));
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), false);
  }

  /**
   * Makes the robot move forward a determined distance in centimeters.
   * 
   * @param dist Distance to travel forward in centimeters.
   */
  public void advanceDist(double dist) {
    leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(Navigation.convertDistance(WHEEL_RADIUS, dist * MOTOR_OFFSET), true);
    rightMotor.rotate(Navigation.convertDistance(WHEEL_RADIUS, dist), true);
    isNavigating = true;
  }


  /* ---NAVIGATION STATUS INTERFACE--- */

  /**
   * Indicates whether the robot is still navigating.
   * 
   * @return Boolean values indicating if the robot is moving.
   */
  public boolean isNavigating() {
    return isNavigating;
  }

  /**
   * This method does not return until the {@code Navigation} class reaches its current target.
   */
  public void waitNavigation() {
    long time = System.currentTimeMillis();
    while (true) {
      if (isNavigating)
        time = System.currentTimeMillis();
      else if (System.currentTimeMillis() - time > MIN_STATIC_INTERVAL) {
        break;
      }
      try {
        Thread.sleep(WAIT_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Computes the distance from the current robot's position to a particular target location in
   * centimeters.
   * 
   * @param x Target X coordinate.
   * @param y Target Y coordinate.
   * @return The distance between the robot and the target coordinates in centimeters.
   */
  public double computeDistance(double x, double y) {
    double[] position = odometer.getXYT();
    return computeRealTarget(position[0], position[1], x * TILE_SIZE, y * TILE_SIZE)[0];
  }


  // ---PRIVATE METHODS---

  /**
   * TODO
   */
  private void goToTarget() {
    double[] position = odometer.getXYT();
    if (target[0] != -1) {
      double dist = target[0] * TILE_SIZE - position[0];
      if (Math.abs(dist) < MIN_TRAVEL_DISTANCE) {
        return;
      }
      if (dist < 0) {
        turnTo(270);
        leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate((int) (convertDistance(WHEEL_RADIUS, -dist) * MOTOR_OFFSET), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, -dist), true);
      } else {
        turnTo(90);
        leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate((int) (convertDistance(WHEEL_RADIUS, dist) * MOTOR_OFFSET), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, dist), true);
      }
    } else if (target[1] != -1) {
      double dist = target[1] * TILE_SIZE - position[1];
      if (Math.abs(dist) < MIN_TRAVEL_DISTANCE) {
        return;
      }
      if (dist < 0) {
        turnTo(180);
        leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate((int) (convertDistance(WHEEL_RADIUS, -dist) * MOTOR_OFFSET), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, -dist), true);
      } else {
        turnTo(0);
        leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate((int) (convertDistance(WHEEL_RADIUS, dist) * MOTOR_OFFSET), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, dist), true);
      }
    }
  }

  /**
   * Computes the absolute angle and distance in centimeters required to reach the target with
   * respect to the current position.
   * 
   * @param currX Current X position in centimeters.
   * @param currY Current Y position in centimeters.
   * @param targetX Target X position in centimeters.
   * @param targetY Target Y position in centimeters.
   * @return Array containing the distance and angle required to reach the target in that order.
   */
  private double[] computeRealTarget(double currX, double currY, double targetX, double targetY) {
    double deltaX = targetX - currX;
    double deltaY = targetY - currY;
    int quadrant = 0;
    double[] computedTarget = new double[2];

    // Determine the quadrant of the target with respect to the current position
    if (deltaX >= 0 && deltaY >= 0)
      quadrant = 1;
    else if (deltaX >= 0 && deltaY < 0)
      quadrant = 2;
    else if (deltaX < 0 && deltaY < 0)
      quadrant = 3;
    else if (deltaX < 0 && deltaY >= 0)
      quadrant = 4;

    // Distance to the target
    double distance = Math.hypot(deltaX, deltaY);

    // Compute the absolute angle of direction to the target
    deltaX = Math.abs(deltaX);
    deltaY = Math.abs(deltaY);

    double targetTheta = 0;
    switch (quadrant) {
      case 1:
        targetTheta = Math.toDegrees(Math.atan(deltaX / deltaY));
        break;
      case 2:
        targetTheta = Math.toDegrees(Math.atan(deltaY / deltaX));
        targetTheta += 90;
        break;
      case 3:
        targetTheta = Math.toDegrees(Math.atan(deltaX / deltaY));
        targetTheta += 180;
        break;
      case 4:
        targetTheta = Math.toDegrees(Math.atan(deltaY / deltaX));
        targetTheta += 270;
        break;
    }

    computedTarget[0] = distance;
    computedTarget[1] = targetTheta;

    return computedTarget;
  }

  /**
   * Converts a value distance to the equivalent degrees of rotation in the wheels. Wheel radius and
   * distance should be in the same units.
   * 
   * @param radius Wheel radius to use for conversion.
   * @param distance Distance to convert.
   * 
   * @return Required angle of rotation for the motors in degrees.
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * Computes the required distance travel of each wheel assuming opposite directions to achieve a
   * certain degree of rotation.
   * 
   * @param radius Wheel radius.
   * @param width Width of the robot.
   * @param angle Desired angle of rotation.
   * 
   * @return Required angle of rotation in degrees on each wheel.
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
