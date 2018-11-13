package ca.mcgill.ecse211.mountev3rest.navigation;

import java.util.LinkedList;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides an interface to move the robot to an arbitrary point on the grid.
 * <p>
 * The {@code Navigation} enables the robot to go to arbitrary locations on the grid by computing
 * the required distance and angle to reach it from the odometer's readings. Additionally, the class
 * runs on its own thread to enable the caller to change direction at any point if an event occurs.
 * <p>
 * Additionally, the {@code Navigation} class can optionally provide trajectory correction while
 * traveling by polling odometer angle reading and recomputing the path of travel.
 * 
 * @author angelortiz
 *
 */
public class Navigation implements Runnable {

  // Class constants
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATE_SPEED = 50;
  private static final int NAVIGATION_PERIOD = 50;
  private static final double TILE_SIZE = 30.48;
  private static final int ANGLE_ERROR_THRESHOLD = 4;
  private final boolean GYRO_CORRECTION;
  private final double MOTOR_OFFSET;
  public final double WHEEL_RADIUS;
  public final double TRACK;

  // Class attributes
  // Motors
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Information about the robot and target
  private Odometer odometer;
  private OdometryCorrector odometryCorrector;
  private LinkedList<double[]> target;
  private double[] location;
  public double targetAngle;

  // State machine flags
  private boolean directionChanged;
  private boolean isMoving;
  private boolean isNavigating;
  private boolean correcting;

  /**
   * Creates a navigator that will operate using the specified track and wheel radius values.
   * 
   * @param leftMotor Left motor of the robot.
   * @param rightMotor Right motor of the robot.
   * @param WHEEL_RADIUS Wheel radius of the robot't wheels measured in centimeters.
   * @param TRACK Measurement of the robot's track in centimeter.
   * @param MOTOR_OFFSET Ration between the speeds of the left and right motors. Used to correct
   *        error in navigation.
   * @param GYRO_CORRECTION Flag indicating whether trajectory correction should be enabled.
   * 
   * @throws OdometerException If the singleton {@code Odometer} class has not been instantiated.
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      OdometryCorrector odometryCorrector, final double WHEEL_RADIUS, final double TRACK,
      final double MOTOR_OFFSET, final boolean GYRO_CORRECTION) throws OdometerException {

    this.odometer = Odometer.getOdometer();
    this.odometryCorrector = odometryCorrector;

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.leftMotor.setAcceleration((int) (this.leftMotor.getAcceleration() * MOTOR_OFFSET));

    this.WHEEL_RADIUS = WHEEL_RADIUS;
    this.TRACK = TRACK;
    this.MOTOR_OFFSET = MOTOR_OFFSET;
    this.GYRO_CORRECTION = GYRO_CORRECTION;

    target = new LinkedList<double[]>();
    location = new double[2];
    location[0] = 1; // TODO: Change for different starting corners.
    location[1] = 1;

    isMoving = false;
    isNavigating = false;
    directionChanged = false;
    correcting = false;
  }

  /**
   * Runs the state machine main logic. This involves setting the direction of the robot to a new
   * target if required and optionally applying trajectory correction.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Main navigator state machine flow

      // If the direction has changed recompute the trajectory of the robot
      if (directionChanged) {
        goToTarget();
        isNavigating = true;
        directionChanged = false;
      }

      // Correct the trajectory if error angle is too large
      if (GYRO_CORRECTION && isNavigating && Math.abs(targetAngle - odometer.getXYT()[2]) > ANGLE_ERROR_THRESHOLD) {
        if (!correcting) {
          directionChanged = true;
          correcting = true;
        }
      } else {
        correcting = false;
      }

      // Set this flag to let other threads know that the robot is currently reaching a waypoint
      if (!leftMotor.isMoving() && !rightMotor.isMoving())
        isMoving = false;

      if (!isMoving && target.size() == 0)
        isNavigating = false;

      if (!isMoving && target.size() > 0) {
        location = target.remove();
        if (target.size() > 0) {
          directionChanged = true;
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

  /**
   * Sets current target and indicates the state machine to retrace the trajectory.
   * 
   * @param x New target X position.
   * @param y New target Y position.
   */
  public void travelTo(double x, double y) {
    // Split the target into X and Y components.
    // double[] newTargetX = {x, location[1]};
    double[] newTargetY = {x, y};

    // target.add(newTargetX);
    target.add(newTargetY);

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * TODO
   * 
   * @param theta
   */
  public void turnTo(double theta) {
    turnTo(theta, false);
  }

  /**
   * Turns to an absolute angle with respect to the grid ensuring minimal rotation. This method
   * constantly polls the gyro sensor angle reading reading to ensure high accuracy. Positive angles
   * are defined as counter-clockwise rotation and vice-versa.
   * 
   * @param theta Desired angle of rotation.
   */
  public void turnTo(double theta, boolean correcting) {
    double currTheta = odometer.getXYT()[2];
    double targetRotation = 0;
    double speedFactor = 1;
    boolean corrected = false;
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

    if (correcting) {
      speedFactor = 0.5;
      corrected = true;
    }


    leftMotor.setSpeed((int) (ROTATE_SPEED * speedFactor));
    rightMotor.setSpeed((int) (ROTATE_SPEED * speedFactor));


    if (direction == 1) {
      leftMotor.forward();
      rightMotor.backward();
    } else {
      leftMotor.backward();
      rightMotor.forward();
    }

    while (Math.abs(odometer.getXYT()[2] - theta) > 1) {
      try {
        Thread.sleep(NAVIGATION_PERIOD / 2);
      } catch (InterruptedException ex) {
        ex.printStackTrace();
      }
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    if (wasEnabled)
      odometryCorrector.enable();
    odometryCorrector.updateHeading();

    if (corrected)
      correcting = false;
  }

  /**
   * Turns to an relative angle with respect to the current position ensuring minimal rotation.
   * Positive angles are defined as counter-clockwise rotation and vice-versa.
   * 
   * @param theta Desired angle of rotation.
   */
  public void turnToRelative(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), false);

    odometryCorrector.updateHeading();
  }

  /**
   * Indicates whether the robot is still navigating.
   * 
   * @return Boolean values indicating if the robot is moving.
   */
  public boolean isNavigating() {
    return isNavigating;
  }

  // REVIEW
  /**
   * Moves the robot directly to the given coordinate and sets the internal current location to that
   * point.
   * 
   * This method must be called once before to enable the {@code Navigation} class to split
   * subsequent paths of travel into their X and Y directions which is required for
   * {@code OdometryCorrector} class to work correctly.
   * 
   * @param x X coordinate to use as reference.
   * @param y Y coordinate to use as reference.
   */
  public void setLocation(int x, int y) {
    double[] newTarget = {x, y};
    target.add(newTarget);
    directionChanged = true;
    isNavigating = true;

    while (isNavigating) {
      try {
        Thread.sleep(NAVIGATION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  public void wait(int targetsToReach) {
    int finalTargets = target.size() - targetsToReach;
    while (target.size() != finalTargets) {
      try {
        Thread.sleep(NAVIGATION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * TODO
   * 
   * @param x
   * @param y
   * @return
   */
  public double computeDistance(double x, double y) {
    double[] position = odometer.getXYT();
    return computeRealTarget(position[0], position[1], x * TILE_SIZE, y * TILE_SIZE)[0];
  }

  /**
   * Moves the robot in the direction of the current target set in the state machine.
   */
  private void goToTarget() {
    isMoving = true;

    // Compute the target's absolute angle and the distance required to reach it
    double[] position = odometer.getXYT();
    double[] realTarget = computeRealTarget(position[0], position[1], target.peek()[0] * TILE_SIZE,
        target.peek()[1] * TILE_SIZE);

    // Turn to target angle
    targetAngle = realTarget[1];
    turnTo(realTarget[1], correcting);

    // Move forward the required target distance
    leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate((int) (convertDistance(WHEEL_RADIUS, realTarget[0]) * MOTOR_OFFSET), true);
    rightMotor.rotate(convertDistance(WHEEL_RADIUS, realTarget[0]), true);
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
