package ca.mcgill.ecse211.mountev3rest.navigation;

import java.util.LinkedList;
import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import lejos.hardware.Button;
import lejos.hardware.Sound;
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
  private static final int FORWARD_SPEED = 120;
  private static final int ROTATE_SPEED = 80;
  private static final int NAVIGATION_PERIOD = 25;
  private static final int CORRECTION_TIME_LIMIT = 1000;
  private static final double TILE_SIZE = 30.48;
  private final double MOTOR_OFFSET;
  private final double SENSOR_OFFSET;
  public final double WHEEL_RADIUS;
  public final double TRACK;

  // Class attributes
  // Motors
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Information about the robot and target
  private Odometer odometer;
  private LightPoller lightPoller;
  private double[] target;
  public double targetAngle;

  // State machine flags
  private boolean directionChanged;
  private boolean isNavigating;
  private boolean trajectoryCorrection;
  private boolean leftInLine;
  private boolean rightInLine;
  private int lastXCorrection;
  private int lastYCorrection;
  private Direction direction;
  private boolean correcting;
  private long timeLeftDetection;
  private long timeRightDetection;

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
   * @throws PollerException If the {@code LightPoller} has not been instantiated.
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      boolean trajectoryCorrection, final double WHEEL_RADIUS, final double TRACK,
      final double MOTOR_OFFSET, final double SENSOR_OFFSET) throws OdometerException, PollerException {

    this.odometer = Odometer.getOdometer();
    this.lightPoller = LightPoller.getLightPoller();

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.leftMotor.setAcceleration((int) (this.leftMotor.getAcceleration() * MOTOR_OFFSET));

    this.WHEEL_RADIUS = WHEEL_RADIUS;
    this.TRACK = TRACK;
    this.MOTOR_OFFSET = MOTOR_OFFSET;
    this.SENSOR_OFFSET = SENSOR_OFFSET;

    target = new double[2];

    isNavigating = false;
    directionChanged = false;
    leftInLine = false;
    rightInLine = false;
    this.trajectoryCorrection = trajectoryCorrection;
    direction = Direction.INIT;
    correcting = false;
  }

  /**
   * Runs the state machine main logic. This involves setting the direction of the robot to a new
   * target if required and optionally applying trajectory correction.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;
    int currLine;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Main navigator state machine flow

      // If the direction has changed recompute the trajectory of the robot
      if (directionChanged) {
        goToTarget();
        isNavigating = true;
        directionChanged = false;
      }

      if (trajectoryCorrection) {
        
        if (lightPoller.leftInLine && System.currentTimeMillis() - timeRightDetection > 300) {
          if (!leftInLine) {
            leftInLine = true;
            timeLeftDetection = System.currentTimeMillis();
            if (!lightPoller.rightInLine)
              adjustTrajectory(1);
          }
        } else {
          leftInLine = false;
        }
        
        if (lightPoller.rightInLine && System.currentTimeMillis() - timeLeftDetection > 300) {
          if (!rightInLine) {
            rightInLine = true;
            timeRightDetection = System.currentTimeMillis();
            if (!lightPoller.leftInLine)
              adjustTrajectory(0);
          }
        } else {
          rightInLine = false;
        }
        
      }
      
      // Set this flag to let other threads know that the robot is currently reaching a waypoint
      if (!leftMotor.isMoving() && !rightMotor.isMoving())
        isNavigating = false;

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
    target[0] = x;
    target[1] = y;

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * Turns to an absolute angle with respect to the grid ensuring minimal rotation. This method
   * constantly polls the gyro sensor angle reading reading to ensure high accuracy. Positive angles
   * are defined as counter-clockwise rotation and vice-versa.
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

    boolean wasEnabled = trajectoryCorrection;
    trajectoryCorrection = false;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, targetRotation) * direction, true);
    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, targetRotation) * direction, false);
    
    trajectoryCorrection = wasEnabled;
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
  }

  /**
   * Indicates whether the robot is still navigating.
   * 
   * @return Boolean values indicating if the robot is moving.
   */
  public boolean isNavigating() {
    return isNavigating;
  }

  public void waitNavigation() {
	long time =System.currentTimeMillis();
    while (true) {
    	if (isNavigating)
    		time =System.currentTimeMillis();
    	else {
    		if (System.currentTimeMillis() - time > 400)
    			break;
    	}
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
   * TODO
   */
  private void updateDirection() {
    double theta = odometer.getXYT()[2];
    if (theta > 45 && theta < 135)
      direction = Direction.EAST;
    else if (theta > 135 && theta < 225)
      direction = Direction.SOUTH;
    else if (theta > 225 && theta < 315)
      direction = Direction.WEST;
    else 
      direction = Direction.NORTH;
  }
  
  /**
   * TODO
   * 
   * @return
   */
  private int estimateCurrentLine() {
    updateDirection();
    double position[] = odometer.getXYT();
    double distInTile;
    double odoReading;
    switch(direction) {
      case NORTH:
        position[1] -= SENSOR_OFFSET;
        odoReading = position[1];
        distInTile = position[1] % TILE_SIZE;
        break;
      case EAST:
        position[0] -= SENSOR_OFFSET;
        odoReading = position[0];
        distInTile = position[0] % TILE_SIZE;
        break;
      case SOUTH:
        position[1] += SENSOR_OFFSET;
        odoReading = position[1];
        distInTile = position[1] % TILE_SIZE;
        break;
      case WEST:
        position[0] += SENSOR_OFFSET;
        odoReading = position[0];
        distInTile = position[0] % TILE_SIZE;
        break;
      default:
        distInTile = 0;
        odoReading = 0;
    }
    
    if (distInTile > TILE_SIZE / 2)
      return (int)((odoReading + (TILE_SIZE - distInTile)) / TILE_SIZE);
    else
      return (int)((odoReading - distInTile)/TILE_SIZE);
  }
  
  private double[] estimateLocation() {
	  double HALF_TILE = TILE_SIZE / 2;
	  
	  double[] ret = new double[2];
	  double[] position = odometer.getXYT();
	  double distInX = position[0] % HALF_TILE;
	  double distInY = position[1] % HALF_TILE;
	    if (distInX > HALF_TILE / 2)
	        ret[0] = (int)((position[0] + (HALF_TILE - distInX)) / TILE_SIZE);
	      else
	        ret[0] = (int)((position[0] - distInX)/TILE_SIZE);
	    
	    if (distInY > HALF_TILE / 2)
	        ret[1] = (int)((position[1] + (HALF_TILE - distInY)) / TILE_SIZE);
	      else
	        ret[1] = (int)((position[1] - distInY)/TILE_SIZE);
	    
	    return ret;
	  
  }
  
  /**
   * TODO
   * 
   * @param laggingSide
   */
  public void adjustTrajectory(int laggingSide) {
	  
	correcting = true;
    
    // Check that this line is no the same as the one for the past correction.
    int lastCorrection = -1;
    if (direction == Direction.NORTH || direction == Direction.SOUTH)
      lastCorrection = lastYCorrection;
    else if (direction == Direction.EAST || direction == Direction.WEST)
      lastCorrection = lastXCorrection;
    
    int currentLine = estimateCurrentLine();
    
    System.out.println("Current line = " + currentLine);
    
    if (lastCorrection == currentLine) {
      Sound.beep();
      Sound.beep();
      Sound.beep();
      Sound.beep();
      correcting = false;
      return;
    }
    
    boolean goBack = false;
    long startTime = System.currentTimeMillis();
    int prevTacho = 0;
    
    // Correct the direction
    if (laggingSide == 0) {
      rightMotor.setSpeed(0);
      leftMotor.setSpeed(FORWARD_SPEED / 2);
      prevTacho = leftMotor.getTachoCount();
      while (true) {
        if (lightPoller.leftInLine) {
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          break; 
        } else if (System.currentTimeMillis() - startTime > CORRECTION_TIME_LIMIT) {
          goBack = true;
          break;
        }
        try {
          Thread.sleep(NAVIGATION_PERIOD);
        } catch (InterruptedException e) {
        }
      }
    } else if (laggingSide == 1) {
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(FORWARD_SPEED / 2);
      prevTacho = rightMotor.getTachoCount();
      while (true) {
        if (lightPoller.rightInLine) {
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          break;
        } else if (System.currentTimeMillis() - startTime > CORRECTION_TIME_LIMIT) {
          goBack = true;
          break;
        }
        try {
          Thread.sleep(NAVIGATION_PERIOD);
        } catch (InterruptedException e) {
        }
      }
    }
    
    if (goBack) {
      if (laggingSide == 0)
        leftMotor.rotate(prevTacho - leftMotor.getTachoCount());
      else if (laggingSide == 1)
        rightMotor.rotate(prevTacho - rightMotor.getTachoCount());
      
      directionChanged = true;
      return;
    }
    
    // Correct the theta value of the odometer
    updateDirection();
    double[] position = odometer.getXYT();
    switch(direction) {
      case NORTH:
        odometer.setXYT(position[0], (currentLine * TILE_SIZE) + SENSOR_OFFSET, 0);
        break;
      case EAST:
        odometer.setTheta(90);
        odometer.setXYT((currentLine * TILE_SIZE) + SENSOR_OFFSET, position[1], 90);
        break;
      case SOUTH:
        odometer.setTheta(180);
        odometer.setXYT(position[0], (currentLine * TILE_SIZE) - SENSOR_OFFSET, 180);
        break;
      case WEST:
        odometer.setTheta(270);
        odometer.setXYT((currentLine * TILE_SIZE) + SENSOR_OFFSET, position[1], 270);
        break;
      default:
    }
    
    if (direction == Direction.NORTH || direction == Direction.SOUTH) 
        lastYCorrection = currentLine;
      else if (direction == Direction.EAST || direction == Direction.WEST)
        lastXCorrection= currentLine;
    
    // Indicate the state machine to recompute the direction.
    directionChanged = true;
    isNavigating = true;
    correcting = false;
    
  }
  
  /**
   * TODO
   */
  public void enableCorrection() {
    trajectoryCorrection = true;
  }
  
  /**
   * TODO
   */
  public void disableCorrection() {
    trajectoryCorrection = false;
  }
  
  /**
   * TODO
   * 
   * @return
   */
  public boolean isCorrectionEnabled() {
    return trajectoryCorrection;
  }

  /**
   * Moves the robot in the direction of the current target set in the state machine.
   */
  private void goToTarget() {
    // Compute the target's absolute angle and the distance required to reach it
    double[] position = odometer.getXYT();
    double[] realTarget =
        computeRealTarget(position[0], position[1], target[0] * TILE_SIZE, target[1] * TILE_SIZE);

    // Turn to target angle
    targetAngle = realTarget[1];
    turnTo(realTarget[1]);

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
  
  private enum Direction {
    INIT, NORTH, EAST, SOUTH, WEST;
  }

}
