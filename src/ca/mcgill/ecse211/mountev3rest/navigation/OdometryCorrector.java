package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Uses line detection on the lower light sensor of the robot to provide periodic corrections to the
 * odometer location estimations.
 * <p>
 * Once instantiated and running, the {@code OdometryCorrector} class runs as a separate thread on
 * the background silently providing corrections to the odometer when lines are detected.
 * 
 * @author angelortiz
 *
 */
public class OdometryCorrector {

  // Constants
  private static final int CORRECTION_TIME_LIMIT = 2200;
  private static final int FORWARD_SPEED = 120;
  private static final int CORRECTION_PERIOD = 25;
  private final double TILE_SIZE;
  private final double SENSOR_OFFSET;

  // Attributes
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;
  private Odometer odometer;
  private LightPoller lightPoller;

  public Direction direction;
  private boolean correctionEnabled;
  private boolean leftInLine;
  private boolean rightInLine;
  private int lastXCorrection;
  private int lastYCorrection;

  /**
   * Creates an {@code OdometryCorrector} that is to be run on its own thread.
   * 
   * @throws PollerException If the {@code LightPoller} has not been instantiated.
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   */
  public OdometryCorrector(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TILE_SIZE, final double SENSOR_OFFSET)
      throws PollerException, OdometerException {

    // Get navigation objects
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    odometer = Odometer.getOdometer();
    lightPoller = LightPoller.getLightPoller();

    // Initialize correction objects
    this.TILE_SIZE = TILE_SIZE;
    this.SENSOR_OFFSET = SENSOR_OFFSET;
    
    leftInLine = false;
    leftInLine = false;
    lastXCorrection = -1;
    lastYCorrection = -1;
    correctionEnabled = true;
    direction = Direction.INIT;
  }

  /**
   * Applies trajectory and odometry correction by detecting the lines on the floor with the two
   * light sensors. If correction is desabled the method returns immediately.
   * 
   * @return 1 if the correction was applied, 0 otherwise.
   */
  public boolean applyCorrection() {
    // Return if correction is disabled.
    if (!correctionEnabled)
      return false;

    lightPoller.poll(); // Update the sensor readings
    boolean ret = false;

    if (lightPoller.leftInLine) {
      if (!leftInLine) {
        leftInLine = true;
        if (!lightPoller.rightInLine)
          ret = adjustTrajectory(1);
      }
    } else {
      leftInLine = false;
    }

    if (lightPoller.rightInLine) {
      if (!rightInLine) {
        rightInLine = true;
        if (!lightPoller.leftInLine)
          ret = adjustTrajectory(0);
      }
    } else {
      rightInLine = false;
    }

    return ret;
  }

  /**
   * Estimates the closest line to the robot according to the odometer's readings and the current
   * heading.
   * 
   * @return An integer indicating the closest line to the robot on the grid according with respect
   *         to the robot's current heading.
   */
  public int estimateCurrentLine() {
    updateDirection();
    double position[] = odometer.getXYT();
    double distInTile;
    double odoReading;
    switch (direction) {
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
      return (int) ((odoReading + (TILE_SIZE - distInTile)) / TILE_SIZE);
    else
      return (int) ((odoReading - distInTile) / TILE_SIZE);
  }

  /**
   * Adjusts the trajectory of the robot by stopping the leading side and waiting until the lagging
   * side sees a line. In case the lagging side takes too long to see a line, the rotation caused by
   * stopping one of the wheels is reversed and the method returns.
   * 
   * @param laggingSide Integer denoting which side is lagging. The encoding is 0 for left and 1 for
   *        right.
   * 
   * @return
   */
  public boolean adjustTrajectory(int laggingSide) {
    // Check that this line is no the same as the one for the past correction.
    int lastCorrection = -1;
    if (direction == Direction.NORTH || direction == Direction.SOUTH)
      lastCorrection = lastYCorrection;
    else if (direction == Direction.EAST || direction == Direction.WEST)
      lastCorrection = lastXCorrection;

    int currentLine = estimateCurrentLine();

    System.out.println("Current line: " + currentLine);
    System.out.println("x: " + odometer.getXYT()[0] + "y: " + odometer.getXYT()[1]);

    if (lastCorrection == currentLine) {
      return false;
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
          Thread.sleep(CORRECTION_PERIOD);
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
          Thread.sleep(CORRECTION_PERIOD);
        } catch (InterruptedException e) {
        }
      }
    }

    if (goBack) {
      if (laggingSide == 0)
        leftMotor.rotate(prevTacho - leftMotor.getTachoCount());
      else if (laggingSide == 1)
        rightMotor.rotate(prevTacho - rightMotor.getTachoCount());

      return false;
    }

    // Correct the theta value of the odometer
    updateDirection();
    double[] position = odometer.getXYT();
    switch (direction) {
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
      lastXCorrection = currentLine;

    return true;
  }

  /**
   * Update the direction variable according to the odometer's reading.
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
   * Enables trajectory and odometry correction using two light sensors.
   */
  public void enable() {
    correctionEnabled = true;
  }

  /**
   * Disables trajectory and odometry correction using two light sensors.
   */
  public void disable() {
    correctionEnabled = false;
  }

  /**
   * Returns true if trajectory and odometry correction is enabled, and false otherwise.
   * 
   * @return A boolean value indicating whether correction is enabled.
   */
  public boolean isEnabled() {
    return correctionEnabled;
  }

  public enum Direction {
    INIT, NORTH, EAST, SOUTH, WEST;
  }

}
