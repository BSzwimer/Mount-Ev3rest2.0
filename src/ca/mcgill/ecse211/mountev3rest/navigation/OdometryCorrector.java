package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class used line detection on the lower light sensor of the robot to provide periodic
 * corrections to the odometer location estimations and the robot's direction.
 * <p>
 * This class corrects the trajectory of the robot by ensuring that both light sensors align with
 * each line traversed, effectively setting the robot's heading to a right angle. Then it adjusts
 * the odometer values by estimating the line that was traversed and using the provided tile size
 * measurement.
 * 
 * @author angelortiz
 *
 */
public class OdometryCorrector {

  // Constants
  private static final int CORRECTION_TIME_LIMIT = 1500;
  private static final int SHORT_CORRECTION_TIME_LIMIT = 1400;
  private static final int ISOLATED_CORRECTION_LIMIT = 7000;
  private static final int FORWARD_SPEED = 120;
  private static final int CORRECTION_SPEED = 80;
  private static final int SAFETY_PAUSE = 200;
  private static final int CORRECTION_PERIOD = 50;
  private final double TILE_SIZE;
  private final double MOTOR_OFFSET;
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
   * Creates an {@code OdometryCorrector} that can be called during navigation.
   * 
   * @param leftMotor Left motor used to correct the trajectory.
   * @param rightMotor Right motor used to correct the trajectory.
   * @param TILE_SIZE Length of the tiles on the grid in centimeters.
   * @param SENSOR_OFFSET Vertical distance from the robot's axis to the lower light sensors in
   *        centimeters.
   * @param MOTOR_OFFSET Ratio between the speed of the left and right motors used to reduce the
   *        error between the motors' different performances.
   * 
   * @throws PollerException If the {@code LightPoller} has not been instantiated.
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   */
  public OdometryCorrector(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TILE_SIZE, final double SENSOR_OFFSET, final double MOTOR_OFFSET)
      throws PollerException, OdometerException {

    // Get navigation objects
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    odometer = Odometer.getOdometer();
    lightPoller = LightPoller.getLightPoller();

    // Initialize correction objects
    this.TILE_SIZE = TILE_SIZE;
    this.SENSOR_OFFSET = SENSOR_OFFSET;
    this.MOTOR_OFFSET = MOTOR_OFFSET;

    leftInLine = false;
    leftInLine = false;
    lastXCorrection = -1;
    lastYCorrection = -1;
    correctionEnabled = true;
    direction = Direction.INIT;
  }

  /**
   * Applies trajectory and odometry correction by detecting the lines on the floor with the two
   * light sensors. If correction is disabled or the robot just applied correction on this line the
   * method returns immediately.
   * <p>
   * This method actively changes the heading of the robot to be aligned to the line being
   * traversed, and overwrites the values of the odometer by estimating the number of line that was
   * detected as well as the dierection and the measurement of the tile length provided.
   * 
   * @return True if the correction was applied, false otherwise.
   */
  public boolean applyCorrection() {
    // Return if correction is disabled.
    if (!correctionEnabled)
      return false;

    lightPoller.poll(); // Update the sensor readings
    boolean ret = false;

    if (lightPoller.leftInLine && lightPoller.rightInLine) {
      return ret;
    }

    if (lightPoller.leftInLine) {
      if (!leftInLine) {
        leftInLine = true;
        if (!lightPoller.rightInLine)
          ret = adjustTrajectory(1);
      }
    } else {
      leftInLine = false;
    }

    if (ret)
      return ret;

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
   * @return An integer indicating the closest line to the robot on the grid according to the
   *         robot's current heading.
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
   * TODO
   * 
   * @param goBackwards
   */
  public void correctOnNextLine(boolean goBackwards) {    
    int prevTachoLeft = leftMotor.getTachoCount();
    int prevTachoRight = rightMotor.getTachoCount();
    long startTime = System.currentTimeMillis();
    boolean goBack = false;
    
    leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);
    if (goBackwards) {
      leftMotor.backward();
      rightMotor.backward();
    } else {
      leftMotor.forward();
      rightMotor.forward();
    }
    
    while (true) {
      lightPoller.poll();
      if (lightPoller.leftInLine) {
        adjustTrajectory(1, goBackwards, false);
        break;
      } else if (lightPoller.rightInLine) {
        adjustTrajectory(0, goBackwards, false);
        break;
      } else if (System.currentTimeMillis() - startTime > ISOLATED_CORRECTION_LIMIT) {
        goBack = true;
        break;
      }
      
      try {
        Thread.sleep(CORRECTION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
    if (goBack) {
      leftMotor.setSpeed((int) (FORWARD_SPEED * MOTOR_OFFSET));
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.rotate(prevTachoLeft - leftMotor.getTachoCount(), true);
      rightMotor.rotate(prevTachoRight - rightMotor.getTachoCount(), false);
    }
    
  }

  /**
   * TODO
   * 
   * @param laggingSide
   * @return
   */
  public boolean adjustTrajectory(int laggingSide) {
    return adjustTrajectory(laggingSide, false, true);
  }

  /**
   * Adjusts the trajectory of the robot by stopping the leading side and waiting until the lagging
   * side sees a line. In case the lagging side takes too long to see a line, the rotation caused by
   * stopping one of the wheels is reversed and the method returns.
   * <p>
   * If the method determines that it just corrected using this line it returns false immediately.
   * 
   * @param laggingSide Integer denoting which side is lagging. The encoding is 0 for left and 1 for
   *        right.
   * 
   * @return True if a correction was actually applied to the odometer, false otherwise.
   */
  public boolean adjustTrajectory(int laggingSide, boolean goingBackwards,
      boolean checkForRepeats) {
    // Check that this line is no the same as the one for the past correction.
    int lastCorrection = -1;
    if (direction == Direction.NORTH || direction == Direction.SOUTH)
      lastCorrection = lastYCorrection;
    else if (direction == Direction.EAST || direction == Direction.WEST)
      lastCorrection = lastXCorrection;

    int currentLine = estimateCurrentLine();

    /*
     * System.out.println("Current line: " + currentLine + "Direction: " + direction);
     * System.out.println("x: " + odometer.getXYT()[0] + "y: " + odometer.getXYT()[1]);
     * Sound.beep();
     */

    // If the last correction was on this line do nothing
    if (lastCorrection == currentLine && checkForRepeats) {
      return false;
    }

    Sound.beep();

    boolean lineDetected = false;

    // Correct the trajectory
    rightMotor.stop(true);
    leftMotor.stop(false);

    // Lagging side is left
    if (laggingSide == 0) {
      lightPoller.poll();

      int prevTachoLeft = leftMotor.getTachoCount();
      int prevTachoRight = rightMotor.getTachoCount();
      long startTime = System.currentTimeMillis();
      boolean goBack = false;

      // Make sure you didn't skip the line while stopping
      while (!lightPoller.rightInLine) {
        lightPoller.poll();
        leftMotor.setSpeed((int) (CORRECTION_SPEED * MOTOR_OFFSET));
        rightMotor.setSpeed(CORRECTION_SPEED);

        if (goingBackwards) {
          leftMotor.forward();
          rightMotor.forward();
        } else {
          leftMotor.backward();
          rightMotor.backward();
        }

        if (System.currentTimeMillis() - startTime > CORRECTION_TIME_LIMIT) {
          goBack = true; // If the line is never seen signal the method to undo the turning
          break;
        }

        try {
          Thread.sleep(CORRECTION_PERIOD);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }

      // Undo the previous travel
      if (goBack) {
        leftMotor.rotate(prevTachoLeft - leftMotor.getTachoCount());
        rightMotor.rotate(prevTachoRight - rightMotor.getTachoCount());
      }

      rightMotor.stop(true);
      leftMotor.stop(false);

      try {
        Thread.sleep(SAFETY_PAUSE);
      } catch (InterruptedException e1) {
        e1.printStackTrace();
      }

      // Correct the lagging wheel
      lineDetected = rotateUntilDetection(0, SHORT_CORRECTION_TIME_LIMIT, goingBackwards);

      if (direction == Direction.NORTH || direction == Direction.SOUTH) {
        lastYCorrection = currentLine;
        lastXCorrection = 0;
      } else if (direction == Direction.EAST || direction == Direction.WEST) {
        lastXCorrection = currentLine;
        lastYCorrection = 0;
      }

      // Lagging side is right
    } else if (laggingSide == 1) {
      lightPoller.poll();

      int prevTachoLeft = leftMotor.getTachoCount();
      int prevTachoRight = rightMotor.getTachoCount();
      long startTime = System.currentTimeMillis();
      boolean goBack = false;

      // Make sure you didn't skip the line while stopping
      while (!lightPoller.leftInLine) {
        lightPoller.poll();
        leftMotor.setSpeed((int) (CORRECTION_SPEED * MOTOR_OFFSET));
        rightMotor.setSpeed(CORRECTION_SPEED);

        if (goingBackwards) {
          leftMotor.forward();
          rightMotor.forward();
        } else {
          leftMotor.backward();
          rightMotor.backward();
        }

        if (System.currentTimeMillis() - startTime > CORRECTION_TIME_LIMIT) {
          goBack = true; // If the line is never seen signal the method to undo the turning
          break;
        }

        try {
          Thread.sleep(CORRECTION_PERIOD);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }

      // Undo the previous travel
      if (goBack) {
        leftMotor.rotate(prevTachoLeft - leftMotor.getTachoCount());
        rightMotor.rotate(prevTachoRight - rightMotor.getTachoCount());
      }

      leftMotor.stop(true);
      rightMotor.stop(false);

      try {
        Thread.sleep(SAFETY_PAUSE);
      } catch (InterruptedException e1) {
        e1.printStackTrace();
      }

      // Correct the lagging wheel
      lineDetected = rotateUntilDetection(1, SHORT_CORRECTION_TIME_LIMIT, goingBackwards);

      if (direction == Direction.NORTH || direction == Direction.SOUTH) {
        lastYCorrection = currentLine;
        lastXCorrection = 0;
      } else if (direction == Direction.EAST || direction == Direction.WEST) {
        lastXCorrection = currentLine;
        lastYCorrection = 0;
      }

    }

    // Correct the theta value of the odometer
    if (lineDetected)
      correctOdometer(currentLine);

    return true;
  }
  
  /**
   * TODO
   * 
   * @param side
   * @return
   */
  public boolean rotateUntilDetection(int side) {
    return rotateUntilDetection(side, CORRECTION_TIME_LIMIT, false);
  }

  /**
   * Rotates one motor until a line is detected on that side or the time limit is exceeded. If no
   * line is seen going forward the method tried going backwards.
   * 
   * @param side Side to rotate. 0 for left, 1 for right.
   * @return True if a line was ever detected, false otherwise.
   */
  public boolean rotateUntilDetection(int side, int timeLimit, boolean startBackwards) {
    // Stop both motors
    leftMotor.stop(true);
    rightMotor.stop(false);
    try {
      Thread.sleep(SAFETY_PAUSE);
    } catch (InterruptedException e2) {
      e2.printStackTrace();
    }

    // Adjust for the requested side of rotation
    EV3LargeRegulatedMotor motor = side == 0 ? leftMotor : rightMotor;
    int speed = side == 0 ? (int) (CORRECTION_SPEED * MOTOR_OFFSET) : CORRECTION_SPEED;

    boolean inLine;
    boolean goBack = false;

    // Rotate forward until a line is detected of the time limit exceeded
    long prevTacho = motor.getTachoCount();
    long startTime = System.currentTimeMillis();
    motor.setSpeed(speed);
    
    if (startBackwards)
      motor.backward();
    else 
      motor.forward();  

    while (true) {
      lightPoller.poll();
      inLine = side == 0 ? lightPoller.leftInLine : lightPoller.rightInLine;
      if (inLine) {
        motor.stop(false);
        break;
      } else if (System.currentTimeMillis() - startTime > timeLimit) {
        goBack = true; // If the line is never seen signal the method to undo the turning
        break;
      }
      try {
        Thread.sleep(CORRECTION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    try {
      Thread.sleep(SAFETY_PAUSE);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    // Undo the turning since a line was never seen
    if (goBack) {
      motor.setSpeed(speed);
      int reverseRotation = side == 0 ? (int) ((prevTacho - motor.getTachoCount()) * MOTOR_OFFSET)
          : (int) (prevTacho - motor.getTachoCount());
      motor.rotate(reverseRotation);
      goBack = false;
    }
    // If a line was seen return
    else
      return true;

    // If no line was seen try going backwards
    prevTacho = motor.getTachoCount();
    startTime = System.currentTimeMillis();

    motor.setSpeed(speed);
    if (startBackwards)
      motor.forward();
    else 
      motor.backward();  

    while (true) {
      lightPoller.poll();
      inLine = side == 0 ? lightPoller.leftInLine : lightPoller.rightInLine;
      if (inLine) {
        motor.stop(false);
        break;
      } else if (System.currentTimeMillis() - startTime > timeLimit) {
        goBack = true; // If the line is never seen signal the method to undo the turning
        break;
      }
      try {
        Thread.sleep(CORRECTION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    try {
      Thread.sleep(SAFETY_PAUSE);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    // Undo the turning since a line was never seen
    if (goBack) {
      motor.setSpeed(speed);
      int reverseRotation = side == 0 ? (int) ((prevTacho - motor.getTachoCount()) * MOTOR_OFFSET)
          : (int) (prevTacho - motor.getTachoCount());
      motor.rotate(reverseRotation);
      return false;
    }

    return true;
  }

  /**
   * Updates the internal direction variable according to the odometer's reading.
   */
  public void updateDirection() {
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

  /**
   * Direction enum used to classify the angle heading of the robot into four groups.
   * 
   * @author angelortiz
   *
   */
  public enum Direction {
    INIT, NORTH, EAST, SOUTH, WEST;
  }

  /**
   * Overwrites the odometer values based on the two light sensor correction results.
   * 
   * @param currentLine Estimated line that was used for the correction.
   */
  private void correctOdometer(int currentLine) {
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
        odometer.setXYT((currentLine * TILE_SIZE) - SENSOR_OFFSET, position[1], 270);
        break;
      default:
    }
  }

}
