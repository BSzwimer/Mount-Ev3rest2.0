package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides localization functionality, which allows the robot to compute its initial location when
 * placed on the grid.
 * <p>
 * The {@code Localizator} class combines ultrasonic and light sensor localization into a single
 * routine that corrects the X, Y and Theta values of the odometer to the location of the robot with
 * respect to the grid.
 * 
 * @author angelortiz
 *
 */
public class Localizer {

  // Constants
  private static final int LOCALIZATION_PERIOD = 25;
  private static final int ROTATE_SPEED = 80;
  private static final int FORWARD_SPEED = 120;
  private static final int THRESHOLD = 45;
  private static final int CORRECTION_TIME_LIMIT = 2200;
  private final double SENSOR_OFFSET;
  private final double TILE_SIZE;

  // Object Attributes
  private Odometer odometer;
  private UltrasonicPoller usPoller;
  private LightPoller lightPoller;
  private Navigation navigation;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Localization Attributes
  private int prevDistance;
  private int currDistance;
  private boolean measurementTaken;
  private double alpha;
  private double beta;

  /**
   * Creates a {@code Localizator} that will operate on the singleton {@code Odometer} class.
   * 
   * @param leftMotor Lower left motor of the robot.
   * @param rightMotor Lower right motor of the robot.
   * @param navigation Navigation object to move the robot on the grid.
   * @param SENSOR_OFFSET Distance between the lower light sensor and the robot's center.
   * @param SENSOR_OFFSET_ANGLE Angle between the lower light sensor and the robot's center.
   * @param TILE_SIZE Size of a tile in the grid in centimeters.
   * 
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   * @throws PollerException If the {@code UltrasonicPoller} of the {@code LightPoller} classes have
   *         not been instantiated.
   * @see Odometer
   */
  public Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Navigation navigation, final double SENSOR_OFFSET, final double TILE_SIZE)
      throws OdometerException, PollerException {
    usPoller = UltrasonicPoller.getUltrasonicPoller();
    lightPoller = LightPoller.getLightPoller();
    odometer = Odometer.getOdometer();

    this.navigation = navigation;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.SENSOR_OFFSET = SENSOR_OFFSET;
    this.TILE_SIZE = TILE_SIZE;
  }

  public void tunnelLocalization() {
	  
	boolean wasEnabled = navigation.isCorrectionEnabled();
	navigation.disableCorrection();
	  
    double[] position;

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.forward();
    rightMotor.forward();

    findLine();
    position = odometer.getXYT();
    int currentLine = navigation.estimateCurrentLine();
    switch (navigation.direction) {
      case NORTH:
        odometer.setXYT(position[0], (currentLine * TILE_SIZE) + SENSOR_OFFSET, 0);
        break;
      case EAST:
        odometer.setXYT((currentLine * TILE_SIZE) + SENSOR_OFFSET, position[1], 90);
        break;
      case SOUTH:
        odometer.setXYT(position[0], (currentLine * TILE_SIZE) - SENSOR_OFFSET, 180);
        break;
      case WEST:
        odometer.setXYT((currentLine * TILE_SIZE) - SENSOR_OFFSET, position[1], 270);
        break;
    }
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), false);
    
    navigation.turnTo(odometer.getXYT()[2] - 90);
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, 2), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, 2), false);

    leftMotor.forward();
    rightMotor.forward();

    findLine();
    position = odometer.getXYT();
    currentLine = navigation.estimateCurrentLine();
    switch (navigation.direction) {
      case NORTH:
        odometer.setXYT(position[0], (currentLine * TILE_SIZE) + SENSOR_OFFSET, 0);
        break;
      case EAST:
        odometer.setXYT((currentLine * TILE_SIZE) + SENSOR_OFFSET, position[1], 90);
        break;
      case SOUTH:
        odometer.setXYT(position[0], (currentLine * TILE_SIZE) - SENSOR_OFFSET, 180);
        break;
      case WEST:
        odometer.setXYT((currentLine * TILE_SIZE) - SENSOR_OFFSET, position[1], 270);
        break;
    }
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), false);

    if (wasEnabled)
    		navigation.enableCorrection();
  }

  /**
   * Performs falling edge ultrasonic localization to correct the angle Theta of the odometer and
   * then it uses light localization to get the correct values of the X and Y coordinates of the
   * robot's location. The values obtained are used to overwrite the odometer's current values.
   * <p>
   * The starting corner parameter is used to adjust the odometer values according the the absolute
   * set of coordinated on the map. The corners are defined using the range {@code [0-3]} with 0
   * being the lower left corner and the range increasing in the counter-clockwise direction.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   * 
   */
  public void localize(int startingCorner, int xLim, int yLim) {
    ultrasonicLocalization(startingCorner);
    lightLocalization(startingCorner, xLim, yLim);
  }

  /**
   * Estimates the angle of the robot with respect to the grid by rotating until a falling edge is
   * detected with the wall on each side. The angle in the middle of the two falling edge detections
   * is used to estimate the robot's real angle.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   * 
   */
  private void ultrasonicLocalization(int startingCorner) {
    boolean wasEnabled = navigation.isCorrectionEnabled();
    navigation.disableCorrection();

    long updateStart, updateEnd;
    prevDistance = -1;
    boolean firstSearch = true;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    // Falling edge localization
    while (true) {
      updateStart = System.currentTimeMillis();
      currDistance = usPoller.poll();
      if (prevDistance < 0) {
        prevDistance = currDistance;
        continue;
      }
      if (prevDistance >= currDistance && prevDistance > THRESHOLD) { // Distance increasing
        if (!measurementTaken && currDistance < THRESHOLD) {
          if (firstSearch) {
            beta = odometer.getXYT()[2];
            leftMotor.backward();
            rightMotor.forward();
            firstSearch = false;
            measurementTaken = true;
          } else {
            alpha = odometer.getXYT()[2];
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            break;
          }
        }
      } else if (prevDistance < currDistance) { // Distance decreasing
        measurementTaken = false;
      }
      prevDistance = currDistance;
      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < LOCALIZATION_PERIOD) {
        try {
          Thread.sleep(LOCALIZATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    correctAngle();

    // Adjust to starting position
    switch (startingCorner) {
      case 0:
        break;
      case 1:
        odometer.update(0, 0, 270);
        break;
      case 2:
        odometer.update(0, 0, 180);
        break;
      case 3:
        odometer.update(0, 0, 90);
        break;
    }

    if (wasEnabled)
      navigation.enableCorrection();

  }

  /**
   * Provides a more accurate set of values for X, Y and Theta to the odometer by using light
   * localization.
   * <p>
   * The light localization routine consists of approaching the reference corner and spinning until
   * four line detections are recorded. The the angles recorded during the line detections are used
   * to correct the odometer values.
   * <p>
   * This method requires that the robot is placed around the middle of a tile and that the odometer
   * knows the angle of the robot to some degree of accuracy. A reference corner and its coordinates
   * must be provided to use to do the localization. The corners are encoded using the range
   * {@code [0-3]} with 0 being the the upper right corner of the tile where the robot lies, and the
   * range increasing in the counter-clockwise direction.
   * 
   * @param referenceCorner Reference corner to use for line localization subroutine.
   * @param refX X coordinate of the reference corner.
   * @param refY Y coordinate of the reference corner.
   */
  public void lightLocalization(int startingCorner, int xLim, int yLim) {
    boolean wasEnabled = navigation.isCorrectionEnabled();
    navigation.disableCorrection();

    // Localize in Y
    navigation.turnTo(0);

    double[] initialPosition = odometer.getXYT();

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    findLine();
    double[] currPosition = odometer.getXYT();
    odometer.setXYT(currPosition[0], SENSOR_OFFSET, 0);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, initialPosition[1] - currPosition[1]),
        true);
    rightMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, initialPosition[1] - currPosition[1]),
        false);

    // Localize in X
    navigation.turnTo(90);

    initialPosition = odometer.getXYT();

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    findLine();
    currPosition = odometer.getXYT();
    odometer.setXYT(SENSOR_OFFSET, currPosition[1], 90);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, initialPosition[0] - currPosition[0]),
        true);
    rightMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, initialPosition[0] - currPosition[0]),
        false);

    switch (startingCorner) {
      case 0:
        odometer.update(TILE_SIZE, TILE_SIZE, 0);
        break;
      case 1:
        odometer.update(TILE_SIZE * (xLim - 1), TILE_SIZE, 0);
        break;
      case 2:
        odometer.update(TILE_SIZE * (xLim - 1), TILE_SIZE * (yLim - 1), 0);
        break;
      case 3:
        odometer.update(TILE_SIZE, TILE_SIZE * (yLim - 1), 0);
        break;
    }

    navigation.travelTo(1, 1);
    navigation.waitNavigation();
    navigation.turnTo(0);
    Button.waitForAnyPress();

    if (wasEnabled)
      navigation.enableCorrection();
  }

  /**
   * TODO
   */
  private void findLine() {
    boolean leftInLine = false;
    boolean rightInLine = true;

    while (true) {
      if (lightPoller.leftInLine) {
        if (!leftInLine) {
          leftInLine = true;
          if (!lightPoller.rightInLine)
            adjustTrajectory(1);
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          break;
        }
      } else {
        leftInLine = false;
      }

      if (lightPoller.rightInLine) {
        if (!rightInLine) {
          rightInLine = true;
          if (!lightPoller.leftInLine)
            adjustTrajectory(0);
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          break;
        }
      } else {
        rightInLine = false;
      }
    }
  }

  /**
   * TODO
   * 
   * @param laggingSide
   */
  private void adjustTrajectory(int laggingSide) {
    boolean goBack = false;
    long startTime = System.currentTimeMillis();
    int prevTacho = 0;

    // Correct the direction
    if (laggingSide == 0) {
      rightMotor.setSpeed(0);
      leftMotor.setSpeed(ROTATE_SPEED / 2);
      prevTacho = leftMotor.getTachoCount();
      while (true) {
        if (lightPoller.leftInLine) {
          break;
        } else if (System.currentTimeMillis() - startTime > CORRECTION_TIME_LIMIT) {
          goBack = true;
          break;
        }
        try {
          Thread.sleep(LOCALIZATION_PERIOD);
        } catch (InterruptedException e) {
        }
      }
    } else if (laggingSide == 1) {
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(ROTATE_SPEED / 2);
      prevTacho = rightMotor.getTachoCount();
      while (true) {
        if (lightPoller.rightInLine) {
          break;
        } else if (System.currentTimeMillis() - startTime > CORRECTION_TIME_LIMIT) {
          goBack = true;
          break;
        }
        try {
          Thread.sleep(LOCALIZATION_PERIOD);
        } catch (InterruptedException e) {
        }
      }
    }

    if (goBack) {
      if (laggingSide == 0)
        leftMotor.rotate(prevTacho - leftMotor.getTachoCount());
      else if (laggingSide == 1)
        rightMotor.rotate(prevTacho - rightMotor.getTachoCount());
      return;
    }
  }

  /**
   * Uses angle measurements from rising or falling edge methods to correct the odometer's theta
   * reading.
   */
  private void correctAngle() {
    double[] position = odometer.getXYT();
    double correctedTheta;
    if (alpha < beta) {
      correctedTheta = position[2] + 45 - ((alpha + beta) / 2);
    } else {
      correctedTheta = position[2] + 225 - ((alpha + beta) / 2);
    }
    odometer.setTheta(correctedTheta);
  }

}
