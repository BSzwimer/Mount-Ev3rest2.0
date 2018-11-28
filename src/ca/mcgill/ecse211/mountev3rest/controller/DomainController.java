package ca.mcgill.ecse211.mountev3rest.controller;

import ca.mcgill.ecse211.mountev3rest.navigation.Display;
import ca.mcgill.ecse211.mountev3rest.navigation.Localizer;
import ca.mcgill.ecse211.mountev3rest.navigation.Navigation;
import ca.mcgill.ecse211.mountev3rest.navigation.Odometer;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometryCorrector;
import ca.mcgill.ecse211.mountev3rest.sensor.ColorDetector;
import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.mountev3rest.util.ArmController;
import ca.mcgill.ecse211.mountev3rest.util.CoordinateMap;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Provides an interface to execute the main subtasks required for the robot to perform the overall
 * routine.
 * <p>
 * The {@code DomainController} represents the low layer of the controller in the system. The class
 * is not concerned with the general state of the main routine execution. Instead, the
 * {@code DomainController} handles all the details related to each of the subtasks required for the
 * main routine. This means that internally, the {@code DomainCotroller} handles all the required
 * calls to more specialized classes and ensures that the requested subtask is completed.
 * <p>
 * It is assumed that the caller of the {@code DomainController} methods (usually the
 * {@code MetaController}) will conform to the external requirements of each subtask. For instance,
 * a call to {@code crossTunnel()} will not produce the expected outcome if the odometer of the
 * robot has not been set to the right initial values through the {@code localize()} method.
 * 
 * @see MetaController
 * 
 * @author angelortiz
 *
 */
public class DomainController {

  // Constants
  private static final double TRACK = 8.45;
  private static final double WHEEL_RADIUS = 2.05;
  private static final double TILE_SIZE = 30.48;
  private static final double MOTOR_OFFSET = 1.015;
  private static final double SENSOR_OFFSET = -2.3;
  private static final int MIN_DIST_TO_TREE = 5;
  private static final int MIN_DIST_TO_AVOID = 20;
  private static final double CORRECTION_DIST = 3;
  private static final double SMALL_DIST = 0.5;

  // Attributes
  CoordinateMap map;
  Odometer odometer;
  OdometryCorrector odometryCorrector;
  Navigation navigation;
  Localizer localizer;
  UltrasonicPoller usPoller;
  LightPoller lightPoller;
  ArmController armController;
  ColorDetector colorDetector;

  // Status attributes
  Zone zone;

  // Threads
  Thread odoThread;
  Thread navThread;

  /**
   * Creates a {@code DomainController} and initializes all the required specialized classes.
   * <p>
   * Instantiation of the {@code DomainController} involves initializing the singleton
   * {@code Odometer}, {@code LightPoller}, and {@code UltrasonicPoller} classes, which is required
   * for the instantiation of other classes.
   * 
   * @throws OdometerException If there is a problem while creating the Odometer singleton instance.
   * @throws PollerException If there is a problem while instantiating the UltrasonicPoller an
   *         LightPoller objects.
   * 
   * @see Odometer
   * @see LightPoller
   * @see UltrasonicPoller
   */
  public DomainController() throws OdometerException, PollerException {

    // Get motor objects
    EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    EV3MediumRegulatedMotor colorSensorMotor =
        new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
    EV3MediumRegulatedMotor armMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

    // Instantiate the sensors
    EV3ColorSensor rightLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    EV3ColorSensor topLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
    EV3ColorSensor leftLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
    EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

    // Create the specialized objects
    usPoller = UltrasonicPoller.getUltrasonicPoller(usSensor);
    lightPoller = LightPoller.getLightPoller(topLightSensor, leftLightSensor, rightLightSensor);
    odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS, MOTOR_OFFSET);
    odometryCorrector =
        new OdometryCorrector(leftMotor, rightMotor, TILE_SIZE, SENSOR_OFFSET, MOTOR_OFFSET);
    navigation =
        new Navigation(leftMotor, rightMotor, odometryCorrector, WHEEL_RADIUS, TRACK, MOTOR_OFFSET);
    localizer = new Localizer(leftMotor, rightMotor, navigation, odometryCorrector, SENSOR_OFFSET,
        TILE_SIZE);
    colorDetector = new ColorDetector(LocalEV3.get().getTextLCD());
    armController = new ArmController(colorSensorMotor, armMotor, leftMotor, rightMotor, navigation,
        colorDetector, SENSOR_OFFSET);

    // Initialize the zone enumeration
    zone = Zone.START;

    // Initialize and start the required extra threads
    odoThread = new Thread(odometer);
    navThread = new Thread(navigation);
    odoThread.start();
    navThread.start();
  }

  /**
   * Sets the map coordinates to be used during each of the tasks.
   * 
   * @param map Map to be used during the tasks.
   */
  public void setMap(CoordinateMap map) {
    this.map = map;
  }

  /**
   * Uses the {@code Localizer} class to provide the odometer with an initial set of coordinates
   * that correspond to the location of the robot with respect to the grid.
   * 
   * @see Localizer
   */
  public void localize() {
    localizer.localize(map.StartCorner, map.LL_x, map.LL_y, map.UR_x, map.UR_y);
  }


  /**
   * TODO
   * 
   * @param targetX
   * @param preferredDetourDirectionY
   * @param direction
   */
  public boolean getToX(double targetX, double preferredDetourDirectionY) {
    double[] position = odometer.getXYT();
    boolean treeInWay;
    boolean otherTreeInWay;
    int coordToAvoid = -1;
    int coordBeforeCollision = -1;
    boolean tookDetour;
    boolean detoured = false;
    Traversal direction;

    if (position[0] < targetX * TILE_SIZE)
      direction = Traversal.EAST;
    else
      direction = Traversal.WEST;

    do {
      tookDetour = false;

      position = odometer.getXYT();
      treeInWay = false;
      otherTreeInWay = false;

      if (Math.abs(map.T_y * TILE_SIZE - position[1]) < MIN_DIST_TO_AVOID
          && ((direction == Traversal.EAST && position[0] < map.T_x * TILE_SIZE
              && map.T_x <= targetX)
              || (direction == Traversal.WEST && position[0] > map.T_x * TILE_SIZE
                  && map.T_x >= targetX))) {
        treeInWay = true;
        coordToAvoid = (int) map.T_y;
        tookDetour = true;
        detoured = true;
      }
      if (Math.abs(map.T_y_o * TILE_SIZE - position[1]) < MIN_DIST_TO_AVOID
          && ((direction == Traversal.EAST && position[0] < map.T_x_o * TILE_SIZE
              && map.T_x_o <= targetX)
              || (direction == Traversal.WEST && position[0] > map.T_x_o * TILE_SIZE
                  && map.T_x_o >= targetX))) {
        otherTreeInWay = true;
        coordToAvoid = (int) map.T_y_o;
        tookDetour = true;
        detoured = true;
      }

      treeInWay = false;
      otherTreeInWay = false;
      tookDetour = false;

      if (treeInWay && otherTreeInWay)
        if (Math.abs(map.T_x * TILE_SIZE - position[0]) < Math
            .abs(map.T_x_o * TILE_SIZE - position[0])) {
          if (direction == Traversal.EAST) {
            navigation.travelToX(map.T_x - 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_x - 1);
          } else if (direction == Traversal.WEST) {
            navigation.travelToX(map.T_x + 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_x + 1);
          }
        } else {
          if (direction == Traversal.EAST) {
            navigation.travelToX(map.T_x_o - 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_x_o - 1);
          } else if (direction == Traversal.WEST) {
            navigation.travelToX(map.T_x_o + 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_x_o + 1);
          }
        }
      else if (treeInWay) {
        if (direction == Traversal.EAST) {
          navigation.travelToX(map.T_x - 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_x - 1);
        } else if (direction == Traversal.WEST) {
          navigation.travelToX(map.T_x + 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_x + 1);
        }
      } else if (otherTreeInWay) {
        if (direction == Traversal.EAST) {
          navigation.travelToX(map.T_x_o - 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_x_o - 1);
        } else if (direction == Traversal.WEST) {
          navigation.travelToX(map.T_x_o + 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_x_o + 1);
        }
      }

      if (tookDetour) {
        if (Math.abs(preferredDetourDirectionY - (coordToAvoid - 1)) < Math
            .abs(preferredDetourDirectionY - (coordToAvoid + 1))) {
          if (zoneContains(coordToAvoid - 1, coordBeforeCollision, false)
              && coordToAvoid - 1 != preferredDetourDirectionY) {
            navigation.travelToY(coordToAvoid - 1);
            navigation.waitNavigation();
          } else {
            navigation.travelToY(coordToAvoid + 1);
            navigation.waitNavigation();
          }
        } else {
          if (zoneContains(coordToAvoid + 1, coordBeforeCollision, false)
              && coordToAvoid + 1 != preferredDetourDirectionY) {
            navigation.travelToY(coordToAvoid + 1);
            navigation.waitNavigation();
          } else {
            navigation.travelToY(coordToAvoid - 1);
            navigation.waitNavigation();
          }
        }
      } else {
        navigation.travelToX(targetX);
        navigation.waitNavigation();
      }
    } while (tookDetour);

    // return detoured;
    return false;
  }

  /**
   * TODO
   * 
   * @param targetY
   * @param preferredDetourDirectionX
   * @param direction
   */
  private boolean getToY(double targetY, double preferredDetourDirectionX) {
    double[] position = odometer.getXYT();;
    boolean treeInWay;
    boolean otherTreeInWay;
    int coordToAvoid = -1;
    int coordBeforeCollision = -1;
    boolean tookDetour;
    boolean detoured = false;
    Traversal direction;

    if (position[1] < targetY * TILE_SIZE)
      direction = Traversal.NORTH;
    else
      direction = Traversal.SOUTH;

    do {
      tookDetour = false;

      position = odometer.getXYT();
      treeInWay = false;
      otherTreeInWay = false;

      if (Math.abs(map.T_x * TILE_SIZE - position[0]) < MIN_DIST_TO_AVOID
          && ((direction == Traversal.NORTH && position[1] < map.T_y * TILE_SIZE
              && map.T_y <= targetY)
              || (direction == Traversal.SOUTH && position[1] > map.T_y * TILE_SIZE
                  && map.T_y >= targetY))) {
        treeInWay = true;
        coordToAvoid = (int) map.T_x;
        tookDetour = true;
        detoured = true;
      }
      if (Math.abs(map.T_x_o * TILE_SIZE - position[0]) < MIN_DIST_TO_AVOID
          && ((direction == Traversal.NORTH && position[1] < map.T_y_o * TILE_SIZE
              && map.T_y_o <= targetY)
              || (direction == Traversal.SOUTH && position[1] > map.T_y_o * TILE_SIZE
                  && map.T_y_o >= targetY))) {
        otherTreeInWay = true;
        coordToAvoid = (int) map.T_x_o;
        tookDetour = true;
        detoured = true;
      }

      treeInWay = false;
      otherTreeInWay = false;
      tookDetour = false;
      if (treeInWay && otherTreeInWay)
        if (Math.abs(map.T_y * TILE_SIZE - position[1]) < Math
            .abs(map.T_y_o * TILE_SIZE - position[1])) {
          if (direction == Traversal.NORTH) {
            navigation.travelToY(map.T_y - 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_y - 1);
          } else if (direction == Traversal.SOUTH) {
            navigation.travelToY(map.T_y + 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_y + 1);
          }
        } else {
          if (direction == Traversal.NORTH) {
            navigation.travelToY(map.T_y_o - 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_y_o - 1);
          } else if (direction == Traversal.SOUTH) {
            navigation.travelToY(map.T_y_o + 1);
            navigation.waitNavigation();
            coordBeforeCollision = (int) (map.T_y_o + 1);
          }
        }
      else if (treeInWay) {
        if (direction == Traversal.NORTH) {
          navigation.travelToY(map.T_y - 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_y - 1);
        } else if (direction == Traversal.SOUTH) {
          navigation.travelToY(map.T_y + 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_y + 1);
        }
      } else if (otherTreeInWay) {
        if (direction == Traversal.NORTH) {
          navigation.travelToY(map.T_y_o - 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_y_o - 1);
        } else if (direction == Traversal.SOUTH) {
          navigation.travelToY(map.T_y_o + 1);
          navigation.waitNavigation();
          coordBeforeCollision = (int) (map.T_y_o + 1);
        }
      }

      if (tookDetour) {
        if (Math.abs(preferredDetourDirectionX - (coordToAvoid - 1)) < Math
            .abs(preferredDetourDirectionX - (coordToAvoid + 1))) {
          if (zoneContains(coordToAvoid - 1, coordBeforeCollision, false)
              && coordToAvoid - 1 != preferredDetourDirectionX) {
            navigation.travelToX(coordToAvoid - 1);
            navigation.waitNavigation();
          } else {
            navigation.travelToX(coordToAvoid + 1);
            navigation.waitNavigation();
          }
        } else {
          if (zoneContains(coordToAvoid + 1, coordBeforeCollision, false)
              && coordToAvoid + 1 != preferredDetourDirectionX) {
            navigation.travelToX(coordToAvoid + 1);
            navigation.waitNavigation();
          } else {
            navigation.travelToX(coordToAvoid - 1);
            navigation.waitNavigation();
          }
        }
      } else {
        navigation.travelToY(targetY);
        navigation.waitNavigation();
      }
    } while (tookDetour);

    // return detoured;
    return false;
  }


  public void crossTunnel() {
    Traversal traversal = null;

    boolean wasEnabled = odometryCorrector.isEnabled();

    // Determine how to traverse the tunnel
    if (zoneContains(map.TN_LL_x, map.TN_LL_y) && zoneContains(map.TN_UR_x, map.TN_LL_y))
      traversal = Traversal.NORTH;
    else if (zoneContains(map.TN_UR_x, map.TN_LL_y) && zoneContains(map.TN_UR_x, map.TN_UR_y))
      traversal = Traversal.WEST;
    else if (zoneContains(map.TN_LL_x, map.TN_UR_y) && zoneContains(map.TN_UR_x, map.TN_UR_y))
      traversal = Traversal.SOUTH;
    else if (zoneContains(map.TN_LL_x, map.TN_LL_y) && zoneContains(map.TN_LL_x, map.TN_UR_y))
      traversal = Traversal.EAST;

    boolean detoured = false;

    switch (traversal) {
      case NORTH: // Bridge is placed vertically and the robot is closer to the lower left corner.
        navigation.travelToY(map.TN_LL_y - 1);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_LL_x + 0.5);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_LL_y - 0.9);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.highSpeedOn();
        navigation.travelToY(map.TN_UR_y + 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        break;
      case WEST: // Bridge is placed horizontally and the robot is closer to the upper right corner.
        navigation.travelToX(map.TN_UR_x + 1);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_UR_y - 0.5);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_UR_x + 0.9);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.highSpeedOn();
        navigation.travelToX(map.TN_LL_x - 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        break;
      case SOUTH: // Bridge is placed vertically and the robot is closer to the upper right corner.
        navigation.travelToY(map.TN_UR_y + 1);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_UR_x - 0.5);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_UR_y + 0.9);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.highSpeedOn();
        navigation.travelToY(map.TN_LL_y - 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        break;
      case EAST: // Bridge is placed horizontally and the robot is closer to the lower left corner.
        navigation.travelToX(map.TN_LL_x - 1);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_LL_y + 0.5);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_LL_x - 0.9);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.highSpeedOn();
        navigation.travelToX(map.TN_UR_x + 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        break;
    }

    // Update the current zone
    switch (zone) {
      case START:
        zone = Zone.SEARCH;
        break;
      case SEARCH:
        zone = Zone.START;
        break;
    }

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * Crosses the tunnel specified by the map given coordinates. The method makes sure that the robot
   * reaches the closest entrance of the tunnel, then it moves through it until the robot is
   * completely outside on the other side.
   * 
   * @see CoordinateMap
   */
  public void crossTunnelOld() {
    double LL_dist = navigation.computeDistance(map.TN_LL_x, map.TN_LL_y);
    double UR_dist = navigation.computeDistance(map.TN_UR_x, map.TN_UR_y);

    boolean wasEnabled = odometryCorrector.isEnabled();

    if (map.TN_UR_x - map.TN_LL_x == 2) { // Bridge is placed horizontally.
      if (LL_dist < UR_dist) { // Robot is closer to the lower left corner.
        navigation.travelToX(map.TN_LL_x - 1);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_LL_y + 0.5);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_LL_x - 0.8);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.travelToX(map.TN_UR_x + 1);
        navigation.waitNavigation();
      } else { // Robot is closer to the upper right corner.
        navigation.travelToX(map.TN_UR_x + 1);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_UR_y - 0.5);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_UR_x + 0.8);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.travelToX(map.TN_LL_x - 1);
        navigation.waitNavigation();
      }
    } else { // Bridge is placed vertically.
      if (LL_dist < UR_dist) { // Robot is closer to the lower left corner.
        navigation.travelToY(map.TN_LL_y - 1);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_LL_x + 0.5);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_LL_y - 0.8);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.travelToY(map.TN_UR_y + 1);
        navigation.waitNavigation();
      } else { // Robot is closer to the upper right corner.
        navigation.travelToY(map.TN_UR_y + 1);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_UR_x - 0.5);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_UR_y + 0.8);
        navigation.waitNavigation();
        odometryCorrector.disable();
        odometryCorrector.correctOnNextLine(true);
        navigation.travelToY(map.TN_LL_y - 1);
        navigation.waitNavigation();
      }
    }

    // localizer.tunnelLocalization();

    if (wasEnabled)
      odometryCorrector.enable();
  }

  public void approachTree() {
    boolean wasEnabled = odometryCorrector.isEnabled();
    odometryCorrector.disable();

    double[] position = odometer.getXYT();
    if (position[0] < map.T_x * TILE_SIZE && position[1] < map.T_y * TILE_SIZE) {
      if (navigation.computeDistance(map.T_x - 1, map.T_y) < navigation.computeDistance(map.T_x,
          map.T_y - 1)) {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x - 1, map.T_y - SMALL_DIST);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(0);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(90);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      } else {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x - SMALL_DIST, map.T_y - 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(90);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(0);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      }
    } else if (position[0] < map.T_x * TILE_SIZE && position[1] > map.T_y * TILE_SIZE) {
      if (navigation.computeDistance(map.T_x - 1, map.T_y) < navigation.computeDistance(map.T_x,
          map.T_y + 1)) {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x - 1, map.T_y + SMALL_DIST);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(180);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(90);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      } else {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x - SMALL_DIST, map.T_y + 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(90);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(180);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      }
    } else if (position[0] > map.T_x * TILE_SIZE && position[1] > map.T_y * TILE_SIZE) {
      if (navigation.computeDistance(map.T_x + 1, map.T_y) < navigation.computeDistance(map.T_x,
          map.T_y + 1)) {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x + 1, map.T_y + SMALL_DIST);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(180);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(270);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      } else {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x + SMALL_DIST, map.T_y + 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(270);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(180);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      }
    } else if (position[0] > map.T_x * TILE_SIZE && position[1] < map.T_y * TILE_SIZE) {
      if (navigation.computeDistance(map.T_x + 1, map.T_y) < navigation.computeDistance(map.T_x,
          map.T_y - 1)) {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x + 1, map.T_y - SMALL_DIST);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(0);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(270);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      } else {
        navigation.highSpeedOn();
        navigation.travelTo(map.T_x + SMALL_DIST, map.T_y - 1);
        navigation.waitNavigation();
        navigation.highSpeedOff();
        navigation.turnTo(270);
        odometryCorrector.correctOnNextLine(false);
        navigation.advanceDist(-SENSOR_OFFSET);
        navigation.waitNavigation();
        navigation.turnTo(0);
        navigation.advanceDist(CORRECTION_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);
      }
    }

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * Approaches the robot to the tree containing the ring set and positions it looking into the
   * nearest face.
   */
  public void approachTreeOld() {
    boolean inY = false;
    boolean inX = false;

    double[] position = odometer.getXYT();
    if (Math.abs(position[1] - TILE_SIZE * (map.T_y)) < MIN_DIST_TO_TREE) {
      inY = true;
    } else if (Math.abs(position[1] - TILE_SIZE * (map.T_y - 1)) < Math
        .abs(position[1] - TILE_SIZE * (map.T_y + 1))) {
      navigation.travelToY(map.T_y - 1);
      navigation.waitNavigation();
    } else {
      navigation.travelToY(map.T_y + 1);
      navigation.waitNavigation();
    }

    position = odometer.getXYT();
    if (Math.abs(position[0] - TILE_SIZE * (map.T_x)) < MIN_DIST_TO_TREE) {
      inX = true;
    } else if (Math.abs(position[0] - TILE_SIZE * (map.T_x - 1)) < Math
        .abs(position[0] - TILE_SIZE * (map.T_x + 1))) {
      navigation.travelToX(map.T_x - 1);
      navigation.waitNavigation();
    } else {
      navigation.travelToX(map.T_x + 1);
      navigation.waitNavigation();
    }

    if (!inX && !inY) {
      navigation.travelToY(map.T_y);
      navigation.waitNavigation();
      inY = true;
    }

    position = odometer.getXYT();
    if (inY) {
      if (position[0] < TILE_SIZE * map.T_x) {
        navigation.turnTo(90);
      } else {
        navigation.turnTo(270);
      }
    } else if (inX) {
      if (position[1] < TILE_SIZE * map.T_y) {
        navigation.turnTo(0);
      } else {
        navigation.turnTo(180);
      }
    }
  }

  /**
   * Moves the robot to the next tree face in the counter-clockwise direction. This method assumes
   * the the robot is already looking into one of the three faces. This can be achieved by calling
   * {@code approachTree()}.
   */
  public void goToNextFace(boolean turn) {
    odometryCorrector.updateDirection();

    switch (odometryCorrector.direction) {
      case NORTH:
        navigation.turnTo(90);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToX(map.T_x + 1);
        navigation.waitNavigation();

        navigation.travelToY(map.T_y);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(270);
        break;
      case EAST:
        navigation.turnTo(180);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToY(map.T_y - 1);
        navigation.waitNavigation();

        navigation.travelToX(map.T_x);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(0);
        break;
      case SOUTH:
        navigation.turnTo(270);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToX(map.T_x - 1);
        navigation.waitNavigation();

        navigation.travelToY(map.T_y);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(90);
        break;
      case WEST:
        navigation.turnTo(0);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToY(map.T_y + 1);
        navigation.waitNavigation();

        navigation.travelToX(map.T_x);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(180);
        break;
      default:
        break;
    }
  }

  /**
   * Moves the robot to the next tree face in the counter-clockwise direction. This method assumes
   * the the robot is already looking into one of the three faces. This can be achieved by calling
   * {@code approachTree()}.
   */
  public void goToPrevFace(boolean turn) {
    odometryCorrector.updateDirection();

    switch (odometryCorrector.direction) {
      case NORTH:
        navigation.turnTo(270);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToX(map.T_x - 1);
        navigation.waitNavigation();

        navigation.travelToY(map.T_y);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(90);
        break;
      case EAST:
        navigation.turnTo(0);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToY(map.T_y + 1);
        navigation.waitNavigation();

        navigation.travelToX(map.T_x);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(180);
        break;
      case SOUTH:
        navigation.turnTo(90);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToX(map.T_x + 1);
        navigation.waitNavigation();

        navigation.travelToY(map.T_y);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(270);
        break;
      case WEST:
        navigation.turnTo(180);
        navigation.advanceDist(SMALL_DIST);
        navigation.waitNavigation();
        odometryCorrector.correctOnNextLine(true);

        navigation.travelToY(map.T_y - 1);
        navigation.waitNavigation();

        navigation.travelToX(map.T_x);
        navigation.waitNavigation();

        if (turn)
          navigation.turnTo(0);
        break;
      default:
        break;
    }
  }

  /**
   * Approaches the tree slowly, detects the color of a ring if any and attempts to get it. This
   * method assumes that the robot is already looking into a face of the tree and positioned on the
   * intersection right in front of it. If a color is detected a sequence of beeps matching the
   * color encoding of the {@code ColorDetector} is issued. Once the collection is completed the
   * robot goes back to the initial intersection where is was located before the method call.
   * 
   * @see ColorDetector
   * @see ArmController
   */
  public void grabRings(boolean correct) {
    boolean wasEnabled = odometryCorrector.isEnabled();
    odometryCorrector.disable();

    if (correct) {
      navigation.advanceDist(2.5);
      navigation.waitNavigation();
      odometryCorrector.correctOnNextLine(true);
    }

    armController.getRing();

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * TODO
   */
  public void releaseRings() {
    boolean wasEnabled = odometryCorrector.isEnabled();
    odometryCorrector.disable();

    navigation.highSpeedOn();

    switch ((int) map.StartCorner) {
      case 0:
        navigation.travelTo(map.LL_x + 1, map.LL_y + 1);
        break;
      case 1:
        navigation.travelTo(map.UR_x - 1, map.LL_y + 1);
        break;
      case 2:
        navigation.travelTo(map.UR_x - 1, map.UR_y - 1);
        break;
      case 3:
        navigation.travelTo(map.LL_x + 1, map.UR_y - 1);
        break;
    }
    navigation.waitNavigation();

    armController.releaseRing();

    navigation.highSpeedOff();

    if (wasEnabled)
      odometryCorrector.enable();
  }


  /* ---PRIVATE METHODS--- */


  private boolean zoneContains(long x, long y) {
    return zoneContains(x, y, true);
  }

  /**
   * TODO
   * 
   * @param x
   * @param y
   * @return
   */
  private boolean zoneContains(long x, long y, boolean includeBoundary) {
    int LL_x, LL_y, UR_x, UR_y;

    switch (zone) {
      case START:
        LL_x = (int) map.LL_x;
        LL_y = (int) map.LL_y;
        UR_x = (int) map.UR_x;
        UR_y = (int) map.UR_y;
        break;
      case SEARCH:
        LL_x = (int) map.I_LL_x;
        LL_y = (int) map.I_LL_y;
        UR_x = (int) map.I_UR_x;
        UR_y = (int) map.I_UR_y;
        break;
      default:
        return false;
    }

    if (includeBoundary) {
      if (x >= LL_x && x <= UR_x && y >= LL_y && y <= UR_y)
        return true;
    } else {
      if (x > LL_x && x < UR_x && y > LL_y && y < UR_y)
        return true;
    }

    return false;
  }

  // REMOVE
  public void testNavigation() throws OdometerException {

    TextLCD lcd = LocalEV3.get().getTextLCD();
    lcd.drawString("      READY      ", 0, 4);

    // Button.waitForAnyPress();

    Display display = new Display(lcd);
    Thread disThread = new Thread(display);
    disThread.start();

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // odometryCorrector.disable();

    /*odometer.setXYT(TILE_SIZE * 1, TILE_SIZE * 1, 0);
    navigation.travelToY(5);
    navigation.waitNavigation();
    navigation.travelToX(6);
    navigation.waitNavigation();*/


    navigation.turnTo(90);
    Button.waitForAnyPress();
    navigation.turnTo(180);
    Button.waitForAnyPress();
    navigation.turnTo(90);
    Button.waitForAnyPress();
    navigation.turnTo(0);
    Button.waitForAnyPress();


    lcd.clear();
    lcd.drawString("       DONE       ", 0, 4);

    Button.waitForAnyPress();
  }

  // REMOVE
  public void testColorDetection() {
    TextLCD lcd = LocalEV3.get().getTextLCD();
    ColorDetector cd = null;
    try {
      cd = new ColorDetector(lcd);
    } catch (PollerException e) {
      e.printStackTrace();
    }
    // cd.demoDetection();
    cd.printRed();
  }

  private enum Zone {
    START, SEARCH
  }

  private enum Traversal {
    NORTH, EAST, SOUTH, WEST
  }

}
