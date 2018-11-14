package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import lejos.hardware.Sound;

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
public class OdometryCorrector extends Thread {
  
  // Constants
  private final double TILE_SIZE;
  private final double SENSOR_OFFSET;
  private final int INTERSECTION_MARGIN = 10;

  // Attributes
  private Odometer odometer;
  private LightPoller lightPoller;

  private Direction direction;
  private boolean enableCorrection;
  private boolean inLine;
  private boolean nextIntersectionIsVertical;
  public int xCount = 0;
  public int yCount = 0;

  /**
   * Creates an {@code OdometryCorrector} that is to be run on its own thread.
   * 
   * @throws PollerException If the {@code LightPoller} has not been instantiated.
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   */
  public OdometryCorrector(final double TILE_SIZE, final double SENSOR_OFFSET) throws PollerException, OdometerException {
    odometer = Odometer.getOdometer();
    lightPoller = LightPoller.getLightPoller();
    enableCorrection = true;
    nextIntersectionIsVertical = false;
    direction = Direction.INIT;
    
    this.TILE_SIZE = TILE_SIZE;
    this.SENSOR_OFFSET = SENSOR_OFFSET;
    
    disable();
  }

  /**
   * Checks for line detections on the lower light sensor as indicated by the {@code LightPoller}
   * and corrects the values of the robot's odometer.
   * 
   * @see LightPoller
   * @see Odometer
   */
  @Override
  public void run() {
    while(true) {
      if (lightPoller.leftInLine) {
        if (!inLine) {
          if (canCorrect() && enableCorrection) {
            updateDirection();
            processLineDetection();
          } else if (!canCorrect() && enableCorrection) {
            Sound.beep();
            Sound.beep();
            Sound.beep();
            Sound.beep();
          }
          updateHeading();
          inLine = true;
        }
      } else {
        inLine = false;
      }
    }
  }
  
  /**
   * TODO
   */
  public void updateHeading() {
    updateDirection();
    double[] position = odometer.getXYT();
    
    double distX = 0;
    double distY = 0;
    double theta = position[2];
    switch(direction) {
      case Q1:
        position[0] -= (SENSOR_OFFSET * Math.sin(Math.toRadians(theta))); 
        position[1] -= (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        distX = (TILE_SIZE - (position[0] % TILE_SIZE)) / Math.sin(Math.toRadians(theta));
        distY = (TILE_SIZE - (position[1] % TILE_SIZE)) / Math.cos(Math.toRadians(theta));
        break;
      case Q2:
        theta -= 90;
        position[0] -= (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        position[1] += (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
        distX = (TILE_SIZE - (position[0] % TILE_SIZE)) / Math.cos(Math.toRadians(theta));
        distY = (position[1] % TILE_SIZE) / Math.sin(Math.toRadians(theta));
        break;
      case Q3:
        theta -= 180;
        position[0] += (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
        position[1] += (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        distX = (position[0] % TILE_SIZE) / Math.sin(Math.toRadians(theta));
        distY = (position[1] % TILE_SIZE) / Math.cos(Math.toRadians(theta));
        break;
      case Q4:
        theta -= 270;
        position[0] += (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        position[1] -= (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
        distX = (position[0] % TILE_SIZE) / Math.cos(Math.toRadians(theta));
        distY = (TILE_SIZE - (position[1] % TILE_SIZE)) / Math.sin(Math.toRadians(theta));
        break;
      default:
        break;
    }
    
    if (distX < distY)
      nextIntersectionIsVertical = true;
    else
      nextIntersectionIsVertical = false;
  }
  
  /**
   * TODO
   */
  public void enable() {
    enableCorrection = true;
  }
  
  /**
   * TODO
   */
  public void disable() {
    enableCorrection = false;
  }
  
  /**
   * TODO
   * 
   * @return
   */
  public boolean isEnabled() {
    return enableCorrection;
  }
  
  /**
   * TODO
   */
  private void updateDirection() {
    double theta = odometer.getXYT()[2];
    if (theta > 0 && theta < 90) {
      direction = Direction.Q1;
    } else if (theta > 90 && theta < 180) {
      direction = Direction.Q2;
    } else if (theta > 180 && theta < 270) {
      direction = Direction.Q3;
    } else {
      direction = Direction.Q4;
    }
  }
  
  /**
   * TODO
   * 
   * @return
   */
  private boolean canCorrect() {
    double[] position = odometer.getXYT();
    double theta = position[2];
    
    switch(direction) {
      case Q1:
        position[0] -= (SENSOR_OFFSET * Math.sin(Math.toRadians(theta))); 
        position[1] -= (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        break;
      case Q2:
        theta -= 90;
        position[0] -= (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        position[1] += (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
        break;
      case Q3:
        theta -= 180;
        position[0] += (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
        position[1] += (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        break;
      case Q4:
        theta -= 270;
        position[0] += (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
        position[1] -= (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
        break;
      default:
        break;
    }
    
    double distXInt = Math.abs((position[0] % TILE_SIZE) - TILE_SIZE);
    double distYInt = Math.abs((position[1] % TILE_SIZE) - TILE_SIZE);
    if (distXInt < INTERSECTION_MARGIN && distYInt < INTERSECTION_MARGIN) {
      return false;
    }
    return true;
  }
  
  /**
   * TODO
   */
  private void processLineDetection() {
    
    // If the intersection if vertical the X coordinate is corrected.
    if (nextIntersectionIsVertical) {
      Sound.beep();
      double lineLocation = estimateLineLocation("X");
      double realX = 0;
      double[] position = odometer.getXYT();
      double theta = position[2];
      switch(direction) {
        case Q1:
          realX = lineLocation + (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
          break;
        case Q2:
          theta -= 90;
          realX = lineLocation + (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
          break;
        case Q3:
          theta -= 180;
          realX = lineLocation - (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
          break;
        case Q4:
          theta -= 270;
          realX = lineLocation - (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
          break;
        default:
          break;
      }
      odometer.setX(realX);
      //System.out.println(String.format("Corrected X: [%.2f] -> [%.2f]", position[0], realX));
      
    // Otherwise the Y coordinate is corrected.
    } else {
      Sound.beep();
      Sound.beep();
      double lineLocation = estimateLineLocation("Y");
      double realY = 0;
      double[] position = odometer.getXYT();
      double theta = position[2];
      switch(direction) {
        case Q1:
          realY = lineLocation + (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
          System.out.println("Correction: " + (SENSOR_OFFSET * Math.cos(Math.toRadians(theta))));
          break;
        case Q2:
          theta -= 90;
          realY = lineLocation - (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
          System.out.println("Correction: " + (SENSOR_OFFSET * Math.sin(Math.toRadians(theta))));
          break;
        case Q3:
          theta -= 180;
          realY = lineLocation - (SENSOR_OFFSET * Math.cos(Math.toRadians(theta)));
          System.out.println("Correction: " + (SENSOR_OFFSET * Math.cos(Math.toRadians(theta))));
          break;
        case Q4:
          theta -= 270;
          realY = lineLocation + (SENSOR_OFFSET * Math.sin(Math.toRadians(theta)));
          System.out.println("Correction: " + (SENSOR_OFFSET * Math.sin(Math.toRadians(theta))));
          break;
      }
      odometer.setY(realY);
      System.out.println(String.format("Corrected Y: [%.2f] -> [%.2f]", position[1], realY));
      System.out.println("Line location: " + lineLocation);
      System.out.println("Theta: " + theta);
    }
  }
  
  /**
   * TODO
   * 
   * @param coordinate
   * @return
   */
  private double estimateLineLocation(String coordinate) {
    double odoReading = coordinate.equals("X") ? odometer.getXYT()[0] : odometer.getXYT()[1];
    double distInTile = odoReading % TILE_SIZE;
    if (distInTile > TILE_SIZE / 2)
      return odoReading + (TILE_SIZE - distInTile);
    else
      return odoReading - distInTile;
  }

  private enum Direction {
    Q1, Q2, Q3, Q4, INIT
  }

}
