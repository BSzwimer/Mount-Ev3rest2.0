package ca.mcgill.ecse211.mountev3rest.controller;

import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.util.ArmController;
import ca.mcgill.ecse211.mountev3rest.util.Map;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Performs all the high level tasks required to complete the overall routine.
 * 
 * The {@code MetaController} class represents the high level of the controller. This class's
 * concern is to ensure that the robot completes the entire routine consisting of reaching a tree of
 * rings, obtaining the rings, and returning them back to the initial location. To achieve this
 * goal, the {@code MetaController} determines which calls should be made to the
 * {@code DomainController} and how the robot should react to the outcome of those calls. The
 * {@code DomainController} class in turn handles the subtleties of each subtask.
 * 
 * Additionally, the {@code MetaController} makes sure that the appropriate external conditions are met
 * before requesting that a given subtask is performed. These external conditions are defined by the
 * executor of the subtask (usually the {@code DomainController}).
 * 
 * @see MetaController
 * 
 * @author angelortiz
 *
 */
public class MetaController {
  
  public static void main(String[] args) throws OdometerException, PollerException {
    double[][] points = {{3.5, 1.5}, {3.5, 3.5}};
    
    TextLCD lcd = LocalEV3.get().getTextLCD();
    
    lcd.drawString("Press any button", 0, 3);
    lcd.drawString("    to start.   ", 0, 4);
    
    Button.waitForAnyPress();
    
    DomainController controller = new DomainController();
    //controller.testNavigation(true, 0, points, lcd);
    controller.grabRings(0);
    //controller.testColorDetection(lcd);
    
    System.exit(0);
  }

  // Attributes
  DomainController domainController;
  Map map;
  int ringCapacity;
  int ringsLeft;
  boolean ringsIdentified;
  int[][] ringMap;

  /**
   * Creates a high level controller that can perform ring collection as a set of subtasks.
   * 
   * @param totalRings Total rings to be found in the tree.
   */
  public MetaController(int totalRings) {
    this.ringsLeft = totalRings;
    ringsIdentified = false;
    ringMap = new int[4][2];

    //domainController = new DomainController();
    map = new Map();
  }

  /**
   * Performs all the required subtasks involved in the ring collection routine. The method runs on
   * an infinite loop that exits when all the rings have been collected.
   */
  public void run() {
    // Get Wi-Fi information.
    // Map map = new WifiClass().getInfo();

    // Create controller.
    //domainController = new DomainController();


    // START RING SEARCH ROUTINE
    /*
     * domainController.localize();
     * 
     * while (ringsLeft > 0) { domainController.crossTunnel(Map.BR_LL, Map.BR_UR, false);
     * domainController.approachTree(Map.T_LL, Map.T_UR);
     * 
     * if (!ringsIdentified) { domainController.identifyRings(ringMap); ringsIdentified = true; }
     * 
     * domainController.grabRings(); domainController.crossTunnel(Map.BR_LL, Map.BR_UR, true);
     * ringsLeft -= ringCapacity; }
     */

  }

}
