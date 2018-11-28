package ca.mcgill.ecse211.mountev3rest.controller;

import java.io.IOException;
import java.util.Map;
import org.json.simple.parser.ParseException;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.mountev3rest.navigation.Display;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.util.CoordinateMap;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

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
 * Additionally, the {@code MetaController} makes sure that the appropriate external conditions are
 * met before requesting that a given subtask is performed. These external conditions are defined by
 * the executor of the subtask (usually the {@code DomainController}).
 * 
 * @see DomainController
 * 
 * @author angelortiz
 *
 */
public class MetaController {

  // Constants
  private static final String SERVER_IP = "192.168.2.18";
  private static final int TEAM_NUMBER = 11;
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;
  private static final int RINGS_TO_COLLECT = 3;

  // Attributes
  private DomainController domainController;

  // Main method
  public static void main(String[] args) throws OdometerException, PollerException {
    // Instantiate the meta controller
    MetaController metaController = new MetaController();

    // Get Wi-Fi Data and pass it to the domain controller.
    CoordinateMap map = getWiFiData();
    metaController.domainController.setMap(map);

    // Run the whole ring routine.
    metaController.run();
    //metaController.testRun();

    Button.waitForAnyPress();
    System.exit(0);
  }

  /**
   * Creates a high level controller that can perform ring collection as a set of subtasks.
   * 
   * @throws PollerException If there is a problem while instantiating the sensor poller classes.
   * @throws OdometerException If there is a problem while instantiating the {@code Odometer} class.
   */
  public MetaController() throws OdometerException, PollerException {
    domainController = new DomainController();
  }

  /**
   * Performs all the required subtasks involved in the ring collection routine. This includes
   * everything from localization, tunnel traversal, and ring collection. The method returns once
   * the entire routine has been performed.
   * 
   * @throws OdometerException If the {@code Odometer} class has not yet been instantiated.
   *
   */
  public void run() throws OdometerException {
    TextLCD lcd = LocalEV3.get().getTextLCD();

    lcd.clear();
    lcd.drawString("       READY       ", 0, 4);

    /*Display display = new Display(lcd);
    Thread disThread = new Thread(display);
    disThread.start();*/
    
    int treeRotations = 0;

    // -- START RING SEARCH ROUTINE --

    // Localize and beep three times
    
    domainController.localize(); 
    Sound.beep(); 
    Sound.beep(); 
    Sound.beep();

    domainController.crossTunnel();
    domainController.approachTree();
    Sound.beep(); 
    Sound.beep(); 
    Sound.beep();
    
    domainController.grabRings(false);
    for(int i = 0; i < RINGS_TO_COLLECT - 1; i++) {
      domainController.goToNextFace(true);
      treeRotations++;
      domainController.grabRings(true);
    }
    
    switch(treeRotations) {
      case 1:
        domainController.goToPrevFace(false);
        break;
      case 2:
        domainController.goToPrevFace(true);
        domainController.goToPrevFace(false);
        break;
      case 3:
        domainController.goToNextFace(false);
        break;
      default:
        break;
    }
    
    domainController.crossTunnel();
    domainController.releaseRings();
    
    Sound.beep(); 
    Sound.beep(); 
    Sound.beep();
    Sound.beep(); 
    Sound.beep();
  }

  /**
   * Gets the map coordinates from a Wi-Fi server running on an external computer and returns a
   * {@code CoordinateMap} containing them.
   * 
   * @return {@code CoordinateMap} object containing the information received through Wi-Fi.
   * 
   * @see CoordinateMap
   */
  @SuppressWarnings("rawtypes")
  private static CoordinateMap getWiFiData() {
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
    Map data = null;
    try {
      data = conn.getData();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }

    return new CoordinateMap(data, TEAM_NUMBER);
  }

  // REMOVE
  public void testRun() throws OdometerException {
    domainController.testNavigation();
    //domainController.testColorDetection();
  }

}
