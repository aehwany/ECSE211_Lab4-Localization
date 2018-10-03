// Lab2.java
package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * @author Ahmed Elehwany 260707540, Barry Chen
 * This the the main class where we 
 * 1. Start the program,
 * 2. Start the UI interface for map selection
 * 3. Call the Navigation or ObstacleAvoidance class and begin threads
 *
 */
public class lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  public static final double WHEEL_RAD = 2.15;
  public static final double TRACK = 13.7;
  
  /*
	 * Initialize 4 different maps:
	 * Map 1: (0,2), (1,1), (2,2), (2,1), (1,0)
	 * Map 2: (1,1), (0,2), (2,2), (2,1), (1,0)
	 * Map 3: (1,0), (2,1), (2,2), (0,2), (1,1)
	 * Map 4: (0,1), (1,2), (1,0), (2,1), (2,2)
	 */
	static int[][] path1 = {{0, 2},		{1, 1},     {2, 2},     {2, 1},   {1, 0}};
	static int[][] path2 = {{1, 1},     {0, 2},     {2, 2},     {2, 1},   {1, 0}};
	static int[][] path3 = {{1, 0},     {2, 1},     {2, 2},     {0, 2},   {1, 1}};
	static int[][] path4 = {{0, 1},     {1, 2},     {1, 0},     {2, 1},   {2, 2}};

	static int[][] finalPath;
	
	/**
	 * This is the main method for this class where the program starts
	 * @param args
	 * @throws Exception
	 */
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);  
    Display odometryDisplay = new Display(lcd); 
    //ObstacleAvoidance obstacleavoidance = new ObstacleAvoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    do {
        // clear the display
        lcd.clear();

	      // Ask the user whether map 1 or 2 / map 3 or 4 should be selected
	      lcd.drawString("<Map 1 | Map 3>", 0, 0);
	      lcd.drawString("       |       ", 0, 1);
	      lcd.drawString(" Map 2 | Map 4 ", 0, 2);
	      lcd.drawString("       |       ", 0, 3);
	      lcd.drawString("       |       ", 0, 4);

        buttonChoice = Button.waitForAnyPress();
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    do {
    if (buttonChoice == Button.ID_LEFT) {
      // Pick between map 1 or map 2
    	 lcd.drawString("<      |      >", 0, 0);
	     lcd.drawString("       |       ", 0, 1);
	     lcd.drawString(" Map 1 | Map 2 ", 0, 2);
	     lcd.drawString("       |       ", 0, 3);
	     lcd.drawString("       |       ", 0, 4);
	     buttonChoice = Button.waitForAnyPress();   // Record button choice
	     if (buttonChoice == Button.ID_LEFT) {     
	    	 finalPath = path1;      // Set map 2
	      }
	     else {
	    	 finalPath = path2;      // Set map 1        
	     }
    }
      else {
    	  lcd.clear();      // clear the display

	      // Pick between map 3 or map 4
	      	 lcd.drawString("<      |      >", 0, 0);
		     lcd.drawString("       |       ", 0, 1);
		     lcd.drawString(" Map 3 | Map 4 ", 0, 2);
		     lcd.drawString("       |       ", 0, 3);
		     lcd.drawString("       |       ", 0, 4);

	      buttonChoice = Button.waitForAnyPress();   // Record choice (left or right press)
	      
	      if (buttonChoice == Button.ID_LEFT) {
		    	 finalPath = path3;      // Set map 4
		      }
		     else {
		    	 finalPath = path4;      // Set map 3
		     }
      }
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
          // clear the display
          lcd.clear();

          // ask the user whether the motors should drive in a square or float
          lcd.drawString("< Left  | Right >", 0, 0);
          lcd.drawString("  No    | with   ", 0, 1);
          lcd.drawString("  obs - | obs-   ", 0, 2);
          lcd.drawString("  avoid | avoid  ", 0, 3);
          lcd.drawString("        |        ", 0, 4);
          
          buttonChoice = Button.waitForAnyPress();
          
       // Start odometer and display threads
          Thread odoThread = new Thread(odometer);
          odoThread.start();
          Thread odoDisplayThread = new Thread(odometryDisplay);
          odoDisplayThread.start();
          
       // Start NavWithoutObstacle thread
          if(buttonChoice == Button.ID_LEFT){
        	 Navigation navigation = new Navigation(leftMotor,rightMotor,TRACK,WHEEL_RAD, finalPath);
        	 navigation.run();
          }
       // Start NavWithObstacle thread
          if(buttonChoice == Button.ID_RIGHT){
        	 ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(leftMotor,rightMotor,TRACK,WHEEL_RAD, finalPath);
        	 obstacleAvoidance.run();
        	 //run the obstacleAvoidance
          }

       // If button is pressed, exit the program
        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
      }
    }