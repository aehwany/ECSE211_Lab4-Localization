/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import java.text.DecimalFormat;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private float color[];
  private static float csData;
  private double countx;
  private double county;
  private float lastColor;
  private double squareLength = 30.48;
  private TextLCD lcd;
  private double[] position;
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    color = new float[Lab2.myColorSample.sampleSize()];
    this.csData = color[0];
    countx = 0.0;
    county = 0.0;
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    
    //fetch color sensor latest reading
    Lab2.myColorSample.fetchSample(color,0);
    while (true) {
      correctionStart = System.currentTimeMillis();
      Lab2.myColorSample.fetchSample(color,0);
      double[] getXYT = odometer.getXYT();
      double theta = getXYT[2];
      csData = color[0];
      csData *= 1000; //Scale intensity value
		//Intensity at line is < 300.
		if(csData < 300 && lastColor < 300) continue; //Only register line when the last measurement DID NOT measure a line
		lastColor = csData;		
		if(csData < 300) //Detected black
		{
			Sound.beep();
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
	      // TODO Trigger correction (When do I have information to correct?)
	      // TODO Calculate new (accurate) robot position
	
	      // TODO Update odometer with new calculated (and more accurate) vales
	      if ( (theta > 350 && theta < 361) || (theta > 0 && theta < 10) ) {
	    	  county++;
	    	  odometer.setXYT(0.0,county * squareLength,0.0);
	    	 // lcd.drawString("apply corr: y=" + numberFormat.format(county * squareLength), 0, 3);
	      }
	      else if ( (theta > 80 && theta < 100) ) {
	    	  countx++;
	    	  odometer.setX(countx * squareLength);
	    	  //lcd.drawString("apply corr: x=" + numberFormat.format(countx * squareLength), 0, 3);
	      }
	      else if ( (theta > 170 && theta < 190) ) {
	    	  county--;
	    	  odometer.setY(county * squareLength);
	    	 // lcd.drawString("apply corr: y=" + numberFormat.format(county * squareLength), 0, 3);
	      }
	      else if ( theta > 260 && theta < 280) {
	    	  countx--;
	    	  odometer.setX(countx * squareLength);
	    	 // lcd.drawString("apply corr: x=" + numberFormat.format(countx * squareLength), 0, 3);
	      }
		}
     // odometer.setXYT(0.3, 19.23, 5.0);
      

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
