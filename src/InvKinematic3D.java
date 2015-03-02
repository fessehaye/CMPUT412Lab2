import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;

/*
 * Inverse Kinematic java class used to solve question 12
 */
public class InvKinematic3D {

	//x,y & z motors
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.A);
	static RegulatedMotor motorMid = new NXTRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.C);
	
	//length of each arm by millimeters and height of the ground
    static double armLenC = (8*20);//in mm
    static double armLenM = (8*8);//in mm
    static double armLenF = (8*11);// in mm
    static double height = 32;//in mm
 

	public static void main(String[] args) {

		motorFar.setSpeed(40);
		motorMid.setSpeed(40);
		motorClose.setSpeed(40);
		
		//Calculates angles for motors
		int[]angles =goHere(80, -100, -100);
		
		motorFar.rotateTo(angles[2]);
		motorMid.rotateTo(angles[1]);
		motorClose.rotateTo(angles[0]);
		
		//resets position
		Button.waitForAnyPress();
		motorFar.rotateTo(0);
		motorMid.rotateTo(0);
		motorClose.rotateTo(0);
	}
	
	/*
	 * Solution to exercise 12
	 * 
	 * Write a program that locates the robot end effector in a given (x,y,z) coordinate.
	 * 
	 */
	public static int[] goHere(double ze, double ye, double xe){
		
		double d, theta1, theta2, theta3;
		
		xe += (armLenC+armLenF+armLenM);
		
		//angle for theta3
		theta3 = Math.atan2(ze,xe);
		
		//translate to zero, zero
		
		d = ((xe*xe)+(ye*ye)-(armLenC*armLenC)-((armLenM+armLenF*Math.cos(theta3))*(armLenM+armLenF*Math.cos(theta3))))/(2*armLenC*(armLenM+armLenF*Math.cos(theta3)));
		
		//Calculates angles for movement
		theta2 = Math.atan2(Math.sqrt(1-(d*d)), d);
		theta1 = Math.atan2(ye, xe) - Math.atan2((armLenM+armLenF*Math.cos(theta3))*Math.sin(theta2),(armLenC+((armLenM+armLenF*Math.cos(theta3))*Math.cos(theta2))));
		
		//degrees to radiant
		theta1 = theta1*180/Math.PI;
		theta2 = theta2*180/Math.PI;
		theta3 = -1*theta3*180/Math.PI;
		
		int[] solution = {(int)theta1,(int)theta2,(int)theta3};
		return solution;
		
	}
	
	
}
