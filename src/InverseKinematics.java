import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;

/*
 * Inverse Kinematic java class used to solve question 6 & 7
 */
public class InverseKinematics {
	//x & y motors
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);
	
	//length of each arm by millimeters
    static double armLenC = (8*29);//in mm
    static double armLenF = (8*14);// in mm

	public static void main(String[] args) {
		//for question 6
		int[]angles =goHere(-100, -100);
		motorFar.rotate(angles[1]);
		motorClose.rotate(angles[0]);
		Button.waitForAnyPress();
		
		//for question 7
		midPoint();
	}
	
	/*
	 * Solution to exercise 6
	 * 
	 * Write a program that receives as input a (x,y) location
	 *  inside the robot working space and locates the robot end
	 *   effector at the input location.
	 * 
	 * uses TwoDOFAnalyticalsolution class
	 */
	public static int[] goHere(double ye, double xe){
		
		double d, theta1, theta2;
		
		//translate to zero, zero
		xe += (armLenC+armLenF);
		
		//Calculates distance between origin and final point
		d = ((xe*xe)+(ye*ye)-(armLenC*armLenC)-(armLenF*armLenF))/(2*armLenC*armLenF);
		
		//Calculates angles for movement
		theta2 = Math.atan2(Math.sqrt(1-(d*d)), d);
		theta1 = Math.atan2(ye, xe) - Math.atan2(armLenF*Math.sin(theta2),(armLenC+(armLenF*Math.cos(theta2))));
		
		//degrees to radiant
		theta1 = theta1*180/Math.PI;
		theta2 = theta2*180/Math.PI;
		
		//remember when moving to add *7 to theta1 because of gear ratio
		int[] solution = {(int)theta1*7,(int)theta2};
		return solution;
		
	}
	
	/*
	 * Solution to exercise 7
	 * 
	 *  Write a program that finds mid points between two points.
	 *  i.e. The user moves the end effector to point1 stores that location then moves the
	 *  end effector to point2 stores the second location and then runs the midpoint 
	 *  algorithm which locates the robots end effector in the middle of the two points.
	 * 
	 * uses TwoDOFAnalyticalsolution class and getHere function
	 */
	public static void midPoint(){

		double midx, midy;
		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution second = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		
		//Records first point of the line
		System.out.println("Press to Record 1");
		Button.waitForAnyPress();
		
		//Calculates angles for the motor for the first point
		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());
		
		//Records second point of the line
        System.out.println("Press to Record 2");
		Button.waitForAnyPress();
		
		//Calculates angles for the motor for the second point
		second.setThetaC(motorClose.getTachoCount()/7);
		second.setThetaF(motorFar.getTachoCount());
		
		//takes average of both x's and y's to find midx midy
		midx = (first.getXe() + second.getXe()) /2;
		midy = (first.getYe() + second.getYe()) /2;
		
		//uses goHere for calculating angles for midpoint
		int[]angles =goHere(midy, midx-(armLenC+armLenF));
		
		//resets to initial position
		motorFar.setSpeed(30);
		
		motorFar.rotateTo(0);
		motorClose.rotateTo(0);
		
		Button.waitForAnyPress();
		
		//rotates to midpoint coordinates
		motorFar.rotateTo(angles[1]);
		motorClose.rotateTo(angles[0]);
		
		//displays midpoint coordinates
		System.out.println("X=" + (int)angles[1]);
		System.out.println("Y=" + (int)angles[0]);
		Button.waitForAnyPress();
		
	}
}
