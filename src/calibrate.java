import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;


public class calibrate {

    static double armLenC = (8*29);//in mm
    static double armLenF = (8*14);// in mm
    static double increment = 5;// in mm
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
//		line(0,0,0,-100);
//		line2(0,0, 135, 100);
//		double[][] points = {
//				{0,0},
//				{100,-50},
//				{100,-75}
//				};
//		arc(points);
		cal();
	}

	private static void cal() {
		motorFar.setSpeed(30);
		//list movements
		pathplanning.line(0,0,40,-50);
		
		Button.waitForAnyPress();
		motorFar.rotateTo(0);
		motorClose.rotateTo(0);
		
	}


	
}
