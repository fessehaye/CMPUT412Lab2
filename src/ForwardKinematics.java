
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;


public class ForwardKinematics {
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);

    static double armLenC = (8*29);//in mm
    static double armLenF = (8*14);// in mm

	public static void main(String[] args) {

		getPosition(90,0);
//		distanceMeasure();
		
	}
	
	public static void getPosition(double degC, double degF ){
		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		double distance, xd, yd;

		motorClose.rotateTo((int)degC*7);
		motorFar.rotateTo((int)degF);

		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());
		
        System.out.println("x: " + (int)(first.getXe()-(armLenC+armLenF)) + "mm");
        System.out.println("y: " + (int)first.getYe() + "mm");
	}
	
	public static void distanceMeasure(){
		
		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution second = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		double distance, xd, yd;

        System.out.println("Press to Record 1");
		Button.waitForAnyPress();

		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());

        System.out.println("Press to Record 2");
		Button.waitForAnyPress();
		
		second.setThetaC(motorClose.getTachoCount()/7);
		second.setThetaF(motorFar.getTachoCount());

		yd = first.getYe()-second.getYe();
		xd = first.getXe()-second.getXe();
		distance = Math.sqrt((xd*xd)+(yd*yd));

        System.out.println("F: " + motorFar.getTachoCount() + "deg");
        System.out.println("C: " + motorClose.getTachoCount() + "deg");
        System.out.println("Distance: " + (int)distance + "mm");

		Button.waitForAnyPress();
        
	}
	
	
	public static void inverseKinematics(double xe, double ye){
		
		double d, theta1, theta2;
		
		d = ((xe*xe)+(ye*ye)-(armLenC*armLenC)-(armLenF*armLenF))/(2*armLenC*armLenF);
		
		theta2 = Math.atan2(Math.sqrt(1-(d*d))/d, 1);
		
		theta1 = Math.asin(armLenF*Math.sin(theta2)) + Math.atan2(ye, xe);
		
		
	}
	
	
	
	
}
