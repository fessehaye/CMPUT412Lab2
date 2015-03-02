/*
 * Class used for solving the Forward Kinematic Solution
 */

public class TwoDOFAnalyticalSolution {
	protected double xe;
	protected double ye;
	protected double thetaC;
	protected double thetaF;
	protected double armLenC;
	protected double armLenF;

	public TwoDOFAnalyticalSolution(double armLenC, double armLenF) {
		super();
		// TODO Auto-generated constructor stub
		this.armLenC = armLenC;
		this.armLenF = armLenF;
	}
	
	//returns Ye for forward Kinematics
	public double getXe() {
		this.xe = this.armLenC * Math.cos(this.thetaC) + this.armLenF * Math.cos(this.thetaC + this.thetaF);

		return this.xe;
	}
	
	//returns Ye for forward Kinematics
	public double getYe() {
		this.ye = this.armLenC * Math.sin(this.thetaC) + this.armLenF * Math.sin(this.thetaC + this.thetaF);

		return this.ye;
	}
	//sets thetaC for forward Kinematics
	public void setThetaC(double thetaC2) {
		this.thetaC = Math.PI*thetaC2/180;
	}
	
	//sets thetaF for forward Kinematics
	public void setThetaF(double thetaF2) {
		this.thetaF = Math.PI*thetaF2/180;
	}


}
