package frc.robot.configuration;

public enum LimelightIMUMode {
	
	EXTERNAL_ONLY(0),
	
	EXTERNAL_SEED(1),
	
	INTERNAL_ONLY(2),
	
	INTERNAL_MT1_ASSIST(3),
	
	INTERNAL_EXTERNAL_ASSIST(4);
	
	public final int modeID;
	
	LimelightIMUMode(int modeID) {
		
		this.modeID = modeID;
		
	}
	
}
