package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LogCommand extends InstantCommand {
	
	public LogCommand(String message) {
		
		super(() -> System.out.println(message));
		
	}
}
