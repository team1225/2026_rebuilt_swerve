package frc.robot.auton.trajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.auton.trajectories.*;
import frc.robot.subsystems.*;
import frc.robot.interfaces.*;
import frc.robot.commands.drivetrain.DrivetrainSwerveRelative;

// The purpose of this class is to help testing trajectories at home in simulation mode
public class TrajectoryGenerationTest extends SequentialCommandGroup {
	
	public TrajectoryGenerationTest(SwerveDrivetrain drivetrain, RobotContainer container, ICamera object_detection_camera, ICamera apriltag_camera) {
		
		addCommands(

			// replace line below by whatever trajectory-generation command you want to test:
			
			//new MoveIn8Shape(drivetrain, container, 3)

			//new MoveInInfinityShape(drivetrain, container, 3)


			//new MoveForwardAndHardLeft(drivetrain, container, 3, 1)

			//new MoveForwardAndHardRight(drivetrain, container, 3, 1)

			//new MoveInReverseAndHardLeft(drivetrain, container, 3, 1)
			
			//new MoveInReverseAndHardRight(drivetrain, container, 3, 1)


			//new MoveForwardAndLeft(drivetrain, container, 3, 1 , 45)

			//new MoveForwardAndRight(drivetrain, container, 3, 1, -45)

			//new MoveInReverseAndLeft(drivetrain, container, 3, 1, 135)
			
			//new MoveInReverseAndRight(drivetrain, container, 3, 1, -135)


			//new MoveInGammaShape(drivetrain, container, 3)

			//new MoveInLShapeInReverse(drivetrain, container, 3)


			// another example, calling a method that generates a trajectory direcly:
			//new DrivetrainSwerveRelative(drivetrain, container, StartingPositionOnePickupSecondNote.createPickupSecondNoteTrajectory(container))

		); 
  
	}
}