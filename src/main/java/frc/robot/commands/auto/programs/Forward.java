package frc.robot.commands.auto.programs;

import java.io.IOException;
import java.nio.file.Path;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CmdSetMotorSpeed;
import frc.robot.commands.CmdSpinMotorNegative;
import frc.robot.commands.CmdSpinMotorPositive;
import frc.robot.commands.SetPoseCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class Forward extends SequentialCommandGroup{

    private Pose2d blueMiddle = new Pose2d(7.120,5.66, new Rotation2d(0));


    public Forward(SwerveSubsystem swerve){
        PathPlannerPath path = null;
        swerve.SetPose(blueMiddle);
        try {
            path = PathPlannerPath.fromPathFile("Forward");
        } catch (Exception e) {
            e.printStackTrace();
        }
        

        addCommands(
            new SetPoseCmd(blueMiddle, swerve),
            new WaitCommand(1),
            AutoBuilder.followPath(path),
            new WaitCommand(1)


            
            
        );
            
    }




}
