package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.Subsystems.Intake;

public class Spit extends Command{
    
    public Intake mIntake;

    public Spit(Intake mIntake){
        this.mIntake = mIntake;

        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        mIntake.IntakeSpit();
    }

    @Override
    public void execute() {
        if(mIntake.PivotEncoder.getPosition() > kIntake.kPositions.spitPOS - 2){
            mIntake.FeedShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.IntakeDown();
    }

}
