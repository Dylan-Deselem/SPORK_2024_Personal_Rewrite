package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
    
    NetworkTable table;
    NetworkTableEntry Tx;
    NetworkTableEntry Ty;
    NetworkTableEntry Ta;

    // creates a blank LimeLight with name Limelight
    public LimeLight(){
        table = NetworkTableInstance.getDefault().getTable("Limelight");
        Tx = table.getEntry("tx");
        Ty = table.getEntry("ty");
        Ta = table.getEntry("ta");
    }

    // Creates a LimeLight with a Designated String as a name
    public LimeLight(String LimeLight){
        table = NetworkTableInstance.getDefault().getTable(LimeLight);
        Tx = table.getEntry("tx");
        Ty = table.getEntry("ty");
        Ta = table.getEntry("ta");
    }

    // periodicly updates the Tx, Ty, and Ta values for testing
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Tx", getTx());
        SmartDashboard.putNumber("Ty", getTy());
        SmartDashboard.putNumber("Ta", getTa());
    }

    // switches the Pipeline of the desired LimeLight
    public void SwitchPipeline(int Pipe){
        table.getEntry("pipeline").setValue(Pipe);
    }

    // gets the Pipeline of the desired LimeLight
    public double getPipeline(){
        return table.getEntry("pipeline").getValue().getDouble();
    }

    // gets the target X pos on the camera 
    public double getTx(){
        return Tx.getDouble(0.0);
    }

    // gets the target Y pos on the camera 
    public double getTy(){
        return Ty.getDouble(0.0);
    }

    // gets the targets area
    public double getTa(){
        return Ta.getDouble(0.0);
    }

    // gets the Robots X position
    public double GetX() {
      return table.getEntry("botpose").getDoubleArray(new double[6])[0];
    }
    
    // gets the Robots Y position
    public double GetY(){
      return table.getEntry("botpose").getDoubleArray(new double[6])[1];
    }
    
    // gets the Robots Z position 
    public double GetZ(){
      return table.getEntry("botpose").getDoubleArray(new double[6])[3];
    }
    
    // gets the pitch of the limeLight
    public double GetPitch(){
      return table.getEntry("botpose").getDoubleArray(new double[6])[4];
    }
    
    // gets the roll of the limeLight
    public double GetRoll(){
      return table.getEntry("botpose").getDoubleArray(new double[6])[5];
    }
    
    // gets the yaw of the limeLight
    public double GetYaw(){
      return table.getEntry("botpose").getDoubleArray(new double[6])[6];
    }

}
