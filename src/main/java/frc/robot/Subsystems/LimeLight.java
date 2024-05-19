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

    public LimeLight(){
        table = NetworkTableInstance.getDefault().getTable("Limelight");
        Tx = table.getEntry("tx");
        Ty = table.getEntry("ty");
        Ta = table.getEntry("ta");
    }

    public LimeLight(String LimeLight){
        table = NetworkTableInstance.getDefault().getTable(LimeLight);
        Tx = table.getEntry("tx");
        Ty = table.getEntry("ty");
        Ta = table.getEntry("ta");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Tx", getTx());
        SmartDashboard.putNumber("Ty", getTy());
        SmartDashboard.putNumber("Ta", getTa());
    }

    public double getTx(){
        return Tx.getDouble(0.0);
    }

    public double getTy(){
        return Ty.getDouble(0.0);
    }

    public double getTa(){
        return Ta.getDouble(0.0);
    }

    public void SwitchPipeline(int Pipe){
        table.getEntry("pipeline").setValue(Pipe);
    }

    public double getPipeline(){
        return table.getEntry("pipeline").getValue().getDouble();
    }


    public double GetX() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[0];
      }
    
      public double GetY(){
        return table.getEntry("botpose").getDoubleArray(new double[6])[1];
      }
    
      public double GetZ(){
        return table.getEntry("botpose").getDoubleArray(new double[6])[3];
      }
    
      public double GetPitch(){
        return table.getEntry("botpose").getDoubleArray(new double[6])[4];
      }
    
      public double GetRoll(){
        return table.getEntry("botpose").getDoubleArray(new double[6])[5];
      }
    
      public double GetYaw(){
        return table.getEntry("botpose").getDoubleArray(new double[6])[6];
      }
}
