package pedroPathing.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightSnapshot {

    private Limelight3A limelight;
    private LLResult result;

    private double x;

    private double length;

    private final double height = 14;
    
    private final double angle = 40;

    public static double k = 1; //TODO: Tune this value

    public LimelightSnapshot(Limelight3A limelight) {
      //  limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public boolean getSnapshot() {
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()) {
            x = result.getTx();
         //   length = result.getLength(result.getTy());
        }
        return result.isValid();
    }

    private double getLength(double y) {
        return height / Math.tan(angle + k * y);
    }

    public double getX() {
        return x;
    }

    public double getLength() {
        return length;
    }
}