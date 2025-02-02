package teleop.misc;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Misc {
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;

    //Yellow, White, Green, Purple
//    public int[] Colors = {Integer.parseInt("FFFF00", 16), Integer.parseInt("FFFFFF", 16), Integer.parseInt("00FF00", 16), Integer.parseInt("800080", 16)};

    public Misc (HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void update(Gamepad gamepad2) {

    }
}