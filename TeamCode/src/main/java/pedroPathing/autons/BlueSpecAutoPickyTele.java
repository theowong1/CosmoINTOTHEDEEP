package pedroPathing.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BlueSpecAutoPickyTele", preselectTeleOp = "CosmoboticsTeleOp")
public class BlueSpecAutoPickyTele extends OpMode {

    private BaseSpecAuto base;

    @Override
    public void loop() {
        base.update(telemetry);
    }

    @Override
    public void init() {
        base = new BaseSpecAuto(hardwareMap);
        base.setAllianceConstants(false, 2, true);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        base.start();
    }

    @Override
    public void stop() {
    }
}
