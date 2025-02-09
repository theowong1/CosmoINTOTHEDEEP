package pedroPathing.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import teleop.AllianceStorage;
import teleop.transport.AutoStorage;
import teleop.transport.TransportFSM;

@Autonomous(name = "BlueNoSpecTele", preselectTeleOp = "CosmoboticsTeleOp")
public class BlueNoAutoSpecTele extends OpMode {

    private TransportFSM transport;

    @Override
    public void loop() {
        transport.update();
        telemetry.update();
    }

    @Override
    public void init() {
        AutoStorage.isAuto = true;
        AllianceStorage.isRed = false;
        AllianceStorage.teleMode = 0;
        AllianceStorage.useColorSensor = true;

        transport = new TransportFSM(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }
}