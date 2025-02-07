package pedroPathing.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import teleop.AllianceStorage;
import teleop.transport.AutoStorage;
import teleop.transport.TransportFSM;

@Autonomous(name = "RedNoAutoSpecTele", preselectTeleOp = "CosmoboticsTeleOp")
public class RedNoAutoSpecTele extends OpMode {

    private TransportFSM transport;

    @Override
    public void loop() {
        transport.update();
        telemetry.update();
    }

    @Override
    public void init() {
        AutoStorage.isAuto = true;
        AllianceStorage.isRed = true;
        AllianceStorage.teleMode = 0;
        AllianceStorage.useColorSensor = true;

        transport = new TransportFSM(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        stop();
    }

    @Override
    public void stop() {
    }
}