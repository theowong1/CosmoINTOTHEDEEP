package test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LimeLight Test", group = "Testing")
public class LimelightTest extends OpMode {

    public static double pX = 0, iX = 0, dX = 0;

    public static double pY = 0, iY = 0, dY = 0;

    private static DcMotorEx frontRight;
    private static DcMotorEx frontLeft;
    private static DcMotorEx backRight;
    private static DcMotorEx backLeft;

    private PIDController xController, yController;

    private Limelight3A limelight;

    @Override
    public void init()
    {
        xController = new PIDController(pX, iX, dX);
        yController = new PIDController(pY, iY, dY);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()) {

            xController.setPID(pX, iX, dX);
            yController.setPID(pY, iY, dY);

            double xPower = xController.calculate(result.getTx(), 0);
            double yPower = xController.calculate(result.getTy(), 0);

            double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower), 1);

            double frontLeftPower = (yPower + xPower) / denominator;
            double backLeftPower = (yPower - xPower) / denominator;
            double frontRightPower = (yPower - xPower) / denominator;
            double backRightPower = (yPower + xPower) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());

        } else {
            telemetry.addData("Limelight", "No Targets");
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }
}
