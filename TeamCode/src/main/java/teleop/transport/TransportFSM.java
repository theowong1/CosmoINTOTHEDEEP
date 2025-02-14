package teleop.transport;

import static teleop.AllianceStorage.isRed;
import static teleop.AllianceStorage.teleMode;
import static teleop.AllianceStorage.useColorSensor;

import android.graphics.Color;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import teleop.utils.Toggle;

public class TransportFSM {
    public boolean test = false;
    public TouchSensor zeroLimit, outLimit;

    public Toggle intakeToggle, isSpecToggle, pickyToggle, colorSensorToggle, redToggle, manualMode, flickerToggle;
    //TODO: MNUL MODE N TEST EVERYTHING
    private ServoImplEx rot, bucketPitch, specClaw, specRot, specArm, flicker;
    private CRServoImplEx outWheel;
    //    private ServoImplEx flap;
//    private ServoImplEx bucketYaw;
    //private ServoImplEx specClawRoll, specClaw;
    public static DcMotorEx extendo, out;
    public DcMotorEx intake;

    private PIDController extendoController, outController, intakeController;
//    private PIDController armController;
//    private ColorSensor sampleColor = new ColorRangeSensor() {
//        @Override
//        public int red() {
//            return 0;
//        }
//
//        @Override
//        public int green() {
//            return 0;
//        }
//
//        @Override
//        public int blue() {
//            return 0;
//        }
//
//        @Override
//        public int alpha() {
//            return 0;
//        }
//
//        @Override
//        public int argb() {
//            return 0;
//        }
//
//        @Override
//        public void enableLed(boolean enable) {
//
//        }
//
//        @Override
//        public void setI2cAddress(I2cAddr newAddress) {
//
//        }
//
//        @Override
//        public I2cAddr getI2cAddress() {
//            return null;
//        }
//
//        @Override
//        public double getDistance(DistanceUnit unit) {
//            return 0;
//        }
//
//        @Override
//        public double getLightDetected() {
//            return 0;
//        }
//
//        @Override
//        public double getRawLightDetected() {
//            return 0;
//        }
//
//        @Override
//        public double getRawLightDetectedMax() {
//            return 0;
//        }
//
//        @Override
//        public String status() {
//            return "";
//        }
//
//        @Override
//        public NormalizedRGBA getNormalizedColors() {
//            return null;
//        }
//
//        @Override
//        public float getGain() {
//            return 0;
//        }
//
//        @Override
//        public void setGain(float newGain) {
//
//        }
//
//        @Override
//        public Manufacturer getManufacturer() {
//            return null;
//        }
//
//        @Override
//        public String getDeviceName() {
//            return "";
//        }
//
//        @Override
//        public String getConnectionInfo() {
//            return "";
//        }
//
//        @Override
//        public int getVersion() {
//            return 0;
//        }
//
//        @Override
//        public void resetDeviceConfigurationForOpMode() {
//
//        }
//
//        @Override
//        public void close() {
//
//        }
//    };

    //COLOR LOGIC:
//    public final int invalid = 0;
//    public final int valid = 1;
//    public final int empty = 2;
    //PID Values:
    public boolean isHighBucket = true;
    public boolean depositObsv = false;

    public final double extendop = .02, extendoi = .0001, extendod = 0.0001, outp = 0.01, outi = .0001, outd = .0001;
    //    public final double armp = 0, armi = 0, armd = 0, armf = 0;
    public double extendopid, outpid, armpid;
//    public double armff;
//    public final double arm_ticks_in_degrees = 1425.1 / 360;
//    public final double zeroOffset = 84;

    //SERVO POSITIONS
    public double rotPos, outWheelPower, bucketPitchPos, flickerPos, specArmPos, specClawPos, specRotPos;

    //MOTOR POSITIONS

    public int extendoPos, outPos, armPos;
    //MOTOR POWER
    public double intakePower;

    //MOTOR TARGETS
    public int extendoTarget, outTarget, armTarget;

    //SERVO VALUES
    public static double flapOpenRotIntake = .5;
    public static double flapClosedrotHome = .35;
    public static double flapClosedrotIntake = 1;

    public static double bucketYawHome = 0.15;
    public static double bucketYawSpit = .5;

    public static double rotIntake = .84;
    public static double rotPrep = .52;
    public static double rotHome = .18;
    public static double rotOuttake = .25;

    public static double bucketPitchHome = .09;
    public static double bucketPitchPrep = .35;
    public static double bucketPitchScore = .7;

    public static double specClawRollIntake = .215;
    public static double specClawRollOuttake = .925;

    public static double specClawOpen = .7;
    public static double specClawClosed = .4;
    public static double specArmHome = .05;
    public static double specRotPrep = .2;
    public static double specArmScore = .53;
    public static double specArmClear = .25;
    //MOTOR POSITIONS
    public static double flickerRetracted = .75;
    public static double flickerOut = .3;
    public static int autoExtendoUpper = 675;
    public static int extendoUpper = 765;
//    public static int extendoUpper = 650;
    public static int extendoLower = 0;
    public static int extended = 115;
    public static int transferTrigger = 0;
    public static int increment = 20;
    public static int outHome = 0;
    public static int outDump = 700;
    //    public final int dumpTrigger = 600;
    public static int outLowBucket = 600;
    //    public final int lowFlipTrigger = 300;
    public static int outHighBucket = 2250;
    //    public final int highFlipTrigger = 2300;
//    public final int armHome = 0;
//    public final int armLowBar = 700;
//    public final int armHighBar = 1500;
    public static double intaking = 1;
    public static double transferring = -1;
    public static double maintaining = 0;
    public static double dormant = 0;
    public static double outtaking = -1;
//    public static double flicking = -.35;
    ColorSensor colorSensor;
    public static double hue;
    public static int validSample = 0;
    public static double blueSample = 222;
    public static double redSample = 23;
    public static double yellowSample = 84;
    public static double hueError = 35;
    public float hsvValues[] = {0F, 0F, 0F};


    //ElapsedTimes:
    ElapsedTime sampleWait;
    ElapsedTime specimenWait;

    //Wait Values:
    public static double shortshortTransferWait = .5;
    public static double longlongTransferWait = 1.75;
    public static double shortTransferWait = .4;
    public static double longTransferWait = .5;
    public static double emergencyWit = .75;
    public static double dumpWait = .75;
    public static double specRotWait = .2;
    public static double specScoreWait = .45;
    public static double specRetractWait = .65;

    public static double specRetrctWit2 = .15;

    //STATE
//    public static boolean isSpec = true;

    //Get Sample Color
//    public String getColor() {
//        if (sampleColor.red() < 128 && sampleColor.blue() > 128) {
//            return "Blue";
//        } else if (sampleColor.red() > 128 && sampleColor.blue() < 128) {
//            return "Red";
//        } else if (sampleColor.red() > 128 && sampleColor.blue() > 128) {
//            return "Yellow";
//        }
//        return "Null";
//    }
//
//    public int isValid() {
//        if (getColor() == "Blue" && isRed) {
//            return invalid;
//        } else if (getColor() == "Red" && !isRed) {
//            return invalid;
//        } else if (getColor() == "Null") {
//            return empty;
//        } else {
//            return valid;
//        }
//    }

    //FSMS:
    public enum SampleTransport {
        SAMPLE_HOME,
        OUTPLUSEXTENDED,
        DUMPPLUSEXTENDED,
        INTAKERETRACTOUT,
        INTAKE,
        EXTENDED,
        RETRACTING,
        OUTTAKE,
        EMERGENCY_OUTTAKE,
        TRANSFER,
        PREP_BUCKET,
        HIGH_BUCKET,
        DUMP,
        RETRACTING_ROT

    }

    public SampleTransport sampleTransport;

    public enum SpecimenTransport {
        SPECIMEN_HOME,
        PREP,
        INTAKE_SPEC,
        SCORE,
        OPEN,
        PREP_HOME,

        PREP_HOME_TWO,
        EMERGENCY_PREP
    }

    public SpecimenTransport specimenTransport = SpecimenTransport.SPECIMEN_HOME;

    //Set States
    public int sampleState, specState;
    public static final int sampleHome = 0, sampleIntake = 1, sampleOuttake = 2, sampleTransfer = 3, sampleLowBucket = 4, sampleHighBucket = 5, sampleDump = 6, specHome = 0, specIntake = 1, specLowBar = 2, specHighBar = 3, specScore = 4;

    public boolean extendSlides, retractSlides;


    public TransportFSM(HardwareMap hardwareMap) {
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");
        flickerToggle = new Toggle(false);
        intakeToggle = new Toggle(false);
        if (teleMode == 0) {
            isSpecToggle = new Toggle(true);
            pickyToggle = new Toggle(false);
        } else if (teleMode == 1) {
            isSpecToggle = new Toggle(false);
            pickyToggle = new Toggle(false);
        } else if (teleMode == 2) {
            isSpecToggle = new Toggle(false);
            pickyToggle = new Toggle(true);
        }

        if (useColorSensor) {
            colorSensorToggle = new Toggle(true);
        } else {
            colorSensorToggle = new Toggle(false);
        }

        if (isRed) {
            redToggle = new Toggle(true);
        } else {
            redToggle = new Toggle(false);
        }
        manualMode = new Toggle(false);

//        if (isRed) {
//            isRedToggle = new Toggle(true);
//        } else {
//            isRedToggle = new Toggle(false);
//        }

        sampleWait = new ElapsedTime();
        specimenWait = new ElapsedTime();
        sampleWait.reset();
        specimenWait.reset();

        //TODO: SAVE POSITION FROM AUTO (OR AUTO RESET) FOR MOTORS

        extendoController = new PIDController(extendop, extendoi, extendod);
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendoController.setPID(extendop, extendoi, extendod);
        if (AutoStorage.isAuto) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendoPos = extendo.getCurrentPosition();
        }

        outController = new PIDController(outp, outi, outd);
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setDirection(DcMotorSimple.Direction.REVERSE);
        outController.setPID(outp, outi, outd);
        if (AutoStorage.isAuto) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outPos = out.getCurrentPosition();
        }


//        armController = new PIDController(armp, armi, armd);
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armController.setPID(armp, armi, armd);
//        armPos = arm.getCurrentPosition();
//
//        intakeController = new PIDController(intakep, intakei, intaked);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeController.setPID(intakep, intakei, intaked);
//        intakePower = intake.getVelocity();

//        specClawRoll = hardwareMap.get(ServoImplEx.class, "specClawRoll");
//        specClaw = hardwareMap.get(ServoImplEx.class, "specClaw");
        rot = hardwareMap.get(ServoImplEx.class, "rot");
        rot.setDirection(Servo.Direction.REVERSE);
//        flap = hardwareMap.get(ServoImplEx.class, "flap");
//        flap.setDirection(Servo.Direction.REVERSE);
        bucketPitch = hardwareMap.get(ServoImplEx.class, "bucketPitch");
        specClaw = hardwareMap.get(ServoImplEx.class, "specClaw");
        specRot = hardwareMap.get(ServoImplEx.class, "specRot");
        specArm = hardwareMap.get(ServoImplEx.class, "specArm");
        flicker = hardwareMap.get(ServoImplEx.class, "flicker");
        outWheel = hardwareMap.get(CRServoImplEx.class, "outWheel");
        outWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//        bucketYaw = hardwareMap.get(ServoImplEx.class, "bucketYaw");

        if (AutoStorage.isAuto) {
            //TODO: DO YOU WANT TO MOVE IN INIT? DO YOU WANT TO MOVE THE SLIDES?
            intake.setPower(dormant);
            outWheel.setPower(dormant);
            rot.setPosition(rotHome);
            bucketPitch.setPosition(bucketPitchHome);
            specClaw.setPosition(specClawClosed);
            specRot.setPosition(specClawRollIntake);
            specArm.setPosition(specArmHome);
            flicker.setPosition(flickerRetracted);
            specimenTransport = SpecimenTransport.INTAKE_SPEC;
            sampleTransport = SampleTransport.SAMPLE_HOME;
        } else {
            //TODO: DO YOU WANT TO MOVE IN INIT? DO YOU WANT TO MOVE THE SLIDES?
            intake.setPower(dormant);
            outWheel.setPower(dormant);
            rot.setPosition(rotHome);
            bucketPitch.setPosition(bucketPitchHome);
            specClaw.setPosition(specClawClosed);
            specRot.setPosition(specClawRollIntake);
            specArm.setPosition(specArmHome);
            flicker.setPosition(flickerRetracted);
            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
            sampleTransport = SampleTransport.SAMPLE_HOME;
        }
//        bucketYaw.setPosition(bucketYawHome);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
        //INIT LED COLOR: purpleRGB!
    }

//    public void setSampleState(int sampleState) {
//        this.sampleState = sampleState;
//    }
//
//    public void setSpecState(int specState) {
//        this.specState = specState;
//    }

    public void update() {
        validSample = updateColor();
        outPos = out.getCurrentPosition();
        outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid / 2.643);

        extendoPos = extendo.getCurrentPosition();
        extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

//        armPos = arm.getCurrentPosition();
//        armpid = outController.calculate(outPos, outTarget);
//        armff = Math.sin(Math.toRadians(armPos / arm_ticks_in_degrees + zeroOffset)) * armf;
//        arm.setPower(armpid + armff);

        intake.setPower(intakePower);

//        specClawRoll.setPosition(specClawRollPos);
//        specClaw.setPosition(specClawPos);
        rot.setPosition(rotPos);
//        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
//        bucketYaw.setPosition(bucketYawPos);
        specArm.setPosition(specArmPos);
        specClaw.setPosition(specClawPos);
        specRot.setPosition(specRotPos);
        flicker.setPosition(flickerPos);
        outWheel.setPower(outWheelPower);

        //TODO: FSM UPDTE FOR UTO
        switch (sampleTransport) {
            case SAMPLE_HOME:
                intakePower = dormant;
                rotPos = rotHome;
                bucketPitchPos = bucketPitchHome;
//                bucketYawPos = bucketYawHome;
                outTarget = outHome;
//                flapPos = flapClosedrotHome;
                extendoTarget = extendoLower;
                outWheelPower = dormant;
                break;
            case EXTENDED:
                outTarget = outHome;
                bucketPitchPos = bucketPitchHome;
                intakePower = maintaining;
                //chnge lter
                rotPos = rotPrep;
                extendoTarget = extendoUpper;
                outWheelPower = dormant;
                break;
            case INTAKERETRACTOUT:
                rotPos = rotIntake;
                intakePower = intaking;
                bucketPitchPos = bucketPitchHome;
                outTarget = outHome;
                break;
            case OUTPLUSEXTENDED:
                outWheelPower = dormant;
                extendoTarget = extended;
                intakePower = dormant;
                outTarget = outHighBucket;
                bucketPitchPos = bucketPitchPrep;
                intakePower = maintaining;
                rotPos = rotPrep;
                extendoTarget = extendoUpper;
                outWheelPower = dormant;
                break;
            case INTAKE:
                intakePower = intaking;
                rotPos = rotIntake;
                outWheelPower = dormant;
//                flapPos = flapClosedrotIntake;
                break;
            case EMERGENCY_OUTTAKE:
                outWheelPower = intaking;
                intakePower = outtaking;
                rotPos = rotIntake;
                break;
            case OUTTAKE:
                outWheelPower = outtaking;
//                if ((sampleWait.seconds() >= longlongTransferWait) && (validSample == 0)) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
//                }
                break;
            case RETRACTING:
                rotPos = rotHome;
                outWheelPower = dormant;
                intakePower = dormant;
                extendoTarget = -5;
                break;
            case RETRACTING_ROT:
                rotPos = rotHome;
                outWheelPower = dormant;
                break;
            case TRANSFER:
                rotPos = rotHome;
                intakePower = transferring;
                outWheelPower = intaking;
                extendoTarget = extendoLower;
                break;
            case HIGH_BUCKET:
                outWheelPower = dormant;
                extendoTarget = extended;
                intakePower = dormant;
                outTarget = outHighBucket;
                bucketPitchPos = bucketPitchPrep;
                break;
            case DUMP:
                outWheelPower = dormant;
                extendoTarget = extended;
                bucketPitchPos = bucketPitchScore;
//                bucketYawPos = bucketYawSpit;
                break;
            case DUMPPLUSEXTENDED:
                bucketPitchPos = bucketPitchScore;
                break;
            default:
                sampleTransport = sampleTransport.SAMPLE_HOME;
        }

        if (zeroLimit.isPressed()) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (outLimit.isPressed()) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        switch (specimenTransport) {
            case SPECIMEN_HOME:
                specArmPos = specArmHome;
                specRotPos = specClawRollIntake;
                specClawPos = specClawOpen;
                break;
            case INTAKE_SPEC:
                specArmPos = specArmHome;
                specRotPos = specClawRollIntake;
                specClawPos = specClawClosed;
                break;
            case PREP:
                specArmPos = specArmScore;
                specRotPos = specClawRollIntake;
                break;
            case SCORE:
                specRotPos = specClawRollOuttake;
                break;
            case OPEN:
                specClawPos = specClawOpen;
                break;
            case PREP_HOME:
                specRotPos = specRotPrep;
                specClawPos = specClawClosed;
                specArmPos = specArmClear;
                break;
            case PREP_HOME_TWO:
                specRotPos = specClawRollIntake;
                specClawPos = specClawClosed;
                break;
            case EMERGENCY_PREP:
                specRotPos = specClawRollIntake;
                break;
        }
    }


    public void setExtendoTarget(int val) {
        extendoTarget = val;
    }

    public void flickerOut(boolean active) {
        if (active) {
            flickerPos = flickerOut;
        } else {
            flickerPos = flickerRetracted;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        intakeToggle.update(gamepad1.a);
        isSpecToggle.update(gamepad1.back);
        pickyToggle.update(gamepad1.dpad_up);
        colorSensorToggle.update(gamepad2.dpad_down);
        manualMode.update(gamepad2.back);
        flickerToggle.update(gamepad2.dpad_right);

        outPos = out.getCurrentPosition();
        outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoPos = extendo.getCurrentPosition();
        extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

//        armPos = arm.getCurrentPosition();
//        armpid = outController.calculate(outPos, outTarget);
//        armff = Math.sin(Math.toRadians(armPos / arm_ticks_in_degrees + zeroOffset)) * armf;
//        arm.setPower(armpid + armff);

//        intakePower = intake.getVelocity();
//        intakepid = intakeController.calculate(intakePower, intakeTarget);
        intake.setPower(intakePower);

//        specClawRoll.setPosition(specClawRollPos);
//        specClaw.setPosition(specClawPos);
        rot.setPosition(rotPos);
//        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
//        bucketYaw.setPosition(bucketYawPos);
        specClaw.setPosition(specClawPos);
        specArm.setPosition(specArmPos);
        specRot.setPosition(specRotPos);
        flicker.setPosition(flickerPos);
        outWheel.setPower(outWheelPower);
        if (!manualMode.value()) {
            switch (sampleTransport) {
                case SAMPLE_HOME:
                    intakePower = dormant;
                    rotPos = rotHome;
                    bucketPitchPos = bucketPitchHome;
//                bucketYawPos = bucketYawHome;
                    outTarget = outHome;
//                flapPos = flapClosedrotHome;
                    extendoTarget = extendoLower;
                    outWheelPower = dormant;
                    if (gamepad1.right_trigger > 0) {
                        sampleTransport = SampleTransport.EXTENDED;
                    }
                    if ((extendoPos <= extended) && gamepad1.right_bumper && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                        sampleWait.reset();
                        isHighBucket = false;
                        sampleTransport = SampleTransport.TRANSFER;
                    }
                    if (gamepad1.y && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                        sampleWait.reset();
                        depositObsv = true;
                        sampleTransport = SampleTransport.EXTENDED;
                    }
                    if (gamepad1.left_bumper && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                        sampleWait.reset();
                        isHighBucket = true;
                        sampleTransport = SampleTransport.TRANSFER;
                    }
                    break;
                case EXTENDED:
                    outTarget = outHome;
                    bucketPitchPos = bucketPitchHome;
                    intakePower = maintaining;
                    //chnge lter
                    rotPos = rotPrep;
                    extendoTarget = extendoUpper;
                    outWheelPower = dormant;
                    if (depositObsv && extendoPos >= extended + 300) {
                        specimenTransport = SpecimenTransport.SPECIMEN_HOME;
                        sampleWait.reset();
                        sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                    }
                    if (gamepad1.left_bumper) {
                        intakeToggle.value = false;
                        sampleWait.reset();
                        isHighBucket = true;
                        sampleTransport = SampleTransport.RETRACTING;
                    }
                    if (gamepad1.right_bumper) {
                        intakeToggle.value = false;
                        sampleWait.reset();
                        isHighBucket = false;
                        sampleTransport = SampleTransport.RETRACTING;
                    }
                    if (gamepad1.left_trigger > 0) {
                        sampleTransport = SampleTransport.SAMPLE_HOME;
                    }
//                if (extendoPos < extended) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
                    if (intakeToggle.value() == true) {
                        sampleTransport = SampleTransport.INTAKE;
                    }
                    if (gamepad1.y && !depositObsv) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                    }
                    if (validSample == 0) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.OUTTAKE;
                    }
                    break;
                case INTAKE:
                    intakePower = intaking;
                    rotPos = rotIntake;
                    outWheelPower = dormant;
//                flapPos = flapClosedrotIntake;
                    if (intakeToggle.value() == false) {
                        sampleTransport = SampleTransport.EXTENDED;
                    }
                    if (gamepad1.left_bumper || (validSample == 1 && teleMode != 0)) {
                        intakeToggle.value = false;
                        sampleWait.reset();
                        isHighBucket = true;
                        sampleTransport = SampleTransport.RETRACTING;
                    }
                    if (gamepad1.right_bumper || (validSample == 1 && teleMode == 0)) {
                        intakeToggle.value = false;
                        sampleWait.reset();
                        isHighBucket = false;
                        sampleTransport = SampleTransport.RETRACTING;
                    }
                    if (gamepad1.y) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                    }
                    if (validSample == 0) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.OUTTAKE;
                    }
                    break;
                case EMERGENCY_OUTTAKE:
                    outWheelPower = intaking;
                    intakePower = outtaking;
                    rotPos = rotIntake;
                    if (sampleWait.seconds() > emergencyWit) {
                        if (depositObsv) {
                            sampleWait.reset();
                            depositObsv = false;
                            sampleTransport = SampleTransport.RETRACTING;
                        } else {
                            sampleTransport = SampleTransport.EXTENDED;
                        }
                    }
                    break;
                case OUTTAKE:
                    outWheelPower = outtaking;
                    if ((validSample != 0)) {
                        sampleTransport = SampleTransport.INTAKE;
                    }
                    if (gamepad1.y) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                    }
//                if ((sampleWait.seconds() >= longlongTransferWait) && (validSample == 0)) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
//                }
                    break;
                case RETRACTING:
                    rotPos = rotHome;
                    outWheelPower = dormant;
//                specimenTransport = SpecimenTransport.SPECIMEN_HOME;
                    if (sampleWait.seconds() > emergencyWit) {
                        intakePower = dormant;
                        extendoTarget = -5;
                    }
                    if (gamepad1.left_bumper) {
                        isHighBucket = true;
                    }
                    if (gamepad1.right_bumper || depositObsv) {
                        isHighBucket = false;
                    }
                    if (gamepad1.right_trigger > 0) {
                        sampleTransport = SampleTransport.EXTENDED;
                    }
                    if (isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                        if (extendoPos <= 50 || gamepad1.y) {
                            sampleWait.reset();
                            sampleTransport = SampleTransport.TRANSFER;
                        }
                    } else {
                        if (extendoPos <= 50 || gamepad1.y) {
//                        sampleWait.reset();
                            depositObsv = false;
                            sampleTransport = SampleTransport.SAMPLE_HOME;
                        }
                    }
                    break;
                case TRANSFER:
                    rotPos = rotHome;
                    intakePower = transferring;
                    outWheelPower = intaking;
                    extendoTarget = extendoLower + 20;
                    if (sampleWait.seconds() > shortTransferWait) {
                        if (isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                            sampleTransport = SampleTransport.HIGH_BUCKET;
                        } else if (!isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                            sampleTransport = SampleTransport.DUMP;
                        } else {
                            sampleTransport = SampleTransport.SAMPLE_HOME;
                        }
                    }
                    break;
                case HIGH_BUCKET:
                    outWheelPower = dormant;
                    extendoTarget = extended;
                    intakePower = dormant;
                    outTarget = outHighBucket;
                    bucketPitchPos = bucketPitchPrep;
                    if (gamepad1.y) {
                        sampleTransport = SampleTransport.DUMP;
                    }
                    if (gamepad1.right_bumper) {
                        sampleTransport = SampleTransport.EXTENDED;
                    }
//                    if (gamepad1.x) {
//                        sampleTransport = SampleTransport.SAMPLE_HOME;
//                    }
                    break;
                case DUMP:
                    outWheelPower = dormant;
                    extendoTarget = extended;
                    bucketPitchPos = bucketPitchScore;
//                bucketYawPos = bucketYawSpit;
                    intakePower = dormant;
                    if (gamepad1.left_trigger > 0) {
                        sampleTransport = SampleTransport.SAMPLE_HOME;
                    }
                    if (gamepad1.right_trigger > 0) {
                        sampleTransport = SampleTransport.EXTENDED;
                    }
                    break;
                default:
                    sampleTransport = sampleTransport.SAMPLE_HOME;
            }

            if (gamepad1.dpad_right && sampleTransport != SampleTransport.SAMPLE_HOME) {
//            extendoTarget = extendoLower;
                sampleTransport = SampleTransport.SAMPLE_HOME;
//            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
            }

            if (zeroLimit.isPressed()) {
                extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (outLimit.isPressed()) {
                out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (pickyToggle.value() == true) {
                teleMode = 2;
            } else {
                if (isSpecToggle.value() == true) {
                    teleMode = 0;
                } else {
                    teleMode = 1;
                }
            }

            if (flickerToggle.value() == true) {
                flickerPos = flickerOut;
            } else {
                flickerPos = flickerRetracted;
            }

            if (colorSensorToggle.value() == true) {
                validSample = updateColor();
//            colorSensor.enableLed(true);
                useColorSensor = true;
            } else {
//            colorSensor.enableLed(false);
                useColorSensor = false;
            }

            if (redToggle.value() == true) {
                isRed = true;
            } else {
                isRed = false;
            }

            switch (specimenTransport) {
                case SPECIMEN_HOME:
                    specArmPos = specArmHome;
                    specRotPos = specClawRollIntake;
                    specClawPos = specClawOpen;
                    if (gamepad1.b) {
                        specimenTransport = SpecimenTransport.INTAKE_SPEC;
                    }
                    break;
                case INTAKE_SPEC:
                    specArmPos = specArmHome;
                    specRotPos = specClawRollIntake;
                    specClawPos = specClawClosed;
                    if (gamepad1.x && (sampleTransport != SampleTransport.HIGH_BUCKET && sampleTransport != SampleTransport.DUMP)) {
                        specimenWait.reset();
                        specimenTransport = SpecimenTransport.PREP;
                    }
                    break;
                case PREP:
                    specArmPos = specArmScore;
                    specRotPos = specClawRollIntake;
                    if (specimenWait.seconds() > specRotWait) {
                        specimenWait.reset();
                        specimenTransport = SpecimenTransport.SCORE;
                    }
                    break;
                case SCORE:
                    specRotPos = specClawRollOuttake;
                    if (gamepad1.b) {
                        specimenWait.reset();
                        specimenTransport = SpecimenTransport.EMERGENCY_PREP;
                    }
                    if (gamepad1.x) {
                        specimenWait.reset();
                        specimenTransport = SpecimenTransport.OPEN;
                    }
                    break;
                case OPEN:
                    specClawPos = specClawOpen;
                    if (specimenWait.seconds() > specRetractWait || gamepad1.b) {
                        specimenWait.reset();
                        specimenTransport = SpecimenTransport.PREP_HOME_TWO;
                    }
                    break;
                case PREP_HOME:
                    specRotPos = specClawRollIntake;
                    specClawPos = specClawClosed;
                    specArmPos = specArmHome;
                    if (specimenWait.seconds() > specScoreWait) {
                        specimenTransport = SpecimenTransport.SPECIMEN_HOME;
                    }
                    break;
                case PREP_HOME_TWO:
                    specRotPos = specClawRollIntake;
                    specClawPos = specClawClosed;
                    if (specimenWait.seconds() > specRetrctWit2) {
                        specimenWait.reset();
                        specimenTransport = SpecimenTransport.PREP_HOME;
                    }
                    break;
                case EMERGENCY_PREP:
                    specRotPos = specClawRollIntake;
                    if (specimenWait.seconds() > specScoreWait) {
                        specimenTransport = SpecimenTransport.INTAKE_SPEC;
                    }
                    break;
            }

            if (gamepad1.dpad_down && specimenTransport != SpecimenTransport.SPECIMEN_HOME) {
                specimenTransport = SpecimenTransport.SPECIMEN_HOME;
            }
        } else {
            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
            sampleTransport = SampleTransport.SAMPLE_HOME;
            if (zeroLimit.isPressed() || gamepad2.a) {
                extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (outLimit.isPressed() || gamepad2.b) {
                out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.left_bumper) {
                outTarget -= 15;
            }

            if (gamepad2.right_bumper) {
                outTarget += 15;
            }

            if (gamepad2.left_trigger > 0) {
                extendoTarget -= 15;
            }

            if (gamepad2.right_trigger > 0) {
                extendoTarget += 15;
            }


        }
        /*
        update():
        if intaking:
        if color is red: LED Red
        if blue blue
        if yellow yellow
        if color is white/Null: LED White
        else if scoring:
        color is green
        else:
        color is purpleRGB
         */

    }

    public int updateColor() {
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            hue = hsvValues[0];
            if (useColorSensor) {
                if (Math.abs(hue - yellowSample) < hueError) {
                    if (teleMode == 0) {
                        return 0;
                    } else {
                        return 1;
                    }
                }
                if (Math.abs(hue - redSample) < hueError) {
                    if (isRed && teleMode != 2) {
                        return 1;
                    } else {
                        return 0;
                    }
                }
                if (Math.abs(hue - blueSample) < hueError) {
                    if (!isRed && teleMode != 2) {
                        return 1;
                    } else {
                        return 0;
                    }
                }
                return 2;
            } else {
                return 2;
            }
    }

//    public void updateFSM() {
//        switch (sampleTransport) {
//            case SAMPLE_HOME:
//                intakePower = dormant;
//                rotPos = rotHome;
//                bucketPitchPos = bucketPitchHome;
//                bucketYawPos = bucketYawHome;
//                outTarget = outHome;
//                flapPos = flapClosed;
//                if (intakeToggle.value() == true) {
//                    sampleTransport = SampleTransport.INTAKE;
//                }
//                if ( (extendoPos <= extended) && gamepad1.y ) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.TRANSFER;
//                }
//                if (sampleState == sampleLowBucket) {
//                    sampleTransport = SampleTransport.LOW_BUCKET;
//                }
//                if (sampleState == sampleHighBucket) {
//                    sampleTransport = SampleTransport.HIGH_BUCKET;
//                }
//                if (sampleState == sampleDump) {
//                    sampleTransport = SampleTransport.DUMP;
//                }
//                break;
//            case INTAKE:
//                intakePower = intaking;
//                rotPos = rotIntake;
//                flapPos = flapClosed;
//                if (intakeToggle.value() == false || sampleState == sampleHome) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
//                if (sampleState == sampleOuttake || (isValid() == invalid)) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.OUTTAKE;
//                }
//                break;
//            case OUTTAKE:
//                //intakePower = outtaking;
//                flapPos = flapOpen;
//                if (isValid() != invalid && sampleWait.seconds() >= shortTransferWait) {
//                    sampleTransport = SampleTransport.INTAKE;
//                }
//                if (isValid() == invalid && sampleWait.seconds() >= longTransferWait) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
//                }
//                break;
//            case EMERGENCY_OUTTAKE:
//                intakePower = transferring;
//                if (sampleWait.seconds() >= longTransferWait) {
//                    sampleTransport = SampleTransport.INTAKE;
//                }
//                break;
//            case TRANSFER:
//                intakePower = transferring;
//                if (sampleWait.seconds() > shortTransferWait) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
//                break;
//            case LOW_BUCKET:
//                outTarget = outLowBucket;
//                if (outPos >= lowFlipTrigger || sampleState == sampleDump) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.DUMP;
//                }
//                break;
//            case HIGH_BUCKET:
//                outTarget = outHighBucket;
//                if (outPos >= highFlipTrigger || sampleState == sampleDump) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.DUMP;
//                }
//                break;
//            case DUMP:
//                outTarget = outDump;
//                if (outPos >= dumpTrigger) {
//                    bucketPitchPos = bucketPitchScore;
//                    bucketYawPos = bucketYawSpit;
//                }
//                if (sampleWait.seconds() > dumpWait) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
//                break;
//            default:
//                sampleTransport = sampleTransport.SAMPLE_HOME;
//        }
//
//
//        /*
//        update():
//        if intaking:
//        if color is red: LED Red
//        if blue blue
//        if yellow yellow
//        if color is white/Null: LED White
//        else if scoring:
//        color is green
//        else:
//        color is purpleRGB
//         */
//
//
//        if (retractSlides && extendoTarget >= extendoLower) {
//            extendoTarget -= increment;
//        }
//
//        if (extendSlides && extendoTarget <= extendoUpper) {
//            extendoTarget += increment;
//        }
//
//        if (sampleState == sampleHome && sampleTransport != SampleTransport.SAMPLE_HOME) {
//            extendoTarget = extendoLower;
//            sampleTransport = SampleTransport.SAMPLE_HOME;
//            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
//        }
//
//        switch (specimenTransport) {
//            case SPECIMEN_HOME:
//                specClawRollPos = specClawRollIntake;
//                specClawPos = specClawOpen;
//                armTarget = armHome;
//                if (specState == specIntake) {
//                    specimenWait.reset();
//                    specimenTransport = SpecimenTransport.INTAKE_SPEC;
//                }
//                break;
//            case INTAKE_SPEC:
//                specClawPos = specClawClosed;
//                if (specState == specHighBar) {
//                    specimenTransport = SpecimenTransport.HIGH_BAR;
//                }
//                if (specState == specLowBar) {
//                    specimenTransport = SpecimenTransport.LOW_BAR;
//                }
//                break;
//            case LOW_BAR:
//                specClawRollPos = specClawRollOuttake;
//                armTarget = armLowBar;
//                if (specState == specScore) {
//                    specimenTransport = SpecimenTransport.SPECIMEN_SCORE;
//                }
//                break;
//            case HIGH_BAR:
//                specClawRollPos = specClawRollOuttake;
//                armTarget = armHighBar;
//                if (specState == specScore) {
//                    specimenTransport = SpecimenTransport.SPECIMEN_SCORE;
//                }
//                break;
//            case SPECIMEN_SCORE:
//                specClawPos = specClawOpen;
//            default:
//                specimenTransport = specimenTransport.SPECIMEN_HOME;
//        }
//    }
}
