package pedroPathing.autons;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import teleop.AllianceStorage;
import teleop.transport.AutoStorage;
import teleop.transport.TransportFSM;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Autonomous(name = "Pedro Spec Auto", preselectTeleOp = "CosmoboticsTeleOp")
public class BaseSpecAuto {

    public TransportFSM transport;

    public Follower follower;

    public Timer pathTimer, actionTimer, opmodeTimer;

    public int pathState;

    public static final double heading = 0;

    public static final double scoreX = 38;

    public static final double scoreY = 68;

    public static final double scoreYInc = .85;

    public static final double scoreXControl = 24;

    public static final double sampleX = 27;
    public static final double pushX = 57;
    public static final Point startPoint = new Point(9, 48, Point.CARTESIAN);

    public static final Point spec1scorePoint = new Point(scoreX, scoreY, Point.CARTESIAN);

    public static final Point sample1_1EndPoint = new Point(37.5, 37.5, Point.CARTESIAN);

    public static final Point sample1_1Control = new Point(24.7,68.8, Point.CARTESIAN);

    public static final Point sample1_2EndPoint = new Point(pushX,27, Point.CARTESIAN);

    public static final Point sample1_2Control = new Point(58,38, Point.CARTESIAN);

    public static final Point sample1_3EndPoint = new Point(sampleX,27, Point.CARTESIAN);

    public static final Point sample2_1EndPoint = new Point(pushX,18.9,Point.CARTESIAN);

    public static final Point sample2_2EndPoint = new Point(sampleX,19, Point.CARTESIAN);

    public static final Point sample3_1EndPoint = new Point(pushX,9,Point.CARTESIAN);

    public static final Point sample3_2EndPoint = new Point(sampleX,9, Point.CARTESIAN);

    public static final Point specPickupPoint = new Point(8.25,33,Point.CARTESIAN);

    public static final Point spec2PickupControl = new Point(22.9,32.7,Point.CARTESIAN);

    public static final Point spec2ScorePoint = new Point(scoreX,scoreY + scoreYInc,Point.CARTESIAN);

    public static final Point specScoreReturnPoint = new Point(scoreXControl,specPickupPoint.getY(),Point.CARTESIAN);

    public static final Path pickupSpec = new Path(new BezierLine(specScoreReturnPoint, specPickupPoint));

    public static final Point spec3ScorePoint = new Point(scoreX, scoreY + 2 * scoreYInc, Point.CARTESIAN);

    public static final Point spec4ScorePoint = new Point(scoreX, scoreY + 3 * scoreYInc, Point.CARTESIAN);

    public static final Point spec5ScorePoint = new Point(scoreX, scoreY + 4 * scoreYInc, Point.CARTESIAN);
    public static final Point spec6ScorePoint = new Point(scoreX, scoreY + 5 * scoreYInc, Point.CARTESIAN);

    public static final Point parkPoint = new Point(17,32.5,Point.CARTESIAN);

    public static Path park;

    public static PathChain moveFirstSample, moveSecondSample, moveThirdSample, scoreSpec1, pickupSpec2, scoreSpec2, pickupSpec3, scoreSpec3, pickupSpec4, scoreSpec4, pickupSpec5, scoreSpec5, pickupSpec6, scoreSpec6, moveFirstSample1, moveFirstSample2, moveFirstSample3, moveSecondSample1, moveSecondSample2, moveThirdSample1, moveThirdSample2;

    public void buildPaths() {



        scoreSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoint, spec1scorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveFirstSample = follower.pathBuilder()
                .addPath(new BezierCurve(spec1scorePoint, sample1_1Control, sample1_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .addPath(new BezierCurve(sample1_1EndPoint, sample1_2Control, sample1_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .addPath(new BezierLine(sample1_2EndPoint, sample1_3EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveSecondSample = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_3EndPoint, sample1_2EndPoint, sample2_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .addPath(new BezierLine(sample2_1EndPoint,sample2_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveThirdSample = follower.pathBuilder()
                .addPath(new BezierCurve(sample2_2EndPoint,sample2_1EndPoint, sample3_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .addPath(new BezierLine(sample3_1EndPoint,sample3_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveFirstSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(spec1scorePoint, sample1_1Control, sample1_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(0)
                .build();

        moveFirstSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_1EndPoint, sample1_2Control, sample1_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(0)
                .build();

        moveFirstSample3 = follower.pathBuilder()
                .addPath(new BezierLine(sample1_2EndPoint, sample1_3EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveSecondSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_3EndPoint, sample1_2EndPoint, sample2_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(0)
                .build();

        moveSecondSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample2_1EndPoint,sample2_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveThirdSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(sample2_2EndPoint,sample2_1EndPoint, sample3_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(0)
                .build();

        moveThirdSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample3_1EndPoint,sample3_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(0)
                .build();

        pickupSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(sample3_2EndPoint, new Point(specPickupPoint.getX() - 1, specPickupPoint.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specPickupPoint.getX() - 1, specPickupPoint.getY(), Point.CARTESIAN), spec2ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        pickupSpec.setConstantHeadingInterpolation(heading);

        pickupSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(spec2ScorePoint, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec3ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        pickupSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(spec3ScorePoint, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec4ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        pickupSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(spec4ScorePoint, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec5ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        pickupSpec6 = follower.pathBuilder()
                .addPath(new BezierLine(spec5ScorePoint, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        scoreSpec6 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec6ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        park = new Path(new BezierLine(spec6ScorePoint, parkPoint));
        park.setConstantHeadingInterpolation(heading);
    }

    public static double intakeSpecTime = 0.05;
    public static double startPathTime = 0.2;

    public static double prepTime = .1;

    public static double scoreTime = 0;
    public static double waitPrepTime = .25;

    public static double extendTime = 1;
    public static double intakeTime = 1;
    public static double dropTime = 1.25;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                follower.followPath(scoreSpec1, true);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() >= extendTime) {
                    transport.sampleTransport = TransportFSM.SampleTransport.EXTENDED;
                }
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() >= intakeTime || transport.updateColor() == 1) {
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING;
                    follower.followPath(moveFirstSample, true);
                    setPathState(2);
                }
                else {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKE;
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime + dropTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime + dropTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetrctWit2 + waitPrepTime + dropTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME_TWO;
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= dropTime) {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                }

                if(!follower.isBusy()) {
                    follower.followPath(moveSecondSample, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(moveThirdSample,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() >= startPathTime) {
                    follower.followPath(scoreSpec2, true);
                    setPathState(6);
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                    follower.followPath(pickupSpec3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME_TWO;
                }

                if(!follower.isBusy()) {
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() >= startPathTime) {
                    follower.followPath(scoreSpec3, true);
                    setPathState(8);
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                    follower.followPath(pickupSpec4, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME_TWO;
                }

                if(!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= startPathTime) {
                    follower.followPath(scoreSpec4, true);
                    setPathState(10);
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                    follower.followPath(pickupSpec5, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME_TWO;
                }

                if(!follower.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() >= startPathTime) {
                    follower.followPath(scoreSpec5, true);
                    setPathState(19);
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                    follower.followPath(pickupSpec6, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME_TWO;
                }

                if(!follower.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds() >= startPathTime) {
                    follower.followPath(scoreSpec6, true);
                    setPathState(12);
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                    follower.followPath(park, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait + TransportFSM.specRetrctWit2 + waitPrepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetrctWit2 + waitPrepTime ) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME_TWO;
                }

                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setAllianceConstants(boolean allianceRed, int telemode, boolean colorSensor) {
        AutoStorage.isAuto = true;
        AllianceStorage.isRed = allianceRed;
        AllianceStorage.teleMode = telemode;
        AllianceStorage.useColorSensor = colorSensor;
    }

    public BaseSpecAuto(HardwareMap hardwareMap) {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), heading));
        buildPaths();

        transport = new TransportFSM(hardwareMap);
    }

    public void update(Telemetry telemetry) {
        follower.update();
        transport.update();
        autonomousPathUpdate();
        transport.flickerOut(false);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("specClw", transport.specClawPos);
        telemetry.addData("trnsportstte", transport.specimenTransport);
        telemetry.update();
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

//    @Override
//    public void loop() {
//        follower.update();
//        transport.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("specClw", transport.specClawPos);
//        telemetry.addData("trnsportstte", transport.specimenTransport);
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), heading));
//        buildPaths();
//
//        transport = new TransportFSM(hardwareMap);
//    }
//
//    @Override
//    public void init_loop() {}
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void stop() {
//    }
}