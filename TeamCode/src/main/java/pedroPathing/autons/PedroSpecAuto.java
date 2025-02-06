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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import teleop.transport.EncoderStorage;
import teleop.transport.TransportFSM;

@Autonomous(name = "Pedro Spec Auto", preselectTeleOp = "CosmoboticsTeleOp")
public class PedroSpecAuto extends OpMode{

    private TransportFSM transport;

    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final double heading = 0;

    private final double scoreX = 37;

    private final double scoreY = 68;

    private final double scoreYInc = .75;

//    private final double sampleX = 22;

    private final double sampleX = 36.5;
    private final Point startPoint = new Point(9, 48, Point.CARTESIAN);

    private final Point spec1scorePoint = new Point(scoreX, scoreY, Point.CARTESIAN);

    private final Point sample1_1EndPoint = new Point(32.7, 36.4, Point.CARTESIAN);

    private final Point sample1_1Control = new Point(24.7,68.8, Point.CARTESIAN);

    private final Point sample1_2EndPoint = new Point(61.2,29, Point.CARTESIAN);

    private final Point sample1_2Control = new Point(58,38, Point.CARTESIAN);

    private final Point sample1_3EndPoint = new Point(sampleX,29.2, Point.CARTESIAN);

    private final Point sample2_1EndPoint = new Point(59,18.9,Point.CARTESIAN);

    private final Point sample2_2EndPoint = new Point(sampleX,19, Point.CARTESIAN);

    private final Point sample3_1EndPoint = new Point(58.9,9,Point.CARTESIAN);

    private final Point sample3_2EndPoint = new Point(sampleX,9, Point.CARTESIAN);

    private final Point specPickupPoint = new Point(9,33,Point.CARTESIAN);

    private final Point spec2Control = new Point(22.9,32.7,Point.CARTESIAN);

    private final Point spec2ScorePoint = new Point(scoreX,scoreY + scoreYInc,Point.CARTESIAN);

    private final Point spec3Control = new Point(15.3,27.1,Point.CARTESIAN);

    private final Point spec3ScorePoint = new Point(scoreX, scoreY + 2 * scoreYInc, Point.CARTESIAN);

    private final Point spec4Control = new Point(21,30,Point.CARTESIAN);

    private final Point spec4ScorePoint = new Point(scoreX, scoreY + 3 * scoreYInc, Point.CARTESIAN);

    private final Point spec5Control = new Point(16.1,27.1,Point.CARTESIAN);

    private final Point spec5ScorePoint = new Point(scoreX, scoreY + 4 * scoreYInc, Point.CARTESIAN);

    private final Point parkPoint = new Point(17,32.5,Point.CARTESIAN);

    private Path park;

    private PathChain moveFirstSample, moveSecondSample, moveThirdSample, scoreSpec1, pickupSpec2, scoreSpec2, pickupSpec3, scoreSpec3, pickupSpec4, scoreSpec4, pickupSpec5, scoreSpec5, moveFirstSample1, moveFirstSample2, moveFirstSample3, moveSecondSample1, moveSecondSample2, moveThirdSample1, moveThirdSample2;

    public void buildPaths() {

        scoreSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoint, spec1scorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.2)
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
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveFirstSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_1EndPoint, sample1_2Control, sample1_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveFirstSample3 = follower.pathBuilder()
                .addPath(new BezierLine(sample1_2EndPoint, sample1_3EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveSecondSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_3EndPoint, sample1_2EndPoint, sample2_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveSecondSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample2_1EndPoint,sample2_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveThirdSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(sample2_2EndPoint,sample2_1EndPoint, sample3_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        moveThirdSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample3_1EndPoint,sample3_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.25)
                .build();

        //TODO: CHIN INTO PICKUP SPEC 2

        pickupSpec2 = follower.pathBuilder()
                .addPath(new BezierCurve(sample3_2EndPoint,spec2Control,specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.5)
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec2ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.2)
                .build();

        pickupSpec3 = follower.pathBuilder()
                .addPath(new BezierCurve(spec2ScorePoint, spec3Control, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.5)
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec3ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.2)
                .build();

        pickupSpec4 = follower.pathBuilder()
                .addPath(new BezierCurve(spec3ScorePoint, spec4Control, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.5)
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec4ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.2)
                .build();

        pickupSpec5 = follower.pathBuilder()
                .addPath(new BezierCurve(spec4ScorePoint, spec5Control, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.5)
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec5ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .setPathEndTimeoutConstraint(.2)
                .build();

        park = new Path(new BezierLine(spec5ScorePoint, parkPoint));
        park.setConstantHeadingInterpolation(heading);
    }

    public static double intakeSpecTime = 2.25;

    public static double prepTime = .1;

    public static double scoreTime = 0;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                follower.followPath(scoreSpec1, true);
                setPathState(1);
                break;
            case 1:
                transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    follower.followPath(moveFirstSample, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy()) {
                    follower.followPath(moveSecondSample, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(moveThirdSample,true);
                    setPathState(8);
                }
                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    follower.followPath(moveSecondSample1,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    follower.followPath(moveSecondSample2,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    follower.followPath(moveThirdSample1,true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()) {
//                    follower.followPath(moveThirdSample2,true);
//                    setPathState(8);
//                }
//                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec2,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime - 1.25) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpec2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= intakeSpecTime + .25) {
                    follower.followPath(scoreSpec3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec4, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= intakeSpecTime + .25) {
                    follower.followPath(scoreSpec4, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                }

                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec5, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= intakeSpecTime + .25) {
                    follower.followPath(scoreSpec5, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }

                if(!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
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

    @Override
    public void loop() {
        follower.update();
        transport.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("specClw", transport.specClawPos);
        telemetry.addData("trnsportstte", transport.specimenTransport);
        telemetry.update();
    }

    @Override
    public void init() {
        EncoderStorage.isAuto = true;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), heading));
        buildPaths();

        transport = new TransportFSM(hardwareMap);
        transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}