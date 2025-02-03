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
    private final double scoreYInc = 1.5;
    private final Point startPoint = new Point(9, 48, Point.CARTESIAN);
    private final Point spec1scorePoint = new Point(scoreX, scoreY, Point.CARTESIAN);
    private final Point sample1_1EndPoint = new Point(48.05600933488915, 34.61376896149358, Point.CARTESIAN);
    private final Point sample1_1Control1 = new Point(26.91588785046729,69.53271028037383, Point.CARTESIAN);
    private final Point sample1_1Control2 = new Point(27.14018691588785,28.93457943925233, Point.CARTESIAN);

    private final Point sample1_2EndPoint = new Point(24.22429906542056,24.672897196261687, Point.CARTESIAN);
    private final Point sample1_2Control = new Point(85.5262543757293,29.236872812135353, Point.CARTESIAN);
    private final Point sample2EndPoint = new Point(23.775700934579437,14.803738317757011,Point.CARTESIAN);
    private final Point sample2Control1 = new Point(74.24299065420563,28.26168224299066, Point.CARTESIAN);
    private final Point sample2Control2 = new Point(75.14018691588785,13.233644859813078, Point.CARTESIAN);
    private final Point sample3EndPoint = new Point(24,9,Point.CARTESIAN);
    private final Point sample3Control1 = new Point(81.64485981308411,16.37383177570093, Point.CARTESIAN);
    private final Point sample3Control2 = new Point(60.33644859813084,5.607476635514011, Point.CARTESIAN);
    private final Point sample3Control3 = new Point(52.71028037383178,7.850467289719619, Point.CARTESIAN);
    private final Point specPickupPoint = new Point(9,33,Point.CARTESIAN);
    private final Point spec2Control = new Point(22.878504672897193,32.74766355140187,Point.CARTESIAN);
    private final Point spec2ScorePoint = new Point(scoreX,scoreY + scoreYInc,Point.CARTESIAN);
    private final Point spec3Control = new Point(15.25233644859813,27.140186915887845,Point.CARTESIAN);
    private final Point spec3ScorePoint = new Point(scoreX, scoreY + 2 * scoreYInc, Point.CARTESIAN);
    private final Point spec4Control = new Point(21,30,Point.CARTESIAN);
    private final Point spec4ScorePoint = new Point(scoreX, scoreY + 3 * scoreYInc, Point.CARTESIAN);
    private final Point spec5Control = new Point(16.149532710280372,27.140186915887845,Point.CARTESIAN);
    private final Point spec5ScorePoint = new Point(scoreX, scoreY + 4 * scoreYInc, Point.CARTESIAN);
    private final Point parkPoint = new Point(17.046728971962615,32.52336448598129,Point.CARTESIAN);
    private Path park;
    private PathChain scoreSpec1, pickupSpec2, scoreSpec2, pickupSpec3, scoreSpec3, pickupSpec4, scoreSpec4, pickupSpec5, scoreSpec5, moveFirstSample1, moveFirstSample2, moveSecondSample, moveThirdSample;

    public void buildPaths() {

        scoreSpec1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoint, spec1scorePoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        moveFirstSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(spec1scorePoint, spec1scorePoint, sample1_1Control1, sample1_1Control2, sample1_1EndPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        moveFirstSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_1EndPoint, sample1_2Control, sample1_2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        moveSecondSample = follower.pathBuilder()
                .addPath(new BezierCurve(sample1_2EndPoint, sample2Control1, sample2Control2, sample2EndPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        moveThirdSample = follower.pathBuilder()
                .addPath(new BezierCurve(sample2EndPoint,sample3Control1,sample3Control2,sample3Control3,sample3EndPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        pickupSpec2 = follower.pathBuilder()
                .addPath(new BezierCurve(sample3EndPoint,spec2Control,specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec2ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        pickupSpec3 = follower.pathBuilder()
                .addPath(new BezierCurve(spec2ScorePoint, spec3Control, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec3ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        pickupSpec4 = follower.pathBuilder()
                .addPath(new BezierCurve(spec3ScorePoint, spec4Control, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec4ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        pickupSpec5 = follower.pathBuilder()
                .addPath(new BezierCurve(spec4ScorePoint, spec5Control, specPickupPoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        scoreSpec5 = follower.pathBuilder()
                .addPath(new BezierLine(specPickupPoint, spec5ScorePoint))
                .setConstantHeadingInterpolation(heading)
                .build();

        park = new Path(new BezierLine(spec5ScorePoint, parkPoint));
        park.setConstantHeadingInterpolation(heading);
    }

    public static double intakeSpecTime = .01;
    public static double prepTime = .1;
    public static double scoreTime = 0;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scoreSpec1);
                setPathState(1);
                break;
            case 1:
                transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime + intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }

                if(!follower.isBusy()) {
                    follower.followPath(moveFirstSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(moveFirstSample2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy()) {
                    follower.followPath(moveSecondSample,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(moveThirdSample,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec2);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpec2);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime + intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }

                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec3);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy()) {
                    follower.followPath(scoreSpec3);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime + intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }

                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec4);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy()) {
                    follower.followPath(scoreSpec4);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime + intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }

                if(!follower.isBusy()) {
                    follower.followPath(pickupSpec5);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specScoreWait + TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SPECIMEN_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRetractWait) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= scoreTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.OPEN;
                }

                if(!follower.isBusy()) {
                    follower.followPath(scoreSpec5);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.specRotWait + prepTime + intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.SCORE;
                } else if (pathTimer.getElapsedTimeSeconds() >= prepTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
                } else if (pathTimer.getElapsedTimeSeconds() >= intakeSpecTime) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.INTAKE_SPEC;
                }

                if(!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(14);
                }
                break;
            case 14:
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