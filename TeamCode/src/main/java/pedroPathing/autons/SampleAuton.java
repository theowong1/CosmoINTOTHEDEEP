package pedroPathing.autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import teleop.AllianceStorage;
import teleop.transport.AutoStorage;
import teleop.transport.TransportFSM;

@Autonomous(name = "Sample Autonomous", preselectTeleOp = "CosmoboticsTeleOp")
public class SampleAuton extends OpMode {
    public TransportFSM transport;

    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private double scoreHeading = Math.toRadians(-45);

    private double startAndSample1Heading = 0;

    private double sample2Heading = Math.toRadians(23.5);

    private double sample3Heading = Math.toRadians(90);

    private double parkHeading = Math.toRadians(-90);

    private final Point startPoint = new Point(9.000, 105.000, Point.CARTESIAN);

    private final Point scorePoint = new Point(11.000, 128.00, Point.CARTESIAN);

    private final Point scorePreloadPoint = new Point(scorePoint.getX()-2, scorePoint.getY()+2, Point.CARTESIAN);

    private final Point preloadControl = new Point(20.681, 106.277, Point.CARTESIAN);

    private final Point sample1Point = new Point(14, 122.3, Point.CARTESIAN);

    private final Point sample2Point = new Point(14.5, 122.5, Point.CARTESIAN);

    private final Point sample3Point = new Point(45, 110.5, Point.CARTESIAN);

    private final Point parkPoint = new Point(67.979, 96.128, Point.CARTESIAN);

    private final Point parkControl = new Point(76.596, 118.532, Point.CARTESIAN);

    private PathChain scorePreload, pickupSample1, pickupSample2, pickupSample3, scorePickup1, scorePickup2, scorePickup3, park;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(startPoint, preloadControl, scorePreloadPoint))
                .setLinearHeadingInterpolation(startAndSample1Heading, scoreHeading)
                .build();

        pickupSample1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePreloadPoint, sample1Point))
                .setLinearHeadingInterpolation(scoreHeading, startAndSample1Heading)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(sample1Point, scorePoint))
                .setLinearHeadingInterpolation(startAndSample1Heading, scoreHeading)
                .build();

        pickupSample2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoint, sample2Point))
                .setLinearHeadingInterpolation(scoreHeading, sample2Heading)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(sample2Point, scorePoint))
                .setLinearHeadingInterpolation(sample2Heading, scoreHeading)
                .build();

        pickupSample3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoint, sample3Point))
                .setLinearHeadingInterpolation(scoreHeading, sample3Heading)
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(sample3Point, scorePoint))
                .setLinearHeadingInterpolation(sample3Heading, scoreHeading)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoint, parkControl, parkPoint))
                .setLinearHeadingInterpolation(scoreHeading, parkHeading)
                .build();
    }

    public static double retractWait = 1.5;
    public static double rotWait = .75;
    public static double intakeWait = 3;
    public static double scoreTimeout = 2.25;
    public static double transferTimeout = 3;
    public static double scorePos = 2000;

    public static double sample3Wait = 1.25;

    public static double sample1Wait = 0.5;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                transport.flickerOut(false);
                transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() >= scoreTimeout - .75) {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMPPLUSEXTENDED;
                }

                if ((!follower.isBusy() && transport.outPos >= scorePos) && pathTimer.getElapsedTimeSeconds() >= scoreTimeout) {
                    follower.followPath(pickupSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > sample1Wait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.OUTPLUSEXTENDED;
                }
                if (!follower.isBusy()) {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKERETRACTOUT;
                }

                if((!follower.isBusy() && transport.updateColor() == 1) || pathTimer.getElapsedTimeSeconds() >= intakeWait) {
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if (transport.extendoPos <= 100 || pathTimer.getElapsedTimeSeconds() >= transferTimeout) {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                    setPathState(9);
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.emergencyWit){
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING;
                } else {
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING_ROT;
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= scoreTimeout - .75) {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMPPLUSEXTENDED;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.shortTransferWait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                }

                if((!follower.isBusy() && transport.outPos >= scorePos) && pathTimer.getElapsedTimeSeconds() >= scoreTimeout) {
                    follower.followPath(pickupSample2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > sample1Wait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.OUTPLUSEXTENDED;
                }
                if (!follower.isBusy()) {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKERETRACTOUT;
                }

                if((!follower.isBusy() && transport.updateColor() == 1) || pathTimer.getElapsedTimeSeconds() >= intakeWait) {
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (transport.extendoPos <= 100 || pathTimer.getElapsedTimeSeconds() >= transferTimeout) {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                    setPathState(10);
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.emergencyWit){
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING;
                } else {
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING_ROT;
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() >= scoreTimeout - .75) {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMPPLUSEXTENDED;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.shortTransferWait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                }

                if((!follower.isBusy() && transport.outPos >= scorePos) && pathTimer.getElapsedTimeSeconds() >= scoreTimeout) {
                    follower.followPath(pickupSample3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > sample3Wait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.OUTPLUSEXTENDED;
                }
                if (!follower.isBusy()) {
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKERETRACTOUT;
                }

                if((!follower.isBusy() && transport.updateColor() == 1) || pathTimer.getElapsedTimeSeconds() >= intakeWait) {
                    follower.followPath(scorePickup3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (transport.extendoPos <= 100 || pathTimer.getElapsedTimeSeconds() >= transferTimeout) {
                    transport.sampleTransport = TransportFSM.SampleTransport.TRANSFER;
                    setPathState(11);
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.emergencyWit){
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING;
                } else {
                    transport.sampleTransport = TransportFSM.SampleTransport.RETRACTING_ROT;
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= scoreTimeout - .75) {
                    transport.sampleTransport = TransportFSM.SampleTransport.DUMPPLUSEXTENDED;
                } else if (pathTimer.getElapsedTimeSeconds() >= TransportFSM.shortTransferWait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.HIGH_BUCKET;
                }

                if((!follower.isBusy() && transport.outPos >= scorePos) && pathTimer.getElapsedTimeSeconds() >= scoreTimeout) {
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() >= retractWait) {
                    transport.sampleTransport = TransportFSM.SampleTransport.SAMPLE_HOME;
                } else if (pathTimer.getElapsedTimeSeconds() >= .25){
                    transport.sampleTransport = TransportFSM.SampleTransport.INTAKERETRACTOUT;
                }

                if(!follower.isBusy()) {
                    transport.specimenTransport = TransportFSM.SpecimenTransport.PREP;
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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        transport.update();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        AutoStorage.isAuto = true;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        AllianceStorage.teleMode = 1;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY(), startAndSample1Heading));
        buildPaths();

        transport = new TransportFSM(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}

