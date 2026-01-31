
package org.firstinspires.ftc.teamcode.autonomous.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.STATES;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogicsOld;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Main", group = "Autonomous")
@Configurable // Panels
public class AutoParkingRed9 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Robot robot;
    TaskLogicsOld task;

    //골대 시작 각도: -36도(블루)

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25, 129, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        this.robot = new Robot(hardwareMap,true);
        this.task = new TaskLogicsOld(this.robot,true);
        this.task.start();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        task.loop();
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




//#region paths










    public static class Paths {
        public PathChain OuttakeStart;
        public PathChain Intake;
        public PathChain IntakeEnd;
        public PathChain Outtake2;
        public PathChain IntakeTop;
        public PathChain IntakeTopEnd;
        public PathChain Outtake3;
        public PathChain Parking;

        public Paths(Follower follower) {
            OuttakeStart = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(118.000, 130.000),
                                    new Pose(111.750, 113.500),
                                    new Pose(106.500, 104.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(45))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(106.500, 104.000),
                                    new Pose(83.270, 79.791),
                                    new Pose(96.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            IntakeEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 60.000),

                                    new Pose(129.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Outtake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.000, 60.000),
                                    new Pose(93.557, 70.857),
                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            IntakeTop = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),

                                    new Pose(96.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            IntakeTopEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 84.000),

                                    new Pose(129.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Outtake3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.000, 84.000),
                                    new Pose(97.105, 80.666),
                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),

                                    new Pose(96.000, 69.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();
        }
    }










//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                delay(0.5);
                followPath(paths.OuttakeStart,1,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    if(task.getTime(STATES.OUTTAKE) > 2){
                        task.setBlockState(BLOCKSTATE.OPEN);
                        task.setTrackingState(TRACKINGSTATE.TRACK);
                        delay(0.1);
                    }
                    if(task.getTime(STATES.OUTTAKE) > 5){
                        task.setOuttakeState(OUTTAKESTATE.REST);
                        task.setBlockState(BLOCKSTATE.BLOCK);
                        task.setTrackingState(TRACKINGSTATE.RESET);
                        task.setIntakeState(INTAKESTATE.STOP);
                        delay(0.1);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    followPath(paths.Intake,1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    followPath(paths.IntakeEnd,0.4,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.STOP);
                    followPath(paths.Outtake2,1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    if(task.getTime(STATES.OUTTAKE) > 2){
                        task.setBlockState(BLOCKSTATE.OPEN);
                        task.setTrackingState(TRACKINGSTATE.TRACK);
                        delay(0.1);
                    }
                    if(task.getTime(STATES.OUTTAKE) > 5){
                        task.setOuttakeState(OUTTAKESTATE.REST);
                        task.setBlockState(BLOCKSTATE.BLOCK);
                        task.setTrackingState(TRACKINGSTATE.RESET);
                        delay(0.1);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    followPath(paths.IntakeTop,1,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    followPath(paths.IntakeTopEnd,0.4,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    task.setIntakeState(INTAKESTATE.STOP);
                    followPath(paths.Outtake3,1,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    if(task.getTime(STATES.OUTTAKE) > 2){
                        task.setBlockState(BLOCKSTATE.OPEN);
                        task.setTrackingState(TRACKINGSTATE.TRACK);
                        delay(0.1);
                    }
                    if(task.getTime(STATES.OUTTAKE) > 5){
                        task.setOuttakeState(OUTTAKESTATE.REST);
                        task.setBlockState(BLOCKSTATE.BLOCK);
                        task.setTrackingState(TRACKINGSTATE.RESET);
                        delay(0.1);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    followPath(paths.Parking,1,true);
                    setPathState(-1);
                }
                break;

            case -1:
                //IDLE
                panelsTelemetry.addData("status: ","complete");
                break;

        }
    }
    void setPathState(int state){
        pathState = state;
    }
    void followPath(PathChain path,double maxPower,boolean holdEnd){
        pathTimer.resetTimer();
        follower.followPath(path,maxPower,holdEnd);
    }
    void delay(double t){
        Timer waitTimer = new Timer();
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTimeSeconds() <= t){
            panelsTelemetry.addData("status: ", "waiting");
            panelsTelemetry.update();
        }
        return;
    }
}