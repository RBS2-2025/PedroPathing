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
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.BLOCKSTATE;
import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.TRACKINGSTATE;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogics;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

@Autonomous(name = "AutoTest", group = "Autonomous")
@Configurable // Panels
public class TaskTest extends OpMode {
    PanelsHelper panels;
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    public static int useTask = 1;
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    Robot robot;
    TaskLogics task;

    //골대 시작 각도: -36도(블루)

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        robot = new Robot(hardwareMap,true);
        task = new TaskLogics(robot,true);

        panels.debug("Status: Initialized");
        panels.update();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        task.loop(); // do task

        // Log values to Panels and Driver Station
        panels.addData("Path State", pathState);
        panels.addData("X", follower.getPose().getX());
        panels.addData("Y", follower.getPose().getY());
        panels.addData("Heading", follower.getPose().getHeading());
        panels.update();
    }



    public static class Paths {
        public PathChain Home;
        public PathChain Shoot;
        public PathChain Right;
        public PathChain Left;
        public PathChain Back;
        public PathChain Straight;

        public Paths(Follower follower) {
            Home = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(72.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                    .build();

            Right = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(96.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Left = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 72.000),

                                    new Pose(48.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Back = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 72.000),

                                    new Pose(72.000, 96.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();

            Straight = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 96.000),

                                    new Pose(72.000, 48.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();
        }
    }




    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                setPathState(useTask);
                break;
            case 1:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    setPathState(-1);
                }
                break;
            case 2:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setIntakeState(INTAKESTATE.DISCHARGE);
                    setPathState(-1);
                }
                break;
            case 3:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    setPathState(-3);
                }
                break;
            case -3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    task.setOuttakeState(OUTTAKESTATE.REST);
                    setPathState(-1);
                }
                break;
            case 4:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setBlockState(BLOCKSTATE.BLOCK);
                    setPathState(-4);
                }
                break;
            case -4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    task.setBlockState(BLOCKSTATE.OPEN);
                    setPathState(-1);
                }
                break;
            case 5:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setTrackingState(TRACKINGSTATE.TRACK);
                    setPathState(-5);
                }
                break;
            case -5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    task.setTrackingState(TRACKINGSTATE.STOP);
                    setPathState(-1);
                }
                break;
            case 6:
                if(!follower.isBusy() && !task.isBusy()){
                    task.setTrackingState(TRACKINGSTATE.RESET);
                    setPathState(-1);
                }
                break;
            case 7:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Straight,1,true);
                    setPathState(-7);
                }
                break;
            case 8:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Left,1,true);
                    setPathState(-7);
                }
                break;
            case 9:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Right,1,true);
                    setPathState(-7);
                }
                break;
            case 10:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Back,1,true);
                    setPathState(-7);
                }
                break;
            case 11:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Shoot,1,true);
                    setPathState(-7);
                }
                break;
            case -7:
                if(!follower.isBusy() && !task.isBusy()){
                    followPath(paths.Home,1,true);
                    setPathState(-1);
                }
                break;

            case -1:
                //IDLE
                if(!follower.isBusy() && !task.isBusy()){
                    task.setIntakeState(INTAKESTATE.STOP);
                    task.setBlockState(BLOCKSTATE.OPEN);
                    task.setOuttakeState(OUTTAKESTATE.REST);
                    task.setTrackingState(TRACKINGSTATE.STOP);
                    panels.addData("status: ","complete");
                }
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
}