
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

import org.firstinspires.ftc.teamcode.enums.INTAKESTATE;
import org.firstinspires.ftc.teamcode.enums.OUTTAKESTATE;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogicsOld;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Parking 6", group = "Autonomous")
@Configurable // Panels
public class AutoParkingRed extends OpMode {
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
        follower.setStartingPose(new Pose(118, 137, Math.toRadians(37)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        this.robot = new Robot(hardwareMap,true);
        this.task = new TaskLogicsOld(this.robot,false);
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
        public PathChain StartOuttake;
        public PathChain Intake;
        public PathChain IntakeEnd;
        public PathChain OuttakeEnd;
        public PathChain Parking;

        public Paths(Follower follower) {
            StartOuttake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.000, 129.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            Intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(63.026, 67.809),
                                    new Pose(45.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 60.000),

                                    new Pose(20.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            OuttakeEnd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.000, 60.000),
                                    new Pose(46.600, 74.452),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118, 137),

                                    new Pose(95, 63)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))

                    .build();
        }
    }





//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                delay(1);
                followPath(paths.StartOuttake,1,true);
                task.setOuttakeState(OUTTAKESTATE.SHOOT);
                delay(2);
                task.setOuttakeState(OUTTAKESTATE.REST);
                setPathState(1);
                follower.getPose();
                break;
            case 1:
                if(!follower.isBusy()) {
                    task.setIntakeState(INTAKESTATE.INTAKE);
                    delay(0.2);
                    followPath(paths.Intake,1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    delay(0.1);
                    followPath(paths.IntakeEnd,0.6,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    delay(0.1);
                    task.setIntakeState(INTAKESTATE.STOP);
                    followPath(paths.OuttakeEnd,1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    task.setOuttakeState(OUTTAKESTATE.SHOOT);
                    delay(2);
                    task.setOuttakeState(OUTTAKESTATE.REST);
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