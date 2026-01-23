
package org.firstinspires.ftc.teamcode.autonomous;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoTriangle", group = "Autonomous")
@Configurable // Panels
public class AutoTirangle extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Servo servo;

    //골대 시작 각도: -36도(블루)

    private enum ServoState {
        IDLE,
        OPEN,
        CLOSE
    }
    private ServoState servoState = ServoState.IDLE;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        servo = hardwareMap.servo.get("servo");
        servo.setPosition(0);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




//#region paths



    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(51.000, 38.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.000, 38.000),

                                    new Pose(93.000, 38.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93.000, 38.000),

                                    new Pose(72.000, 72.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }




//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                followPath(paths.Path1,1,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    delay(0.1);
                    followPath(paths.Path2,1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    delay(0.1);
                    followPath(paths.Path3,1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    delay(0.1);
                    followPath(paths.Path1,1,true);
                    setPathState(1);
                }
            case -1:
                //IDLE
                panelsTelemetry.addData("status: ","complete");
                break;

        }
        switch (servoState){
            case IDLE:
                break;
            case OPEN:
                servo.setPosition(1);
                setServoState(ServoState.IDLE);
                break;
            case CLOSE:
                servo.setPosition(0);
                setServoState(ServoState.IDLE);
                break;
        }
    }
    void setPathState(int state){
        pathState = state;
    }
    void setServoState(ServoState state){
        servoState = state;
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