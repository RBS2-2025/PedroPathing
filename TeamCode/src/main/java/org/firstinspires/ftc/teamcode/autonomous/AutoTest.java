
package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoTest", group = "Autonomous")
@Configurable // Panels
public class AutoTest extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, waitTimer;
    private Servo servo;

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
        follower.setStartingPose(new Pose(56, 136, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();
        waitTimer = new Timer();
        waitTimer.resetTimer();

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


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 136.000),

                                    new Pose(33.670, 109.757)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.670, 109.757),

                                    new Pose(34.174, 58.957)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.174, 58.957),

                                    new Pose(18.783, 59.687)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.783, 59.687),

                                    new Pose(48.513, 97.757)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.513, 97.757),

                                    new Pose(11.609, 60.739)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.609, 60.739),

                                    new Pose(42.826, 100.174)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(135))

                    .build();

        }
    }


    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                followPath(paths.Path1,false);
                setServoState(ServoState.OPEN);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    followPath(paths.Path2,true);
                    setServoState(ServoState.CLOSE);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    followPath(paths.Path3,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    followPath(paths.Path4,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    followPath(paths.Path5,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    followPath(paths.Path6,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(waitTimer.getElapsedTime() > 1){
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            case -1:
                //IDLE
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
        waitTimer.resetTimer();
    }
    void setServoState(ServoState state){
        servoState = state;
    }
    void followPath(PathChain path,boolean holdEnd){
        pathTimer.resetTimer();
        follower.followPath(path,holdEnd);
    }
}
    