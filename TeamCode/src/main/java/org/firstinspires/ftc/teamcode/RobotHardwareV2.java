package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class RobotHardwareV2 {
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor intake;
    public DcMotorEx flywheel;

    public Servo feeder;
    public Servo intakeRamp;
    public Servo rotateLauncher;
    public Servo trajectoryServo;
    public Servo indicatorLightMotor;
    public Servo indicatorLightLocation;
    public Servo gate2;
    public Servo gate3;
    public CRServo spin;

    public Limelight3A limelight;

    public double backVelocity = 1575; // 1600; //1550
    public double frontVelocity = 1000; //1100;
    public double middleVelocity = 1175; //1200

    private double flywheelTargetVelocity = 0;
    final double TIME_BETWEEN_SHOTS = .9;
    final double FEED_TIME = 0.2;

    final double TIME_BETWEEN_SHOTS_BACK = 1.3;
    final double FEED_TIME_BACK = 0.5;

    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();

    private LaunchState launchState = LaunchState.IDLE;
    private SortAndLaunchState sortAndLaunchState = SortAndLaunchState.IDLE;
    private int[] sortAndLaunchSequence;
    private double[] sortAndLaunchWaits;  // [waitAfterDrop0, waitAfterDrop1, waitAfterDrop2]
    private ElapsedTime sortAndLaunchTimer = new ElapsedTime();





    public int getDetectedAprilTag(Telemetry telemetry){
        LLResult llResult = limelight.getLatestResult();


        // telemetry.addLine("Detect April Tag");
        // telemetry.update();

        if (llResult != null && llResult.isValid()) {
            /* for debugging
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
             */


            // telemetry.addLine("got llresult" + llResult);
            // telemetry.update();

            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial

                telemetry.addData("April Tag Detected is: ", id);
                telemetry.update();
                if (id == 21 || id == 22 || id == 23){
                    return id;
                }
            }
        }
        telemetry.update();
        return 0;
    }

    public void adjustLauncherBlue(Telemetry telemetry){
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0;
        int id = 0;

        if (llResult == null || !llResult.isValid()){
            return;
        }


        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20){
                id = fiducial.getFiducialId(); // The ID number of the fiducial
                Tx = fiducial.getTargetXDegrees();

                telemetry.addData("ID:", id);
                telemetry.addData("Tx:", Tx);
                telemetry.update();
                break;
            }
        }


        if (id == 20) {

            if (Tx >= 0) {
                if ((rotateLauncher.getPosition() - (0.002*Math.abs(Tx))) >= 0.164){
                    rotateLauncher.setPosition(rotateLauncher.getPosition() - (0.002*Math.abs(Tx)));
                } else {
                    setBlueAngle();
                }
            } else if (Tx <= -2){ //Tx <= -3
                if ((rotateLauncher.getPosition() - (0.002*Math.abs(Tx))) <= 0.222){
                    rotateLauncher.setPosition(rotateLauncher.getPosition() + (0.002*Math.abs(Tx+3)));
                } else {
                    setRedAngle();
                }
            }
        }
    }

    public void adjustLauncherRed(){
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0;
        int id = 0;

        if (llResult == null || !llResult.isValid()){
            return;
        }


        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 24){
                id = fiducial.getFiducialId(); // The ID number of the fiducial
                Tx = fiducial.getTargetXDegrees();
                break;
            }
        }


        if (id == 24) {

            if (Tx >= 5) {
                if ((rotateLauncher.getPosition() - (0.002 * Math.abs(Tx))) >= 0.164) {
                    rotateLauncher.setPosition(rotateLauncher.getPosition() - (0.002 * Math.abs(Tx)));
                } else {
                    setBlueAngle();
                }
            } else if (Tx <= -1) {
                if ((rotateLauncher.getPosition() - (0.002 * Math.abs(Tx))) <= 0.222) {
                    rotateLauncher.setPosition(rotateLauncher.getPosition() + (0.002 * Math.abs(Tx + 3)));
                } else {
                    setRedAngle();
                }
            }
        }
    }

    public boolean adjustLauncherUsingAprilTagRedBack(){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

            if (Tx >= 0) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() - 0.002);
                return false;
            } else if (Tx <= -3){
                rotateLauncher.setPosition(rotateLauncher.getPosition() + 0.002);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    public boolean adjustLauncherUsingAprilTagBlueBack(){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

             if (Tx >= 5) {
                 rotateLauncher.setPosition(rotateLauncher.getPosition() - 0.002);
                 return false;
             } else if (Tx <= -1) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() + 0.002);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    public boolean adjustLauncherUsingAprilTagRedFront(Telemetry telemetry){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

            telemetry.addData("Tx: ", Tx);
            telemetry.addData("launchPos: ", rotateLauncher.getPosition());
            if (Tx >= 4) {
                double newPosition = rotateLauncher.getPosition() - 0.002;
                rotateLauncher.setPosition(newPosition);
                telemetry.addData("newPosition minus", newPosition);
                return false;
            } else if (Tx <= -6){
                double newPosition = rotateLauncher.getPosition() + 0.002;
                rotateLauncher.setPosition(newPosition);
                telemetry.addData("newPosition plus", newPosition);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    public boolean adjustLauncherUsingAprilTagBlueFront(){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

            if (Tx >= 5) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() - 0.002);
                return false;
            } else if (Tx <= -5) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() + 0.002);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    /** Maps launchSequence string (e.g. "1,2,3") to [waitFirst, waitSecond, waitThird] in seconds */
    private static final Map<String, double[]> SORT_AND_LAUNCH_WAIT_TIMES = new HashMap<>();
    static {
        SORT_AND_LAUNCH_WAIT_TIMES.put("1,2,3", new double[]{0.5, 0.5, 1.5});
        SORT_AND_LAUNCH_WAIT_TIMES.put("1,3,2", new double[]{0.7, 1.5, 1.2});
        SORT_AND_LAUNCH_WAIT_TIMES.put("2,1,3", new double[]{1.9, 0.3, 1.5});
        SORT_AND_LAUNCH_WAIT_TIMES.put("2,3,1", new double[]{1.5, 1.5, 0.5});
        SORT_AND_LAUNCH_WAIT_TIMES.put("3,1,2", new double[]{1.7, 0.3, 0.7});
        SORT_AND_LAUNCH_WAIT_TIMES.put("3,2,1", new double[]{1.5, 0.3, 0.3});
    }

//
//    public void launch(){
//        feeder.setPosition(.77);
//        sleep(200);
//        feeder.setPosition(0.47);
//    }

    public void dropGate2(){
        gate2.setPosition(0.84);
    }

    public void dropGate3(){
        gate3.setPosition(0.89);
    }

    public void gate2up() {
        gate2.setPosition(0.42); // ????
    }
    public void gate3up() {
        gate3.setPosition(0.47); // ???
    }
    public void gate2middle() {
        gate2.setPosition(0.48);
    }
    public void gate3middle() {
        gate3.setPosition(0.5);
    }

    private void dropByPosition(int position) {
        switch (position) {
            case 1: dropGate2(); break;
            case 2: dropGate3(); break;
            case 3: intakeRampDown();break;
        }
    }

    private enum LaunchState {
        IDLE,
        START_LAUNCH,
        WAIT_LAUNCH_COMPLETE,
    }

    private enum SortAndLaunchState {
        IDLE,
        DROP_0,
        WAIT_AFTER_DROP_0,
        DROP_1,
        WAIT_AFTER_DROP_1,
        START_LAUNCH_0,
        WAIT_LAUNCH_COMPLETE_0,
        DROP_2,
        START_LAUNCH_1,
        WAIT_LAUNCH_COMPLETE_1,
        WAIT_AFTER_DROP_2,
        START_LAUNCH_2,
        WAIT_LAUNCH_COMPLETE_2,
        START_LAUNCH_EXTRA,
        WAIT_LAUNCH_COMPLETE_EXTRA,
        DONE,
    }
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.START_LAUNCH;
                    shotTimer.reset();
                    stopSpin();
                }
                break;
            case START_LAUNCH:
                if (flywheel.getVelocity() > flywheelTargetVelocity - 60 && flywheel.getVelocity() < flywheelTargetVelocity + 40){
                    launchState = LaunchState.WAIT_LAUNCH_COMPLETE;
                    //stopSpin();
                    setFeedLaunch();
                    //feeder.setPosition(0.40);


                    feederTimer.reset();
                }
                break;
            case WAIT_LAUNCH_COMPLETE:
                if (feederTimer.seconds() > FEED_TIME) {
                    setFeedDown();
                    //feeder.setPosition(0.66);
                    startSpin();

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    /**
     * sortAndLaunch - State machine for sorted launch sequence.
     * Transition order is driven by launchSequence: drop first, wait, drop second, wait, launch 1,
     * drop third, wait (waits[2]), launch 2, launch 3. All waits use WAIT_AFTER_DROP.
     * @param launchSequence Array of 3 integers (1-3) specifying gate order
     * @return true when entire sequence is complete, false if sequence not in wait-times map
     *
     *
     *
     *
     * State sequence:
        IDLE
        DROP_0 → drop sequence[0]
        WAIT_AFTER_DROP_0 → wait waits[0]
        DROP_1 → drop sequence[1]
        WAIT_AFTER_DROP_1 → wait waits[1], then require okToLaunch for first launch
        START_LAUNCH_1 → launch 1 (requires okToLaunch)
        WAIT_LAUNCH_COMPLETE_1
        DROP_2 → drop sequence[2] (third item), start timer for waits[2]
        START_LAUNCH_2 → launch 2
        WAIT_LAUNCH_COMPLETE_2
        WAIT_AFTER_DROP_2 → wait waits[2] (only if longer than launch 2 time; timer started in DROP_2)
        START_LAUNCH_3 → launch 3
        WAIT_LAUNCH_COMPLETE_3 → IDLE
     */
    public void resetLaunchAndSortState(){
        sortAndLaunchState = SortAndLaunchState.IDLE;
    }
    boolean sortAndLaunch(boolean okToLaunch, int[] launchSequence, Telemetry telemetry) {
        if (launchSequence == null || launchSequence.length != 3) {
            return false;
        }

        switch (sortAndLaunchState) {
            case IDLE:
                String key = launchSequence[0] + "," + launchSequence[1] + "," + launchSequence[2];
                double[] waits = SORT_AND_LAUNCH_WAIT_TIMES.get(key);
                if (waits == null || waits.length < 3) {
                    return false;
                }
                sortAndLaunchSequence = launchSequence;
                sortAndLaunchWaits = waits;
                sortAndLaunchState = SortAndLaunchState.DROP_0;
                // set gates to stow positions
                gate2up();
                gate3up();
                intakeRampMiddle();
                reverseIntake();
                sleep(500);
                stopIntake();
                break;
            case DROP_0:
                telemetry.addLine("start drop_0");
                if (sortAndLaunchSequence[0] == 3) {
                    startIntake();
                }
                dropByPosition(sortAndLaunchSequence[0]);
                sortAndLaunchTimer.reset();
                startSpin();
                sortAndLaunchState = SortAndLaunchState.WAIT_AFTER_DROP_0;
                break;
            case WAIT_AFTER_DROP_0:
                if (sortAndLaunchTimer.seconds() > sortAndLaunchWaits[0]) {
                    telemetry.addLine("done drop_0");
                    sortAndLaunchState = SortAndLaunchState.DROP_1;
                    if (sortAndLaunchSequence[0] == 3) {
                        stopIntake();
                    }
                }
                break;
            case DROP_1:
                telemetry.addLine("start drop_1");
                if (sortAndLaunchSequence[1] == 3) {
                    startIntake();
                }
                dropByPosition(sortAndLaunchSequence[1]);
                sortAndLaunchTimer.reset();
                sortAndLaunchState = SortAndLaunchState.WAIT_AFTER_DROP_1;
                break;
            case WAIT_AFTER_DROP_1:
                if (sortAndLaunchTimer.seconds() > sortAndLaunchWaits[1]) {
                    telemetry.addLine("done drop_1");
                    if (sortAndLaunchSequence[1] == 3) {
                        stopIntake();
                    }
                    stopSpin();
                    if (!okToLaunch) {
                        break;
                    }
                    sortAndLaunchState = SortAndLaunchState.START_LAUNCH_0;
                    shotTimer.reset();
                    feederTimer.reset();
                }
                break;
            case START_LAUNCH_0:
                if (flywheel.getVelocity() > flywheelTargetVelocity - 60 && flywheel.getVelocity() < flywheelTargetVelocity + 40) {
                    telemetry.addLine("start launch_0");

                    stopSpin();
                    setFeedLaunch();
                    //feeder.setPosition(0.40);
                    feederTimer.reset();
                    sortAndLaunchState = SortAndLaunchState.WAIT_LAUNCH_COMPLETE_0;
                }
                break;
            case WAIT_LAUNCH_COMPLETE_0:
                if (feederTimer.seconds() > FEED_TIME) {
                    telemetry.addLine("done launch_0");
                    setFeedDown();
                    //feeder.setPosition(0.66);
                    startSpin();
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        sortAndLaunchState = SortAndLaunchState.START_LAUNCH_1;
                        shotTimer.reset();
                    }
                }
                break;
            case START_LAUNCH_1:
                if (flywheel.getVelocity() > flywheelTargetVelocity - 60 && flywheel.getVelocity() < flywheelTargetVelocity + 40) {
                    telemetry.addLine("start launch_1");
                    stopSpin();
                    setFeedLaunch();
                    //feeder.setPosition(0.40);
                    feederTimer.reset();
                    sortAndLaunchState = SortAndLaunchState.WAIT_LAUNCH_COMPLETE_1;
                }
                break;
            case WAIT_LAUNCH_COMPLETE_1:
                if (feederTimer.seconds() > FEED_TIME) {
                    telemetry.addLine("done launch_1");
                    setFeedDown();
                    //feeder.setPosition(0.66);
                    startSpin();
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        sortAndLaunchState = SortAndLaunchState.DROP_2;

                    }
                }
                break;
            case DROP_2:
                telemetry.addLine("start drop_2");
                if (sortAndLaunchSequence[2] == 3) {
                    startIntake();
                }
                startSpin();
                dropByPosition(sortAndLaunchSequence[2]);
                sortAndLaunchTimer.reset();
                sortAndLaunchState = SortAndLaunchState.WAIT_AFTER_DROP_2;
                shotTimer.reset();
                feederTimer.reset();
                break;

            case WAIT_AFTER_DROP_2:
                if (sortAndLaunchTimer.seconds() > sortAndLaunchWaits[2]) {
                    telemetry.addLine("done drop_2");
                    if (sortAndLaunchSequence[2] == 3) {
                        stopIntake();
                    }
                    sortAndLaunchState = SortAndLaunchState.START_LAUNCH_2;
                    shotTimer.reset();
                    feederTimer.reset();
                }
                break;
            case START_LAUNCH_2:
                if (flywheel.getVelocity() > flywheelTargetVelocity - 60 && flywheel.getVelocity() < flywheelTargetVelocity + 40) {
                    telemetry.addLine("start launch_2");

                    stopSpin();
                    setFeedLaunch();
                    //feeder.setPosition(0.40);
                    feederTimer.reset();
                    sortAndLaunchState = SortAndLaunchState.WAIT_LAUNCH_COMPLETE_2;
                }
                break;
            case WAIT_LAUNCH_COMPLETE_2:
                if (feederTimer.seconds() > FEED_TIME) {
                    telemetry.addLine("done launch_2");
                    setFeedDown();
                    //feeder.setPosition(0.66);
                    startSpin();
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        sortAndLaunchState = SortAndLaunchState.START_LAUNCH_EXTRA;
                        shotTimer.reset();
                        stopIntake();
                    }
                }
                break;
                case START_LAUNCH_EXTRA:
                    if (flywheel.getVelocity() > flywheelTargetVelocity - 60 && flywheel.getVelocity() < flywheelTargetVelocity + 40) {
                        gate2middle();
                        gate3middle();
                        stopSpin();
                        setFeedLaunch();
                        //feeder.setPosition(0.40);
                        feederTimer.reset();
                        sortAndLaunchState = SortAndLaunchState.WAIT_LAUNCH_COMPLETE_EXTRA;
                    }
                    break;
                case WAIT_LAUNCH_COMPLETE_EXTRA:
                    if (feederTimer.seconds() > FEED_TIME) {
                        setFeedDown();
                        //feeder.setPosition(0.66);
                        sortAndLaunchState = SortAndLaunchState.DONE;
                        return true;
                    }
                    break;
            default:
                break;
        }
        telemetry.update();

        return false;
    }

    boolean launchBack(boolean shotRequested){

        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.START_LAUNCH;
                    shotTimer.reset();
                }
                break;
            case START_LAUNCH:
                if (flywheel.getVelocity() > flywheelTargetVelocity - 60 && flywheel.getVelocity() < flywheelTargetVelocity + 40){
                    launchState = LaunchState.WAIT_LAUNCH_COMPLETE;
                    stopSpin();
                    setFeedLaunch();
                    //feeder.setPosition(0.40);

                    feederTimer.reset();
                }
                break;
            case WAIT_LAUNCH_COMPLETE:
                if (feederTimer.seconds() > FEED_TIME_BACK) {
                    setFeedDown();
                    //feeder.setPosition(0.66);
                    startSpin();

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS_BACK){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    public void setFeedLaunch(){
        feeder.setPosition(0.40);
    }

    public void setFeedDown(){
        feeder.setPosition(0.66);
    }

    public void startSpin(){
        spin.setPower(-1);
    }

    public void reverseSpin(){
        spin.setPower(1);
    }

    public void stopSpin(){
        spin.setPower(0);
    }


    public void launchPt2(){
        //feeder.setPosition(0.68);
    }

    public void startIntake(){
        intake.setPower(.70);
    }

    public void stopIntake(){
        intake.setPower(0);
    }


    public void resetMechanismsUp(){
        setFeedDown();
        //feeder.setPosition(0.66);

        intakeRampUp();
        //intakeRamp.setPosition(0.519);
        setAngleStraight();
        //rotateLauncher.setPosition(0.2);
        gate2up();
        gate3up();
        /*
        gate2.setPosition(0.45);
        gate3.setPosition(0.5);
         */
    }

    public void resetMechanismsMiddle(){
        setFeedDown();
        intakeRampUp();
        gate2middle();
        gate3middle();
    }



    public void resetMechanisms(){
        setFeedDown();
        //feeder.setPosition(0.66);
        intakeRampDown();
        //intakeRamp.setPosition(0.4655);
        setAngleStraight();
        //rotateLauncher.setPosition(0.2);
        dropGate2();
        dropGate3();
        /*
        gate2.setPosition(0.84);
        gate3.setPosition(0.89);
         */
    }

    public void reverseIntake(){
        intake.setPower(-0.7);
    }

    public void stopFlywheel(){
        flywheel.setVelocity(0.0);
    }

    public void setFlywheelSpeedBackPosition(){
        flywheel.setVelocity(backVelocity);
        flywheelTargetVelocity = backVelocity;
    }

    public void intakeRampUp(){
        intakeRamp.setPosition(0.5183);
    }

    public void intakeRampMiddle(){
        intakeRamp.setPosition(0.505);
    }

    public void intakeRampDown(){
        intakeRamp.setPosition(0.462);
    }

    public void setFlywheelSpeedFrontPosition(){
        flywheel.setVelocity(frontVelocity);
        flywheelTargetVelocity = frontVelocity;
    }

    public void setFlywheelSpeedMiddlePosition(){
        flywheel.setVelocity(middleVelocity);
        flywheelTargetVelocity = middleVelocity;
    }

    public double getFlywheelVelocityError() {
        return flywheelTargetVelocity - flywheel.getVelocity();
    }

    public void motorLightOff(){
        indicatorLightMotor.setPosition(0.0);
    }
    public void setColorGreenMotor(){
        indicatorLightMotor.setPosition(0.5);
    }

    public void setColorRedMotor(){
        indicatorLightMotor.setPosition(0.277);
    }

    public void locationLightOff(){
        indicatorLightLocation.setPosition(0.0);
    }

    public void setColorBlueLocation(){
        indicatorLightLocation.setPosition(0.611);
    }

    public void setColorOrangeLocation(){
        indicatorLightLocation.setPosition(0.388);
    }

    public void setNormalTrajectory(){
        trajectoryServo.setPosition(0.0);
    }

    public void setFarTrajectory(){
        trajectoryServo.setPosition(0.3);
    }

    public void setBlueAngle(){
        rotateLauncher.setPosition(0.222); //-19.7+2.9 //0.222
    }

    public void setRedAngle(){
        rotateLauncher.setPosition(0.164);
    }

    public void setAngleStraight(){
        rotateLauncher.setPosition(0.195); //0.031/18
    }
}
