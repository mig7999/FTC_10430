
package org.swerverobotics.library.examples;


        import org.swerverobotics.library.*;
        import org.swerverobotics.library.interfaces.*;

        import com.qualcomm.ftccommon.DbgLog;
        import com.qualcomm.robotcore.hardware.*;
        import com.qualcomm.robotcore.util.*;

        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import static org.swerverobotics.library.internal.Util.handleCapturedInterrupt;

/**
 * SynchIMUDemo gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 * http://www.adafruit.com/products/2472
 */
@TeleOp(name="WW_BLUE_Alliance", group="Swerve Examples")

public class WW_Blue_Alliance_Aut extends SynchronousOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // Our sensors, motors, and other devices go here, along with other long term state
    IBNO055IMU              imu;
    ElapsedTime             elapsed    = new ElapsedTime();
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    EulerAngles angles;
    Position position;
    Velocity velocity;

    DcMotor frontleft = null;
    DcMotor backleft = null;
    DcMotor frontright  = null;
    DcMotor backright = null;


    OpticalDistanceSensor v_sensor_ods = null;
    OpticalDistanceSensor v_sensor_ods_SIDE = null;


    Servo s1 = null;
    Servo s2 = null;
    Servo s3 = null;
    Servo s4 = null;
    Servo s5 = null;


    double Retrieve_Arm_custom_Positions[] = {0,0,0,0,0,0};
    double Drop_off_Climbers_Positions[] = {1.0,0.2,0.65,0.85,0.5};
    boolean Rest_Positions_Enabled = true;
    boolean Drop_off_Climbers_Enabled = false;
    double Rest_Positions[] = {1.0,0.2,1.0,0.85,0.5};


    int                     loopCycles;
    int                     i2cCycles;
    int                     value;
    double                  ms;
    double msCopy = 0;
    int ServoMotionState = 0;

    double TempMaxdrivePower = 0.07;
    double NormalMaxdrivePower = 0.13;
    double maxdrivePower = 0.13;
    double TargetHeading = 0.0;
    double TurningSpeed = 0.115;

    Boolean MaxDrivePowerAchieved = false;
    double HeadingShiftValue = 1000.0;
    double TimeElapsed= 0;
    double LastStraightRangeSensorVal = 10.0;
    double LastSideRangeSensorVal = 10.0;
    double TurningPowercopy = 0.0;


    double Targetdistance = 0;
    double NewLeftMotorPower = 0.0;
    double NewRightMotorPower = 0.0;
    double HeadingError = 0.0;
    double NormalizedHeading = 0.0;
    double X_Position_Inches = 0.0;
    double InitialPitchValue = 0.0;
    Boolean RobotShouldBeInMotion = true;



    double position_s1 = 0.7;
    double position_s2 = .5;
    double position_s3 = .5;
    double position_s4 = 0;
    double position_s5 = 0;

    boolean OneStepAtAttime = true;
    public enum States {
        APROACH_CENTERLINE,
        TURN_RIGHT_TWRDS_BEACONS, // turn 45 degrees right
        MOVE_FWD_TWD_BEACONS,// move slowly use ods
        TURN_RIGHT_2ND_TIME_TWRDS_BEACONS,// 45 more degrees.
        MOVE_FWD_APPROACH_BEACONS,// move slowly use ods
        TURN_LEFT_FOR_BEACON_SWEEP,
        MOVE_SWEEP_BEACONS,
        DRIVE_BACK_TO_MIDPT,
        TURN_RIGHT_FACE_BEACON_MIDPT,
        DROP_OFF_CLIMBERS,
        RETRIEVE_ARM,
        TURN_AROUND,
        MOVE_AWAY_FROM_BEACONS,// move slowly use ods
        TURN_LEFT_TWRDS_RAMP,// turn 45 degrees left
        MOVE_BACK_TWRDS_RAMP,
        TURN_LEFT_FACE_RAMP,// turn 90 degrees left
        TURN_LEFT_BEFORE_RAMP_SCAN,
        SCAN_RAMP,
        ROTATE_LEFT_TO_MID_POINT,
        GET_ON_THE_RAMP,
        CHILL,
    }

    States CurrentState;
    int SweepState= 0;
    double Side_ods_Distance = 100.0;
    double Front_ods_Distance = 100.0;
    double NewTurningSpeed = 0.0;
    //----------------------------------------------------------------------------------------------
    // main() loop
    //----------------------------------------------------------------------------------------------

    @Override public void main() throws InterruptedException {
        int SubState = 0;
        double LeftEdge = 0;
        double RightEdge = 0;
        frontright = hardwareMap.dcMotor.get("frontright");
        backright = hardwareMap.dcMotor.get("backright");
        frontleft = hardwareMap.dcMotor.get("frontleft");
        backleft = hardwareMap.dcMotor.get("backleft");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

        s1 = this.hardwareMap.servo.get("servo1");
        s2 = this.hardwareMap.servo.get("servo2");
        s3 = this.hardwareMap.servo.get("servo3");
        s4 = this.hardwareMap.servo.get("servo4");
        s5 = this.hardwareMap.servo.get("servo5");


        Reset_Encoders();


        parameters.angleunit = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = ",BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);


        v_sensor_ods = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        v_sensor_ods_SIDE = hardwareMap.opticalDistanceSensor.get("sensor_ods_side");


        Targetdistance = 15.0;

        // Wait until we're told to go
        CurrentState = States.APROACH_CENTERLINE;
        delay(50);
        imu.startAccelerationIntegration(new Position(), new Velocity());
        delay(50);
        angles = imu.getAngularOrientation();
        InitialPitchValue = normalizeDegrees(angles.heading);

        // Set up our dashboard computations
        composeDashboard();

        idle();

        boolean IsIMUDisc = IsIMUDisconnected();

        if (IsIMUDisc == false) {
            waitForStart();
            delay(50);
            IsIMUDisc = IsIMUDisconnected();
            // Enable reporting of position using the naive integrator

            RobotShouldBeInMotion = true;

            //THIS IS OUR MAIN LOOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            while (opModeIsActive() && (IsIMUDisc == false)) {
                idle();
                angles = imu.getAngularOrientation();
                NormalizedHeading = NormalizePitchReading(normalizeDegrees(angles.heading), InitialPitchValue);
                X_Position_Inches = convert_encoder_dat_to_inches((backleft.getCurrentPosition() + backright.getCurrentPosition()) / 2);
                Side_ods_Distance = get_ods_side_value_in_inches();
                Front_ods_Distance = get_ods_value_in_inches();

                telemetry.update();


                if (OneStepAtAttime) // use gamepad1 as a way to step through
                {
                    SetServoPositions();
                    updateGamepads();
                    if (this.gamepad1.x) {
                        OneStepAtAttime = false;
                    }

                } else if (Rest_Positions_Enabled) // only handle the initial Set_Rest_Servo_Positions
                {
                    Set_Rest_Servo_Positions();
                    SetServoPositions();
                } else {

                    SetServoPositions();
                    switch (CurrentState) {
                        case APROACH_CENTERLINE: //A
                            if (RobotShouldBeInMotion) {
                                goStraightKeepStraight(Targetdistance);
                                if (X_Position_Inches >= Targetdistance) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_RIGHT_TWRDS_BEACONS;
                                    RobotShouldBeInMotion = true;
                                    TargetHeading = 45.0;//
                                    OneStepAtAttime = true;
                                }

                            }

                            break;
                        case TURN_RIGHT_TWRDS_BEACONS: //B
                            if (RobotShouldBeInMotion) {
                                if (NormalizedHeading >= TargetHeading) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnRight();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.MOVE_FWD_TWD_BEACONS;
                                    RobotShouldBeInMotion = true;
                                    Targetdistance = 50.0;//66 made it shorted to come in ahead of sweep
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }

                            break;

                        case MOVE_FWD_TWD_BEACONS: //C
                            if (RobotShouldBeInMotion) {
                                goStraightKeepStraight(Targetdistance);
                                if (X_Position_Inches >= Targetdistance) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_RIGHT_2ND_TIME_TWRDS_BEACONS;
                                    RobotShouldBeInMotion = true;
                                    TargetHeading = 90.0;//
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;

                        case TURN_RIGHT_2ND_TIME_TWRDS_BEACONS: //D
                            if (RobotShouldBeInMotion) {
                                if (NormalizedHeading >= TargetHeading) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnRight();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.MOVE_FWD_APPROACH_BEACONS;
                                    RobotShouldBeInMotion = true;
                                    Targetdistance = 35.0;// 29 inches would be close to the edge, but we want to almost hit the wall
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                    //TempMaxdrivePower = 0.075;
                                    //NormalMaxdrivePower = 0.1;
                                    maxdrivePower = TempMaxdrivePower;

                                }

                            }
                            break;

                        case MOVE_FWD_APPROACH_BEACONS: //E
                            if (RobotShouldBeInMotion) {
                                goStraightKeepStraight(Targetdistance);
                                // don't allow it to get too close to the beacons. || (get_ods_value_in_inches() < 2.0)
                                if ((X_Position_Inches >= Targetdistance)|| (Front_ods_Distance <= 5.5))
                                {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }

                                if ((X_Position_Inches >= 3.0)&&(DidRoboStoppedtMoving()))// if front sensor gets blind
                                {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }

                                if (X_Position_Inches >= 3.0)// drop speed after intial approach
                                {
                                    maxdrivePower = TempMaxdrivePower;
                                }

                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_LEFT_FOR_BEACON_SWEEP;
                                    RobotShouldBeInMotion = true;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                    TargetHeading = 180.0;//CANNOT SELECT 180 BECAUSE CAUSES ERRORS
                                }

                            }
                            break;

                        case TURN_LEFT_FOR_BEACON_SWEEP: //F.1
                            if (RobotShouldBeInMotion) {
                                // going from -90 to 180 left turn. We just need to switch signs
                                // then stop right away
                                if ((NormalizedHeading >= 0.0) ||(NormalizedHeading <= -175.0 ))
                                {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnLeft();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.MOVE_SWEEP_BEACONS;
                                    RobotShouldBeInMotion = true;
                                    Targetdistance = 30.0;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;

                        case MOVE_SWEEP_BEACONS:// move in front of beacons F.2
                            if (RobotShouldBeInMotion) {

                                goStraightKeepStraight(Targetdistance);

                                LastSideRangeSensorVal = (Side_ods_Distance*0.95)+(LastSideRangeSensorVal*0.05);
                                //LastSideRangeSensorVal = Side_ods_Dist_inch;

                                if ((RightEdge == 0) && (LastSideRangeSensorVal < 5.0))
                                {
                                    RightEdge = X_Position_Inches;
                                    TurningPowercopy = 6.0;
                                }
                                if ((RightEdge != 0) &&(LeftEdge == 0) && (LastSideRangeSensorVal > 5.0))
                                {
                                    LeftEdge = X_Position_Inches;
                                    //stopRobot();
                                    //RobotShouldBeInMotion = false;
                                    TurningPowercopy = 9.0;

                                }
                                if ((X_Position_Inches >= Targetdistance)|| ( (LeftEdge != 0) && ((X_Position_Inches-LeftEdge) >= 3.0)))
                                {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }

                            }
                            else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_RIGHT_FACE_BEACON_MIDPT;
                                    RobotShouldBeInMotion = true;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                    TargetHeading = -90.0;
                                    //Targetdistance = (LeftEdge-RightEdge)/2.0;//
                                }

                            }
                            break;
                        //DriveBackwards
/*                        case DRIVE_BACK_TO_MIDPT: //G
                            if (RobotShouldBeInMotion) {
                                DriveBackwards(-Targetdistance);// Targetdistance is
                                if (X_Position_Inches <= (-Targetdistance)) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_RIGHT_FACE_BEACON_MIDPT;
                                    RobotShouldBeInMotion = true;
                                    TargetHeading = -90.0;//
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;*/

                        case TURN_RIGHT_FACE_BEACON_MIDPT: //H
                            if (RobotShouldBeInMotion) {
                                if (NormalizedHeading >= TargetHeading) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnRight();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.DROP_OFF_CLIMBERS;
                                    RobotShouldBeInMotion = true;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;

                        case DROP_OFF_CLIMBERS: //I
                            if (Drop_off_Climbers())// wait until all servo positions have been achieved
                            {
                                CurrentState = States.RETRIEVE_ARM;
                                OneStepAtAttime = true;
                            }
                            break;

                        case RETRIEVE_ARM:
                            if (Set_Rest_Servo_Positions())// wait until all servo positions are returned to normal
                            {
                                CurrentState = States.TURN_AROUND;
                                TargetHeading = -90.0;
                                RobotShouldBeInMotion = true;
                                OneStepAtAttime = true;
                                maxdrivePower = NormalMaxdrivePower;

                            }
                            break;
                        case TURN_AROUND:// turn left from 90 to -90 degrees J
                            if (RobotShouldBeInMotion) {
                                if ((NormalizedHeading < 0.0) && (NormalizedHeading <= TargetHeading)) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnLeft();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.MOVE_AWAY_FROM_BEACONS;
                                    RobotShouldBeInMotion = true;
                                    Targetdistance = 29.0;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;
                        case MOVE_AWAY_FROM_BEACONS: //K
                            if (RobotShouldBeInMotion) {
                                goStraightKeepStraight(Targetdistance);
                                if (X_Position_Inches >= Targetdistance) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_LEFT_TWRDS_RAMP;
                                    RobotShouldBeInMotion = true;
                                    TargetHeading = -135.0;//
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;
                        case TURN_LEFT_TWRDS_RAMP: //L
                            if (RobotShouldBeInMotion) {
                                if (NormalizedHeading <= TargetHeading) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnLeft();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.MOVE_BACK_TWRDS_RAMP;
                                    Targetdistance = 12.0;
                                    RobotShouldBeInMotion = true;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;
                        case MOVE_BACK_TWRDS_RAMP: //M
                            if (RobotShouldBeInMotion) {
                                goStraightKeepStraight(Targetdistance);
                                if (X_Position_Inches >= Targetdistance) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.TURN_LEFT_FACE_RAMP;
                                    RobotShouldBeInMotion = true;
                                    TargetHeading = 135.0;//
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;
                        case TURN_LEFT_FACE_RAMP: //N
                            if (RobotShouldBeInMotion) {
                                if ((NormalizedHeading > 0) && (NormalizedHeading >= TargetHeading))// make sure we are in correct quadrants
                                {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                } else {
                                    turnLeft();
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.GET_ON_THE_RAMP;
                                    RobotShouldBeInMotion = true;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                    Targetdistance = 30.0;
                                }

                            }
                            break;

                        case GET_ON_THE_RAMP: //O
                            if (RobotShouldBeInMotion) {
                                goStraightKeepStraight(Targetdistance);
                                if (X_Position_Inches >= Targetdistance) {
                                    stopRobot();
                                    RobotShouldBeInMotion = false;
                                }
                            } else {
                                if (DidRoboStoppedtMoving()) {
                                    CurrentState = States.CHILL;
                                    RobotShouldBeInMotion = true;
                                    Reset_Encoders();
                                    OneStepAtAttime = true;
                                }

                            }
                            break;
                        case CHILL:
                            break;
                        default:
                            break;

                    }
                }

            }// while

            //Thread.yield();

        }
    }


    //----------------------------------------------------------------------------------------------
    // dashboard configuration
    //----------------------------------------------------------------------------------------------
    void goStraightKeepStraight(double TargetDistance)
    {
        double Distancedifference;
        double AdjusteddrivePower;

        Distancedifference = Targetdistance- X_Position_Inches;

        if (Distancedifference > 0)
        {
            if (MaxDrivePowerAchieved == false)// startup power
            {

                AdjusteddrivePower = 0.065+(Math.pow(X_Position_Inches+2,3))/5000.0;
            }
            else// slow down power as we approach target position
            {
                AdjusteddrivePower = 0.055+(Math.pow(Distancedifference+1,2))/5000.0;
            }

            if (AdjusteddrivePower > maxdrivePower)
            {
                MaxDrivePowerAchieved = true;
                AdjusteddrivePower = maxdrivePower;
            }


            if ((TimeElapsed != 0) && (velocity.velocX <= 0.03))
            {
                if ((ms-TimeElapsed)> 1500)
                {
                    AdjusteddrivePower = 0.3;// spin the wheels for a bit
                }
            }

            //TimeElapsed was reset. Count time from no movement
            if ((TimeElapsed == 0) && (velocity.velocX == 0.0))
            {
                TimeElapsed = ms;// copy global counter
            }

            if (velocity.velocX != 0.0) TimeElapsed = ms;// reset if movement occurs



            //HeadingShiftValue shifts the number calculations away from zero to avoid division by zero.
            if (TargetHeading == 180.0)
            {
                if (NormalizedHeading >= 0.0)// if heading is around 180
                {
                    NormalizedHeading = 180.0-(NormalizedHeading-180.0);
                    HeadingError = (((NormalizedHeading+HeadingShiftValue)-(TargetHeading+HeadingShiftValue))/(TargetHeading+HeadingShiftValue))*(5.0);
                }
                else// NormalizedHeading is less than zero
                {
                    HeadingError = (((NormalizedHeading+HeadingShiftValue)-(TargetHeading+HeadingShiftValue))/(TargetHeading+HeadingShiftValue))*(5.0);
                }


            }
            else
            {
                HeadingError = (((NormalizedHeading+HeadingShiftValue)-(TargetHeading+HeadingShiftValue))/(TargetHeading+HeadingShiftValue))*5.0;
            }

            if ( HeadingError > 0.0)// we are skewed to the right. apply more power to that side.
            {
                if ( HeadingError > 1.0) HeadingError = 1.0;
                NewRightMotorPower = AdjusteddrivePower*(1.0+HeadingError);
                NewLeftMotorPower = AdjusteddrivePower*(1.0-HeadingError);
            }

            else if ( HeadingError <= 0.0)// we are skewed to the left. apply more power to that side.
            {
                if ( HeadingError < -1.0) HeadingError = -1.0;
                NewRightMotorPower = AdjusteddrivePower*(1.0+HeadingError);
                NewLeftMotorPower = AdjusteddrivePower*(1.0-HeadingError);
            }

            frontright.setPower(NewRightMotorPower);
            backright.setPower(NewRightMotorPower);

            frontleft.setPower(NewLeftMotorPower);
            backleft.setPower(NewLeftMotorPower);

        }

    }
    void DriveBackwards(double TargetDistance)// Target and X_position_inches are both negative
    {
        double Distancedifference;
        double AdjusteddrivePower;

        Distancedifference = Math.abs(-Targetdistance- X_Position_Inches);

        if (Distancedifference > 0)
        {
            if (MaxDrivePowerAchieved == false)// startup power
            {

                AdjusteddrivePower = 0.045+(Math.pow(-X_Position_Inches+1,3))/5000.0;
            }
            else// slow down power as we approach target position
            {
                AdjusteddrivePower = 0.045+(Math.pow(Distancedifference+1,3))/5000.0;
            }

            if (AdjusteddrivePower > maxdrivePower)
            {
                MaxDrivePowerAchieved = true;
                AdjusteddrivePower = maxdrivePower;
            }

            //HeadingShiftValue shifts the number calculations away from zero to avoid division by zero.
            HeadingError = (((NormalizedHeading+HeadingShiftValue)-(TargetHeading+HeadingShiftValue))/(TargetHeading+HeadingShiftValue))*(-5.0);
            if ( HeadingError > 0.0)// we are skewed to the right. apply more power to that side.
            {
                if ( HeadingError > 1.0) HeadingError = 1.0;
                NewRightMotorPower = AdjusteddrivePower*(1.0-HeadingError);
                NewLeftMotorPower = AdjusteddrivePower*(1.0+HeadingError);
            }
            else if ( HeadingError <= 0.0)// we are skewed to the left. apply more power to that side.
            {
                if ( HeadingError < -1.0) HeadingError = -1.0;
                NewRightMotorPower = AdjusteddrivePower*(1.0-HeadingError);
                NewLeftMotorPower = AdjusteddrivePower*(1.0+HeadingError);
            }
            frontright.setPower(-NewRightMotorPower);
            backright.setPower(-NewRightMotorPower);

            frontleft.setPower(-NewLeftMotorPower);
            backleft.setPower(-NewLeftMotorPower);

        }

    }
    void turnLeft()// inverted power application to compensate for gearing.
    {
        NewTurningSpeed = SlowTurning();

        frontright.setPower(NewTurningSpeed);
        backright.setPower(NewTurningSpeed);
        frontleft.setPower(-NewTurningSpeed);
        backleft.setPower(-NewTurningSpeed);

    }
    void turnRight()
    {
        NewTurningSpeed = SlowTurning();
        frontright.setPower(-NewTurningSpeed);
        backright.setPower(-NewTurningSpeed);
        frontleft.setPower(NewTurningSpeed);
        backleft.setPower(NewTurningSpeed);
    }
    void stopRobot()
    {
        frontright.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        backleft.setPower(0);
    }
    double SlowTurning()
    {
        double AdjustedTurningSpeed;
        double AngularDifference;
        AngularDifference = Math.abs((NormalizedHeading+200)-(TargetHeading+200));
        if (MaxDrivePowerAchieved == false)// startup power
        {
            AdjustedTurningSpeed = TurningSpeed;
        }
        else {
            if (TargetHeading == 180.0)
            {
                if (NormalizedHeading >= 0.0)//
                {
                    AdjustedTurningSpeed = 0.06+((AngularDifference*1.5)/1000.0);
                }
                else// NormalizedHeading is less than zero i e -179
                {
                    AdjustedTurningSpeed = 0.06+(((180+NormalizedHeading)*1.5)/1000.0);
                }


            }
            AdjustedTurningSpeed = 0.06+((AngularDifference*1.5)/1000.0);
        }

        if (AdjustedTurningSpeed >= TurningSpeed)
        {
            AdjustedTurningSpeed = TurningSpeed;
            MaxDrivePowerAchieved = true;
        }

        TurningPowercopy = AdjustedTurningSpeed;
        return AdjustedTurningSpeed;
    }
    Boolean DidRoboStoppedtMoving()
    {
        if ( (velocity.velocX < 0.02) && (velocity.velocY < 0.02) && (velocity.velocZ < 0.02) )
        {
            MaxDrivePowerAchieved = false;
            TimeElapsed= 0;
            return true;
        }
        else return false;
    }

    Boolean IsIMUDisconnected()
    {
        if ( (angles.heading == 0.0) && (angles.roll == 0.0)  && (angles.pitch == 0.0)  )
        {
            return true;
        }
        else return false;
    }
    void composeDashboard()
    {
        // The default dashboard update rate is a little to slow for us, so we update faster
        telemetry.setUpdateIntervalMs(100);

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.

                position = imu.getPosition();
                velocity = imu.getVelocity();

                // The rest of this is pretty cheap to acquire, but we may as well do it
                // all while we're gathering the above.
                //loopCycles = getLoopCount();
                //i2cCycles = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
                ms = elapsed.time() * 1000.0;
            }
        });


        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeStatus(imu.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeCalibration(imu.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.heading);
                    }
                }),

                telemetry.item("roll: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.roll);
                    }
                }),
                telemetry.item("pitch: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.pitch);
                    }
                }));

        telemetry.addLine(
                telemetry.item("tARGETheading: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatDecimal(TargetHeading);
                    }
                }),
                telemetry.item("X_Position_Inches ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatDecimal(X_Position_Inches);
                    }
                })
        );

/*        telemetry.addLine(
                telemetry.item("x: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.x);
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.y);
                    }
                }),
                telemetry.item("z: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.z);
                    }
                }));*/
/*        telemetry.addLine(
                telemetry.item("Sx: ", new IFunc<Object>() {
                    public Object value() {
                        return formatVelocity(velocity.velocX);
                    }
                }),
                telemetry.item("Sy: ", new IFunc<Object>() {
                    public Object value() {
                        return formatVelocity(velocity.velocY);
                    }
                }),
                telemetry.item("Sz: ", new IFunc<Object>() {
                    public Object value() {
                        return formatVelocity(velocity.velocZ);
                    }
                }));*/

        telemetry.addLine(
                telemetry.item("State: ", new IFunc<Object>() {
                    public Object value() {
                        return CurrentState;
                    }
                }),
                telemetry.item("HeadingErr: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(HeadingError);
                    }
                }),
                telemetry.item("TargDist: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(Targetdistance);
                    }
                }));

        telemetry.addLine(
                telemetry.item("encoderleft: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(backleft.getCurrentPosition());
                    }
                }),
                telemetry.item("encoderright: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(backright.getCurrentPosition());
                    }
                }),
                telemetry.item("NormalizedHeading: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(NormalizedHeading);
                    }
                })
        );

        telemetry.addLine(
                telemetry.item("ods_in_Side: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(Side_ods_Distance);
                    }
                }),

                telemetry.item("odsin: ", new IFunc<Object>() {
                            public Object value() {
                                return formatDecimal(get_ods_value_in_inches());
                            }
                        }
                )

        );
        telemetry.addLine(
                telemetry.item("ods: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(v_sensor_ods.getLightDetected());
                    }
                }),

                telemetry.item("ods_Side: ", new IFunc<Object>() {
                            public Object value() {
                                return formatDecimal(v_sensor_ods_SIDE.getLightDetected());
                            }
                        }
                )

        );

        telemetry.addLine(
                telemetry.item("TurningPowercopy: ", new IFunc<Object>() {
                    public Object value() {
                        return formatDecimal(TurningPowercopy);
                    }
                })



        );


        this.telemetry.addLine
                (
                        this.telemetry.item("frontright:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatDecimal(frontright.getPower());
                            }
                        }),
                        this.telemetry.item("backright: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatDecimal(backright.getPower());
                            }
                        }),
                        this.telemetry.item("frontleft: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatDecimal(frontleft.getPower());
                            }
                        }),
                        this.telemetry.item("backleft: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatDecimal(backleft.getPower());
                            }
                        })
                );

    }

    String formatAngle(double angle)
    {
        return parameters.angleunit==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
    }
    String formatRadians(double radians)
    {
        return formatDegrees(degreesFromRadians(radians));
    }
    String formatDegrees(double degrees)
    {
        return String.format("%.1f", normalizeDegrees(degrees));
    }
    String formatRate(double cyclesPerSecond)
    {
        return String.format("%.2f", cyclesPerSecond);
    }
    String formatPosition(double coordinate)
    {
        String unit = parameters.accelunit== IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }
    String formatVelocity(double coordinate)
    {

        return String.format("%.2f%s", coordinate, " mPerSec");
    }

    String formatDecimal(double decimalVal)
    {

        return String.format("%.3f%s", decimalVal, " ");
    }
    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /** Normalize the angle into the range [-180,180) */
    double normalizeDegrees(double degrees)
    {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
    double degreesFromRadians(double radians)
    {
        return radians * 180.0 / Math.PI;
    }

    /** Turn a system status into something that's reasonable to show in telemetry */
    String decodeStatus(int status)
    {
        switch (status)
        {
            case 0: return "idle ";
            case 1: return "syserr ";
            case 2: return "periph ";
            case 3: return "sysinit ";
            case 4: return "selftest ";
            case 5: return "fusion ";
            case 6: return "running ";
            default:return (String.format("S: %d", (status)));
        }

    }

    /** Turn a calibration code into something that is reasonable to show in telemetry */
    String decodeCalibration(int status)
    {
        StringBuilder result = new StringBuilder();

        result.append(String.format("C: %d", (status)));  // MAG calibration status
        result.append(". ");
        result.append(String.format("s%d", (status >> 6) & 0x03));  // SYS calibration status
        result.append(" ");
        result.append(String.format("g%d", (status >> 4) & 0x03));  // GYR calibration status
        result.append(" ");
        result.append(String.format("a%d", (status >> 2) & 0x03));  // ACC calibration status
        result.append(" ");
        result.append(String.format("m%d", (status >> 0) & 0x03));  // MAG calibration status


        return result.toString();
    }

    double a_ods_light_detected ()

    {
        double l_return = 0.0;

        if (v_sensor_ods != null)
        {
            v_sensor_ods.getLightDetected();
        }

        return l_return;

    } // a_ods_light_detected

    double get_ods_value_in_inches ()
    {
        //=0.2117*G6^(-0.589)
        double ExpProduct;
        ExpProduct = Math.pow(v_sensor_ods.getLightDetected(), -0.589);
        return  (0.2117 *ExpProduct);

    }

    double get_ods_side_value_in_inches ()
    {
        //0.0237*G6^(-1.164)
        double ExpProduct;
        double dist;
        ExpProduct = Math.pow(v_sensor_ods_SIDE.getLightDetected(), -0.589);
        dist = (0.2117 *ExpProduct);
        if ((dist <20.0) && (dist >=0.0)) return dist;
        else return 20.0;

    }

    double convert_encoder_dat_to_inches (double encoderVal)
    {
        return (encoderVal)/80.0;

    }

    void Reset_Encoders()  throws InterruptedException
    {
        backright.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backleft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backright.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        backleft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        frontright.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        frontleft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


    }

    void SetServoPositions()
    {
        s1.setPosition(position_s1);
        s2.setPosition(position_s2);
        s3.setPosition(position_s3);
        s4.setPosition(position_s4);
        s5.setPosition(position_s5);
    }

    boolean Set_Rest_Servo_Positions()
    {

        boolean ProcessCompleted = false;
        if ((ms -msCopy)> 500)// wait one second before switching.
        {
            msCopy = ms;
            switch(ServoMotionState)
            {
                case 0:
                    position_s1 = Rest_Positions[0];
                    ServoMotionState++;
                    break;
                case 1:
                    position_s2 = Rest_Positions[1];
                    ServoMotionState++;
                    break;
                case 2:
                    position_s3 = Rest_Positions[2];
                    ServoMotionState++;
                    break;
                case 3:
                    position_s4 = Rest_Positions[3];
                    ServoMotionState++;
                    break;
                case 4:
                    position_s5 = Rest_Positions[4];
                    Rest_Positions_Enabled = false;
                    ServoMotionState = 0;
                    ProcessCompleted = true;
                    break;
            }
        }



        return ProcessCompleted;
    }

    boolean Drop_off_Climbers()
    {

        boolean ProcessCompleted = false;
        if ((ms -msCopy)> 500)// wait one second before switching.
        {
            msCopy = ms;
            switch(ServoMotionState)
            {
                case 0:
                    position_s1 = Drop_off_Climbers_Positions[0];
                    ServoMotionState++;
                    break;
                case 1:
                    position_s2 = Drop_off_Climbers_Positions[1];
                    ServoMotionState++;
                    break;
                case 2:
                    position_s3 = Drop_off_Climbers_Positions[2];
                    ServoMotionState++;
                    break;
                case 3:
                    position_s4 = Drop_off_Climbers_Positions[3];
                    ServoMotionState++;
                    break;
                case 4:
                    position_s5 = Drop_off_Climbers_Positions[4];
                    Rest_Positions_Enabled = false;
                    ServoMotionState = 0;
                    ProcessCompleted = true;
                    break;
            }
        }



        return ProcessCompleted;
    }
    double NormalizePitchReading(double PitchReading, double OffsetValue)
    {
        double AdjustedPitch;
        AdjustedPitch = normalizeDegrees((PitchReading+200)-(OffsetValue+200));

        return AdjustedPitch;
    }

    private void delay(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch (InterruptedException e)
        {
            handleCapturedInterrupt(e);
        }
    }
}
