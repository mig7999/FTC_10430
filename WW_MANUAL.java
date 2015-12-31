
package org.swerverobotics.library.examples;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.swerverobotics.library.internal.Util.handleCapturedInterrupt;

/**
 * An example of a synchronous opmode that implements a simple drive-a-bot. 
 */
@TeleOp(name="WW_MANUAL_mode", group="Swerve Examples")
//@Disabled

public class WW_MANUAL extends SynchronousOpMode
    {
    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.


    DcMotor frontright  = null;
    DcMotor backright = null;
    DcMotor frontleft = null;
    DcMotor backleft = null;
    OpticalDistanceSensor v_sensor_ods = null;

        Servo s1 = null;
        Servo s2 = null;
        Servo s3 = null;
        Servo s4 = null;
        Servo s5 = null;

    EulerAngles angles;
    Position position;
    Velocity velocity;

    IBNO055IMU              imu;
    ElapsedTime             elapsed    = new ElapsedTime();
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

        double NormalizedPitch = 0.0;
        double LastNormalizedPitch = -179.0;
        double MaximumPitchChange_LIMIT = 45.0;
        double FirstGear = 0.2f;
        double SecondGear = 0.6f;
        double ThirdGear = 1.0f;
        double CurrentGear = 0.0f;
        double RearWheelsFlippingAvoidanceVal = 1.0f;
        double                  ms;
        double msCopy = 0;
        double Rest_Positions[] = {0.7,0.25,0.6,0.85,0.5};

        double x_custom_Positions[] = {0.6,0.9,0.9,0.5,0.5};
        int ServoMotionState = 0;
        boolean Rest_Positions_Enabled = true;
        boolean x_custom_Enabled = false;
        double position_s1 = 0.5;
        double position_s2 = 0.5;
        double position_s3 = 0.5;
        double position_s4 = 0.5;
        double position_s5 = 0.5;
        boolean KeepRunning = true;




    @Override protected void main() throws InterruptedException
        {
        // Initialize our hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names you assigned during the robot configuration
        // step you did in the FTC Robot Controller app on the phone.
           // devMode = DcMotorController.DeviceMode.WRITE_ONLY;

/*
        frontright = hardwareMap.dcMotor.get("frontright");
        //    frontright =    (DcMotor)HardwareUtil.getHardwareOn("frontright",  hardwareMap.dcMotor);
        backright = hardwareMap.dcMotor.get("backright");

        frontleft = hardwareMap.dcMotor.get("frontleft");
        backleft = hardwareMap.dcMotor.get("backleft");
*/
            frontright =    (DcMotor)HardwareUtil.getHardwareOn("frontright",  hardwareMap.dcMotor);
            backright =    (DcMotor)HardwareUtil.getHardwareOn("backright",  hardwareMap.dcMotor);
            frontleft =    (DcMotor)HardwareUtil.getHardwareOn("frontleft",  hardwareMap.dcMotor);
            backleft =    (DcMotor)HardwareUtil.getHardwareOn("backleft",  hardwareMap.dcMotor);

/*            s1 = hardwareMap.servo.get("servo1");
            s2 = hardwareMap.servo.get("servo2");
            s3 = hardwareMap.servo.get("servo3");
            s4 = hardwareMap.servo.get("servo4");
            s5 = hardwareMap.servo.get("servo5");*/
            s1 = (Servo)HardwareUtil.getHardwareOn("servo1",  hardwareMap.servo);
            s2 = (Servo)HardwareUtil.getHardwareOn("servo2",  hardwareMap.servo);
            s3 = (Servo)HardwareUtil.getHardwareOn("servo3",  hardwareMap.servo);
            s4 = (Servo)HardwareUtil.getHardwareOn("servo4",  hardwareMap.servo);
            s5 = (Servo)HardwareUtil.getHardwareOn("servo5",  hardwareMap.servo);

            double s1_MIN_MAX_LIMITS[] = {0.0,1.0};
            double s2_MIN_MAX_LIMITS[] = {0.1,0.99};
            double s3_MIN_MAX_LIMITS[] = {0.1,0.99};
            double s4_MIN_MAX_LIMITS[] = {0.2,0.99};
            double s5_MIN_MAX_LIMITS[] = {0.1,0.99};

        Reset_Encoders();

        // One of the two motors (here, the left) should be set to reversed direction
        // so that it can take the same power level values as the other motor.
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);





        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = ",BNO055";

            try
            {
                imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);
                //    imu = (IBNO055IMU)HardwareUtil.getHardwareOn("imu",  hardwareMap.i2cDevice);
                //v_sensor_ods = hardwareMap.opticalDistanceSensor.get ("sensor_ods");
                this.configureDashboard();
            }
            catch (Exception e)
            {
                KeepRunning = false;
                //System.exit(0);
            }

        v_sensor_ods = (OpticalDistanceSensor)HardwareUtil.getHardwareOn("sensor_ods",  hardwareMap.opticalDistanceSensor);
        // Wait until we've been given the ok to go
        waitForStart();

            // Configure the dashboard however we want it


            // Enter a loop processing all the input we receive
        while ((opModeIsActive())&& (KeepRunning))
            {
            // Emit telemetry with the freshest possible values
            telemetry.update();

            NormalizedPitch = (double) normalizeDegrees(angles.pitch);
            NormalizedPitch = (NormalizedPitch*0.95)+(LastNormalizedPitch*0.05);

            LastNormalizedPitch = NormalizedPitch;

            if (this.updateGamepads())
            {
                // There is (likely) new gamepad input available.
                // Do something with that! Here, we just drive.

                if (this.gamepad1.dpad_down) {// for testing purposes
                    if (this.gamepad1.a) frontright.setPower(0.15);
                    else frontright.setPower(0.0);

                    if (this.gamepad1.b) frontleft.setPower(0.15);
                    else frontleft.setPower(0.0);

                    if (this.gamepad1.x) backleft.setPower(0.15);
                    else backleft.setPower(0.0);

                    if (this.gamepad1.y) backright.setPower(0.15);
                    else backright.setPower(0.0);

                    if (this.gamepad1.left_bumper) Reset_Encoders();
                }
                else
                {
                    doManualDrivingControl(gamepad1);
                    if (this.gamepad1.right_bumper) {
                        CurrentGear = ThirdGear;
                    } else if (this.gamepad1.left_bumper) {
                        CurrentGear = SecondGear;
                    } else {
                        CurrentGear = FirstGear;
                    }


                }

            }
/*                else
            {
                frontright.setPower(0);
                frontleft.setPower(0);

                backright.setPower(0);
                backleft.setPower(0);
            }*/

            // left_stick_y
            position_s1 = Update_Stick_Position_w_limit(gamepad2.left_stick_x,position_s1 , s1_MIN_MAX_LIMITS[0], s1_MIN_MAX_LIMITS[1]);
            position_s2 = Update_Stick_Position_w_limit(gamepad2.left_stick_y,position_s2, s2_MIN_MAX_LIMITS[0], s2_MIN_MAX_LIMITS[1]);
            position_s3 = Update_Stick_Position_w_limit(-gamepad2.right_stick_y,position_s3, s3_MIN_MAX_LIMITS[0], s3_MIN_MAX_LIMITS[1]);
            position_s4 = Update_Stick_Position_w_limit(gamepad2.right_stick_x,position_s4, s4_MIN_MAX_LIMITS[0], s4_MIN_MAX_LIMITS[1]);
            if (gamepad2.left_bumper) position_s5 = Update_Stick_Position_w_limit(1,position_s5 , s5_MIN_MAX_LIMITS[0], s5_MIN_MAX_LIMITS[1]);
            if (gamepad2.right_bumper) position_s5 = Update_Stick_Position_w_limit(-1,position_s5 , s5_MIN_MAX_LIMITS[0], s5_MIN_MAX_LIMITS[1]);

            if (gamepad2.x)
            {
                x_custom_Enabled = true;
            }

            if (gamepad2.b)
            {
                Rest_Positions_Enabled = true;
            }


            if (x_custom_Enabled) Set_Custom_x_Servo_Positions();
            if (Rest_Positions_Enabled) Set_Rest_Servo_Positions();
                SetServoPositions();


            //waitOneFullHardwareCycle();

            // Let the rest of the system run until there's a stimulus from the robot controller runtime.
            if (KeepRunning) this.idle();
            }
        }



    /**
     * Implement a simple two-motor driving logic using the left and right
     * right joysticks on the indicated game pad.
     */
    void doManualDrivingControl(Gamepad pad) throws InterruptedException
        {
        // Remember that the gamepad sticks range from -1 to +1, and that the motor
        // power levels range over the same amount
            double ctlPowerLeft    =  pad.left_stick_y;
            double ctlPowerRight =  pad.right_stick_y;

       ctlPowerLeft = ctlPowerLeft*CurrentGear;
            ctlPowerRight = ctlPowerRight*CurrentGear;

        if ((NormalizedPitch >= -140.0) && (NormalizedPitch < -1.0) )// we are in the correct quadrant. tipping occurs when pitch reads > -135
        {

            RearWheelsFlippingAvoidanceVal = 0.05;
        }
        else
            {
                RearWheelsFlippingAvoidanceVal = 0.99;
            }

        ctlPowerRight = Range.clip(ctlPowerRight, -1f, 1f);
        ctlPowerLeft  = Range.clip(ctlPowerLeft, -1f, 1f);

            double powerRightRear = Range.clip(ctlPowerRight * RearWheelsFlippingAvoidanceVal, -1f, 1f);
            double powerLeftRear  = Range.clip(ctlPowerLeft * RearWheelsFlippingAvoidanceVal, -1f, 1f);

        // Tell the motors
        frontright.setPower(ctlPowerRight);
        frontleft.setPower(ctlPowerLeft);

        backright.setPower(powerRightRear);
        backleft.setPower(powerLeftRear);
        }


    void configureDashboard()
        {
        // Configure the dashboard. Here, it will have one line, which will contain three items
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation();
                ms = elapsed.time() * 1000.0;

            }
        });
/*        this.telemetry.addLine
                (
                        this.telemetry.item("frontright:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(frontright.getPower());
                            }
                        }),
                        this.telemetry.item("backright: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(backright.getPower());
                            }
                        }),
                        this.telemetry.item("frontleft: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(frontleft.getPower());
                            }
                        }),
                        this.telemetry.item("backleft: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(backleft.getPower());
                            }
                        })
                );*/
/*            telemetry.addLine(
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

                    telemetry.item("ods: ", new IFunc<Object>() {
                        public Object value() {
                            return formatDecimal4digits(v_sensor_ods.getLightDetected());
                        }
                    }));*/
            telemetry.addLine(
                    telemetry.item("encoderleftIn: ", new IFunc<Object>() {
                        public Object value() {
                            return formatDecimal(convert_encoder_dat_to_inches(backleft.getCurrentPosition()));
                        }
                    }),
                    telemetry.item("encoderrightIn: ", new IFunc<Object>() {
                        public Object value() {
                            return formatDecimal(convert_encoder_dat_to_inches(backright.getCurrentPosition()));
                        }
                    }),

                    telemetry.item("odsIn: ", new IFunc<Object>() {
                        public Object value() {
                            return formatDecimal4digits(convert_ods_volt_to_inches(v_sensor_ods.getLightDetected()));
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
                    telemetry.item("NormalizedPitch ", new IFunc<Object>()
                    {
                        public Object value()
                        {
                            return formatDecimal(NormalizedPitch);
                        }
                    }),
                    telemetry.item("RearWheelsFlippingAvoidanceVal: ", new IFunc<Object>()
                    {
                        public Object value()
                        {
                            return formatDecimal(RearWheelsFlippingAvoidanceVal);
                        }
                    }));
/*            this.telemetry.addLine
                    (
                            this.telemetry.item("s1:", new IFunc<Object>() {
                                @Override
                                public Object value() {
                                    return formatDecimal(s1.getPosition());
                                }
                            }),
                            this.telemetry.item("s2: ", new IFunc<Object>() {
                                public Object value() {
                                    return formatDecimal(s2.getPosition());
                                }
                            }),
                            this.telemetry.item("s3: ", new IFunc<Object>() {
                                public Object value() {
                                    return formatDecimal(s3.getPosition());
                                }
                            }),
                            this.telemetry.item("s4: ", new IFunc<Object>() {
                                public Object value() {
                                    return formatDecimal(s4.getPosition());
                                }
                            })
                    );*/
            this.telemetry.addLine
                    (
                            this.telemetry.item("joystick:", new IFunc<Object>() {
                                @Override
                                public Object value() {
                                    return formatDecimal(gamepad2.left_stick_x);
                                }
                            })

                    );
        }

    // Handy functions for formatting data for the dashboard
    String format(double d)
        {
        return String.format("%.1f", d);
        }

        String formatDecimal(double decimalVal)
        {

            return String.format("%.2f%s", decimalVal, " ");
        }
        String formatDecimal4digits(double decimalVal)
        {

            return String.format("%.4f%s", decimalVal, " ");
        }

        double convert_ods_volt_to_inches (double volts)
        {
            //0.0237*G6^(-1.164)
            double ExpProduct;
            ExpProduct = Math.pow(volts, -1.164);
            return  (0.0237 *ExpProduct);

        }

        double convert_encoder_dat_to_inches (double encoderVal)
        {
            return (encoderVal)/80.0;

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
        void Reset_Encoders()
        {
            this.backright.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            this.backleft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

            this.frontright.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            this.backright.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

            this.frontleft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            this.backleft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }

        void SetServoPositions()
        {
            try
            {
                s1.setPosition(position_s1);
                s2.setPosition(position_s2);
                s3.setPosition(position_s3);
                s4.setPosition(position_s4);
                s5.setPosition(position_s5);
            }
            catch (Exception e)
            {
                KeepRunning = false;
                //System.exit(0);
            }


        }

        double Update_Stick_Position_w_limit(double stick_value, double position_value, double min_Limit, double max_limit )
        {
            double Threshold = 0.01;
            double increment = 0.05;

            if (stick_value > Threshold)
            {
                position_value = position_value+increment;
            }

            if (stick_value < (-Threshold))
            {
                position_value = position_value-increment;
            }
            position_value = Range.clip(position_value, min_Limit, max_limit);
            return Range.clip(position_value, 0, 1);
        }

        boolean Set_Custom_x_Servo_Positions()
        {

            boolean ProcessCompleted = false;
            if ((ms -msCopy)> 500)// wait one second before switching.
            {
                msCopy = ms;
                switch(ServoMotionState)
                {
                    case 0:
                        position_s1 = x_custom_Positions[0];
                        ServoMotionState++;
                        break;
                    case 1:
                        position_s2 = x_custom_Positions[1];
                        ServoMotionState++;
                        break;
                    case 2:
                        position_s3 = x_custom_Positions[2];
                        ServoMotionState++;
                        break;
                    case 3:
                        position_s4 = x_custom_Positions[3];
                        ServoMotionState++;
                        break;
                    case 4:
                        position_s5 = x_custom_Positions[4];
                        x_custom_Enabled = false;
                        ServoMotionState = 0;
                        break;
                }
            }



            return ProcessCompleted;
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



}


