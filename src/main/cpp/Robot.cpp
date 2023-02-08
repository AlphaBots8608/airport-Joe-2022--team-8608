// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Compressor.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/DoubleSolenoid.h>
#include <cameraserver/CameraServer.h>
#include <frc/CAN.h>
#include <rev/CANSparkMax.h>
class Robot : public frc::TimedRobot
{
private:

    //default speed and turn limit
    double speedLimit = 0.8;
    double turnSpeed = 0.65;
    // is the climbing arm tilted out?
    //DEFAULT false -- because we leave it in the full upright position
    bool IsTiltOut = false;

    //is the ball Shooter currently in the UP positon
    //DEFAULT TRUE -- becuase we start each match with our 
    bool IsActuatorLifted = true;
    // Robot drive system
    rev::CANSparkMax mLeftFront{10,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//Driver Side Drive motors
    rev::CANSparkMax mLeftRear{14,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//Driver Side Drive motors
    rev::CANSparkMax mRightFront{9,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//Passenger Side Drive motors
    rev::CANSparkMax mRightRear{15,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//Passenger Side Drive motors
    
    //frc::PWMSparkMax m_left{0};//driver
    //frc::PWMSparkMax m_right{9};//passenger
    frc::MotorControllerGroup mLeft{mLeftFront, mLeftRear};
    frc::MotorControllerGroup mRight{mRightFront, mRightRear};

    frc::DifferentialDrive m_robotDrive{mLeft, mRight};
    
    rev::CANSparkMax ballPickUp{8,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//ball pickup
    //frc::PWMSparkMax ballPickUp{8};//bpup

    rev::CANSparkMax climberWinch{6,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//Vertical Hook adjust
    //rev::CANSparkMax ArmTilter{14,rev::CANSparkMaxLowLevel::MotorType::kBrushed};//Tilt In/Out Hook Adjust
    
    frc::DoubleSolenoid actuator{frc::PneumaticsModuleType::CTREPCM, 0, 1};
    frc::DoubleSolenoid armTilter{frc::PneumaticsModuleType::CTREPCM, 2, 3};

    frc::PneumaticsControlModule pcm;
    frc::Joystick driveStick{0};
    frc::Joystick controlStick{1};
    frc::Timer m_timer;

    

public:
    Robot()
    {
        frc::CameraServer::StartAutomaticCapture();

        mLeft.SetInverted(true);
        m_robotDrive.SetExpiration(100_ms);
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_timer.Start();
    }

    void AutonomousInit() override
    {
        
        m_timer.Reset();
        m_timer.Start();
    }

    void AutonomousPeriodic() override
    {
        double speedLimit = 0.6;

        if(m_timer.Get() < 2.2_s)
        {
            m_robotDrive.ArcadeDrive(-1 * speedLimit, 0);
        }
        else if(m_timer.Get() < 3_s)
        {
            m_robotDrive.ArcadeDrive(0, 0);
        }
        else if(m_timer.Get() < 6_s)
        {
            m_robotDrive.ArcadeDrive(speedLimit, 0);
        }
        else if(m_timer.Get() < 6.5_s)
        {
            m_robotDrive.ArcadeDrive(0, 0);
        }
        else if(m_timer.Get() < 7_s)
        {
            m_robotDrive.ArcadeDrive(0, 0);
            ballPickUp.Set(1);
        }
        else if(m_timer.Get() < 8.5_s) // back up 1.5 seconds
        {
            ballPickUp.Set(0);
            m_robotDrive.ArcadeDrive(-1 * speedLimit, 0);
        }
        else if(m_timer.Get() < 9_s) // lower ball pickup
        {
            ballPickUp.Set(0);
            m_robotDrive.ArcadeDrive(0, 0);
            actuator.Set(frc::DoubleSolenoid::Value::kForward);
        }
        else // everything off
        {
            ballPickUp.Set(0);
            m_robotDrive.ArcadeDrive(0, 0);
            actuator.Set(frc::DoubleSolenoid::Value::kOff);
        }
    }
    

    void TeleopInit() override
    {
    }

    void TeleopPeriodic() override
    {
        // Drive with arcade style (use right stick)
        //   m_robotDrive.ArcadeDrive(0.5, 0.0);
        

        //Special Modes
        if(driveStick.GetRawButton(9))
        {
            speedLimit = .4;
            turnSpeed = .50;
        }
        if(driveStick.GetRawButton(10))
        {
            speedLimit = .8;
            turnSpeed = .65;
        }
        //End Special Modes

        //Drive Control
        if(driveStick.GetRawButton(4))
            m_robotDrive.ArcadeDrive(0, -1);
        else
            m_robotDrive.ArcadeDrive(-1 * (driveStick.GetRawAxis(1)) * speedLimit, driveStick.GetRawAxis(2) * turnSpeed);
        //END Drive Control

        //Ball Pickup
        double BallPickupSpeed = 0.85;
        double BallKickOutSpeed = 1;
        int SpinInButton = 4; // release
        int SpinOutButton = 2; // pick up
        if(controlStick.GetRawButton(SpinInButton))
            ballPickUp.Set(BallKickOutSpeed);
        else if(controlStick.GetRawButton(SpinOutButton))
            ballPickUp.Set(-BallPickupSpeed);
        else    
            ballPickUp.Set(0);
        //End Ball Pickup

        //Ball Collect Lift Actuator
        double LiftAxis = controlStick.GetRawAxis(1);
        double LiftActuatorDeadzone = 0.8;
        //lift ball actuator
        if(LiftAxis < -LiftActuatorDeadzone)
        {
            if (!IsTiltOut)
            {
                actuator.Set(frc::DoubleSolenoid::Value::kReverse);
                IsActuatorLifted = true;
            }
        }
        //Lower ball actuator
        else if(LiftAxis > LiftActuatorDeadzone)
        {
            actuator.Set(frc::DoubleSolenoid::Value::kForward);
            IsActuatorLifted = false;
        }
        else
            actuator.Set(frc::DoubleSolenoid::Value::kOff);
        //END Ball Lift Actuator

        //Climber Axis Control
        double climberAxis = controlStick.GetRawAxis(3);
        double LiftSpeed = -1;
        double deadZoneClimb = 0.1;
        if (climberAxis <-deadZoneClimb || climberAxis > deadZoneClimb)
        {
            climberWinch.Set(climberAxis * LiftSpeed);
            pcm.DisableCompressor();
        }
        else 
        {
            climberWinch.Set(0);
            pcm.EnableCompressorDigital();
        }
        //END Climber Axis Control

        //Hook Tilt Control
        int HooksForward = 7;
        int HooksBack = 5;
        double tiltSpeed = 1;

        if(controlStick.GetRawButton(HooksForward))
        {
            if (!IsActuatorLifted)
            {
                armTilter.Set(frc::DoubleSolenoid::Value::kForward);
                IsTiltOut = true;
            } 
        }        
        else if(controlStick.GetRawButton(HooksBack))
        {
            armTilter.Set(frc::DoubleSolenoid::Value::kReverse);
            IsTiltOut = false;
        }
        else    
            armTilter.Set(frc::DoubleSolenoid::Value::kOff);
        //END Hook Tilt Control

        //reset faults for some reason?
        pcm.ClearAllStickyFaults();

    }

    void TestInit() override {}

    void TestPeriodic() override {}
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
