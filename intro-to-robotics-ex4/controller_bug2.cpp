//Author: Raz Yacov Chai ID:327821484
#include "controller_bug2.hpp"

namespace argos
{

    /****************************************/
    /****************************************/

    void ControllerBug2::Init(TConfigurationNode &t_tree)
    {
        /* Get the actuators and sensors */
        m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
        m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
        m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
        m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
        m_pcCamera->Enable();
        m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
        m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
        m_pcPositioning->Enable();

        // read target position from .argos file into m_cTargetPosition
        TConfigurationNode &tTargetNode = GetNode(t_tree, "target_position");
        Real targetX, targetY;
        GetNodeAttribute(tTargetNode, "x", targetX);
        GetNodeAttribute(tTargetNode, "y", targetY);
        m_cTargetPosition.Set(targetX, targetY, 0.0);

        /* your Init code here */
        // reports the the type of algoritem used
        LOG << "This is BUG2" << std::endl;
        
        // Initialalizing state
        m_state = EState::GO2GOUL;

        // Initiates the LEDs with the color BLUE
        m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);

        // Initialaize the arial path and angle to the goul
        airPath = m_cTargetPosition - m_pcPositioning->GetReading().Position;
        airAngle = airPath.GetZAngle().UnsignedNormalize();
    }

    void ControllerBug2::ControlStep()
    {
        auto &readings = m_pcCamera->GetReadings(); // check's the camera

        // if the robot reached his goul, finish the search and report saccess
        if (!m_pcCamera->GetReadings().BlobList.empty())
        {
            m_pcWheels->SetLinearVelocity(0, 0);         // stops the robot
            m_pcColoredLEDs->SetRingLEDs(CColor::GREEN); // turns the leds grean
            LOG << "target reached!" << std::endl;       // reports to the log that the robot had reached his goul
            return;
        }
        
        switch (m_state)
        {
        case EState::GO2GOUL:
        { // the robot goes in the direction of the goul
            // if the robot detects an obsical infront of him he switch state to Circumnavigate
            if (IsObstacleAhead())
            {
                // switching to Circumnavigate state
                m_state = EState::Circumnavigate;
                //initialize the turned varuble 
                turned = false;
                // turns the LEDs yellow for passing an obstical
                m_pcColoredLEDs->SetRingLEDs(CColor::YELLOW);
                // reports the new state
                LOG << "state: Circumnavigate" << std::endl;
            }
            else
            { // the robots go twords the goul
                // Get current orientation (yaw angle)
                CRadians cCurrentHeading, cPitch, cRoll;
                m_pcPositioning->GetReading().Orientation.ToEulerAngles(cCurrentHeading, cPitch, cRoll);

                // Calculate angle difference (relative to current heading)
                CRadians cAngleDifference = (airAngle - cCurrentHeading).UnsignedNormalize();
                
                // if the robot is not heading twords the goul, turn twords the goul else go strate twords the goul
                if (cAngleDifference > THRESHOLD_RAD && cAngleDifference < (CRadians ::TWO_PI - THRESHOLD_RAD))
                {
                    m_pcWheels->SetLinearVelocity(0.05, -0.05);
                }
                else
                {
                    m_pcWheels->SetLinearVelocity(4, 4);
                }
            }
            break;
        }

        case EState::Circumnavigate: //the robot drive around the obstical until finding the air path agin
        {
            // calculates the robors position
            CVector3 pos = m_pcPositioning->GetReading().Position;
            
            //caculates the angle of the vector between the robot and the goul
            CRadians angle2goul = (m_cTargetPosition - pos).GetZAngle().UnsignedNormalize(); 
            
            // Calculate angle difference between the robots angle to goul and the air angle to goul
            CRadians cAngleDifference = (airAngle - angle2goul).UnsignedNormalize();
            if (turned && cAngleDifference <= THRESHOLD_RAD && cAngleDifference <= (CRadians ::TWO_PI - THRESHOLD_RAD))
            {
                m_state = EState::GO2GOUL;
                // turns the LEDs blue for searching
                m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
                LOG << "state: GO2GOUL" << std::endl; // reports the new state
                break;
            }
            else{
                // handel the driving of the robot
                if (IsObstacleAhead())
                { // in order to avoid running into walls turns right in the place
                    m_pcWheels->SetLinearVelocity(0.04, -0.04);
                }
                else if (IsObstacleDetected(5))
                { // while the obsticals is to the left of the robot drive forward
                    m_pcWheels->SetLinearVelocity(4, 4);
                }
                else
                { // when losing the obstical continue driving left
                    m_pcWheels->SetLinearVelocity(0.02, 0.5);
                    turned = true;
                }
            }
            
            break;
        }
        }
    }

    bool ControllerBug2::IsObstacleAhead()
    { // returns true if their is an obstical 10cm infront of the robot
        bool bObstical = false;

        m_pcRangefinders->Visit([this, &bObstical](const CCI_PiPuckRangefindersSensor::SInterface &sensor) { // a function to implement for every range finder sensor
            if (sensor.Label != 0 && sensor.Label != 7)
                return; // continues only for the two frot range finder sensor
            if (sensor.Proximity < 0.1)
            { ////if the proximity the sensor detects is less then 10 CMs return true
                bObstical = true;
            }
        });

        return bObstical;
    }

    bool ControllerBug2::IsObstacleDetected(int sensor_index)
    { // returns true if their is an obstical 10cm infront of the sensor with the given index
        bool bObstical = false;

        m_pcRangefinders->Visit([this, &bObstical, &sensor_index](const CCI_PiPuckRangefindersSensor::SInterface &sensor) { // a function to implement for every range finder sensor
            if (sensor.Label != sensor_index)
                return; // continues only for the two frot range finder sensor
            if (sensor.Proximity < 0.1)
            { // if the proximity the sensor detects is less then 10 CMs return true
                bObstical = true;
            }
        });

        return bObstical;
    }

    /****************************************/
    /****************************************/

    REGISTER_CONTROLLER(ControllerBug2, "controller_bug2");

}
