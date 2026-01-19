#include "team1.hpp"
#include <iostream>



namespace argos {

	/****************************************/
	/****************************************/

	void Controller1::Init(TConfigurationNode& t_tree) {
		ForagingController::Init(t_tree);

		/* Your Init code goes here */
		eState = STATE_EXPLORE;
		walkCounter = 0;
		currMovementDuration = 60; // ערך התחלתי
		leftWheelSpeed = 0.0f;
		rightWheelSpeed = 0.0f;
		chooseCurrMovement();


	}

	void Controller1::ControlStep() {
		/* Your ControlStep code goes here*/
		// start of switch case
		switch (eState){
			case STATE_EXPLORE:{
				if(isObstacleAhead()){
					eState = STATE_AVOID_OBSTACLE;

				}
				else{
					randomWalk();
				}
				break;
			}
			case STATE_AVOID_OBSTACLE:{
				avoidObstacle();
				break;
			}
		}

		// end of switch case
		// end of control step
	}
	// state functions
	void Controller1::randomWalk(){
		walkCounter++;
		if(walkCounter >= currMovementDuration){
			walkCounter = 0;
			chooseCurrMovement();
		}
		m_pcWheels->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);
		
	}
	void Controller1:: chooseCurrMovement(){
	
		Real movement = m_rng->Uniform(CRange<Real>(0.0f,1.0f));
		if(movement < 0.55f){
			currMovementDuration = 60 +  m_rng->Uniform(CRange<UInt32>(0,60));
			leftWheelSpeed = 0.157f;
			rightWheelSpeed = 0.157f;
			std::cout << "Moving straight" << std::endl;
		}
		else if(movement < 0.70f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = -0.05f;
			rightWheelSpeed = 0.05f;
			std::cout << "Rotating left in place" << std::endl;
		}
		else if(movement < 0.85f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = 0.05f;
			rightWheelSpeed = -0.05f;
			std::cout << "Rotating right in place" << std::endl;
		}
		else{
			currMovementDuration = 40 +  m_rng->Uniform(CRange<UInt32>(0,40));
			Real fSpeedRatio = m_rng->Uniform(CRange<Real>(0.85, 0.95));
			if(m_rng->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
				// מעגל שמאלה
				leftWheelSpeed = 0.157f * fSpeedRatio;
				rightWheelSpeed = 0.157f;
				std::cout << "Circular movement left" << std::endl;
			} else {
				// מעגל ימינה
				leftWheelSpeed = 0.157f;
				rightWheelSpeed = 0.157f * fSpeedRatio;
				std::cout << "Circular movement right" << std::endl;
			}
		}
		}

	void Controller1::avoidObstacle(){
		m_pcWheels->SetLinearVelocity(-0.08f, 0.08f); // סיבוב שמאלה
		
		// בדוק אם הדרך כבר פנויה
		if(!isObstacleAhead()){
			std::cout << "Path clear! Returning to EXPLORE" << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0; // אפס את הקאונטר
			chooseCurrMovement(); // בחר תנועה חדשה
		}

	}
	
	// helper functions
	 bool Controller1::isObstacleAhead() const {
      /* Check the front rangefinders */
      bool bObstacle = false;
      
      // define a lambda function to process each sensor reading
      std::function<void(const CCI_PiPuckRangefindersSensor::SInterface&)> fCheckSensor =
         [&bObstacle](const auto& sensor) {
            const auto& [id, pos, ori, range] = sensor.Configuration;
            if (sensor.Label != 0 && sensor.Label != 7) return; 
            if (sensor.Proximity < range) {
               bObstacle = true;
            }
         };
      // visit each sensor reading, modifying bObstacle if any of the front sensors detected
      m_pcRangefinders->Visit(fCheckSensor);
      return bObstacle;
   }
	CVector2 Controller1::getRobotPosition() const{
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      return CVector2(reading.Position.GetX(), reading.Position.GetY());
   }
   CRadians Controller1::getRobotHeading() const{
      CRadians cHeading, cPitch, cRoll;
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      reading.Orientation.ToEulerAngles(cHeading, cPitch, cRoll);
      return cHeading;
   }

	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}

