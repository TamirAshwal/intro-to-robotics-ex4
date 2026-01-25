#include "team2.hpp"
#include <iostream>



namespace argos {

	/****************************************/
	/****************************************/

	void controller2::Init(TConfigurationNode& t_tree) {
		ForagingController::Init(t_tree);

		/* Your Init code goes here */
		eState = STATE_EXPLORE;
		walkCounter = 0;
		currMovementDuration = 60; 
		leftWheelSpeed = 0.0f;
		rightWheelSpeed = 0.0f;
		hasFood = false;
		chooseCurrMovement();


	}

	void controller2::ControlStep() {
		//* Your ControlStep code goes here*/
		m_Readings = m_pcCamera->GetReadings();
		// start of switch case
		switch (eState){
			case STATE_EXPLORE:{
				if(isObstacleAhead()){
					eState = STATE_AVOID_OBSTACLE;
				}
				else if(!getFoodPositions().empty() && !hasFood){
					eState = STATE_TO_FOOD;
					std::cout << "Food detected! Switching to TO_FOOD state" << std::endl;
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
			case STATE_TO_FOOD:{
				moveToFood2();
				break;

			}
			case STATE_RETURN_TO_BASE:{
				returnToBase();
				break;
			}
		}

		// end of switch case
		// end of control step
	}
	// state functions
	void controller2::randomWalk(){
		walkCounter++;
		if(walkCounter >= currMovementDuration){
			walkCounter = 0;
			chooseCurrMovement();
		}
		m_pcWheels->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);
		
	}
	void controller2:: chooseCurrMovement(){
	
		Real movement = m_rng->Uniform(CRange<Real>(0.0f,1.0f));
		if(movement < 0.50f){
			currMovementDuration = 50 +  m_rng->Uniform(CRange<UInt32>(0,50));
			leftWheelSpeed = 0.157f;
			rightWheelSpeed = 0.157f;
			std::cout << "Moving straight" << std::endl;
		}
		else if(movement < 0.68f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = -0.05f;
			rightWheelSpeed = 0.05f;
			std::cout << "Rotating left in place" << std::endl;
		}
		else if(movement < 0.86f){
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
	// we saw at least one food so we go to it
	void controller2::moveToFood(){
		Real minDistance = std::numeric_limits<Real>::max();
		CRadians relativeAngle;
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == CColor::GRAY80){
				if(blob->Distance < minDistance){
					minDistance = blob->Distance;
					relativeAngle = blob->Angle;
					relativeAngle = blob->Angle.SignedNormalize();
				}
			}
		}
		if(minDistance >= std::numeric_limits<Real>::max()){
			std::cout << "Lost sight of food, returning to explore" << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0;
			chooseCurrMovement();
			return;
		}
		// check if we got to the food
		if(minDistance < 0.15f){
			std::cout << "Reached food! Returning to base" << std::endl;
			hasFood = true;
			eState = STATE_RETURN_TO_BASE;
		
			return;
		}
		
		// move the robot towards the food
		if(relativeAngle.GetAbsoluteValue() < CRadians::PI_OVER_FOUR.GetValue()){
			
			m_pcWheels->SetLinearVelocity(0.157f, 0.157f);
		}
		else if(relativeAngle.GetValue() > 0){
			
			m_pcWheels->SetLinearVelocity(-0.157f, 0.157f);
		}
		else{
		
			m_pcWheels->SetLinearVelocity(0.157f, -0.157f);
		}

	}
	void controller2::returnToBase(){
		    std::cout << "=== In returnToBase ===" << std::endl;

		CVector2 robotPos = getRobotPosition();
		CRadians robotHeading = getRobotHeading();
		CVector2 basePosition = getBasePosition();
		CVector2 toBase = basePosition - robotPos;
		Real distance = toBase.Length();
		// check if we reached the base
		if(distance < 0.08f){
			std::cout << "Reached base! Dropping food" << std::endl;
			eState = STATE_EXPLORE;
			hasFood = false;
			walkCounter = 0;
			chooseCurrMovement();
			return;
		}
		// navigate to the base
		CRadians targetAngle = toBase.Angle();
		CRadians relativeAngle = (targetAngle - robotHeading).SignedNormalize();
		 if(isObstacleAhead()){
        // פשוט התחמק (או תוכל להשתמש ב-Bug2 כאן!)
        m_pcWheels->SetLinearVelocity(-0.08f, 0.08f);
    }
    // כוון לבסיס
    else if(relativeAngle.GetAbsoluteValue() < CRadians::PI_OVER_FOUR.GetValue()){
        // לך ישר
        m_pcWheels->SetLinearVelocity(0.157f, 0.157f);
    }
    else if(relativeAngle.GetValue() > 0){
        // פנה שמאלה
        m_pcWheels->SetLinearVelocity(-0.157f, 0.157f);
    }
    else{
        // פנה ימינה
        m_pcWheels->SetLinearVelocity(0.157f, -0.157f);
    }

	}

	void controller2::avoidObstacle(){
		m_pcWheels->SetLinearVelocity(-0.08f, 0.08f); 
		if(!isObstacleAhead()){
			std::cout << "Path clear! Returning to EXPLORE" << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0; 
			chooseCurrMovement(); 
		}
	}
	bool controller2::isFoodVisible() const{
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == CColor::GRAY80){
				return true;
			}
		}
		return false;
	}
	CVector2 controller2::getBasePosition() const{
		if(m_basePositions.empty()){
			std::cout << " base positions not initialized!" << std::endl;
			return CVector2(0.0f, 0.0f);
		}
		else{
			return CVector2(m_basePositions[0].GetX(),
							m_basePositions[0].GetY());
		}
	}
	// helper functions
	 bool controller2::isObstacleAhead() const {
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
	CVector2 controller2::getRobotPosition() const{
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      return CVector2(reading.Position.GetX(), reading.Position.GetY());
   }
   CRadians controller2::getRobotHeading() const{
      CRadians cHeading, cPitch, cRoll;
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      reading.Orientation.ToEulerAngles(cHeading, cPitch, cRoll);
      return cHeading;
   }

   std::vector<CVector2> controller2::getFoodPositions() const{
		std::vector<CVector2> foodPositions;

		for(const auto& foodblob : m_Readings.BlobList){
			if(foodblob->Color == CColor::GRAY80 ){
				bool taken = false;
				
				for(const auto& robotblob : m_Readings.BlobList){
					if((robotblob->Color == CColor::BLUE || robotblob->Color == CColor::RED) && (foodblob->Angle - robotblob->Angle).UnsignedNormalize() < CRadians::PI_OVER_THREE){
						taken = true;
						break;
					}	
				}

				if(!taken)
					foodPositions.emplace_back(foodblob->Distance, foodblob->Angle); 
			}
		}

		return foodPositions;
	}

	void controller2::moveToFood2(){
		Real minDistance = std::numeric_limits<Real>::max();
		CRadians relativeAngle;
		
		std::vector<CVector2> foodPositions = getFoodPositions();
		for(const auto& food : foodPositions){
			if(food.Length() < minDistance){
				minDistance = food.Length();
				relativeAngle = food.Angle().SignedNormalize();
			}
		}

		if(minDistance >= std::numeric_limits<Real>::max()){
			std::cout << "Lost sight of food, returning to explore" << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0;
			chooseCurrMovement();
			return;
		}
		// check if we got to the food
		if(minDistance < 0.15f){
			std::cout << "Reached food! Returning to base" << std::endl;
			hasFood = true;
			eState = STATE_RETURN_TO_BASE;
		
			return;
		}
		
		// move the robot towards the food
		if(relativeAngle.GetAbsoluteValue() < CRadians::PI_OVER_FOUR.GetValue()){
			
			m_pcWheels->SetLinearVelocity(0.157f, 0.157f);
		}
		else if(relativeAngle.GetValue() > 0){
			
			m_pcWheels->SetLinearVelocity(-0.157f, 0.157f);
		}
		else{
		
			m_pcWheels->SetLinearVelocity(0.157f, -0.157f);
		}
	}
	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(controller2, "controller2");

}

