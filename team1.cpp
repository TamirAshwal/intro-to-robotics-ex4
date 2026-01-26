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
		currMovementDuration = 60; 
		leftWheelSpeed = 0.0f;
		rightWheelSpeed = 0.0f;
		hasFood = false;

		//blocking varubels
		enemyBaseKnown = false;
		followingEnemy = false;
		
		//interception varubels
		lastAngleToTarget = 0;
		filterdError = 0;
      	interception_alpha = 0.6;
		interception_K = 5.0f;
		isRobotCW = true;

		blockMode = true;//CRandom::CreateRNG("argos")->Uniform(CRange<Real>(0.0f, 1.0f)) < 0.25;
		
		chooseCurrMovement();

	}

	void Controller1::ControlStep() {
		/* Your ControlStep code goes here*/
		m_Readings = m_pcCamera->GetReadings();
		// start of switch case
		switch (eState){
			case STATE_EXPLORE:{
				if(isObstacleAhead()){
					eState = STATE_AVOID_OBSTACLE;
				}
				else if(blockMode && !enemyBaseKnown && !enemyWithFoodPossitions().empty()){
					eState = STATE_BLOCK;
					std::cout << "enemy with Food detected! Switching to Block state!" << std::endl;
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
			case STATE_BLOCK:{
				block();
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
	void Controller1::moveToFood(){
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
	void Controller1::returnToBase(){
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

	void Controller1::avoidObstacle(){
		m_pcWheels->SetLinearVelocity(-0.08f, 0.08f); 
		if(!isObstacleAhead()){
			std::cout << "Path clear! Returning to EXPLORE" << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0; 
			chooseCurrMovement(); 
		}
	}

	bool Controller1::isFoodVisible() const{
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == CColor::GRAY80){
				return true;
			}
		}
		return false;
	}
	
	CVector2 Controller1::getBasePosition() const{
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
      return cHeading.UnsignedNormalize();
   }

	std::vector<CVector2> Controller1::getFoodPositions() const{
		std::vector<CVector2> foodPositions;

		for(const auto& foodblob : m_Readings.BlobList){
			if(foodblob->Color == CColor::GRAY80){
				bool taken = false;
				
				for(const auto& robotblob : m_Readings.BlobList){
					if(robotblob->Color != CColor::GRAY80 && (foodblob->Angle - robotblob->Angle).UnsignedNormalize() < CRadians::PI_OVER_THREE){
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

	CRadians Controller1::angle2TeamAhead() const{
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == m_teamColor && blob->Angle.SignedNormalize() < CRadians::PI/6){
				return blob->Angle.SignedNormalize();
			}
		}

		return CRadians::PI;
	}

	//overides!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// improved move to food that don't follow robots
	void Controller1::moveToFood2(){
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

	//improved avoidObstacle that clears the path for robots with food
	void Controller1::avoidObstacle2(){
		CRadians angle2Team = angle2TeamAhead();
		if(angle2Team != CRadians::PI){ //only checks if angel2Team exsistes, it will never be pi
			if (hasFood)
				return;
			else
				m_pcWheels->SetLinearVelocity(0.145f - 0.01f * angle2Team.GetValue(), 0.145f + 0.01f * angle2Team.GetValue()); 
		}	
		else{
			m_pcWheels->SetLinearVelocity(-0.04f, 0.04f); 
		}

		if(!isObstacleAhead()){
			std::cout << "Path clear! Returning to EXPLORE" << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0; 
			chooseCurrMovement(); 
		}
	}

	//blocking effort
	
	CVector2 Controller1::relToAbsPosition(const CVector2 blob) const{
		CVector2 robotPos = getRobotPosition();
		CRadians robotHeading = getRobotHeading();

		CVector2 blobRel(blob.Length()* Cos(blob.Angle()),blob.Length() * Sin(blob.Angle()));

		blobRel.Rotate(robotHeading);

		return robotPos + blobRel;
	}


	std::vector<CVector2> Controller1::enemyWithFoodPossitions() const {

		std::vector<CVector2> positions;

		for(const auto& enemy : m_Readings.BlobList) {
			//if(enemy->Color == CColor::GRAY80 || enemy->Color == m_teamColor) continue;
			if(enemy->Color == CColor::RED) {
				for(const auto& food : m_Readings.BlobList) {
					if(food->Color != CColor::GRAY80) continue;

					if((enemy->Angle - food->Angle).UnsignedNormalize() < CRadians::PI / 12) {
						positions.emplace_back(enemy->Distance, enemy->Angle);
						break;
					}
				}
			}
		}
		return positions;
	}

	void Controller1::followEnemy(const CRadians relativeAngle) {
		//good code **********************************************************8
		if(relativeAngle.GetAbsoluteValue() < CRadians::PI_OVER_SIX.GetValue() && !isObstacleAhead()){
			
			m_pcWheels->SetLinearVelocity(0.157f, 0.157f);
		}
		else if(relativeAngle.GetValue() > 0){
			
			m_pcWheels->SetLinearVelocity(-0.157f, 0.157f);
		}
		else{
		
			m_pcWheels->SetLinearVelocity(0.157f, -0.157f);
		}
		//********************************************************************88888 */

		//// bad interceptoin code **************************
		// CVector2 robotPos = getRobotPosition();
		// CRadians robotHeading = getRobotHeading();
		// CRadians angle = relativeAngle;
		
		// float angleToTarget = angle.SignedNormalize().GetValue();
		// float angleError = CRadians(angleToTarget - lastAngleToTarget).SignedNormalize().GetValue();
		// lastAngleToTarget = angleToTarget;

		// filterdError = interception_alpha * angleError + (1-interception_alpha) * filterdError;

		// m_pcWheels->SetLinearVelocity(0.25f - interception_K * filterdError, 0.25f + interception_K * filterdError);
	}

	void Controller1::enemyBlocking(CVector2 enemyPos, CVector2 enemyGoal){
		CVector2 robotPos = getRobotPosition();
		CRadians robotHeading = getRobotHeading();

		// --- Target-centered angles ---
		CVector2 vE = enemyPos - enemyGoal;
		CVector2 vR = robotPos - enemyGoal;

		CRadians angE = vE.Angle();
		CRadians angR = vR.Angle();

		CRadians deltaAng = (angE - angR).SignedNormalize();

		float speed = ((deltaAng * isRobotCW).GetValue() > 0)? 1.2f : -1.2f;

		m_pcWheels->SetLinearVelocity(speed , speed);
	}

	void Controller1::turn4defence(CVector2 enemyBasePos){
		CVector2 robotPos = getRobotPosition();
		CRadians robotHeading = getRobotHeading();

		CVector2 toTarget =  enemyBasePos - robotPos;
		CRadians headingErrPlus = CRadians(toTarget.Angle() + CRadians::PI_OVER_TWO - robotHeading).SignedNormalize();
		CRadians headingErrMinus = CRadians(toTarget.Angle() - CRadians::PI_OVER_TWO - robotHeading).SignedNormalize();

		if(headingErrPlus < headingErrMinus && headingErrPlus > CRadians(0.1)){
			m_pcWheels->SetLinearVelocity(-0.1, 0.1);
			isRobotCW = -1;
		}
		else if ( headingErrMinus < headingErrPlus && headingErrMinus > CRadians(0.1))
		{
			m_pcWheels->SetLinearVelocity(0.1, -0.1);
			isRobotCW = 1;
		}
		else{
			m_pcWheels->SetLinearVelocity(0, 0);
			isRobotCW = (headingErrPlus < headingErrMinus)? -1 : 1;
		}
	}

	void Controller1::block() {

		Real minDistance = std::numeric_limits<Real>::max();
		CRadians relativeAngle;
		
		std::vector<CVector2> enemyPositions = enemyWithFoodPossitions();

		if(enemyBaseKnown && enemyPositions.empty()) {
				turn4defence(enemyBasePos);
				return;
		}

		if(enemyPositions.empty() && followingEnemy) {
			for(const auto& enemy : m_Readings.BlobList) {
				if(enemy->Color != CColor::GRAY80 && enemy->Color != m_teamColor) {
					if((relToAbsPosition(CVector2(enemy->Distance/100, enemy->Angle)) - enemyWithFoodPos).Length() < 0.2f){
						enemyBasePos = enemyWithFoodPos;
						enemyBaseKnown = true; 
						std::cout << "found emeny base at:" << enemyBasePos.GetX() << "," << enemyBasePos.GetY() << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
						turn4defence(enemyBasePos); //turn for defence
						return;
					}
				}
			}
			followingEnemy = false;
			eState = STATE_EXPLORE;
			return;
		}

		for(const auto& enemy : enemyPositions){
			if(enemy.Length() < minDistance){
				minDistance = enemy.Length();
				relativeAngle = enemy.Angle();
			}
		}
		enemyWithFoodPos = relToAbsPosition(CVector2(minDistance/100, relativeAngle));
		if(enemyBaseKnown){
			turn4defence(enemyBasePos);
			enemyBlocking(enemyWithFoodPos, enemyBasePos);
		}
		else{
			followingEnemy = true;
			followEnemy(relativeAngle);
		}
	}
	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}

