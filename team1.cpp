// 209374867
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
		chooseCurrMovement();
		// variables for stuck detection
		m_lastStuckPosition = getRobotPosition();
		m_stuckTimer = 0;
		m_unstickStepCounter = 0;
		m_isUnsticking = false;
		m_proximityStuckTimer = 0;
		enemyBaseKnown = false;
    	followingEnemy = false;
		lastAngleToTarget = 0;
		filterdError = 0;
		interception_alpha = 0.6f;
		interception_K = 5.0f;
		isRobotCW = true;
		// blockMode = false; //blockMode = CRandom::CreateRNG("argos")->Uniform(CRange<Real>(0.0f, 1.0f)) < 0.25;

	}

	void Controller1::ControlStep() {
		/* Your ControlStep code goes here*/
		m_Readings = m_pcCamera->GetReadings();
		bool shouldCheckStuck = !enemyBaseKnown;

		// Unstuck mechanism
		// firstly check if we are stuck this is true after the robot decided it is stuck
		if(shouldCheckStuck){
		if (m_isUnsticking) {
			m_unstickStepCounter++;
			// we are stuck so first we  reverse for 20 steps
			if (m_unstickStepCounter < 20) {
				m_pcWheels->SetLinearVelocity(-0.1f, -0.1f); // הגברתי טיפה מהירות רוורס
			}
			// then we turn for 30 steps
			else if (m_unstickStepCounter < 50) {
				m_pcWheels->SetLinearVelocity(0.02f, -0.02f); 
			}
			// after the stuck menuver is done which is after 50 steps we reset the variables and return to previous state
			else {
				m_isUnsticking = false;
				m_stuckTimer = 0;
                m_proximityStuckTimer = 0; 
				m_lastStuckPosition = getRobotPosition(); 
				// super important check to see if ew were in the middle of returning to base
                if (hasFood) {
                    std::cout << "Unstuck with food! Recalculating path to base." << std::endl;
                    eState = STATE_RETURN_TO_BASE;
                    m_bFacingBase = false; 
                }
                else {
					// if we don;t have food we just conntinue with exploring 
				    eState = STATE_EXPLORE; 
				    chooseCurrMovement(); 
                }
			}
			return; 
		}
		// this is after the first if statement meaning we are not in the middle of the menuver
        bool isStuck = false;
        bool robotClose = false;
		// check if there are robots which can be teamates or enemies really clise to us
        for(const auto& blob : m_Readings.BlobList) {
            if (blob->Color != CColor::GRAY80) {
                if (blob->Distance < 15.0f && Abs(blob->Angle.GetValue()) < 0.5f) {
                    robotClose = true;
					
                    break;
                }
            }
        }
		// we upadte the proximityStuckTimer according to if there are robots close to us
        if (robotClose) {
            m_proximityStuckTimer++;
        } 
		else {
            if(m_proximityStuckTimer > 0) m_proximityStuckTimer--;
        }
		// if the timer is above 30 we are stuck
        if (m_proximityStuckTimer > 30) {
            std::cout << "PROXIMITY STUCK DETECTED! (Blocked by robot)" << std::endl;
            isStuck = true;
        }
		// this is the same logic as before but we use location instead of the camera
		CVector2 currentPos = getRobotPosition();
		Real distMoved = (currentPos - m_lastStuckPosition).Length();

		if (distMoved < STUCK_THRESHOLD_DIST) {
			m_stuckTimer++;
		} 
		else {
			m_stuckTimer = 0;
			m_lastStuckPosition = currentPos;
		}
		// if we are stuck for sime tine make the flag true
		if (m_stuckTimer > STUCK_THRESHOLD_TIME) {
			std::cout << "POSITION STUCK DETECTED!" << std::endl;
            isStuck = true;
		}
		// we are stuck so we make sure we start the menuver
        if (isStuck) {
            m_isUnsticking = true;
			m_unstickStepCounter = 0;
			return;
        }
	}
		// start of switch case
		switch (eState){
			case STATE_EXPLORE:{
				if(isObstacleAhead()){
					eState = STATE_AVOID_OBSTACLE;
				}
				// else if(!enemyWithFoodPossitions().empty()){
				// 	eState = STATE_BLOCK;
				// 	std::cout << "enemy with Food detected! Switching to Block state" << std::endl;
				// }
				// // Logic from File 1: Enter block mode if assigned and enemy seen
            	// else if(blockMode && !enemyBaseKnown && !enemyWithFoodPossitions().empty()){
                // eState = STATE_BLOCK;
                // std::cout << "Enemy with Food detected! Switching to Block state" << std::endl;
           		// }
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
				avoidObstacle2();
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
		// check if we need to choose a new movement
		if(walkCounter >= currMovementDuration){
			walkCounter = 0;
			chooseCurrMovement();
		}
		m_pcWheels->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);
	}
	// fucntion that choose the next movement randomally
	void Controller1:: chooseCurrMovement(){
	
		Real movement = m_rng->Uniform(CRange<Real>(0.0f,1.0f));
		// going straight
		if(movement < 0.50f){
			currMovementDuration = 50 +  m_rng->Uniform(CRange<UInt32>(0,50));
			leftWheelSpeed = 0.157f;
			rightWheelSpeed = 0.157f;
			
		}
		// rotating left in place 
		else if(movement < 0.68f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = -0.05f;
			rightWheelSpeed = 0.05f;
			
		}
		// ratating right in place
		else if(movement < 0.86f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = 0.05f;
			rightWheelSpeed = -0.05f;
			
		}
		// moving in a circular path 
		else{
			currMovementDuration = 40 +  m_rng->Uniform(CRange<UInt32>(0,40));
			Real fSpeedRatio = m_rng->Uniform(CRange<Real>(0.85, 0.95));
			if(m_rng->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
				// left circle
				leftWheelSpeed = 0.157f * fSpeedRatio;
				rightWheelSpeed = 0.157f;
			} else {
				// right circle
				leftWheelSpeed = 0.157f;
				rightWheelSpeed = 0.157f * fSpeedRatio;
			}
		}
	}
	
	void Controller1::returnToBase() {
		
		// firstly check if we got to the base
		CVector2 currPos = getRobotPosition();
		CVector2 basePos = getBasePosition();
		Real distanceToBase = (currPos - basePos).Length();
		// if so drop the food return to explore and update the flags
		if(distanceToBase < 0.10f){ 
			m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
			hasFood = false;
			m_carriedFoodId = std::nullopt;
			std::cout << "Dropped food at base! Back to Explore." << std::endl;
			eState = STATE_EXPLORE;
			walkCounter = 0;
			chooseCurrMovement();
			return;
		}
		// check if we are facing the base if not we rotate the robot
		if (!m_bFacingBase) {
			CVector2 toBase = basePos - currPos;
			CRadians targetHeading = toBase.Angle();
			CRadians currentHeading = getRobotHeading();
			CRadians error = (targetHeading - currentHeading).SignedNormalize();
			const Real epsilon = 0.05f; 
			// fix the robot heading 
			if (Abs(error.GetValue()) > epsilon) {
				std::cout << "readjust the heading... Error: " << error.GetValue() << std::endl;
				// move it by a speed proportional to the error
				Real speed = error.GetValue() * 0.5f; 
				if(speed > 0.2f) speed = 0.2f;
				if(speed < -0.2f) speed = -0.2f;
				if(speed > 0.0f && speed < 0.02f) speed = 0.02f;
				if(speed < 0.0f && speed > -0.02f) speed = -0.02f;
				m_pcWheels->SetLinearVelocity(-speed, speed);
			} 
			// the heading is correct so update the flags
			else {
				m_bFacingBase = true;
				m_cReturnLineHeading = targetHeading; 
				m_pcWheels->SetLinearVelocity(0.0f, 0.0f); 
				std::cout << "Aligned with base! Starting M-Line." << std::endl;
			}
			return;
		}
		// the heading is correct
		else {
			CRadians currHeading = getRobotHeading();
			CRadians error = (m_cReturnLineHeading - currHeading).SignedNormalize();
			const Real K_P = 0.5f; 
			Real turnCorrection = error.GetValue() * K_P;
			m_pcWheels->SetLinearVelocity(0.15f - turnCorrection, 0.15f + turnCorrection);
		}

	}
		
	// helper functions to check for food and base position
	bool Controller1::isFoodVisible() const{
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == CColor::GRAY80){
				return true;
			}
		}
		return false;
	}
	// get the base position from the ForagingController class
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
   // get the robot position from the positioning sensor
	CVector2 Controller1::getRobotPosition() const{
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      return CVector2(reading.Position.GetX(), reading.Position.GetY());
   }
   // get the robot heading from the positioning sensor
   CRadians Controller1::getRobotHeading() const{
      CRadians cHeading, cPitch, cRoll;
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      reading.Orientation.ToEulerAngles(cHeading, cPitch, cRoll);
      return cHeading;
   }

    std::vector<CVector2> Controller1::getFoodPositions() const {
		std::vector<CVector2> foodPositions;

		for (const auto& foodblob : m_Readings.BlobList) {
			if (foodblob->Color == CColor::GRAY80) {
				bool taken = false;
				// vector from me to the food
				CVector2 vMeToFood;
				vMeToFood.FromPolarCoordinates(foodblob->Distance, foodblob->Angle);
				for (const auto& robotblob : m_Readings.BlobList) {
					// check only other robots
					if (robotblob->Color != CColor::GRAY80) {
						// distance between the food and the robot
						CVector2 vMeToRobot;
						vMeToRobot.FromPolarCoordinates(robotblob->Distance, robotblob->Angle);
						// vector from the robot to the food
						CVector2 vRobotToFood = vMeToFood - vMeToRobot;
						// if the robot is closer to the food than me, mark the food as taken
						if (vRobotToFood.Length() < vMeToFood.Length()) {
							taken = true;
							break;
						}
					}
				}

				if (!taken) {
					foodPositions.emplace_back(foodblob->Distance, foodblob->Angle);
				}
			}
		}

		return foodPositions;
	}
	// angle to teammate ahead
	CRadians Controller1::angle2TeamAhead() const{
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == m_teamColor && blob->Angle.SignedNormalize() < CRadians::PI/6){
				return blob->Angle.SignedNormalize();
			}
		}

		return CRadians::PI;
	}
	// check to see if we need this function
	bool Controller1::isTeammateAhead() const {
    for(const auto& blob : m_Readings.BlobList) {
        if(blob->Color == m_teamColor) {  
            CRadians angle = blob->Angle.SignedNormalize();
            // בדיקה אם הרובוט מלפנים (בטווח 30 מעלות) וקרוב
            if(Abs(angle.GetValue()) < CRadians::PI_OVER_SIX.GetValue() && 
               blob->Distance < 50.0f) {  
                return true;
            }
        }
    }
    return false;
}

	
	// improved move to food that don't follow robots
	void Controller1::moveToFood2(){
		Real minDistance = std::numeric_limits<Real>::max();
		CRadians relativeAngle;
		// get food positions that are not taken by other robots
		std::vector<CVector2> foodPositions = getFoodPositions();
		for(const auto& food : foodPositions){
			if(food.Length() < minDistance){
				minDistance = food.Length();
				relativeAngle = food.Angle().SignedNormalize();
			}
		}
		// check if we lost sight of the food
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
			m_bFacingBase = false;
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
		m_pcWheels->SetLinearVelocity(-0.08f, 0.08f); // סיבוב במקום
    if(!isObstacleAhead()){ 
        eState = STATE_EXPLORE; 
        walkCounter = 0; 
        chooseCurrMovement(); 
    }
	}

	//blocking effort
	std::vector<CVector2> Controller1::enemyWithFoodPossitions() const {
		std::vector<CVector2> positions;
		// find enemies that have food (food directly in front of them)
		for(const auto& enemy : m_Readings.BlobList) {
			if(enemy->Color == CColor::GRAY80 || enemy->Color == m_teamColor) continue;
			// check if there is food directly in front of the enemy
			for(const auto& food : m_Readings.BlobList) {
				if(food->Color != CColor::GRAY80) continue;
				if((enemy->Angle - food->Angle).UnsignedNormalize() < CRadians::PI / 12) {
					positions.emplace_back(food->Distance, food->Angle);
					break;
				}
			}
		}
		return positions;
	}
	// follow the enemy with food
	void Controller1::followEnemy(const CRadians& relativeAngle) {
		// move the robot towards the enemy
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
	// store the enemy base position
	void Controller1::StoreEnemyBasePosition() {
		enemyBasePos = enemyWithFoodPos;
		enemyBaseKnown = true;

		std::cout << "Enemy base discovered at "
				<< enemyBasePos << std::endl;
	}
	// go to enemy base and camp there
	void Controller1::GoToEnemyBaseAndCamp() {
		const CVector3& pos3D = m_pcPositioning->GetReading().Position;
		CVector2 robotPos(pos3D.GetX(), pos3D.GetY());
		// vector to enemy base
		CVector2 toBase = enemyBasePos - robotPos;
		CRadians targetAngle = toBase.Angle();
		// get robot heading
		CRadians heading, pitch, roll;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(heading, pitch, roll);
		// angle difference
		CRadians diff = (targetAngle - heading).SignedNormalize();
		// check if we got to the base
		if(toBase.Length() < 0.15f) {
			m_pcWheels->SetLinearVelocity(0.0f, 0.0f); // CAMP
			return;
		}
		// move towards enemy base
		if(Abs(diff) < CRadians::PI_OVER_SIX) {
			m_pcWheels->SetLinearVelocity(0.15f, 0.15f);
		}
		else if(diff.GetValue() > 0) {
			m_pcWheels->SetLinearVelocity(-0.1f, 0.1f);
		}
		else {
			m_pcWheels->SetLinearVelocity(0.1f, -0.1f);
		}
	}
	// blocking state function
	void Controller1::block() {
		Real minDistance = std::numeric_limits<Real>::max();
		CRadians relativeAngle;
		std::vector<CVector2> enemyPositions = enemyWithFoodPossitions();
		// Case 1: We know where the enemy base is, but see no enemies -> Guard the base
		if(enemyBaseKnown && enemyPositions.empty()) {
				if (turn4defence(enemyBasePos)) m_pcWheels->SetLinearVelocity(0,0);
				return;
		}
    	// Case 2: We were following an enemy, lost them, and now need to see if we found their base
		if(enemyPositions.empty() && followingEnemy) {
			for(const auto& enemy : m_Readings.BlobList) {
				// Check all non-food, non-team blobs
				if(enemy->Color != CColor::GRAY80 && enemy->Color != m_teamColor) { // Make sure m_teamColor is defined or use !Blue/!Red logic
					// Distance check to define "Base Found"
					if((relToAbsPosition(CVector2(enemy->Distance/100.0f, enemy->Angle)) - enemyWithFoodPos).Length() < 0.2f){
						enemyBasePos = enemyWithFoodPos;
						enemyBaseKnown = true; 
						std::cout << "Found Enemy Base at: " << enemyBasePos.GetX() << ',' << enemyBasePos.GetY() <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1"<< std::endl;
						turn4defence(enemyBasePos); 
						return;
					}
				}
			}
			followingEnemy = false;
			eState = STATE_EXPLORE;
			return;
		}

		// Case 3: We see enemies with food -> Chase or Block
		for(const auto& enemy : enemyPositions){
			if(enemy.Length() < minDistance){
				minDistance = enemy.Length();
				relativeAngle = enemy.Angle();
			}
		}
		// Update predicted enemy position
		// Note: Divide distance by 100 if your logic assumes cm vs meters, check consistency with relToAbsPosition
		enemyWithFoodPos = relToAbsPosition(CVector2(minDistance/100.0f, relativeAngle)); 

    	if(enemyBaseKnown){
        enemyBlocking(enemyWithFoodPos, enemyBasePos);
    	}
    	else{
        followingEnemy = true;
        followEnemy(relativeAngle);
    	}
	}
	// convert relative position to absolute position
	CVector2 Controller1::relToAbsPosition(const CVector2 blob) const{
			CVector2 robotPos = getRobotPosition();
			CRadians robotHeading = getRobotHeading();

			CVector2 blobRel(blob.Length()* Cos(blob.Angle()),blob.Length() * Sin(blob.Angle()));

			blobRel.Rotate(robotHeading);

			return robotPos + blobRel;
		}
	// enemy blocking function
	void Controller1::enemyBlocking(CVector2 enemyPos, CVector2 enemyGoal){
		if(!turn4defence(enemyGoal)){
			 return;
		}
		CVector2 robotPos = getRobotPosition();
		CRadians robotHeading = getRobotHeading();

		//Target-centered angles
		CVector2 vE = enemyPos - enemyGoal;
		CVector2 vR = robotPos - enemyGoal;
		CRadians angE = vE.Angle();
		CRadians angR = vR.Angle();
		// angle difference
		CRadians deltaAng = (angE - angR).SignedNormalize();
		float speed = ((deltaAng * isRobotCW).GetValue() > 0)? 0.12f : -0.12f;
		m_pcWheels->SetLinearVelocity(speed , speed);
	}
	// turn to defence position function
	bool Controller1::turn4defence(CVector2 enemyBasePos){
		CVector2 robotPos = getRobotPosition();
		CRadians robotHeading = getRobotHeading();
		// vector to target defence position
		CVector2 toTarget =  enemyBasePos - robotPos;
		Real headingErrPlus = CRadians(toTarget.Angle() + CRadians::PI_OVER_TWO - robotHeading).SignedNormalize().GetAbsoluteValue();
		Real headingErrMinus = CRadians(toTarget.Angle() - CRadians::PI_OVER_TWO - robotHeading).SignedNormalize().GetAbsoluteValue();
		// decide direction
		isRobotCW = (headingErrPlus < headingErrMinus)? -1 : 1;
		if(headingErrPlus < headingErrMinus) {
			isRobotCW = -1;
			if(headingErrPlus < 0.1) return true;
			m_pcWheels->SetLinearVelocity(-0.01, 0.01);
		}
		else {
			isRobotCW = +1;
			if(headingErrMinus < 0.1) return true;
			m_pcWheels->SetLinearVelocity(0.01, -0.01);
		}

		return false;
	}




	


	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}