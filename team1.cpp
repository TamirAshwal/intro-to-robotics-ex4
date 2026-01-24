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
				else if(!enemyWithFoodPossitions().empty()){
					eState = STATE_BLOCK;
					std::cout << "enemy with Food detected! Switching to Block state" << std::endl;
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
			
		}
		else if(movement < 0.68f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = -0.05f;
			rightWheelSpeed = 0.05f;
			
		}
		else if(movement < 0.86f){
			currMovementDuration = 5 +  m_rng->Uniform(CRange<UInt32>(0,5));
			leftWheelSpeed = 0.05f;
			rightWheelSpeed = -0.05f;
			
		}
		else{
			currMovementDuration = 40 +  m_rng->Uniform(CRange<UInt32>(0,40));
			Real fSpeedRatio = m_rng->Uniform(CRange<Real>(0.85, 0.95));
			if(m_rng->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
				// מעגל שמאלה
				leftWheelSpeed = 0.157f * fSpeedRatio;
				rightWheelSpeed = 0.157f;
				
			} else {
				// מעגל ימינה
				leftWheelSpeed = 0.157f;
				rightWheelSpeed = 0.157f * fSpeedRatio;
				
			}
		}
	}
	
	void Controller1::returnToBase() {
    // שלב 1: בדיקת הגעה לבסיס (ללא שינוי)
    CVector2 currPos = getRobotPosition();
    CVector2 basePos = getBasePosition();
    Real distanceToBase = (currPos - basePos).Length();

    if(distanceToBase < 0.10f){ // הגדלתי טיפה ל-0.10 שיהיה קל יותר לפגוע
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        hasFood = false;
        m_carriedFoodId = std::nullopt;
        std::cout << "Dropped food at base! Back to Explore." << std::endl;
        eState = STATE_EXPLORE;
        walkCounter = 0;
        chooseCurrMovement();
        return;
    }

    // --- שלב א': סיבוב במקום ---
    if (!m_bFacingBase) {
        
        // חישוב וקטור לבסיס
        CVector2 toBase = basePos - currPos;
        
        // חישוב הזווית הרצויה והנוכחית
        CRadians targetHeading = toBase.Angle();
        CRadians currentHeading = getRobotHeading();
        
        // תיקון חשוב: חיסור רגיל ואז SignedNormalize לתוצאה
        // זה מבטיח שנקבל זווית בין -PI ל-PI (הדרך הקצרה)
        CRadians error = (targetHeading - currentHeading).SignedNormalize();
        
        // הגדרת סף דיוק (קצת יותר סלחני)
        const Real epsilon = 0.05f; // כ-3 מעלות

        if (Abs(error.GetValue()) > epsilon) {
            std::cout << "readjust the heading... Error: " << error.GetValue() << std::endl;
            
            // --- הפתרון לבעיית הרעידות ---
            // במקום מהירות קבועה, המהירות תלויה בגודל השגיאה.
            // ככל שמתקרבים - המהירות יורדת.
            Real speed = error.GetValue() * 0.5f; 
            
            // הגבלת מהירות מינימלית ומקסימלית כדי שלא ייתקע או ישתולל
            if(speed > 0.2f) speed = 0.2f;
            if(speed < -0.2f) speed = -0.2f;
            // אם המהירות ממש נמוכה אבל עדיין לא הגענו, תן דחיפה קטנה
            if(speed > 0.0f && speed < 0.02f) speed = 0.02f;
            if(speed < 0.0f && speed > -0.02f) speed = -0.02f;

            // סיבוב במקום (גלגלים הפוכים)
            m_pcWheels->SetLinearVelocity(-speed, speed);
        } 
        else {
            // הגענו לזווית!
            m_bFacingBase = true;
            m_cReturnLineHeading = targetHeading; // שמירת ה-M-Line
            
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f); // עצירה מוחלטת לפני תחילת תנועה
            std::cout << "Aligned with base! Starting M-Line." << std::endl;
        }
        return;
    }

    // --- שלב ב': נסיעה על ה-M-Line ---
    else {
        CRadians currHeading = getRobotHeading();
        
        // חישוב שגיאה ביחס לקו המקורי ששמרנו
        CRadians error = (m_cReturnLineHeading - currHeading).SignedNormalize();
        
        // כאן משתמשים בתיקון עדין תוך כדי נסיעה קדימה (כמו ב-Bug2)
        const Real K_P = 0.5f; // מקדם תיקון
        Real turnCorrection = error.GetValue() * K_P;
        
        // מהירות בסיס קדימה + תיקון
        m_pcWheels->SetLinearVelocity(0.15f - turnCorrection, 0.15f + turnCorrection);
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
      return cHeading;
   }

   //Raz's addition
   //new!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   //returns all the positions of all the food that is not taken or about to be taken
	std::vector<CVector2> Controller1::getFoodPositions() const{
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

	CRadians Controller1::angle2TeamAhead() const{
		for(const auto& blob : m_Readings.BlobList){
			if(blob->Color == CColor::BLUE && blob->Angle.SignedNormalize() < CRadians::PI/6){
				return blob->Angle.SignedNormalize();
			}
		}

		return CRadians::PI;
	}
	bool Controller1::isTeammateAhead() const {
    for(const auto& blob : m_Readings.BlobList) {
        if(blob->Color == CColor::BLUE) {  // כחול = teammate
            CRadians angle = blob->Angle.SignedNormalize();
            // בדיקה אם הרובוט מלפנים (בטווח 30 מעלות) וקרוב
            if(Abs(angle.GetValue()) < CRadians::PI_OVER_SIX.GetValue() && 
               blob->Distance < 50.0f) {  // 50 ס"מ במערכת המצלמה
                return true;
            }
        }
    }
    return false;
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
		CRadians angle2Team = angle2TeamAhead();
		if(angle2Team != CRadians::PI){ //only checks if angel2Team exsistes, it will never be pi
			if (hasFood){
				return;
			}
			else{
				m_pcWheels->SetLinearVelocity(0.145f - 0.01f * angle2Team.GetValue(), 0.145f + 0.01f * angle2Team.GetValue()); 
			}
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
	//blocking effort
	std::vector<CVector2> Controller1::enemyWithFoodPossitions() const {
		// std::vector<CVector2> positions;

		// for(const auto& enemyblob : m_Readings.BlobList) {
		// 	if(enemyblob->Color == CColor::RED)
		// 		for(const auto& foodblob : m_Readings.BlobList) {
		// 			if(foodblob->Color == CColor::GRAY80)
		// 				positions.emplace_back(enemyblob->Distance, enemyblob->Angle);
		// 		}
		// }
		// return positions;

		std::vector<CVector2> positions;

		for(const auto& enemy : m_Readings.BlobList) {
			if(enemy->Color != CColor::RED) continue;

			for(const auto& food : m_Readings.BlobList) {
				if(food->Color != CColor::GRAY80) continue;

				CRadians dAngle =(enemy->Angle - food->Angle).UnsignedNormalize();

				if(dAngle < CRadians::PI / 12) {
					positions.emplace_back(enemy->Distance, enemy->Angle);
					break;
				}
			}
		}
		return positions;
	}

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

	void Controller1::StoreEnemyBasePosition() {
		enemyBasePos = enemyWithFoodPos;
		enemyBaseKnown = true;

		std::cout << "Enemy base discovered at "
				<< enemyBasePos << std::endl;
	}

	void Controller1::GoToEnemyBaseAndCamp() {
		const CVector3& pos3D = m_pcPositioning->GetReading().Position;
		CVector2 robotPos(pos3D.GetX(), pos3D.GetY());

		CVector2 toBase = enemyBasePos - robotPos;
		CRadians targetAngle = toBase.Angle();

		CRadians heading, pitch, roll;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(heading, pitch, roll);

		CRadians diff = (targetAngle - heading).SignedNormalize();

		if(toBase.Length() < 0.15f) {
			m_pcWheels->SetLinearVelocity(0.0f, 0.0f); // CAMP
			return;
		}

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
	

	void Controller1::block() {

		if(enemyBaseKnown) {
			GoToEnemyBaseAndCamp();
			return;
		}

		if(enemyWithFoodPossitions().empty()) {
			for(const auto& enemyblob : m_Readings.BlobList) {
				if(enemyblob->Color == CColor::RED) {
					if((CVector2(enemyblob->Distance, enemyblob->Angle) - enemyWithFoodPos).Length() < 0.5f){
					// if(foodLostCounter >= 5){
						std::cout << "found emeny base!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
						StoreEnemyBasePosition();
						followingEnemy = false;
						GoToEnemyBaseAndCamp();
					}
					break;
				}
			}
			followingEnemy = false;
			// foodLostCounter = 0;
			eState = STATE_EXPLORE;
			return;

		// 	if(enemyStillVisible) {
        //     foodLostCounter++;
        // } else {
        //     foodLostCounter = 0;
        // }

        // if(foodLostCounter >= FOOD_LOST_CONFIRM_STEPS) {
        //     std::cout << "Enemy base detected!" << std::endl;
        //     StoreEnemyBasePosition();
        //     GoToEnemyBaseAndCamp();
        //     return;
        // }
		}

		Real minDistance = std::numeric_limits<Real>::max();
		CRadians relativeAngle;
		
		std::vector<CVector2> enemyPositions = enemyWithFoodPossitions();
		followingEnemy = true;
		for(const auto& enemy : enemyPositions){
			if(enemy.Length() < minDistance){
				minDistance = enemy.Length();
				relativeAngle = enemy.Angle().SignedNormalize();
			}
		}
		enemyWithFoodPos = CVector2(minDistance, relativeAngle);
		followEnemy(relativeAngle);
	}

	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}