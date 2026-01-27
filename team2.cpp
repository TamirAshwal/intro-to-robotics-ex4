// 209374867 327821484
#include "team2.hpp"
#include <iostream>
#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <optional>
#include <functional>

namespace argos {

    /****************************************/
    /****************************************/

    void controller2::Init(TConfigurationNode& t_tree) {
        ForagingController::Init(t_tree);
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
        blockMode = false;
    }

    void controller2::ControlStep() {
        /* Your ControlStep code goes here*/
        m_Readings = m_pcCamera->GetReadings();
        bool shouldCheckStuck = !enemyBaseKnown;

        // Unstuck mechanism
        if (shouldCheckStuck) {
            if (m_isUnsticking) {
                m_unstickStepCounter++;
                // we are stuck so first we reverse for 20 steps
                if (m_unstickStepCounter < 20) {
                    m_pcWheels->SetLinearVelocity(-0.1f, -0.1f);
                }
                // then we turn for 30 steps
                else if (m_unstickStepCounter < 50) {
                    m_pcWheels->SetLinearVelocity(0.02f, -0.02f);
                }
                // after the stuck maneuver is done which is after 50 steps we reset the variables and return to previous state
                else {
                    m_isUnsticking = false;
                    m_stuckTimer = 0;
                    m_proximityStuckTimer = 0;
                    m_lastStuckPosition = getRobotPosition();
                    
                    // super important check to see if we were in the middle of returning to base
                    if (hasFood) {
                        std::cout << "Unstuck with food! Recalculating path to base." << std::endl;
                        eState = STATE_RETURN_TO_BASE;
                        m_bFacingBase = false;
                    } else {
                        // if we don't have food we just continue with exploring
                        eState = STATE_EXPLORE;
                        chooseCurrMovement();
                    }
                }
                return;
            }

            // this is after the first if statement meaning we are not in the middle of the maneuver
            bool isStuck = false;
            bool robotClose = false;
            
            // check if there are robots which can be teammates or enemies really close to us
            for (const auto& blob : m_Readings.BlobList) {
                if (blob->Color == CColor::BLUE || blob->Color == CColor::RED) {
                    if (blob->Distance < 15.0f && Abs(blob->Angle.GetValue()) < 0.5f) {
                        robotClose = true;
                        break;
                    }
                }
            }

            // we update the proximityStuckTimer according to if there are robots close to us
            if (robotClose) {
                m_proximityStuckTimer++;
            } else {
                if (m_proximityStuckTimer > 0) m_proximityStuckTimer--;
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
            } else {
                m_stuckTimer = 0;
                m_lastStuckPosition = currentPos;
            }

            // if we are stuck for some time make the flag true
            if (m_stuckTimer > STUCK_THRESHOLD_TIME) {
                std::cout << "POSITION STUCK DETECTED!" << std::endl;
                isStuck = true;
            }

            // we are stuck so we make sure we start the maneuver
            if (isStuck) {
                m_isUnsticking = true;
                m_unstickStepCounter = 0;
                return;
            }
        } // End of shouldCheckStuck logic

        // start of switch case
        switch (eState) {
            case STATE_EXPLORE: {
                if (isObstacleAhead()) {
                    eState = STATE_AVOID_OBSTACLE;
                }
                // else if(!enemyWithFoodPossitions().empty()){
                // 	eState = STATE_BLOCK;
                // 	std::cout << "enemy with Food detected! Switching to Block state" << std::endl;
                // }
                else if (!getFoodPositions().empty() && !hasFood) {
                    eState = STATE_TO_FOOD;
                    std::cout << "Food detected! Switching to TO_FOOD state" << std::endl;
                } else {
                    randomWalk();
                }
                break;
            }
            case STATE_AVOID_OBSTACLE: {
                avoidObstacle2();
                break;
            }
            case STATE_TO_FOOD: {
                moveToFood2();
                break;
            }
            case STATE_RETURN_TO_BASE: {
                returnToBase();
                break;
            }
            case STATE_BLOCK: {
                block();
                break;
            }
        }
        // end of control step
    }

    // state functions
    void controller2::randomWalk() {
        walkCounter++;
        // check if we need to choose a new movement
        if (walkCounter >= currMovementDuration) {
            walkCounter = 0;
            chooseCurrMovement();
        }
        m_pcWheels->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);
    }

    // function that choose the next movement randomly
    void controller2::chooseCurrMovement() {
        Real movement = m_rng->Uniform(CRange<Real>(0.0f, 1.0f));
        // going straight
        if (movement < 0.50f) {
            currMovementDuration = 50 + m_rng->Uniform(CRange<UInt32>(0, 50));
            leftWheelSpeed = 0.157f;
            rightWheelSpeed = 0.157f;
        }
        // rotating left in place
        else if (movement < 0.68f) {
            currMovementDuration = 5 + m_rng->Uniform(CRange<UInt32>(0, 5));
            leftWheelSpeed = -0.05f;
            rightWheelSpeed = 0.05f;
        }
        // rotating right in place
        else if (movement < 0.86f) {
            currMovementDuration = 5 + m_rng->Uniform(CRange<UInt32>(0, 5));
            leftWheelSpeed = 0.05f;
            rightWheelSpeed = -0.05f;
        }
        // moving in a circular path
        else {
            currMovementDuration = 40 + m_rng->Uniform(CRange<UInt32>(0, 40));
            Real fSpeedRatio = m_rng->Uniform(CRange<Real>(0.85, 0.95));
            if (m_rng->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
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

    void controller2::returnToBase() {
        // firstly check if we got to the base
        CVector2 currPos = getRobotPosition();
        CVector2 basePos = getBasePosition();
        Real distanceToBase = (currPos - basePos).Length();
        
        // if so drop the food return to explore and update the flags
        if (distanceToBase < 0.10f) {
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
                if (speed > 0.2f) speed = 0.2f;
                if (speed < -0.2f) speed = -0.2f;
                if (speed > 0.0f && speed < 0.02f) speed = 0.02f;
                if (speed < 0.0f && speed > -0.02f) speed = -0.02f;
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

    bool controller2::isFoodVisible() const {
        for (const auto& blob : m_Readings.BlobList) {
            if (blob->Color == CColor::GRAY80) {
                return true;
            }
        }
        return false;
    }

    CVector2 controller2::getBasePosition() const {
        if (m_basePositions.empty()) {
            std::cout << " base positions not initialized!" << std::endl;
            return CVector2(0.0f, 0.0f);
        } else {
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

    CVector2 controller2::getRobotPosition() const {
        const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
        return CVector2(reading.Position.GetX(), reading.Position.GetY());
    }

    CRadians controller2::getRobotHeading() const {
        CRadians cHeading, cPitch, cRoll;
        const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
        reading.Orientation.ToEulerAngles(cHeading, cPitch, cRoll);
        return cHeading;
    }

    // Raz's addition
    // returns all the positions of all the food that is not taken or about to be taken
    std::vector<CVector2> controller2::getFoodPositions() const {
        std::vector<CVector2> foodPositions;

        for (const auto& foodblob : m_Readings.BlobList) {
            if (foodblob->Color == CColor::GRAY80) {
                bool taken = false;

                // 1. Vector from me to food
                CVector2 vMeToFood;
                vMeToFood.FromPolarCoordinates(foodblob->Distance, foodblob->Angle);

                for (const auto& robotblob : m_Readings.BlobList) {
                    // Check if it is a robot (not gray)
                    if (robotblob->Color != CColor::GRAY80) {
                        
                        // 2. Vector from me to other robot
                        CVector2 vMeToRobot;
                        vMeToRobot.FromPolarCoordinates(robotblob->Distance, robotblob->Angle);

                        // 3. Vector from other robot to food
                        CVector2 vRobotToFood = vMeToFood - vMeToRobot;

                        // 4. Compare distances
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

    CRadians controller2::angle2TeamAhead() const {
        for (const auto& blob : m_Readings.BlobList) {
            if (blob->Color == CColor::BLUE && blob->Angle.SignedNormalize() < CRadians::PI / 6) {
                return blob->Angle.SignedNormalize();
            }
        }
        return CRadians::PI;
    }

    bool controller2::isTeammateAhead() const {
        for (const auto& blob : m_Readings.BlobList) {
            if (blob->Color == CColor::BLUE) { // Teammate
                CRadians angle = blob->Angle.SignedNormalize();
                if (Abs(angle.GetValue()) < CRadians::PI_OVER_SIX.GetValue() && blob->Distance < 50.0f) {
                    return true;
                }
            }
        }
        return false;
    }

    // improved move to food that don't follow robots
    void controller2::moveToFood2() {
        Real minDistance = std::numeric_limits<Real>::max();
        CRadians relativeAngle;

        std::vector<CVector2> foodPositions = getFoodPositions();
        for (const auto& food : foodPositions) {
            if (food.Length() < minDistance) {
                minDistance = food.Length();
                relativeAngle = food.Angle().SignedNormalize();
            }
        }

        if (minDistance >= std::numeric_limits<Real>::max()) {
            std::cout << "Lost sight of food, returning to explore" << std::endl;
            eState = STATE_EXPLORE;
            walkCounter = 0;
            chooseCurrMovement();
            return;
        }
        // check if we got to the food
        if (minDistance < 0.15f) {
            std::cout << "Reached food! Returning to base" << std::endl;
            hasFood = true;
            eState = STATE_RETURN_TO_BASE;
            m_bFacingBase = false;
            return;
        }

        // move the robot towards the food
        if (relativeAngle.GetAbsoluteValue() < CRadians::PI_OVER_FOUR.GetValue()) {
            m_pcWheels->SetLinearVelocity(0.157f, 0.157f);
        } else if (relativeAngle.GetValue() > 0) {
            m_pcWheels->SetLinearVelocity(-0.157f, 0.157f);
        } else {
            m_pcWheels->SetLinearVelocity(0.157f, -0.157f);
        }
    }

    // improved avoidObstacle that clears the path for robots with food
    void controller2::avoidObstacle2() {
        m_pcWheels->SetLinearVelocity(-0.08f, 0.08f); // Spin in place
        if (!isObstacleAhead()) { // If clear
            eState = STATE_EXPLORE; // Back to explore
            walkCounter = 0;
            chooseCurrMovement();
        }
    }

    // blocking effort
    std::vector<CVector2> controller2::enemyWithFoodPossitions() const {
        std::vector<CVector2> positions;

        for (const auto& enemy : m_Readings.BlobList) {
            if (enemy->Color == CColor::GRAY80 || enemy->Color == m_teamColor) continue;
            
            // Note: The original code had a commented out if(RED) with a hanging brace.
            // I removed the extra brace here.
            
            for (const auto& food : m_Readings.BlobList) {
                if (food->Color != CColor::GRAY80) continue;
                if ((enemy->Angle - food->Angle).UnsignedNormalize() < CRadians::PI / 12) {
                    positions.emplace_back(enemy->Distance, enemy->Angle);
                    break;
                }
            }
        }
        return positions;
    }

    void controller2::followEnemy(const CRadians& relativeAngle) {
        // move the robot towards the enemy
        if (relativeAngle.GetAbsoluteValue() < CRadians::PI_OVER_FOUR.GetValue()) {
            m_pcWheels->SetLinearVelocity(0.157f, 0.157f);
        } else if (relativeAngle.GetValue() > 0) {
            m_pcWheels->SetLinearVelocity(-0.157f, 0.157f);
        } else {
            m_pcWheels->SetLinearVelocity(0.157f, -0.157f);
        }
    }

    void controller2::StoreEnemyBasePosition() {
        enemyBasePos = enemyWithFoodPos;
        enemyBaseKnown = true;
        std::cout << "Enemy base discovered at " << enemyBasePos << std::endl;
    }

    void controller2::GoToEnemyBaseAndCamp() {
        const CVector3& pos3D = m_pcPositioning->GetReading().Position;
        CVector2 robotPos(pos3D.GetX(), pos3D.GetY());

        CVector2 toBase = enemyBasePos - robotPos;
        CRadians targetAngle = toBase.Angle();

        CRadians heading, pitch, roll;
        m_pcPositioning->GetReading().Orientation.ToEulerAngles(heading, pitch, roll);

        CRadians diff = (targetAngle - heading).SignedNormalize();

        if (toBase.Length() < 0.15f) {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f); // CAMP
            return;
        }

        if (Abs(diff) < CRadians::PI_OVER_SIX) {
            m_pcWheels->SetLinearVelocity(0.15f, 0.15f);
        } else if (diff.GetValue() > 0) {
            m_pcWheels->SetLinearVelocity(-0.1f, 0.1f);
        } else {
            m_pcWheels->SetLinearVelocity(0.1f, -0.1f);
        }
    }

    void controller2::block() {
        Real minDistance = std::numeric_limits<Real>::max();
        CRadians relativeAngle;

        std::vector<CVector2> enemyPositions = enemyWithFoodPossitions();

        // Case 1: We know where the enemy base is, but see no enemies -> Guard the base
        if (enemyBaseKnown && enemyPositions.empty()) {
            turn4defence(enemyBasePos);
            return;
        }

        // Case 2: We were following an enemy, lost them, and now need to see if we found their base
        if (enemyPositions.empty() && followingEnemy) {
            for (const auto& enemy : m_Readings.BlobList) {
                // Check all non-food, non-team blobs
                if (enemy->Color != CColor::GRAY80 && enemy->Color != m_teamColor) {
                    // Distance check to define "Base Found"
                    if ((relToAbsPosition(CVector2(enemy->Distance / 100.0f, enemy->Angle)) - enemyWithFoodPos).Length() < 0.2f) {
                        enemyBasePos = enemyWithFoodPos;
                        enemyBaseKnown = true;
                        std::cout << "Found Enemy Base!" << std::endl;
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
        for (const auto& enemy : enemyPositions) {
            if (enemy.Length() < minDistance) {
                minDistance = enemy.Length();
                relativeAngle = enemy.Angle();
            }
        }

        // Update predicted enemy position
        enemyWithFoodPos = relToAbsPosition(CVector2(minDistance / 100.0f, relativeAngle));

        if (enemyBaseKnown) {
            turn4defence(enemyBasePos);
            enemyBlocking(enemyWithFoodPos, enemyBasePos);
        } else {
            followingEnemy = true;
            followEnemy(relativeAngle);
        }
    }

    CVector2 controller2::relToAbsPosition(const CVector2 blob) const {
        CVector2 robotPos = getRobotPosition();
        CRadians robotHeading = getRobotHeading();

        CVector2 blobRel(blob.Length() * Cos(blob.Angle()), blob.Length() * Sin(blob.Angle()));

        blobRel.Rotate(robotHeading);

        return robotPos + blobRel;
    }

    void controller2::enemyBlocking(CVector2 enemyPos, CVector2 enemyGoal) {
        CVector2 robotPos = getRobotPosition();
        CRadians robotHeading = getRobotHeading();

        // --- Target-centered angles ---
        CVector2 vE = enemyPos - enemyGoal;
        CVector2 vR = robotPos - enemyGoal;

        CRadians angE = vE.Angle();
        CRadians angR = vR.Angle();

        CRadians deltaAng = (angE - angR).SignedNormalize();

        float speed = ((deltaAng * isRobotCW).GetValue() > 0) ? 1.2f : -1.2f;

        m_pcWheels->SetLinearVelocity(speed, speed);
    }

    void controller2::turn4defence(CVector2 enemyBasePos) {
        CVector2 robotPos = getRobotPosition();
        CRadians robotHeading = getRobotHeading();

        CVector2 toTarget = enemyBasePos - robotPos;
        CRadians headingErrPlus = CRadians(toTarget.Angle() + CRadians::PI_OVER_TWO - robotHeading).SignedNormalize();
        CRadians headingErrMinus = CRadians(toTarget.Angle() - CRadians::PI_OVER_TWO - robotHeading).SignedNormalize();

        if (headingErrPlus < headingErrMinus && headingErrPlus > CRadians(0.1)) {
            m_pcWheels->SetLinearVelocity(-0.1, 0.1);
            isRobotCW = -1;
        } else if (headingErrMinus < headingErrPlus && headingErrMinus > CRadians(0.1)) {
            m_pcWheels->SetLinearVelocity(0.1, -0.1);
            isRobotCW = 1;
        } else {
            m_pcWheels->SetLinearVelocity(0, 0);
            isRobotCW = (headingErrPlus < headingErrMinus) ? -1 : 1;
        }
    }

    /****************************************/
    /****************************************/

    REGISTER_CONTROLLER(controller2, "controller2");

}