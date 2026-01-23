#include "foraging.hpp"

namespace argos {

   class Controller1 : public ForagingController {

   public:

      Controller1() {}
      virtual ~Controller1() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

      uint8_t getTeamId() const override { return 1; }
      private:
      /* state enum*/
      enum Estate{
         STATE_EXPLORE,
         STATE_AVOID_OBSTACLE,
         STATE_TO_FOOD,
         STATE_RETURN_TO_BASE,
         //Raz's addition
         STATE_BLOCK
      };
      CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings m_Readings;
      Estate eState;
      // parameters
      
      UInt32 walkCounter;
      UInt32 currMovementDuration;
		Real leftWheelSpeed;
		Real rightWheelSpeed;
      bool hasFood;
      //Raz's addition
      bool blockMode;
      CVector2 enemyBasePos;
      //blocking effort
      bool enemyBaseKnown;
      bool followingEnemy;
      CVector2 enemyWithFoodPos;
      int foodLostCounter;
         // Unstuck mechanism variables
   


      // helper functions
      bool isObstacleAhead() const;
      CVector2 getRobotPosition() const;
      CRadians getRobotHeading() const;
      void chooseCurrMovement();
      bool isFoodVisible() const;
      CVector2 getBasePosition() const;
      //Raz's addition
      std::vector<CVector2> getFoodPositions() const;
      CRadians angle2TeamAhead() const;
      //blocking effort
      std::vector<CVector2> enemyWithFoodPossitions() const;
      void GoToEnemyBaseAndCamp();
      void StoreEnemyBasePosition();
      void followEnemy(const CRadians& relativeAngle);
      bool isTeammateAhead() const;
   
CVector2 calculateRepulsiveForce() const;

      // state functions
      void randomWalk();
      void avoidObstacle();
      void moveToFood();
      void returnToBase();
      //Raz's addition
      void moveToFood2();
      void avoidObstacle2();
      void block();

   };
}