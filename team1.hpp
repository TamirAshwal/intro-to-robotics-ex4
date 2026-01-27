// 209374867
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
            STATE_BLOCK
         };
         CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings m_Readings;
         Estate eState;
      // parameters

      // variables for stuck detection and mechanism
      CVector2 m_lastStuckPosition;     
      UInt32 m_stuckTimer;             
      UInt32 m_unstickStepCounter;      
      bool m_isUnsticking;             
      const UInt32 STUCK_THRESHOLD_TIME = 50;  
      const Real STUCK_THRESHOLD_DIST = 0.02f; 
      UInt32 m_proximityStuckTimer;

      // variables for random walk and explore state
      UInt32 walkCounter;
      UInt32 currMovementDuration;
		Real leftWheelSpeed;
		Real rightWheelSpeed;
      
      bool hasFood;
      bool blockMode;
      CVector2 enemyBasePos;
      //blocking effort
      bool enemyBaseKnown;
      bool followingEnemy;
      CVector2 enemyWithFoodPos;
      int foodLostCounter;
      // return to base variables 
      bool m_bFacingBase = false;        
      CRadians m_cReturnLineHeading;
      // interception specific variables (FROM FILE 1)
      bool isRobotCW; // Clockwise direction flag for blocking
      float lastAngleToTarget;
      float filterdError;
      float interception_alpha;
      float interception_K;

      // helper functions
      bool isObstacleAhead() const;
      CVector2 getRobotPosition() const;
      CRadians getRobotHeading() const;
      void chooseCurrMovement();
      bool isFoodVisible() const;
      CVector2 getBasePosition() const;
      CVector2 relToAbsPosition(const CVector2 blob) const;
      void enemyBlocking(CVector2 enemyPos, CVector2 enemyGoal);
      bool turn4defence(CVector2 enemyBasePos);
      std::vector<CVector2> getFoodPositions() const;
      CRadians angle2TeamAhead() const;
      //blocking effort
      std::vector<CVector2> enemyWithFoodPossitions() const;
      void GoToEnemyBaseAndCamp();
      void StoreEnemyBasePosition();
      void followEnemy(const CRadians& relativeAngle);
      bool isTeammateAhead() const;
   
      

      // state functions
      void randomWalk();
      void returnToBase();

      void moveToFood2();
      void avoidObstacle2();
      void block();

   };
}