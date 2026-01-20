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
         STATE_RETURN_TO_BASE
      };
      CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings m_Readings;
      Estate eState;
      // parameters
      
      UInt32 walkCounter;
      UInt32 currMovementDuration;
		Real leftWheelSpeed;
		Real rightWheelSpeed;
      bool hasFood;
   


      // helper functions
      bool isObstacleAhead() const;
      CVector2 getRobotPosition() const;
      CRadians getRobotHeading() const;
      void chooseCurrMovement();
      bool isFoodVisible() const;
      CVector2 getBasePosition() const;
      
      // state functions
      void randomWalk();
      void avoidObstacle();
      void moveToFood();
      void returnToBase();

   };
}
