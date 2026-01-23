//Author: Raz Yacov Chai ID:327821484
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos {

   class ControllerBug2 : public CCI_Controller {

   public:

      ControllerBug2() {}

      virtual ~ControllerBug2() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

   private:
      
      /* Sensors and Actuators */
      CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
      CCI_PositioningSensor* m_pcPositioning = nullptr;
      CVector3 m_cTargetPosition;

      //robot states
      enum class EState {GO2GOUL, Circumnavigate};
      EState m_state;

      //thresholds for minimal angle and position diffrece to prevent robot from being to sensative
      CRadians THRESHOLD_RAD = CRadians(0.1);
      Real THRESHOLD_POS = 0.1;

      //decler the varubales that will store the arial vector and angle to the goul
      CVector3 airPath;
      CRadians airAngle;
      
      //fleg indicating wether the robot already took a turn
      bool turned;
      
      //decler my funtions
      bool IsObstacleAhead();                      //function for detecting obsticales in front of the robot
      bool IsObstacleDetected(int sensor_index);   //function for detecting obsticalse by sensor index
   };
}
