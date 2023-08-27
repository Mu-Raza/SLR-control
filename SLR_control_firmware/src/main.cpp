/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
class motor_class{
  private:
    int dir_pin;
    int step_pin; 
  public:
    motor_class(int _dir_pin, int _step_pin){
      pinMode(_dir_pin, OUTPUT);
      pinMode(_step_pin, OUTPUT);
      dir_pin = _dir_pin;
      step_pin = _step_pin;
    }
    void motor_direction(){

    }

};
class robot{
  private:
    float l1;
    float l2;
    float l3;
    float th1_min;
    float th1_max;
    float th2_min;
    float th2_max;
    float th3_min;
    float th3_max;
    float angular_velocity_per_motor;
    float minimum_step_delay;
    int steps_per_revolution = 2048;
  public:
    robot(float _param[])
    {
      l1 = _param[0];
      l2 = _param[1];
      l3 = _param[2];
      th1_min = _param[3];
      th1_max = _param[4];
      th2_min = _param[5];
      th2_max = _param[6];
      th3_min = _param[7];
      th3_max = _param[8];
      angular_velocity_per_motor = _param[9];
      if(angular_velocity_per_motor < 5)
        angular_velocity_per_motor = 5; // rpm
      minimum_step_delay = 60/(angular_velocity_per_motor*steps_per_revolution*2); // 2048 number of steps each revolution ,, 60 seconds ,, 2 transition between high to low ,,  
    }
    bool achieveableable_corrdinated_flag = 0;
    bool trajectory_status = 0;    
    struct robot_orientation_object{
      bool direction_th1;
      bool direction_th2;
      bool direction_th3;
      float actual_angle_th1;
      float actual_angle_th2;
      float actual_angle_th3;
      float desired_angle_th1;
      float desired_angle_th2;
      float desired_angle_th3;
      float difference_angle_th1;
      float difference_angle_th2;
      float difference_angle_th3;
      int steps_m1;
      int steps_m2;
      int steps_m3;
      int step_delay_m1;
      int step_delay_m2;
      int step_delay_m3;
      float x_coordinates;
      float y_coordinates;
      float z_coordinates;
    };
    robot_orientation_object robot_orientation;

    void forward_kinematics(void){
      float c23 = cos(robot_orientation.desired_angle_th2)*cos(robot_orientation.desired_angle_th3) - sin(robot_orientation.desired_angle_th2)*sin(robot_orientation.desired_angle_th3);
      float s23 = sin(robot_orientation.desired_angle_th2)*cos(robot_orientation.desired_angle_th3) + cos(robot_orientation.desired_angle_th2)*sin(robot_orientation.desired_angle_th3);

      robot_orientation.x_coordinates = cos(robot_orientation.desired_angle_th1)*(l2*cos(robot_orientation.desired_angle_th2) + l3*c23);
      robot_orientation.y_coordinates = sin(robot_orientation.desired_angle_th1)*(l2*cos(robot_orientation.desired_angle_th2) + l3*c23);
      robot_orientation.z_coordinates = l1 - l2*sin(robot_orientation.desired_angle_th2) - l3*s23;
    }

    void inverse_kinematics(void){
      float th1 = atan2(robot_orientation.y_coordinates,robot_orientation.x_coordinates);
      if(th1<th1_max & th1>th1_min)
      {
        robot_orientation.desired_angle_th1 = th1;
        float c3 = (pow(cos(th1)*robot_orientation.x_coordinates + sin(th1)*robot_orientation.y_coordinates,2) + pow(robot_orientation.z_coordinates - pow(l1,2),2) - pow(l2,2) - pow(l3,2))/(2*l2*l3);
        float s3_eu = sqrt(1 - pow(c3,2));
        float s3_ed = -1*sqrt(1 - pow(c3,2));
        float th3_eu = atan2(s3_eu,c3);
        float th3_ed = atan2(s3_ed,c3);
        if(th3_eu<th3_max & th3_eu>th3_min)
        {
          robot_orientation.desired_angle_th1 = th3_eu;
          float c2 = ((cos(th1)*robot_orientation.x_coordinates + sin(th1)*robot_orientation.y_coordinates)*(cos(th3_eu)*l3 + l2) - (robot_orientation.z_coordinates - l1)*sin(th3_eu)*l3)/(pow(cos(th3_eu*l3 + l2),2) + pow(sin(th3_eu)*l3,2));
          float s2 = -1*((cos(th1)*robot_orientation.x_coordinates + sin(th1)*robot_orientation.y_coordinates)*sin(th3_eu)*l3 + (robot_orientation.z_coordinates - l1)*(cos(th3_eu)*l3 + l2))/(pow(cos(th3_eu*l3 + l2),2) + pow(sin(th3_eu)*l3,2));
          robot_orientation.desired_angle_th2 = atan2(s2,c2);
          if(robot_orientation.desired_angle_th2<th2_max & robot_orientation.desired_angle_th2>th2_min)
          {
            achieveableable_corrdinated_flag = 1;
          }
          else
          {
            achieveableable_corrdinated_flag = 0;
          }
        }
        else if (th3_ed<th3_max & th3_ed>th3_min)
        {
          robot_orientation.desired_angle_th1 = th3_ed;
          float c2 = ((cos(th1)*robot_orientation.x_coordinates + sin(th1)*robot_orientation.y_coordinates)*(cos(th3_ed)*l3 + l2) - (robot_orientation.z_coordinates - l1)*sin(th3_ed)*l3)/(pow(cos(th3_ed*l3 + l2),2) + pow(sin(th3_ed)*l3,2));
          float s2 = -1*((cos(th1)*robot_orientation.x_coordinates + sin(th1)*robot_orientation.y_coordinates)*sin(th3_ed)*l3 + (robot_orientation.z_coordinates - l1)*(cos(th3_ed)*l3 + l2))/(pow(cos(th3_ed*l3 + l2),2) + pow(sin(th3_ed)*l3,2));
          robot_orientation.desired_angle_th2 = atan2(s2,c2);
          if(robot_orientation.desired_angle_th2<th2_max & robot_orientation.desired_angle_th2>th2_min)
          {
            achieveableable_corrdinated_flag = 1;
          }
          else
          {
            achieveableable_corrdinated_flag = 0;
          }
        }
        else
        {
          achieveableable_corrdinated_flag = 0;
        }
      }
      else
      {
        achieveableable_corrdinated_flag = 0;
      }
      
    }
    
    void th_to_step(void){
      robot_orientation.difference_angle_th1 = robot_orientation.desired_angle_th1 - robot_orientation.actual_angle_th1;
      robot_orientation.steps_m1 = (int)((abs(robot_orientation.difference_angle_th1)/360)*float(steps_per_revolution));
      if(robot_orientation.difference_angle_th1>0){
        robot_orientation.direction_th1 = 1;
      }
      else{
        robot_orientation.direction_th1 = 0; 
      }
      robot_orientation.difference_angle_th2 = robot_orientation.desired_angle_th2 - robot_orientation.actual_angle_th2;
      robot_orientation.steps_m2 = (int)((abs(robot_orientation.difference_angle_th2)/360)*float(steps_per_revolution));
      if(robot_orientation.difference_angle_th2>0){
        robot_orientation.direction_th2 = 1;
      }
      else{
        robot_orientation.direction_th2 = 0; 
      }
      robot_orientation.difference_angle_th3 = robot_orientation.desired_angle_th3 - robot_orientation.actual_angle_th3;
      robot_orientation.steps_m3 = (int)((abs(robot_orientation.difference_angle_th3)/360)*float(steps_per_revolution));
      if(robot_orientation.difference_angle_th3>0){
        robot_orientation.direction_th3 = 1;
      }
      else{
        robot_orientation.direction_th3 = 0;
      }
    }

    void determine_step_delay()
    {      
      robot_orientation.difference_angle_th1 = abs(robot_orientation.desired_angle_th1 - robot_orientation.actual_angle_th1);
      robot_orientation.difference_angle_th2 = abs(robot_orientation.desired_angle_th2 - robot_orientation.actual_angle_th2);
      robot_orientation.difference_angle_th3 = abs(robot_orientation.desired_angle_th3 - robot_orientation.actual_angle_th3);

      if(robot_orientation.difference_angle_th1 >= robot_orientation.difference_angle_th2 & robot_orientation.difference_angle_th1 >= robot_orientation.difference_angle_th3){
        robot_orientation.step_delay_m1 = (int)minimum_step_delay;
        robot_orientation.step_delay_m2 = (int)((robot_orientation.difference_angle_th1/robot_orientation.difference_angle_th2)*minimum_step_delay);
        robot_orientation.step_delay_m3 = (int)((robot_orientation.difference_angle_th1/robot_orientation.difference_angle_th3)*minimum_step_delay);
      }
      else if (robot_orientation.difference_angle_th2 >= robot_orientation.difference_angle_th1 & robot_orientation.difference_angle_th2 >= robot_orientation.difference_angle_th3){
        robot_orientation.step_delay_m2 = (int)minimum_step_delay;
        robot_orientation.step_delay_m1 = (int)((robot_orientation.difference_angle_th2/robot_orientation.difference_angle_th1)*minimum_step_delay);
        robot_orientation.step_delay_m3 = (int)((robot_orientation.difference_angle_th2/robot_orientation.difference_angle_th3)*minimum_step_delay);
      }else if (robot_orientation.difference_angle_th3 >= robot_orientation.difference_angle_th1 & robot_orientation.difference_angle_th3 >= robot_orientation.difference_angle_th2){
        robot_orientation.step_delay_m3 = (int)minimum_step_delay;
        robot_orientation.step_delay_m1 = (int)((robot_orientation.difference_angle_th3/robot_orientation.difference_angle_th1)*minimum_step_delay);
        robot_orientation.step_delay_m2 = (int)((robot_orientation.difference_angle_th3/robot_orientation.difference_angle_th2)*minimum_step_delay);
      }
      else{
        robot_orientation.step_delay_m1 = (int)minimum_step_delay;
        robot_orientation.step_delay_m2 = (int)((robot_orientation.difference_angle_th1/robot_orientation.difference_angle_th2)*minimum_step_delay);
        robot_orientation.step_delay_m3 = (int)((robot_orientation.difference_angle_th1/robot_orientation.difference_angle_th3)*minimum_step_delay);      
      }
    }
  
    void get_home_position(void)
    {
      robot_orientation.actual_angle_th1 = 0;
      robot_orientation.actual_angle_th2 = 0;
      robot_orientation.actual_angle_th3 = 0;
    }
    void goto_home_position(void)
    {
      robot_orientation.desired_angle_th1 = 0;
      robot_orientation.desired_angle_th2 = 0;
      robot_orientation.desired_angle_th3 = 0;
      determine_step_delay();
      th_to_step();
    }
};


#define LED 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("LED is off");
  delay(1000);
}

int myMultiplyFunction(int x, int y){

  int result;

  result = x * y;

  return result;

}
