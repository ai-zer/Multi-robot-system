#include <webots/Robot.hpp>
#include <iostream>
#include <webots/Emitter.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <cmath>
#include <vector>
#include <webots/Lidar.hpp>

#define Time_step 96
#define r 0.021   //here 'r' is wheel's radius
#define L 0.1054 //here 'L' is distance between two wheels
using namespace webots;

// y coord is always zero
float start[]={0,0};
float goal[]={-1.5,1.5};
float goal_thresh=0.1;
float epsilon=0.1;             // potential force constant
float v_limit=0.08 ; // in m/s  //linear velocity limit of the robot
float k=0.2;  // 
float obs_thresh= 0.55; 
float distance(float arr1[],float arr2[])
{
return sqrt(pow(arr1[0]-arr2[0],2)+pow(arr1[1]-arr2[1],2));
}
float *attractive_force(float curr[],float goal[])
{
static float temp[2];
temp[0]= -1*epsilon*(curr[0]-goal[0]);
temp[1]= -1*epsilon*(curr[1]-goal[1]);
return temp;
}
float *repulsive_force(float curr[],float obs[],float dist)
{
float term=epsilon*pow((1.0/dist)-(1.0/obs_thresh),2)*(1.0/pow(dist,2));
static float temp2[2];
temp2[0]=term*((curr[0]-obs[0])/abs(curr[0]-obs[0]));
temp2[1]=term*((curr[1]-obs[1])/abs(curr[1]-obs[1]));
return temp2;
}
void find_net_obs_vec(float sensor_dist[],float curr[],float theta,float vec[])
{
float temp[2];
float *p;
if (sensor_dist[0]<=obs_thresh)
{
temp[0]=sensor_dist[0]*cos(theta)+curr[0];
temp[1]=sensor_dist[0]*cos(theta)+curr[1];
p=repulsive_force(curr,temp,sensor_dist[0]);
vec[0]=vec[0]+ *p;
vec[1]=vec[1]+ *(p+1);
}
if (sensor_dist[1]<=obs_thresh)
{
temp[0]=sensor_dist[1]*cos(theta-M_PI/4.0)+(curr[0]+0.5);
temp[1]=sensor_dist[1]*cos(theta-M_PI/4.0)+(curr[1]+0.5);
p=repulsive_force(curr,temp,sensor_dist[1]);
vec[0]=vec[0]+ *p;
vec[1]=vec[1]+ *(p+1);
}
if (sensor_dist[2]<=obs_thresh)
{
temp[0]=sensor_dist[2]*cos(theta+M_PI/4.0)+(curr[0]-0.5);
temp[1]=sensor_dist[2]*cos(theta+M_PI/4.0)+(curr[1]+0.5);
p=repulsive_force(curr,temp,sensor_dist[2]);
vec[0]=vec[0]+ *p;
vec[1]=vec[1]+ *(p+1);
}
}

int main() {
Robot *robot=new Robot();
PositionSensor *pos_r=robot->getPositionSensor("right wheel sensor");
PositionSensor *pos_l=robot->getPositionSensor("left wheel sensor");
// min from ultrasonic sensor is 0.25, max is 2
//angular distance between sensors is 45degree
DistanceSensor *front_sensor=robot->getDistanceSensor("Sharp's IR sensor GP2Y0A02YK0F");
DistanceSensor *frontr_sensor=robot->getDistanceSensor("Sharp's IR sensor GP2Y0A02YK0F(1)");
DistanceSensor *frontl_sensor=robot->getDistanceSensor("Sharp's IR sensor GP2Y0A02YK0F(2)");

front_sensor->enable(Time_step);
frontr_sensor->enable(Time_step);
frontl_sensor->enable(Time_step);
pos_r->enable(Time_step);
pos_l->enable(Time_step);
Motor *motor_l=robot->getMotor("left wheel motor");
Motor *motor_r=robot->getMotor("right wheel motor");
float revc_r=0;
float revc_l=0;
float revp_r=0;
float revp_l=0;
float sensor_dist[3];
float omega_r;
float omega_l;
float omega_b;
float v=0;
float theta= M_PI/2.0;
float vec_overall[2];
float omega_des;
float theta_des;
float curr[]={0,0};
float *p; 
for (int i=0;i<2;i++)
curr[i]=start[i];
while(robot->step(Time_step)!=-1 && distance(curr,goal)>=goal_thresh)
{
revc_r=pos_r->getValue();
revc_l=pos_l->getValue();
sensor_dist[0]=front_sensor->getValue();
sensor_dist[0]=0.7611*pow(sensor_dist[0],-0.9313)+0.03;
sensor_dist[1]=frontr_sensor->getValue();
sensor_dist[1]=0.7611*pow(sensor_dist[1],-0.9313)+0.03;
sensor_dist[2]=frontl_sensor->getValue();
sensor_dist[2]=0.7611*pow(sensor_dist[2],-0.9313)+0.03;
//std::cout<<"front sensor value is "<<sensor_dist[0]<<std::endl;
//std::cout<<front_sensor->getMinValue();
omega_r=(revc_r-revp_r)/(Time_step/1000.0);
omega_l=(revc_l-revp_l)/(Time_step/1000.0);
//std::cout<<"omega_l and omega_r from encoders are: "<<omega_r<<"and "<<omega_l<<std::endl;
revp_r=revc_r;
revp_l=revc_l;
v=(r/2.0)*(omega_r+omega_l);
//std::cout<<v<<std::endl;

curr[0]=curr[0] + v*cos(theta)*(Time_step/1000.0);
curr[1]=curr[1] +  v*sin(theta)*(Time_step/1000.0);
std::cout<<"current x and y are : "<<curr[0]<<" and "<<curr[1]<<std::endl;
//std::cout<<theta<<std::endl;
omega_b=(r/L)*(omega_r-omega_l);    //original eqn=(r/L)*(omega_r-omega_l)
//std::cout<<"body omega from encoder is  "<<omega_b<<std::endl;
theta= theta + omega_b*(Time_step/1000.0);

if (theta > M_PI)
{theta=theta- 2*M_PI;}
if (theta < -M_PI)
{theta=theta+2*M_PI;}

p=attractive_force(curr,goal);
vec_overall[0]= *p;
vec_overall[1]= *(p+1); 
//std::cout<<" before  is "<<vec_overall[0]<<" and" <<vec_overall[1]<<std::endl;
find_net_obs_vec(sensor_dist,curr,theta,vec_overall);
//std::cout<<" after is "<<vec_overall[0]<<" and" <<vec_overall[1]<<std::endl;
theta_des= atan2(vec_overall[1],vec_overall[0]);


omega_des=(theta_des-theta)*k;
//std::cout<<"omega required"<<omega_des<<std::endl;
omega_r= (v_limit + (L/2)*omega_des)/r; //original eqn: omega_r= (v_limit + (L/2)*omega_des)/r
omega_l= (v_limit - (L/2)*omega_des)/r; // original eqn: omega_l= (v_limit - (L/2)*omega_des)/r;
//std::cout<<"omega_l and omega_r from APF are: "<<omega_r<<"and "<<omega_l<<std::endl;
motor_l->setPosition(INFINITY);
motor_l->setVelocity(omega_l);
motor_r->setPosition(INFINITY);
motor_r->setVelocity(omega_r);

}
while(robot->step(Time_step)!=-1)
{
motor_l->setPosition(INFINITY);
motor_l->setVelocity(0);
motor_r->setPosition(INFINITY);
motor_r->setVelocity(0);
} 
}

