#include<iostream>
#include<time.h>
#include<vector>
using namespace std;
const int num_tasks=10; //we assume num_of_robots=num_of_tasks
float utility[num_tasks][num_tasks]={{0.169,0.631,0.389,0.194,0.513,0.955,0.128,0.524,0.861,0.962},{0.762,0,0.147,0.061,0.925,0.344,0.063,0.463,0.638,0.344},{0.967,0.073,0.743,0.519,0.075,0.542,0.893,0.606,0.594,0.491},{0.991,0.093,0.953,0.451,0.497,0.938,0.779,0.550,1,0.976},{0.787,0.472,0.944,0.814,0.234,0.724,0.995,0.477,0.090,0.741},{0.806,0.606,1.000,0.857,0.747,0.465,0.322,0.683,0.400,0.563},{0.486,0.911,0.270,0.155,0.529,0.083,0.598,0.407,0.757,0.352},{0.205,0.517,0.116,0.774,0.746,0.553,0.000,0.148,0.305,0.311},{0.744,0.955,0.719,0.959,0.953,0.403,0.549,0.751,0.342,0.134},{0.095,0.600,0.214,0.606,0.376,0.782,0.260,0.690,0.456,0.561}}; //rows correspond to robot and column correspond to task
const int pop_size=60; // size of population
int max_iterations=500; //The program will run for at max this many iterations.
int obx_size=3; //number of genes directly taken from the teacher in OBX operation
void tlbo(float utility[][num_tasks],const int pop_size,const int num_tasks,int max_iterations,int obx_size);
void evaluation(float utility[][num_tasks],float *pop,const int pop_size,float *eval);
int find_teacher(int pop_size,float eval_pop[]);
void random(float *arr,int vect,int num,int range); 
bool lin_search(float arr[],int size,int element);
void print_array(float *arr,int nrows,int ncolumns);
void gen_newcand(float *pop,float *new_cand,int teacher,int num_tasks,int obx_size,int curr_cand);



int main()
{
tlbo(utility,pop_size,num_tasks,max_iterations,obx_size);
}

void tlbo(float utility[][num_tasks], const int pop_size, const int num_tasks,int max_iterations,int obx_size)
{
 //**POPULATION INITIALIZATION** 
  
 //Encoding: [1,4,6,3,2,7,5,9,8,10] => robot 1 is assigned task 1,robot 2 is assigned task 4,robot 3 is assigned task 6 and so on
 srand(time(0));
 float pop[pop_size][num_tasks];
 int curr_best[num_tasks];
 float eval_currbest=0;
 float eval_pop[pop_size];
 float new_cand[num_tasks];
 float eval_new;
 int teacher;
 int learning_partner;
  
 //Below function initialises the population randomly 
 random((float *)pop,pop_size,num_tasks,num_tasks);
  
 //Prints the population
 cout<<endl<<"Initial random population is "<<endl;
 cout<<endl;
 print_array((float *)pop,pop_size,num_tasks);
  
 //**IMPLEMENT LEARNING PROCESS**
 //Evaluation of the population
 
 evaluation(utility,(float *)pop,pop_size,eval_pop);
 print_array(eval_pop,1,pop_size);
   
 //Find the teacher
 teacher=find_teacher(pop_size,eval_pop);
 //cout<<teacher<<endl;
 
 for (int g=0;g<max_iterations;g++)
 {
  for (int curr_cand=0;curr_cand<pop_size;curr_cand++)
  { 
   //Use the current teacher to teach the individual using order based recombination operator
   if (curr_cand!= (teacher-1))
   {
    gen_newcand((float *)pop,new_cand,teacher,num_tasks,obx_size,curr_cand);
    evaluation(utility,new_cand,1,&eval_new);
    if (eval_new >=eval_pop[curr_cand])
    {
     eval_pop[curr_cand]=eval_new;
     for (int i=0;i<num_tasks;i++)
     {
      pop[curr_cand][i]=new_cand[i];
     }
    }
   }
   teacher=find_teacher(pop_size,eval_pop);
 
   //Use a randomly generated learner to learn from that individual using order based recombination operator
   vector<int> list;
   //generate learning partner
   for (int i=1;i<pop_size;i++)
   {
    if(i!=teacher && (i-1)!=curr_cand)
    {
     list.push_back(i);
    }     
   }
   learning_partner=list[rand()%(list.size())]; 
   //gen new candidate from the learning partner and curr_candidate
   gen_newcand((float *)pop,new_cand,learning_partner,num_tasks,obx_size,curr_cand);
   evaluation(utility,new_cand,1,&eval_new);
   if (eval_new >=eval_pop[curr_cand])
   {
    eval_pop[curr_cand]=eval_new;
    for (int i=0;i<num_tasks;i++)
    {
     pop[curr_cand][i]=new_cand[i];
    }
   }
   teacher=find_teacher(pop_size,eval_pop);
  }
 }
 cout<<endl<<"Final population is "<<endl;
 cout<<endl;
 print_array((float *)pop,pop_size,num_tasks);
 cout<<endl<<"Fitness value is "<<endl;
 cout<<endl;
 print_array(eval_pop,1,pop_size);
}

void evaluation(float utility[][num_tasks],float *pop,const int pop_size,float *eval)
{
 for (int i=0;i<pop_size;i++)
 {
  *(eval+i)=0; 
  {
   for (int j=0;j<num_tasks;j++)
   { 
    *(eval+i)=*(eval+i)+utility[j][int(*((pop+i*num_tasks)+j)-1)]; 
   }
  }   
 }
}

int find_teacher(int pop_size,float eval_pop[])
{
 int teacher=1; 
 for (int i=0;i<pop_size;i++)
 {
  if (eval_pop[i]>eval_pop[teacher-1])
  {
   teacher=i+1;
  }   
 }
 return teacher;
}

void random(float *arr,int vect,int num,int range)
{
 int temp;
 int index;
 vector<int> list;
 vector<int>::iterator it;
 for (int i=0;i<vect;i++)
 {
  temp=range;
  for (int i=0;i<range;i++)
  {
   list.push_back(i+1);
  } 
  for(int j=0;j<num;j++)
  {
   index= rand()%temp;
   *(arr+i*num+j)=list[index];
   it=list.begin()+index;
   list.erase(it);
   temp=temp-1;
  }
 }
} 
bool lin_search(float arr[],int size,int element)
{
 bool ans=false;
 for (int i=0;i<size;i++)
 {
  if (arr[i]==element)
  {
   ans=true;
   return ans;
  }
 }
 return ans;
} 

void print_array(float *arr,int nrows,int ncolumns)
{
 for (int i=0;i<nrows;i++)
 {
  for (int j=0;j<ncolumns;j++)
  {
   cout<<*(arr+j+i*ncolumns)<<" ";
  }
  cout<<endl;
 }
}

void gen_newcand(float *pop,float *new_cand,int teacher,int num_tasks,int obx_size,int curr_cand)
{
 //Initialse new_cand to 0
 for (int k=0;k<num_tasks;k++)
 {
  new_cand[k]=0;
 }
 //Generate random indexes equal to obx_size for obx_operator to work on
 float obx_pos[obx_size];
 random(obx_pos,1,obx_size,num_tasks);
 //print_array(obx_pos,1,obx_size);
 // Apply obx_operator to generate genes of new_cand from teacher
 for (int k=0;k<obx_size;k++)
 {
  new_cand[int(obx_pos[k]-1)]=*(pop+(teacher-1)*num_tasks+ int(obx_pos[k]-1));
 }
 // Apply obx _operator to generate rest of the genes of new_cand
 for (int k=0;k<num_tasks;k++)
 {
  if (new_cand[k]==0)
  {
   for (int i=0;i<num_tasks;i++)
   {
    if(!lin_search(new_cand,num_tasks,*(pop+(curr_cand)*num_tasks+i)))
    {
     new_cand[k]=*(pop+(curr_cand)*num_tasks+i);
     break;
    }
   }
  }
 }
}













