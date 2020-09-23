#include "NS_Task_1_Predef.h"

/*
*Team Id:1776
*Author List: Sharma Pooja Balrambhai, Tarun Kumar, Vaibhav Tiwari, Sathyam Rajpal.
*Filename: NS_Task_1_Sandbox.cpp
*Theme: Nutty Squirrel.
*Functions: Square(),line_follow(),Task_1_1(),Task_1_2(),forward_wls(),left_turn_wls(),right_turn_wls(),
			final(),make_north(),cleaner(),check_color(),refresh(),directions(),planner(),reroute(),align(),
			calculate(),go_place(),Square(),pick_it_up(),move().
*Global Variables: counter, left_sensor_reading, right_sensor_reading, middle_sensor_reading,
				   n15, n20, n21, n18, p0, p1, p2, p4, p5, p6,Kp,Ki,Kd,
				   last_picked,last_placed,lastStartNode,lastEndNode,ns_backup,ne_backup,
				   o,po,picked,trial[4],arena[NUMBER][NUMBER],previous_error,correction,error,total_error,
				   path[30],obstacle,prev_obstacle,i,b,g,flagger,x[NUMBER],y[NUMBER],begin,end.
*/

extern unsigned int color_sensor_pulse_count;

void forward_wls(unsigned char nodes);

void left_turn_wls(void);

void right_turn_wls(void);

char move(char o, int ns, int ne);

char final(int start, int pick, int goal, char start_orientation, char pick_point_orientation);

void make_north(char c);

void cleaner(int arr[]);

char check_color(void);

void refresh(int goal);

void directions(int path[30],char c);

void planner(int G[NUMBER][NUMBER], int startnode, int endnode);

void reroute(int arena[NUMBER][NUMBER],int new_begin, int goal,char o);

void align(void);

char calculate(int begin, int goal);

void go_place(int start, int goal, char orientation, char color);

void Square(void);

char pick_it_up(int s, int goal, char orientation);

void line_follow(void);

void Task_1_1(void);

void Task_1_2(void);