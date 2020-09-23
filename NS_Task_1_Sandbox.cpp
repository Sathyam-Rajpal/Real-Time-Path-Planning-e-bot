
/*
*Team Id:1776
*Author List: Sathyam Rajpal, Vaibhav Tiwari .
*Filename: NS_Task_1_Sandbox.cpp
*Theme: Nutty Squirrel.
*Functions: Square(),line_follow(),Task_1_1(),Task_1_2(),forward_wls(),left_turn_wls(),right_turn_wls(),
			final(),make_north(),cleaner(),check_color(),refresh(),directions(),planner(),reroute(),align(),
			calculate(),go_place(),Square(),pick_it_up(),move().
*Global Variables: counter, left_sensor_reading, right_sensor_reading, middle_sensor_reading,
				   n15, n20, n21, n18, p0, p1, p2, p4, p5, p6,Kp,Ki,Kd,
				   last_picked,last_placed,lastStartNode,lastEndNode,ns_backup,ne_backup,
				   o,po,picked,trial[4],arena[26][26],previous_error,correction,error,total_error,
				   path[30],obstacle,prev_obstacle,i,b,g,flagger,x[26],y[26],begin,end.
*/

/*NOTE: PLEASE FIND THE WEIGHTED MAP OF THE ARENA MARKED WITH THE DISTANCE AND THE NODE NUMBERS IN THE JOURNAL ATTACHED*/
/*ALTHOUGH WE TRIED OUR BEST BUT WITH THE INCONSISTENCIES OF V-REP THE SIMULATION SUMTIMES GIVES DIFFERENT RESULT ON THE SAME CODE.*/
#include "NS_Task_1_Sandbox.h"

#define THRESHOLD  200
/*Variable nj=1 means the nut has not been placed at the jth position.*/
int n15 = 1, n20 = 1, n21 = 1, n18 = 1;
/*Variable pj=1 means the nut has not been picked from the jth position.*/
int p0 = 1, p1 = 1, p2 = 1, p4 = 1, p5 = 1, p6 = 1;
/*Stores the position of the last pick point, it updates at the start argument of the final() if not nut is found.*/
int last_picked = -1;
/*Stores the position of the point where the nut was last placed, updates to the start argument of final() once a nut is found-picked-placed*/
int last_placed = -1;
/*Stores the value of the previous start node in the last iteration of move() which is called repetetively in the recursions of reroute() via directions()*/
int lastStartNode = -1;
/*Stores the node which couldn't be reached beacause of the obstacle, deals with the back half of the circle */
int lastEndNode = -1;
/*Variables used to sensitize the motion of the bot at the back half of the circle*/
int ns_backup = -1;
/*Variables used to sensitize the motion of the bot at the back half of the circle*/
int ne_backup = -1;

/*Stores the current position of the bot.*/
char po;
/*Variable picked is 1 when the bot picks the nut and picked is 0 as soon as the bot leaves the pickup point.*/
int picked = 0;
/*Trial array is created to deal with the conditional motion of the bot at the back half of the circle.*/
int trial[4] = { -1,-1,-1,-1 };
/*Stores the previously calculated error.*/
double previous_error = 0;
/*The corrrection factor in velocity that is needed to be fed to the wheels of the bot to remain in the center of the line.*/
double correction = 0;
/*Stores the calculated error in position of the sensors from the center of the line.*/
double error = 0;
/*Cumulative sum of the error calculated.*/
double total_error = 0;
/*Kp is the constant for the Proportional Controller. It basically gives an idea of how sensitive you want the controller to be.*/
double Kp = 105;
/*Ki is the constant for the Integral Controller. The controller adds up the errors from each of the previous loops from the beginning.*/
double Ki = 0.458;
/*is the constant for the Derivative. It tries to anticipate the relative correction at a future instance.*/
double Kd = 5.15;
/*A flag to save the computational time by terrminating the useless successive recursions.*/
int obstacle = 0;
/*A flag to save the computational time by terrminating the useless successive recursions.*/
int prev_obstacle = 0;
/* arena: Distance wieghted map of the arena.*/
int arena[26][26] = { {0,84,0,0,0,0,0,207,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},         //0
					  {84,0,84,0,0,0,0,0,119,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//1
					  {0,84,0,84,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},			//2
					  {0,0,84,0,84,0,0,0,0,0,265,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//3
					  {0,0,0,84,0,84,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},			//4
					  {0,0,0,0,84,0,84,0,0,0,0,0,119,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//5
					  {0,0,0,0,0,84,0,0,0,0,0,0,0,207,0,0,0,0,0,0,0,0,0,0,0,0,0},			//6
					  {207,0,0,0,0,0,0,0,113,0,0,0,0,0,195,0,0,0,0,0,0,0,0,0,0,0,0},		//7
					  {0,119,0,0,0,0,0,113,0,200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//8
					  {0,0,0,0,0,0,0,0,200,0,95,0,0,0,0,0,95,0,0,0,0,0,0,0,0,0,0},		//9
					  {0,0,0,265,0,0,0,0,0,95,0,90,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//10
					  {0,0,0,0,0,0,0,0,0,0,90,0,200,0,0,0,0,95,0,0,0,0,0,0,0,0,0},		//11
					  {0,0,0,0,0,119,0,0,0,0,0,200,0,113,0,0,0,0,0,0,0,0,0,0,0,0,0},		//12
					  {0,0,0,0,0,0,207,0,0,0,0,0,113,0,0,0,0,0,0,195,0,0,0,0,0,0,0},		//13
					  {0,0,0,0,0,0,0,195,0,0,0,0,0,0,0,83,0,0,0,0,81,0,0,0,0,0,0},		//14
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,83,0,39,0,0,0,0,0,0,0,0,0,0},			//15
					  {0,0,0,0,0,0,0,0,0,95,0,0,0,0,0,39,0,383,0,0,0,0,190,0,0,0,0},		//16
					  {0,0,0,0,0,0,0,0,0,0,0,95,0,0,0,0,383,0,39,0,0,0,190,0,0,0,0},		//17
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,39,0,83,0,0,0,0,0,0,0},			//18
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,195,0,0,0,0,83,0,0,81,0,0,0,0,0},		//19
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,81,0,0,0,0,0,0,0,0,0,0,0,0},			//20
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,81,0,0,0,0,0,0,0}			//21
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,190,190,0,0,0,0,0,1,0,0,0},		//26
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0},			//23
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1},		//24
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},			//25
					  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0}			//26
};
/*Used as counter variable*/
int filter = 0;
/*Used as counter variable*/
int i = 0;
/*Variable b stands for begin,stores the start point of the reroute().*/
int b = 10;
/*Variable g stands for goal,stores the end point of the reroute().*/
int g = 0;
/*Stores the last state 'Left' or 'Right' of the bot. Flagger = 2 when bot takes a left turn and Flagger = 3 when bot takes a right turn.
   Used for auto alignment of the bot if the three sensors are reading white.*/
int flagger = 0;

/* x: ordinates of each point.With 10th node as origin.*/
double x[26] = { -4,-2,-1,0,1,2,4,-4,-2,-2,0,2,2,4,-4,-3,-2.1,2.1,3,4,-4,4,0,0,0,-2,2 };

/* y: ordinates of each point.With 10th node as origin.*/
double y[26] = { 5,5,5,5,5,5,5,3,4,0,0,0,4,3,2,1,0,0,1,2,-1,-1,-1,-2,-3,-3,-3 };

/* counter: Keeps track of number of turns forming a square.*/
int counter = 0;

/*left_sensor_reading: Stores the values of left white line sensor*/
unsigned char left_sensor_reading = 0;

/*right_sensor_reading: Stores the values of right white line sensor*/
unsigned char right_sensor_reading = 0;

/*middle_sensor_reading: Stores tha values of middle white line sensor*/
unsigned char middle_sensor_reading = 0;

/*path: Stores the shortest path*/
int path[30] = { -1,-1,-1,-1,-1,-1,
	 -1,-1,-1,-1,-1,-1,
	 -1,-1,-1,-1,-1,-1,
	 -1,-1,-1,-1,-1,-1,
	 -1,-1,-1,-1,-1,-1 };


/*path: Stores the initial orientation*/
static char o = 'N';

/*begin: Stores the former of last path pair before meeting the obstacle.*/
int begin = -1;
/*end: Stores the later of the path pairs before meeting the obstacle.*/
int end = -1;

/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
void forward_wls(unsigned char nodes)
{   /*Variable used as a filter to remove the noise values of the proximity sensor.*/
	int obstacle_filter = 0;
	_delay_ms(5);
	//The variable node stores true if the bot touches a node,that facilitates counting the node else its false.   
	bool node = false;
	//Variable node_count keeps track of the number of nodes crossed.
	int node_count = 0;

	// runs until the goal,the 13th node is reached.
	do {

		_delay_ms(1);
		//The value of the left white line sensor is stored in the variable left_sensor_reading using predefined ADC_Conversion() function.
		left_sensor_reading = ADC_Conversion(1);
		//The value of the middle white line sensor is stored in the variable middle_sensor_reading using predefined ADC_Conversion() function.
		middle_sensor_reading = ADC_Conversion(2);
		//The value of the right white line sensor is stored in the variable right_sensor_reading using predefined ADC_Conversion() function.
		right_sensor_reading = ADC_Conversion(3);
		//This condition checks if the bot is on black path.

		/*This funtion is called to calculate the correction value that is needed to be fed to the left and right wheels via velocity functon.*/
		align();
		/*Condition checks if all the 3 sensors are on a node i.e. giving corresponding values of balck color.If true then the bot detects node.*/
		if (left_sensor_reading > THRESHOLD && middle_sensor_reading > THRESHOLD && right_sensor_reading > THRESHOLD)//The condition is true if the bot reaches a node.
		{
			//If the value of node is false this means the bot was on black line in the previous loop, and has just now entered the node.
			if (node == false)
			{
				//Makes the value of node to true as the bot is now at a node.
				node = true;
				//Increase the node count when the bot is at a node.
				node_count = node_count + 1; printf("NODE: %d\n", node_count);
			}
		}
		/*Condition checks if all the middle sensor is giving corresponding values of balck color.If true then the bot moves forward.*/
		else if (middle_sensor_reading > THRESHOLD)
		{
			flagger = 1;
			velocity(265, 265);
			forward();
			//Sets the node to false if the bot is on a black line and the node value
			//is true, this is to rest the loop and tell us the node has been crossed in the previous loop.
			if (node == true)
				node = false;
		}
		//To keep the bot on black line , the left sensor if senses the black colour rather than white - it takes a slight left turn to realign the bot.
		if (left_sensor_reading > THRESHOLD && middle_sensor_reading < THRESHOLD && right_sensor_reading < THRESHOLD)
		{   /*Condition checks if the bot is currently at either of the two starting points of the back-half circle,If true then
			  the bot's velocity is conditionally sensitized.*/
			if ((ns_backup == 16 && ne_backup == 17) || (ns_backup == 17 && ne_backup == 16))
			{
				flagger = 2;
				velocity((70 - correction), (70 - correction));
				soft_left();
			}
			/*Else the velocity correction of left turn is constant for rest of thr arena.*/
			else
			{
				flagger = 2;
				velocity((60 - correction), (60 - correction));
				soft_left();
			}
		}
		//To keep the bot on black line , the right sensor if senses the black colour rather than white - it takes a slight right turn to realign the bot.
		if (right_sensor_reading > THRESHOLD && middle_sensor_reading < THRESHOLD && left_sensor_reading < THRESHOLD)
		{   /*Condition checks if the bot is currently at either of the two starting points of the back-half circle,If true then
			  the bot's velocity is conditionally sensitized.*/
			if ((ns_backup == 16 && ne_backup == 17) || (ns_backup == 17 && ne_backup == 16))
			{
				flagger = 3;
				velocity((70 + correction), (70 + correction));
				soft_right();
			}
			/*Else the velocity correction of right turn is constant for rest of thr arena.*/
			else {
				flagger = 3;
				velocity((60 + correction), (60 + correction));
				soft_right();
			}
		}
		/*Condition checks if all the 3 sensors are giving corresponding values of white color.If true then the bot acts according to its previous state to realign itself.*/
		if ((middle_sensor_reading < THRESHOLD) && (left_sensor_reading < THRESHOLD) && (right_sensor_reading < THRESHOLD))
		{   /*If the last state of the bot was right*/
			if (flagger == 3)
			{
				velocity(50 + correction, 50 + correction);
				right();
			}
			/*If the last state of the bot was left*/
			else if (flagger == 2)
			{
				velocity(50 - correction, 50 - correction);
				left();
			}

		}
		/*This condition checks if the bot is nearing an obstacle having no colour.If true then the bot detects there is an obstacle.*/
		if (ADC_Conversion(4) < 50 && check_color() == 'X')
		{   /*Counter variable used in the function.*/
			int i = 0, x = 0;
			obstacle_filter++;
			/*If the proximity sensor reads a consistent value for more than a specified times.*/
			if (obstacle_filter > 20)
			{
				printf("|n---OBSTACLE---|n");
				obstacle_filter = 0;
				/*Bot takes a U turn*/
				back();
				_delay_ms(100);
				right_turn_wls();
				stop();
				/*This condition resets the path array as -1.*/
				if (end == -1 && begin == -1)
				{
					for (i = 0; i < 30; i++)
					{
						if (path[i] == -1)
						{
							x = i;
							break;
						}
					}
					end = path[x - 2]; begin = path[x - 1];
				}
				/*Sets the cost of the path in which the obstacle was found equal to 5000, hence the bot never goes to that path again.*/
				arena[begin][end] = 0;
				arena[end][begin] = 0;
				/*To update the orientation of the bot if the obstacle is encounterd.*/
				o = calculate(end, begin);
				printf(" |%d--%d|& %c ", begin, end, o);
				lastEndNode = end;
				forward_wls(1);
				stop();
				reroute(arena, begin, g, o);
				obstacle = obstacle + 1;
			}
		}
	} while (node_count < nodes);

}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void)
{   // to keep on going left until the next black line is found
	left(); _delay_ms(250);
	while (ADC_Conversion(2) < 200);
	//Stops when the next black line is encountered
	stop();
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{
	// to keep on going right until the next black line is found
	right(); _delay_ms(250);
	while (ADC_Conversion(2) < 200);
	//Stops when the next black line is encountered
	stop();
}

/*
*
* Function Name: Square
* Input: void
* Output: void
* Logic: Use this function to make the robot trace square path on the arena.
*
*        NOTE : THE VALUES USED FOR _delay_ms() ARE IN ACCORDANCE WITH REAL TIME SIMULATION OF THE V-REP SIMULATOR.
*
* Example Call: Square();
*/
void Square(void)
{   //While loop runs till the counter value reaches 4 repeatedly making the bot go forward and left.
	while (counter < 4)
	{
		forward();
		//Go forward for 3 seconds.
		_delay_ms(3000);
		left();
		//Turns left by 90 approximately degrees, simulation sofware is rather inconsistent.
		_delay_ms(476);
		stop();
		//Increases the counter by 1 after every turn.
		counter = counter + 1;
	}
}

/*
* Function Name: line_follow
* Input: void
* Output: void
* Logic: Use this function to make the bot trace a black line on the arena.
* Example Call: line_follow();
*/
void line_follow(void)
{   //This loop aligns the bot to a black line.

	//int node_count;
	bool node = false;
	int n = 0;
	do
	{
		_delay_ms(1);
		//The value of the left white line sensor is stored in the variable left_sensor_reading using predefined ADC_Conversion() function.
		left_sensor_reading = ADC_Conversion(1);
		//The value of the middle white line sensor is stored in the variable middle_sensor_reading using predefined ADC_Conversion() function.
		middle_sensor_reading = ADC_Conversion(2);
		//The value of the right white line sensor is stored in the variable right_sensor_reading using predefined ADC_Conversion() function.
		right_sensor_reading = ADC_Conversion(3);
		//This condition checks if the bot is on black path.

		align();

		if (middle_sensor_reading > THRESHOLD)
		{
			flagger = 1;
			velocity(200, 200);
			forward();
			//Sets the node to false if the bot is on a black line and the node value
			//is true, this is to rest the loop and tell us the node has been crossed in the previous loop.
			if (node == true)
				node = false;
		}

		if (left_sensor_reading > THRESHOLD && middle_sensor_reading > THRESHOLD && right_sensor_reading > THRESHOLD)//The condition is true if the bot reaches a node.
		{
			//If the value of node is false this means the bot was on black line in the previous loop, and has just now entered the node.
			if (node == false)
			{
				//Makes the value of node to true as the bot is now at a node.
				node = true;
				//Increase the node count when the bot is at a node.
				//node_count = node_count + 1;
			}
		}
		//To keep the bot on black line , the left sensor if senses the black colour rather than white - it takes a slight left turn to realign the bot.
		if (left_sensor_reading > THRESHOLD && middle_sensor_reading < THRESHOLD && right_sensor_reading < THRESHOLD)
		{
			flagger = 2;
			//printf("WHEEL 1:%d,WHEEL 2:%d\n", abs(50 - int(correction)), 50 + int(correction));
			velocity((50 - correction), (50 - correction));
			soft_left();
		}
		//To keep the bot on black line , the right sensor if senses the black colour rather than white - it takes a slight right turn to realign the bot.
		if (right_sensor_reading > THRESHOLD && middle_sensor_reading < THRESHOLD && left_sensor_reading < THRESHOLD)
		{
			flagger = 3;
			//printf("WHEEL 1:%d,WHEEL 2:%d\n", 50 + int(correction), abs(50 - int(correction)));
			velocity((50 + correction), (50 + correction));
			soft_right();
		}
		if ((middle_sensor_reading < THRESHOLD) && (left_sensor_reading < THRESHOLD) && (right_sensor_reading < THRESHOLD))
		{
			if (flagger == 3)
			{
				velocity(50 + correction, 50 + correction);
				right();
				//_delay_ms(250);
			}
			else if (flagger == 2)
			{
				velocity(50 - correction, 50 - correction);
				left();
				//_delay_ms(250);
			}
		}
	} while (true);
}

/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to run our Task 1.1 logic
*
*       NOTE: THE VALUES UBD IN FUNCTION _delay_ms() ARE IN ACCORDANCE TO REAL TIME SIMULATION OF THE V-REP SIMULATOR.
*
* Example Call: Task_1_1();
*/
void Task_1_1(void)
{
	_delay_ms(5);
	//The variable node stores true if the bot touches a node,that facilitates counting the node else its false.   
	bool node = false;
	//Variable node_count keeps track of the number of nodes crossed.
	int node_count = 0;

	// runs until the goal,the 13th node is reached.
	do {
		_delay_ms(1);
		//The value of the left white line sensor is stored in the variable left_sensor_reading using predefined ADC_Conversion() function.
		left_sensor_reading = ADC_Conversion(1);
		//The value of the middle white line sensor is stored in the variable middle_sensor_reading using predefined ADC_Conversion() function.
		middle_sensor_reading = ADC_Conversion(2);
		//The value of the right white line sensor is stored in the variable right_sensor_reading using predefined ADC_Conversion() function.
		right_sensor_reading = ADC_Conversion(3);
		//This condition checks if the bot is on black path.
		if (left_sensor_reading == 0 && right_sensor_reading == 0 && middle_sensor_reading != 0) {
			//If fulfilled then only the bot moves ahead.
			forward();
			//Sets the node to false if the bot is on a black line and the node value
			//is true, this is to rest the loop and tell us the node has been crossed in the previous loop.
			if (node == true)
				node = false;
		}

		else if (left_sensor_reading != 0 && middle_sensor_reading != 0 && right_sensor_reading != 0)//The condition is true if the bot reaches a node.
		{   //If the value of node is false this means the bot was on black line in the previous loop, and has just now entered the node.
			if (node == false) {
				//Makes the value of node to true as the bot is now at a node.
				node = true;
				//Increase the node count when the bot is at a node.
				node_count = node_count + 1;
				//print the node number on the console.
				printf("%d ", node_count);
			}
			//Checks the number of node on which the bot is and acts accordingly.
			if (node_count == 1)
				//did not use the wls functions as the nodeis surrounded by many black lines.
				right();
			else if (node_count == 6)
			{//just a little push for alignment on the black line.
				soft_right();
			}
			else if (node_count == 7)
			{//just a little push for alignment on the black line.
				soft_left();
			}
			else if (node_count == 4 || node_count == 10)
			{//pushes the bot beyond the node before turning.
				forward();
				_delay_ms(300);
				stop();
				//turns left.
				left_turn_wls();
			}
			if (node_count == 9)
			{
				forward();
				_delay_ms(250);
				stop();
				left_turn_wls();
			}
			if (node_count == 3)
			{
				forward();
				_delay_ms(250);
				stop();
				left_turn_wls();
			}
			if (node_count == 2)
			{
				forward();
				_delay_ms(500);
				right();
				_delay_ms(150);
				stop();
				left_turn_wls();
			}
			else if (node_count == 8 || node_count == 5)
			{
				forward();
				_delay_ms(300);
				stop();
				right_turn_wls();

			}
			else if (node_count == 11)
			{
				soft_left(); right();
			}
			else if (node_count == 12)
			{
				forward();
				_delay_ms(350);
				stop();
				right_turn_wls();
			}
			//The goal is counted as node 13, so the condition is fulfilled the bot stops and the execution of loop gets terminated.
			else if (node_count == 13)
			{
				forward();
				_delay_ms(300);
				stop();
				// The loop terminates upon reaching the goal counted as the 13th node.
				break;
			}

		}
		//To keep the bot on black line , the left sensor if senses the black colour rather than white - it takes a slight left turn to realign the bot.
		else if (left_sensor_reading != 0)
		{

			soft_left();
		}
		//To keep the bot on black line , the right sensor if senses the black colour rather than white - it takes a slight right turn to realign the bot.
		else if (right_sensor_reading != 0)
		{

			soft_right();
		}
		//The execution keeps on running until termination upon reaching the goal.	
		if (node_count == 13)
			break;
	} while (true);

}

/*
* Function Name: directions
* Input: int path[],char c. Character c stores the current orientation of the robot.
* Output: void
* Logic: The function takes the path array as the input along with current orientation and creates a set of moves to be taekn by the bot
*        to reach the goal position.This function recursively calls the function move() which basically the motion directives.
* Example Call: directions(path[30],c);
*/
void directions(int path[30], char c)
{   //To terrminate all futile recursions.
	if (obstacle == prev_obstacle)
	{
		o = c;
		int i;
		printf("\n");
		int j = 0;
		for (i = 28; i > 0; i--)
		{
			if (path[i] != -1)
			{

				if ((path[i - 1] == 16 && path[i] == 17) || (path[i - 1] == 17 && path[i] == 16))
				{

					printf("THE TRIAL: %d=>%d %d=>%d %d=>%d %d=>%d", path[i - 2], i - 2, path[i - 1], i - 1, path[i], i, path[i + 1], i + 1);
					trial[0] = path[i + 1];
					trial[1] = path[i];
					trial[2] = path[i - 1];
					trial[3] = path[i - 2];
					if (trial[0] == -1)
					{
						if (trial[1] == 16 && trial[2] == 17)
						{
							trial[0] = 9;

						}
						else if (trial[1] == 17 && trial[2] == 16)
						{
							trial[0] = 11;
						}
					}

					break;
				}
			}
		}
		for (i = 28; i > 0; i--)
		{
			if (path[i] != -1)
			{
				o = move(o, path[i], path[i - 1]);
				begin = path[i - 1];
				end = path[i - 2];
				j++;

			}
		}
		printf("%c x", o);
	}
}

/*
*
* Function Name: planner
* Input: arena[][], start node,end node.
* Output: path[]
* Logic: Uses Djkistra's algorithm to find out the shortest possible unblocked path b/w current start and end nodes using greedy techniques and findng local optimum one by one.
* Example Call: planner(arena[26][26],startnode,endnode); //Gives shortest possible path b/w startnode and endnode.
*
*/
void planner(int G[26][26], int startnode, int endnode)
{   /*Variable used as a counter*/
	int q = 0;
	begin = -1; end = -1;
	//cleaning path,end and begin.
	for (q = 0; q < 30; q++)
	{
		path[q] = -1;
	}

	int cost[26][26], distance[26], pred[26];
	int visited[26], count, mindistance, nextnode, i, j;

	//pred[] stores the predecessor of each node.
	//count gives the number of nodes traversed so far.
	//create the cost matrix
	for (i = 0; i < 26; i++)
		for (j = 0; j < 26; j++)
			if (G[i][j] == 0)
				cost[i][j] = 5000;
			else
				cost[i][j] = G[i][j];

	//initialize pred[],distance[] and visited[]

	for (i = 0; i < 26; i++)
	{
		distance[i] = cost[startnode][i];
		pred[i] = startnode;
		visited[i] = 0;
	}

	distance[startnode] = 0;
	visited[startnode] = 1;
	count = 1;

	while (count < 21)
	{
		mindistance = 5000;

		//nextnode gives the node at minimum distance
		for (i = 0; i < 26; i++)
			if (distance[i] < mindistance && !visited[i])
			{
				mindistance = distance[i];
				nextnode = i;
			}

		//check if a better path exists through nextnode            
		visited[nextnode] = 1;
		for (i = 0; i < 26; i++)
			if (!visited[i])
				if (mindistance + cost[nextnode][i] < distance[i])
				{
					distance[i] = mindistance + cost[nextnode][i];
					pred[i] = nextnode;
				}
		count++;
	}

	if (endnode != startnode)
	{
		int i = 0;
		printf("\nDistance of node%d=%d", endnode, distance[endnode]);

		path[0] = endnode;
		j = endnode;
		do
		{
			j = pred[j];
			if (j != -1)
				path[i + 1] = j;
			printf(" %d <-", path[i]);
			i = i + 1;
		} while (j != startnode);
	}
}

/*
*
* Function Name: move
* Input: char o [orientation], int ns [current node], int ne [succeeding node].
* Output: char o [orientation].
* Logic: By comparing the x,y co-ordinates of the current node and the next node the function assigns the required movement
*        that should be taken by the bot along with the orientation.
*        N = NORTH, S = SOUTH, E = EAST, W = WEST A = NORTHEAST, B = SOUTHEAST, C = SOUTHWEST, D = NORTHWEST.
* Example Call: move(o,ns,ne);
*
*/
char move(char o, int ns, int ne)
{
	int ns_backup = -1;
	int ne_backup = -1;
	/*If this condition true then the recursion terminates.*/
	if (prev_obstacle == obstacle)
	{
		char movement;
		printf("Picked: %d \n", picked);
		/*Condition checks if the x ordinate of the current node is equal to the x ordinate of succeeding node
		  OR if the y co-ordinate of the current node is equal to the y co-ordinate of succeeding node.If true then then
		  the orientation and the movement required is assigned to the bot as per the graphical representation of the arena.*/
		if (x[ns] == x[ne] || y[ns] == y[ne])
		{
			if (o == 'N')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'r';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'l';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'f';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'u';
				}

			}
			else if (o == 'S')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'l';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'r';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'u';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'f';
				}
			}
			else if (o == 'E')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'f';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'u';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'l';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'r';
				}

			}
			else if (o == 'W')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'u';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'f';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'r';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'l';
				}

			}
			else if (o == 'D')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'r';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'l';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'r';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'l';
				}
			}
			else if (o == 'A')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'r';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'l';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'l';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'r';
				}
			}
			else if (o == 'B')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'l';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'r';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'l';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'r';
				}
			}
			else if (o == 'C')
			{
				if (x[ns] < x[ne])
				{
					o = 'E';
					movement = 'l';
				}
				if (x[ns] > x[ne])
				{
					o = 'W';
					movement = 'r';
				}
				if (y[ns] < y[ne])
				{
					o = 'N';
					movement = 'r';
				}
				if (y[ns] > y[ne])
				{
					o = 'S';
					movement = 'l';
				}
			}
		}
		/*Condition checks if the x ordinate of the current node is different rom the x ordinate of succeeding node
		  OR if the y co-ordinate of the current node is different from the y co-ordinate of succeeding node.If true then then
		  the orientation and the movement required is assigned to the bot as per the graphical representation of the arena.*/
		else if (x[ns] != x[ne] && y[ns] != y[ne])
		{
			if (o == 'N')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'l';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'r';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'l';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'r';
				}


			}
			else if (o == 'S')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'r';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'l';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'r';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'l';
				}



			}
			else if (o == 'E')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'l';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'r';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'r';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'l';
				}


			}
			else if (o == 'W')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'r';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'l';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'l';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'r';
				}


			}
			else if (o == 'A')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'l';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'r';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'u';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'f';
				}


			}
			else if (o == 'D')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'f';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'u';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'l';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'r';
				}


			}
			else if (o == 'B')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'u';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'f';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'r';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'l';
				}


			}
			else if (o == 'C')
			{
				if (x[ns] > x[ne] && y[ns] < y[ne])
				{
					o = 'D';
					movement = 'r';
				}

				if (x[ns] < x[ne] && y[ne] < y[ns])
				{
					o = 'B';
					movement = 'l';
				}
				if (y[ns] > y[ne] && x[ns] > x[ne])
				{
					o = 'C';
					movement = 'f';
				}
				if (y[ns] < y[ne] && x[ns] < x[ne])
				{
					o = 'A';
					movement = 'u';
				}


			}
		}
		/*Sets the previous orientation equal to current orientation if the ne-ending node is equal to g-goal.*/
		if (ne == g)
			po = o;
		/*Specific set of conditions for the movement of the bot at the back half of the circle.*/
		if ((ns == 16 && ne == 17))
		{
			if (trial[0] == 15)
				movement = 'r';
			else if (trial[0] == 9)
				movement = 'f';
		}
		if (ns == 17 && ne == 16)
		{
			if (trial[0] == 18)
				movement = 'l';
			else if (trial[0] == 11)
				movement = 'f';
		}
		if (ns == 17 && ne == trial[3])
		{
			if (trial[3] == 18)
				movement = 'r';
			if (trial[3] == 11)
				movement = 'f';
		}
		if (ns == 16 && ne == trial[3])
		{
			if (trial[3] == 15)
				movement = 'l';
			if (trial[3] == 9)
				movement = 'f';
		}

		printf("LAST END NODE: %d CURRENT NODE:%d NEXT NODE: %d \n", lastEndNode, ns, ne);

		/*Specific set of conditions for the movement of the bot at the back half of the circle.*/
		if (lastEndNode == 17 && ns == 16 && ne == 9)
		{
			printf("TRIAL TRUE");
			movement = 'f';
		}
		else if (lastEndNode == 17 && ns == 16 && ne == 15)
		{
			printf("TRIAL TRUE");
			movement = 'l';
		}
		else if (lastEndNode == 16 && ns == 17 && ne == 11)
		{
			printf("TRIAL TRUE ");
			movement = 'f';
		}
		else if (lastEndNode == 16 && ns == 17 && ne == 18)
		{
			printf("TRIAL TRUE");
			movement = 'r';
		}

		/*Specific set of conditions for the movement of the bot at the back half of the circle.*/
		else if (lastEndNode == 11 && ns == 17 && ne == 18)
		{
			printf("TRIAL TRUE");
			movement = 'l';
		}
		else if (lastEndNode == 11 && ns == 17 && ne == 16)
		{
			printf("TRIAL TRUE");
			movement = 'f';
		}
		else if (lastEndNode == 9 && ns == 16 && ne == 15)
		{
			printf("TRIAL TRUE");
			movement = 'r';
		}
		else if (lastEndNode == 9 && ns == 16 && ne == 17)
		{
			printf("TRIAL TRUE");
			movement = 'f';
		}

		printf("Moving %c between %d and %d facing %c\n", movement, ns, ne, o);

		/*If the movement of the bot is calculated to be left as per the orientation and its position.*/
		if (movement == 'l')
		{   /*Specific conditions to optimise the movement of the bot when the bot has picked a nut.*/
			if (picked == 1)
			{
				stop();
				left_turn_wls();
				forward_wls(1);
				stop();
				picked = 0;
			}
			/*Specific conditions to optimise the movement of the bot.*/
			else
			{
				if (ns == 18 || ns == 15)
				{
					stop();
					left_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 12 && ne == 13)
				{
					forward();
					_delay_ms(350);
					stop();
					left_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 13 && ne == 12)
				{
					forward();
					_delay_ms(250);
					stop();
					left_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 17 && ne == 18)
				{
					forward(); _delay_ms(100);
					stop();
					left_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 8 && ne == 1)
				{
					forward(); _delay_ms(325);
					stop();
					left_turn_wls();
					//Stops when the next black line is encountered
					stop();
					forward_wls(1);
					stop();
				}
				else if (ns == 14 && ne == 6)
				{
					forward(); _delay_ms(163);
					stop();
					left_turn_wls();
					//Stops when the next black line is encountered
					stop();
					forward_wls(1);
					stop();
				}
				else if (ns == 19 && ne == 13)
				{
					forward(); _delay_ms(163);
					stop();
					left_turn_wls();
					//Stops when the next black line is encountered
					stop();
					forward_wls(1);
					stop();
				}
				else
				{
					forward(); _delay_ms(175);
					while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
					stop();
					left_turn_wls();
					forward_wls(1);
					stop();
				}
			}
		}
		/*If the movement of the bot is calculated to be right as per the orientation and its position.*/
		if (movement == 'r')
		{   /*Specific conditions to optimise the movement of the bot when it ha picked a nut.*/
			if (picked == 1)
			{
				stop();
				right_turn_wls();
				forward_wls(1);
				stop();
				picked = 0;
			}
			/*Specific conditions to optimise the movement of the bot.*/
			else
			{
				if (ns == 18 || ns == 15)
				{
					stop();
					right_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 12 && ne == 5)
				{
					forward(); _delay_ms(325);
					stop();
					right_turn_wls();
					//Stops when the next black line is encountered
					stop();
					forward_wls(1);
					stop();
				}
				else if (ns == 8 && ne == 7)
				{
					forward();
					_delay_ms(350);
					stop();
					right_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 7 && ne == 8)
				{
					forward(); _delay_ms(250);
					stop();
					while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
					right_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 16 && ne == 15)
				{
					forward(); _delay_ms(100);
					stop();
					right_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 14 && ne == 7)
				{
					forward(); _delay_ms(163);
					stop();
					right_turn_wls();
					stop();
					forward_wls(1);
					stop();
				}
				else if (ns == 19 && ne == 21)
				{
					forward(); _delay_ms(163);
					stop();
					right_turn_wls();
					stop();
					forward_wls(1);
					stop();
				}
				else
				{
					forward(); _delay_ms(175);
					while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
					stop();
					right_turn_wls();
					forward_wls(1);
					stop();
				}
			}
		}
		/*If the movement of the bot is calculated to be forward as per the orientation and its position.*/
		if (movement == 'f')
		{   /*Specific conditions to optimise the movement of the bot.*/
			if ((ns == 9 || ns == 10 || ns == 11) && (o == 'E' || o == 'W'))
			{
				if (o == 'E'&&ns == 9)
				{
					forward(); _delay_ms(200);
					while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
					soft_right();
					while (ADC_Conversion(2) > 200);
					forward_wls(1);
					stop();
				}
				else if (o == 'W'&&ns == 11)
				{
					forward(); _delay_ms(200);
					while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
					soft_left();
					while (ADC_Conversion(2) > 200);
					forward_wls(1);
					stop();
				}
				else
				{
					if (o == 'E')
					{
						forward(); _delay_ms(175);
						while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
						soft_right();
						while (ADC_Conversion(2) > 200);
						forward_wls(1);
						stop();
					}
					else if (o == 'W')
					{
						forward(); _delay_ms(175);
						while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
						soft_left();
						while (ADC_Conversion(2) > 200);
						forward_wls(1);
						stop();
					}
				}
			}
			else if (ns == 17 && o == 'W'&&ne == 16)
			{
				ns_backup = ns;
				ne_backup = ne;
				forward(); _delay_ms(150);
				while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
				soft_right();
				while (ADC_Conversion(2) > 200);
				forward_wls(1);
				stop();
			}
			else if (ns == 17 && ne == 11)
			{
				forward(); _delay_ms(150);
				soft_left();
				while (ADC_Conversion(2) > 200);
				forward_wls(1);
				stop();
			}
			else if (ns == 16 && o == 'E'&&ne == 17)
			{
				ns_backup = ns;
				ne_backup = ne;
				forward(); _delay_ms(150);
				while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
				soft_left();
				while (ADC_Conversion(2) > 200);
				forward_wls(1);
				stop();
			}
			else if (ns == 16 && ne == 9)
			{
				forward(); _delay_ms(150);
				soft_right();
				while (ADC_Conversion(2) > 200);
				forward_wls(1);
				stop();
			}
			else if ((ns == 8 || ns == 12) && o == 'S')
			{
				forward();
				_delay_ms(240);
				forward_wls(1);
				stop();
			}
			else if ((ns == 8 || ns == 12) && o == 'N')
			{
				forward();
				_delay_ms(200);
				forward_wls(1);
				stop();
			}
			else if ((ns == 18 || ns == 15) && (ne == 14 || ne == 19))
			{
				forward();
				while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
				forward_wls(1);
				stop();
			}
			else
			{
				forward(); _delay_ms(175);
				while (ADC_Conversion(1) > THRESHOLD && ADC_Conversion(2) > THRESHOLD && ADC_Conversion(3) > THRESHOLD);
				forward_wls(1);
				stop();
			}
		}
		/*If the movement of the bot is calculated to be U Turn as per the orientation and its position.*/
		if (movement == 'u')
		{   /*Specific conditions to optimise the movement of the bot if the bot has pickeda  nut.*/
			if (picked = 1)
			{
				picked = 0;
				if (ns == 0)
				{
					stop();
					left_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 6)
				{
					stop();
					right_turn_wls();
					forward_wls(1);
					stop();
				}
				else if (ns == 1 || ns == 5)
				{
					stop();
					right_turn_wls();
					right_turn_wls();
					forward_wls(1);
					stop();
				}

			}
			/*Specific conditions to optimise the movement of the bot.*/
			else
			{
				printf("To cross verify the updated delays at u turns");
				back(); _delay_ms(350); stop();
				right_turn_wls();
				forward_wls(1);
				stop();
			}
		}

		lastEndNode = -1;
		lastStartNode = ns;
	}
	return o;
}

/*
*
* Function Name: reroute
* Input: int arena[26][26] - The path matrix, int B - Begin position, int G - Goal position, char c- Start orientation.
* Output: void
* Logic: A function created to guide the robot using planner() and directions().
* Example Call: reroute(arena,startnode,endnode,orientation);
*/
void reroute(int arena[26][26], int B, int G, char c)
{   /*Initialises the previous obstacle variable eaual to obstacle variable to indicate the rerouting.*/
	prev_obstacle = obstacle;
	/*Calls the function planner() that stores the shortest path from the start position to the specified goal.*/
	planner(arena, B, G);
	/*After the planner function makes returns the shortest path in the path array, reroute function calls the function
	directions () for creating the set of moves that will lead the bot the desired goal position.*/
	directions(path, c);
}

/*
*
* Function Name: calculate.
* Input: int begin - last start node, int goal - Succeeding node.
* Output: char ret - Updated Orientation.
* Logic: When the bot detects an obstacle in the path, as the bot takes an U turn and reroutes the new orientation
		 of the bot is updated by this function as the bot reroutes itself using a newer path.
* Example Call: calculate(begin,goal);
*/
char calculate(int begin, int goal)
{
	char ret;
	/*If any of the x, y co-ordinates are equal then the bot's direction is just reversed.*/
	if (x[begin] == x[goal] || y[begin] == y[goal])
	{
		if (x[begin] < x[goal])
			ret = 'E';
		else if (x[begin] > x[goal])
			ret = 'W';
		else if (y[begin] < y[goal])
			ret = 'N';
		else
			ret = 'S';
	}
	else

	{   /*Specific conditions are applied to reverse the bot's direction for the intermediate directions.*/
		if (x[begin] < x[goal])
		{
			if (y[begin] < y[goal])
				ret = 'A';
			else if (y[begin] > y[goal])
				ret = 'B';
		}
		else
		{
			if (y[begin] > y[goal])
				ret = 'C';
			else if (y[begin] < y[goal])
				ret = 'D';
		}
	}
	printf("Updated Orientation: %c ", ret);
	/*Returns the updated orientation.*/
	return ret;
}

/*
*
* Function Name: cleaner.
* Input: int arr[] - path array.
* Output: void.
* Logic: Resets the path to prevent the satck corruption.
* Example Call: cleaner(arr[]);
*/
void cleaner(int arr[])
{
	printf("\n");
	int s = 0; int length = 30;
	printf("______________");
	for (s = 0; s < length; s++)
	{
		printf("%d Element before:%d\t", i, arr[i]);
	}
}

/*
*
* Function Name: align.
* Input: void.
* Output: void.
* Logic: The function with help of the PID algorithm calculates a velocity correction for aligning the bot to the center of the line.
* Example Call: align();
*/
void align(void)
{ // save previous error for differential 
	previous_error = error;
	double s;
	s = (ADC_Conversion(1)) + (ADC_Conversion(3)) + (ADC_Conversion(2));
	// Calculate the error in how much robot deviate from center.
	error = (((1 * ADC_Conversion(1)) + (3 * ADC_Conversion(3)) + (2 * ADC_Conversion(2))) / s) - 1.9;
	// Accumulate error for integral
	total_error += error;
	/*The correction in velocity that needs to be deducted in left turns and added in right turns.*/
	correction = (Kp*error) + (Kd*(error - previous_error)) + (Ki*total_error);
}

/*
*
* Function Name: refresh.
* Input: void.
* Output: void.
* Logic: Re-initialises all the parameters into their initial value to pevent stack corruption.
* Example Call: refresh();
*/
void refresh(int goal)
{
	g = goal;
	int s = 0; int length = 30;
	obstacle = 0; prev_obstacle = 0;
	i = 0;
	begin = -1;
	end = -1;
	for (s = 0; s < length; s++)
	{
		path[i] = -1;
	}
}

/*
*
* Function Name: make_north.
* Input: char c - current orientation.
* Output: void.
* Logic: Resets the direction of the bot to north.
* Example Call: make-north(po);
*/
void make_north(char c)
{
	if (c == 'E')
	{
		forward();
		_delay_ms(300);
		left_turn_wls();
		stop();
		o = 'N';
	}
	else if (c == 'W')
	{
		forward();
		_delay_ms(300);
		right_turn_wls();
		stop();
		o = 'N';
	}
	else if (c == 'N')
	{
		forward();
		_delay_ms(300);
		stop();
	}
}

/*
*
* Function Name: pick_it_up.
* Input: int s - Starting point position ,int goal - Pick point position,char c - pick point orientation.
* Output: char.  It returns the colour of the object if detected otherwide 'X'
* Logic: Uses the function check_color() to check for a nut, if  found then it picks up the up the nut and returns its colour to the calling function.
* Example Call: pick_it_up(,goal,'N');  Picking point orientation is always NORTH.
*/
char pick_it_up(int s, int goal, char orientation)
{
	char co;
	int flag = 0;
	int count = 0;
	/*Pick point position*/
	g = goal;
	/*Calls the function reroute() to take the bot to the picking point using the shortest path.*/
	reroute(arena, s, goal, orientation);
	/*Sets the variable picked for custum alignment of the bot for the bot to follow the next movement command*/
	picked = 1;
	printf("/n Previous Orientation: %c/n Orientation: %c/n", po, o);
	/*Calls the function make_north() to make the orientation of the bot to north upon reaching the pickup point.*/
	make_north(po);
	stop();
	/*Specific conditions to align the bot perpendicularly for nut detection.*/
	right_turn_wls();
	left_turn_wls();

	/*Checking for the nut at the pickup point.*/
	while (1)
	{   /*Uses the function check_color() for checking the colour.*/
		co = check_color();
		if (co != 'X')
		{
			printf("/n%c %d izs th color/n", co, count);
			co = check_color();
			break;
		}
		count++;
	}
	printf("\n %c %d is the colour", co, count);
	/*This condition runs if any of the colour either Green or Red is found.*/
	if (co != 'X')
	{
		pick();
		stop();
		return co;
	}
	else
		return 'X';
}

/*
*
* Function Name: make_south.
* Input: char c - current orientation.
* Output: void.
* Logic: Resets the direction of the bot to south.
* Example Call: make-north(po);
*/
void make_south(char c)
{   /*Specific conditions according to previous orientation for making the bot face south before placing.*/
	if (c == 'A')
	{
		forward();
		_delay_ms(300);
		right_turn_wls();
		stop();
		_delay_ms(1);
		o = 'S';
	}
	else if (c == 'C')
	{
		forward();
		_delay_ms(300);
		left_turn_wls();
		stop();
		_delay_ms(1);
		o = 'S';
	}
	else if (c == 'D')
	{
		forward();
		_delay_ms(300);
		left_turn_wls();
		stop();
		_delay_ms(1);
		o = 'S';
	}
	else if (c == 'B')
	{
		forward();
		_delay_ms(300);
		right_turn_wls();
		stop();
		_delay_ms(1);
		o = 'S';
	}
}

/*
*
* Function Name: placing.
* Input: char color, int goal - placing position.
* Output: void.
* Logic: Takes the values of the goal and places the nuts using some specific alignment condition.
* Example Call: placing(color,goal);
*/
void placing(char color, int goal)
{
	if (goal == 20)
	{
		forward();
		_delay_ms(300);
		make_south(po);
		right_turn_wls();
		left_turn_wls();
		_delay_ms(2);
		place();
	}
	else if (goal == 21)
	{
		forward();
		_delay_ms(300);
		make_south(po);
		right_turn_wls();
		left_turn_wls();
		_delay_ms(2);
		place();
	}
	else if (goal == 15)
	{
		make_south(po);
		right_turn_wls();
		left_turn_wls();
		_delay_ms(2);
		place();
	}
	else if (goal == 18)
	{
		make_south(po);
		right_turn_wls();
		left_turn_wls();
		_delay_ms(2);
		place();
	}
}

/*
*
* Function Name: go_place.
* Input: int start - Pick point location, int goal - Place point location, char orientation, char color..
* Output: void.
* Logic: Uses the function reroute() to take the bot to the placing location and then the function placing() to keep the nut.
* Example Call: go_place(start,goal,orientation,color);
*/
void go_place(int start, int goal, char orientation, char color)
{
	g = goal;
	/*Takes the bot to the desired empty placing location.*/
	reroute(arena, start, goal, orientation);
	/*Place the nut by making the orientation south first.*/
	placing(color, goal);
}

/*
*
* Function Name: check_color.
* Input: void.
* Output: char col - Colour of the nut.
* Logic: Uses the predefined the functions to check for the colour of the bot.
* Example Call: check_color();
*/
char check_color(void)
{
	unsigned int red_pulse_count, green_pulse_count;
	char col;
	filter_red();
	red_pulse_count = color_sensor_pulse_count;
	filter_green();
	green_pulse_count = color_sensor_pulse_count;

	if (red_pulse_count > 3500)
	{
		col = 'R';
	}
	else if (green_pulse_count > 3500)
	{
		col = 'G';
	}
	else
	{
		col = 'X';
	}
	return col;
}

/*
*
* Function Name: Final.
* Input: int start - Current position, int pick - Pick point position, int goal - Placing position,
		 char start_orientation - Placing point orientation of the bot, char pick_pont_orientation.
* Output: char color.
* Logic: Uses th pick_it_up and go_place functions to perform the pick-up and place operation.
* Example Call: final(int start, int pick, int goal, char start_orientation, char pick_point_orientation);
*/
char final(int start, int pick, int goal, char start_orientation, char pick_point_orientation)
{
	char color;
	/*This function takes the bot to the */
	color = pick_it_up(start, pick, start_orientation);
	/*If the colour found is GREEN, then the goal is set specifically to the empty spots in the green zone D-2*/
	if (color == 'G')
	{
		if (n18 == -1 && n21 == 1)
		{
			goal = 21; n21 = -1;
		}
		else if (n18 == 1 && n21 == -1)
		{
			goal = 18; n18 = -1;
		}
		else if (n18 == 1 && n21 == 1)
		{
			goal = 21; n21 = -1;
		}
		else
			printf("ERROR");
	}
	/*If the colour found is RED, then the goal is set specifically to the empty spots in the red zone D-!*/
	if (color == 'R')
	{
		if (n15 == -1 && n20 == 1)
		{
			goal = 20; n20 = -1;
		}
		else if (n15 == 1 && n20 == -1)
		{
			goal = 15; n15 = -1;
		}
		else if (n15 == 1 && n20 == 1)
		{
			goal = 20; n20 = -1;
		}
		else
			printf("Placing Fail - V-Rep Inconsistencies.");
	}
	printf("In Final color: %c", color);
	/*If the colour found is green or red, go_place function is called with the desired position to place the nut.*/
	if (color != 'X')
	{
		printf("\nPicked from %d and placed at%d", pick, goal);
		go_place(pick, goal, pick_point_orientation, color);
		/*Specific cases for placing at some place points.*/
		if (goal == 20 || goal == 21)
		{
			right_turn_wls(); forward_wls(1);
		}
		/*Setting the pick up point and placing point backup in variables.*/
		last_picked = pick;
		last_placed = goal;
		return color;
	}
	/*This condition is used if there was no nut found at the pickup point.*/
	else if (color == 'X')
	{
		int b; b = pick;
		if (pick == 2)
			pick = 4;
		else
			pick = pick + 1;
		printf("\nNo nut found at %d ,was oriented %c,going for, checking at %d ", b, pick_point_orientation, pick);
		/*Recursive use of function final().*/
		return (final(b, pick, goal, pick_point_orientation, 'N'));
	}
}

/*
*
* Function Name: Task_1_2.
* Input: void.
* Output: void.
* Logic: Task_1_2 logic is encapsulated in this function to complete the assigned task.
* Example Call: Task_1_2();
*/
void Task_1_2(void)
{
	char COLOR_FOUND;
	int destination = -1;
	/*Taking the bot to the arena origin (as assumed) node 10.*/
	forward_wls(2);
	forward();
	_delay_ms(150);
	//Function call 4 times for placing all the nuts. 
	/*COLOR_FOUND = final(10, 0, destination, 'N', 'N');
	COLOR_FOUND = final(last_placed, last_picked + 1, destination, 'S', 'N');
	COLOR_FOUND = final(last_placed, last_picked + 1, destination, 'S', 'N');
	COLOR_FOUND = final(last_placed, last_picked + 1, destination, 'S', 'N');
	stop();
	//Takes the bot to the starting position of simulation.
	

	if (o == 'E')
	{
		forward();
		_delay_ms(100);
		stop();
		right_turn_wls();
		forward_wls(1);
		stop();
		{forward();
		while (ADC_Conversion(1) > THRESHOLD || ADC_Conversion(2) > THRESHOLD || ADC_Conversion(3) > THRESHOLD); }
		stop();
		forward();
		_delay_ms(70);
		stop();
	}
	else if (o == 'W')
	{
		forward();
		_delay_ms(100);
		stop();
		left_turn_wls();
		forward_wls(1);
		stop();
		{forward();
		while (ADC_Conversion(1) > THRESHOLD || ADC_Conversion(2) > THRESHOLD || ADC_Conversion(3) > THRESHOLD); }
		stop();
		forward();
		_delay_ms(70);
		stop();
	}*/
	reroute(arena, 10, 26, 'N');
	printf("\n COMPLETED - SIMULATION END \n");
	_delay_ms(1000);
}
