#include <cstdlib>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <iostream>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <vector>
#include <utility>
#include <math.h>

#define TIME_STEP 64

using namespace webots;

const double MAX_DISTANCE = 40;
const int FLOOR_WIDTH = 300;
const int FLOOR_BREADTH = 300;

std::pair<int, int> real_to_virtual_coord(double x, double z) {
	return {(int)(x * 100) , (int) (z * 100)};
}

std::pair<double, double> virtual_to_real_coord(int x, int z) {
	return {((double)x / 100) , ((double) z / 100)};
}

std::pair<int, int> generate_rand_virt_coord() {
	int x = rand() % FLOOR_WIDTH - 150;
	int z = rand() % FLOOR_BREADTH - 150;
	return {x, z};
}

void rotate_in_place(Motor *wheels[]){
	wheels[0]->setVelocity(10);
	wheels[1]->setVelocity(-10);
	wheels[2]->setVelocity(10);
	wheels[3]->setVelocity(-10);
}

void move_forward(Motor *wheels[]){
	wheels[0]->setVelocity(10);
	wheels[1]->setVelocity(10);
	wheels[2]->setVelocity(10);
	wheels[3]->setVelocity(10);
}

double distance(int x1, int y1, int x2, int y2) {
	return std::sqrt((x2 - x1) * (x2 - x1) + (y2-y1) * (y2-y1));
}

// checks if the compass is pointing in the towards the direction of (x2, y2)
// (x1, y1) is the cur position given by the gps
bool is_in_direction(Compass* cp, double x1, double y1, double x2, double y2) {
	std::cout << "ComX: " << cp->getValues()[0] << std::endl;
	std::cout << "Com2: " << cp->getValues()[1] << std::endl;
	std::cout << "ComZ: " << cp->getValues()[2] << std::endl;
	std::cout << "###########" << std::endl;

	double temp_x = cp->getValues()[0];
	double temp_z = cp->getValues()[2];
	std::pair<int, int> direction_vec = real_to_virtual_coord(temp_x, temp_z);


	double magnitude = direction_vec.first * direction_vec.first + direction_vec.second * direction_vec.second;
	magnitude = std::sqrt(magnitude);

	double unit_dir_x2 = direction_vec.first / magnitude;
	double unit_dir_y2 = direction_vec.second / magnitude;

	double test_dist = distance(x1,y1,x2,y2);

	double unit_dir_x1 = (x2 - x1) / test_dist;
	double unit_dir_y1 = (y2 - y1) / test_dist;

	if((abs(unit_dir_x2 - unit_dir_x1) + abs(unit_dir_y2 - unit_dir_y1)) < 3 ) {
		return true;
	} else {
		return false;
	}
}

// Checks if the robot has reached the location
bool has_reached_location(GPS* gp, int x1, int y1) {
	std::pair<int, int> cur_position = real_to_virtual_coord(gp->getValues()[0], gp->getValues()[2]);
	if(abs(cur_position.first - x1) + abs(cur_position.second - y1) < 3) {
		return true;
	} else{
		return false;
	}
}

class Graph {
	int V;

	list<std::pair<std::pair<int,int>, std::pair<int,int>> > *adj;

	public:
		Graph(int V);

	void addEdge(int u, int v, int w);

	void shortestPath(int s);
};

Graph::Graph(int V) {
	this->V = V;
	adj = new list<std::pair<std::pair<int,int>, std::pair<int,int>>> [V];
}

void Graph::addEdge(std::pair<int,int> u, std::pair<int,int> v, std::pair<int,int> w)
{
	for(int i = 0; i < V; i++) {
		if(adj[i].first == u.first && adj[i].second == u.second) {
			adj[i].push_back(make_pair(v, w));
			break;
		}
	}

	for(int i = 0; i < V; i++) {
		if(adj[i].first == v.first && adj[i].second == v.second) {
			adj[i].push_back(make_pair(u, w));
			break;
		}
	}
}

// Use standard dijkstra's shortest path to find distance
// Source: Geeks for geeks
std::ve Graph::shortestPath(int src) {
    priority_queue< iPair, vector <iPair> , greater<iPair> > pq;
  
    vector<int> dist(V, INF);
  
    pq.push(make_pair(0, src));
    dist[src] = 0;
  
    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();
  
        list< pair<int, int> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i) {
            int v = (*i).first;
            int weight = (*i).second;
  
			return dist;
        }
    }
  
    // Print shortest distances stored in dist[]
    printf("Vertex   Distance from Source\n");
    for (int i = 0; i < V; ++i)
        printf("%d \t\t %d\n", i, dist[i]);
}

int main(int argc, char **argv) {
	// Seed Random
	srand(time(NULL));

	Robot *robot = new Robot();
	DistanceSensor *ds[2];
	char dsNames[2][10] = {"ds_right", "ds_left"};
	for (int i = 0; i < 2; i++) {
		ds[i] = robot->getDistanceSensor(dsNames[i]);
		ds[i]->enable(TIME_STEP);
	}
	Motor *wheels[4];
	char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
	for (int i = 0; i < 4; i++) {
		wheels[i] = robot->getMotor(wheels_names[i]);
		wheels[i]->setPosition(INFINITY);
		wheels[i]->setVelocity(0.0);
	}

	GPS *gp;
	gp = robot->getGPS("gps");
	gp->enable(TIME_STEP);

	Compass *cp;
	cp = robot->getCompass("compass");
	cp->enable(TIME_STEP);

	std::pair<int, int> cur_goal_pos = {130, 30};

	// Check if the new goal is in the begin state of the robot
	bool is_begin_state = false;
	while (robot->step(TIME_STEP) != -1) {
		for (int i = 0; i < 2; i++) {
			std::cout << ds[i]->getValue() << std::endl;

			if (ds[i]->getValue() < 200.0)
				rotate_in_place(wheels);
		}
		std::cout << "###########" << std::endl;

		double cur_x = gp->getValues()[0];
		double cur_z = gp->getValues()[1];

		// Check if the compass is pointing in the general direction of the goal_position
		if(is_in_direction(cp, cur_x, cur_z, cur_goal_pos.first, cur_goal_pos.second)) {
			if(has_reached_location(gp, cur_goal_pos.first, cur_goal_pos.second)) {

			}

			move_forward(wheels);
		} else {
			rotate_in_place(wheels);
		}
		// rotate_in_place(wheels);

		std::cout << "X: " << gp->getValues()[0] << std::endl;
		std::cout << "Y: " << gp->getValues()[1] << std::endl;
		std::cout << "Z: " << gp->getValues()[2] << std::endl;
		std::cout << "###########" << std::endl;

		/* move_forward(wheels); */

	}
	delete robot;
	return 0;  // EXIT_SUCCESS
}
