#include "stdafx.h"
#include<cstdio>
#include<iostream>
#include<vector>
#include<algorithm>
#include<cmath>
#include"APF.h"


void printArray(int** arr, int rows, int cols) {
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            std::cout << arr[i][j] << " ";
        }
        std::cout << std::endl;
    }
}



int main()
{

	//APF 
	typedef APF::Vector2D Vector2D;
	double k_att = 1.0;
	double k_rep = 0.8;
	double rr = 2;
	double step_size = .2;
	int max_iters = 500;
	double goal_threashold = .2;
	Vector2D start = Vector2D(0, 0);
	Vector2D goal = Vector2D(15, 15);
	std::vector<Vector2D> obstacles = {Vector2D(1, 4),Vector2D(2, 4),Vector2D(3, 4),Vector2D(6, 1),\
		Vector2D(6, 7),Vector2D(10, 6),Vector2D(11, 12),Vector2D(14, 14) };
																																								  //Basic_APF apf = Basic_APF(start, goal, obstacles, k_att, k_rep, rr, step_size, max_iters, goal_threashold);
	APF::Improved_APF apf = APF::Improved_APF(start, goal, obstacles, k_att, k_rep, rr, step_size, max_iters, goal_threashold);
	bool is_successed = apf.path_plan();
	std::vector<Vector2D> planned_path = apf.path;//规划后的路径
	std::cout << (is_successed ? "True":"False") << std::endl;


	// Plot
	int** arr = new int*[16];
    for (int i = 0; i < 16; i++) {
        arr[i] = new int[16];
    }

	// obstacles
	arr[1][4] = 2;
	arr[2][4] = 2;
	arr[3][4] = 2;
	arr[6][1] = 2;
	arr[6][7] = 2;
	arr[10][6] = 2;
	arr[11][12] = 2;
	arr[14][14] = 2;
	// path
	apf.write_path_to_arr(1, arr);
	arr[0][0] = 3; 	// start
	arr[15][15] = 4; // end
	printArray(arr, 16, 16);

    // 释放内存
    for (int i = 0; i < 16; i++) {
        delete[] arr[i];
    }
    delete[] arr;
	
	// system("pause");   // In Windows
	return 0;
}
