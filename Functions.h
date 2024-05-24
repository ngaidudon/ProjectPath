#pragma once
#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <cmath>
void setGrid();
void displayGrid();
void setAdj();
void displayAdj();
void setWall(vector<vector<double>>& adj, int wallNode);
void resetWalls();
void BFS(vector<vector<double>>& matrix, int start, int end);
void Dijkstra(vector<vector<double>>& matrix, int start, int end);
double euclideanHeuristic(int u, int end);
void AStar(vector<vector<double>>& graph, int start, int end);