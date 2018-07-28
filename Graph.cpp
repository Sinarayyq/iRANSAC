#pragma once

#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <QDebug>

using namespace std;

class Graph
{
	unordered_map<int, const unordered_map<int, int>> vertices;

public:
	void add_vertex(int name, const unordered_map<int, int>& edges)
	{
		// Insert the connected nodes in unordered map
		vertices.insert(unordered_map<int, const unordered_map<int, int>>::value_type(name, edges));
	}

	vector<int> shortest_path(int start, int finish)
	{
		// Second arguments -> distances
		// Find the smallest distance in the already in closed list and push it in -> previous
		unordered_map<int, int> distances;
		unordered_map<int, int> previous;
		vector<int> nodes; // Open list
		vector<int> path; // Closed list

		auto comparator = [&](int left, int right) { return distances[left] > distances[right]; };

		for (auto& vertex : vertices)
		{
			if (vertex.first == start)
			{
				distances[vertex.first] = 0;
			}
			else
			{
				distances[vertex.first] = numeric_limits<int>::max();
			}

			nodes.push_back(vertex.first);
			push_heap(begin(nodes), end(nodes), comparator);
		}

		while (!nodes.empty())
		{
			pop_heap(begin(nodes), end(nodes), comparator);
			int smallest = nodes.back();
			nodes.pop_back();

			std::cout << "Open list: ";
			for (std::vector<int>::const_iterator i = nodes.begin(); i != nodes.end(); ++i)
				std::cout << *i << ' ';
			std::cout << std::endl;

			if (smallest == finish)
			{
				while (previous.find(smallest) != end(previous))
				{
					//QDebug mdebug = qDebug();
					path.push_back(smallest);
					smallest = previous[smallest];
					std::cout << "Closed list: ";
					//mdebug << "Closed list: ";
					for (std::vector<int>::const_iterator i = path.begin(); i != path.end(); ++i)
						//mdebug << *i << ' ';
						std::cout << *i << ' ';
					//mdebug << endl;
					std::cout << std::endl;
				}

				break;
			}

			if (distances[smallest] == numeric_limits<int>::max())
			{
				break;
			}

			for (auto& neighbor : vertices[smallest])
			{
				int alt = distances[smallest] + neighbor.second;
				if (alt < distances[neighbor.first])
				{
					distances[neighbor.first] = alt;
					previous[neighbor.first] = smallest;
					make_heap(begin(nodes), end(nodes), comparator);
				}
			}
		}

		return path;
	}
};