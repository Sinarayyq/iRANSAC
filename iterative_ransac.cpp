#include "iterative_ransac.h"
#include "io.h"







//================================dijkstra=====================================
#pragma region
// 示例类：边的结构体(用来演示)
class EData
{
public:
	int start; // 边的起点
	int end;   // 边的终点
	double weight; // 边的权重

public:
	EData() {}
	EData(int s, int e, double w) :start(s), end(e), weight(w) {}
};

// 邻接表
class ListUDG
{
#define MAX 10000
#define INF         (~(0x1<<31))        // 最大值(即0X7FFFFFFF)
private: // 内部类

	class ENode             // 邻接表中表对应的链表的顶点
	{
		int ivex;           // 该边所指向的顶点的位置
		double weight;         // 该边的权
		ENode *nextEdge;    // 指向下一条弧的指针
		friend class ListUDG;
	};


	class VNode             // 邻接表中表的顶点
	{
		int data;          // 顶点信息
		ENode *firstEdge;   // 指向第一条依附该顶点的弧
		friend class ListUDG;
	};

private: // 私有成员
	int mVexNum;             // 图的顶点的数目
	int mEdgNum;             // 图的边的数目
	VNode mVexs[MAX];

public:
	// 创建邻接表对应的图(自己输入)
	//ListUDG();
	// 创建邻接表对应的图(用已提供的数据)
	ListUDG(int vexs[], int vlen, EData *edges[], int elen);
	~ListUDG();

	void dijkstra(int vs, int vexs[], double dist[]);


private:

	// 返回ch的位置
	int getPosition(int ch);

	// 将node节点链接到list的最后
	void linkLast(ENode *list, ENode *node);
	// 获取边<start, end>的权值；若start和end不是连通的，则返回无穷大。
	double getWeight(int start, int end);

};

/*
* 创建邻接表对应的图(用已提供的数据)
*/
ListUDG::ListUDG(int vexs[], int vlen, EData *edges[], int elen)
{
	int c1, c2;
	int i, p1, p2;
	double weight;
	ENode *node1, *node2;

	// 初始化"顶点数"和"边数"
	mVexNum = vlen;
	mEdgNum = elen;
	// 初始化"邻接表"的顶点
	for (i = 0; i<mVexNum; i++)
	{
		mVexs[i].data = vexs[i];
		mVexs[i].firstEdge = NULL;
	}

	// 初始化"邻接表"的边
	for (i = 0; i<mEdgNum; i++)
	{
		// 读取边的起始顶点和结束顶点
		c1 = edges[i]->start;
		c2 = edges[i]->end;
		weight = edges[i]->weight;

		p1 = getPosition(c1);
		p2 = getPosition(c2);
		// 初始化node1
		node1 = new ENode();
		node1->ivex = p2;
		node1->weight = weight;
		// 将node1链接到"p1所在链表的末尾"
		if (mVexs[p1].firstEdge == NULL)
			mVexs[p1].firstEdge = node1;
		else
			linkLast(mVexs[p1].firstEdge, node1);
		// 初始化node2
		node2 = new ENode();
		node2->ivex = p1;
		node2->weight = weight;
		// 将node2链接到"p2所在链表的末尾"
		if (mVexs[p2].firstEdge == NULL)
			mVexs[p2].firstEdge = node2;
		else
			linkLast(mVexs[p2].firstEdge, node2);
	}
}

/*
* 析构函数
*/
ListUDG::~ListUDG()
{
}

/*
* 将node节点链接到list的最后
*/
void ListUDG::linkLast(ENode *list, ENode *node)
{
	ENode *p = list;

	while (p->nextEdge)
		p = p->nextEdge;
	p->nextEdge = node;
}

/*
* 返回ch的位置
*/
int ListUDG::getPosition(int ch)
{
	int i;
	for (i = 0; i<mVexNum; i++)
		if (mVexs[i].data == ch)
			return i;
	return -1;
}

/*
* 获取边<start, end>的权值；若start和end不是连通的，则返回无穷大。
*/
double ListUDG::getWeight(int start, int end)
{
	ENode *node;

	if (start == end)
		return 0;

	node = mVexs[start].firstEdge;
	while (node != NULL)
	{
		if (end == node->ivex)
			return node->weight;
		node = node->nextEdge;
	}

	return INF;
}

/*
* Dijkstra最短路径。
* 即，统计图中"顶点vs"到其它各个顶点的最短路径。
*
* 参数说明：
*       vs -- 起始顶点(start vertex)。即计算"顶点vs"到其它顶点的最短路径。
*     prev -- 前驱顶点数组。即，prev[i]的值是"顶点vs"到"顶点i"的最短路径所经历的全部顶点中，位于"顶点i"之前的那个顶点。
*     dist -- 长度数组。即，dist[i]是"顶点vs"到"顶点i"的最短路径的长度。
*/
void ListUDG::dijkstra(int vs, int prev[], double dist[])
{
	int i, j, k;
	double min;
	double tmp;
	int flag[MAX];      // flag[i]=1表示"顶点vs"到"顶点i"的最短路径已成功获取。

	// 初始化
	for (i = 0; i < mVexNum; i++)
	{
		flag[i] = 0;                // 顶点i的最短路径还没获取到。
		prev[i] = 0;                // 顶点i的前驱顶点为0。
		dist[i] = getWeight(vs, i);  // 顶点i的最短路径为"顶点vs"到"顶点i"的权。
	}

	// 对"顶点vs"自身进行初始化
	flag[vs] = 1;
	dist[vs] = 0;

	// 遍历mVexNum-1次；每次找出一个顶点的最短路径。
	for (i = 1; i < mVexNum; i++)
	{
		// 寻找当前最小的路径；
		// 即，在未获取最短路径的顶点中，找到离vs最近的顶点(k)。
		min = INF;
		for (j = 0; j < mVexNum; j++)
		{
			if (flag[j] == 0 && dist[j]<min)
			{
				min = dist[j];
				k = j;
			}
		}
		// 标记"顶点k"为已经获取到最短路径
		flag[k] = 1;

		// 修正当前最短路径和前驱顶点
		// 即，当已经"顶点k的最短路径"之后，更新"未获取最短路径的顶点的最短路径和前驱顶点"。
		for (j = 0; j < mVexNum; j++)
		{
			tmp = getWeight(k, j);
			tmp = (tmp == INF ? INF : (min + tmp)); // 防止溢出
			if (flag[j] == 0 && (tmp  < dist[j]))
			{
				dist[j] = tmp;
				prev[j] = k;
			}
		}
	}

	// 打印dijkstra最短路径的结果
	/*std::cout << "dijkstra(" << mVexs[vs].data << "): " << std::endl;
	for (i = 0; i < mVexNum; i++)
	std::cout << "  shortest(" << mVexs[vs].data << ", " << mVexs[i].data << ")=" << dist[i] << std::endl;*/
}

void sortrows(std::vector<std::vector<double>>& flat_points_matrix, int col)
{
	std::sort(flat_points_matrix.begin(),
		flat_points_matrix.end(),
		[col](const std::vector<double>& lhs, const std::vector<double>& rhs) {
		return lhs[col] < rhs[col];
	});
}

int OutputNormals(std::vector<FMesh::Normal> subsample_points_normals)
{
	std::ofstream outdata;
	std::string name_file = "C:\\Users\\sinara\\Desktop\\subsample_points_normals.txt";
	outdata.open(name_file, std::ios::out);
	outdata.clear();

	const size_t size = subsample_points_normals.size();
	for (std::size_t i = 0; i < size; ++i)
	{
		outdata << subsample_points_normals[i].data()[0] << " " << subsample_points_normals[i].data()[1] << " " << subsample_points_normals[i].data()[2] << std::endl;
		//outdata << points_on_mesh[i].x() << " " << points_on_mesh[i].y() << " " << points_on_mesh[i].z() << std::endl;
	}
	return 0;
}

int DijkstraAlgorithm(FMesh *mesh, std::vector<FMesh::Point> &points, FMesh::Point &start_point, std::vector<std::vector<double>> &distance)
{
	/*size_t size_points_on_mesh = mesh.number_of_vertices();
	size_t size_edges = mesh.edges().size();*/
	size_t size_points_on_mesh = (*mesh).n_vertices();
	size_t size_edges = (*mesh).n_edges();
	//FMesh  m;

	int prev[MAX] = { 0 };
	double dist[MAX] = { 0 };

	// 顶点
	//char vexs[] = { '0', '1', '2', '3', '4', '5', '16' };
	int *vexs = new int[size_points_on_mesh];
	for (size_t i = 0; i < size_points_on_mesh; i++)
	{
		vexs[i] = i;
		//std::cout << vexs[i] << std::endl;
	}

	//weight

	//EData* edges_temp = new EData[size_edges];
	EData **edges;
	edges = (EData **)malloc(size_edges * sizeof(EData *));
	/*for (edge_iterator it = mesh.edges_begin(); it != mesh.edges_end(); ++it)
	{
	vertex_descriptor s = mesh.vertex((*it), 0);
	vertex_descriptor t = mesh.vertex((*it), 1);
	edges[count] = (EData *)malloc(sizeof(EData));
	edges[count]->start = (int)s;
	edges[count]->end = (int)t;
	edges[count]->weight = sqrt((mesh.point(s) - mesh.point(t)).squared_length());
	count++;
	}*/
	int count = 0;
	FMesh::VertexHandle s, t;
	for (FMesh::EdgeIter it = (*mesh).edges_begin(); it != (*mesh).edges_end(); ++it)
	{
		s = (*mesh).to_vertex_handle((*mesh).halfedge_handle(*it, 0));
		t = (*mesh).from_vertex_handle((*mesh).halfedge_handle(*it, 0));
		edges[count] = (EData *)malloc(sizeof(EData));
		edges[count]->start = s.idx();
		edges[count]->end = t.idx();
		edges[count]->weight = sqrt((*mesh).calc_edge_sqr_length((*it)));
		count++;
	}
	/*std::cout << mesh.point(s) << std::endl;
	std::cout << mesh.point(t) << std::endl;
	std::cout << "count =" << count << std::endl;
	std::cout << "edges[count]->start = " << edges[count - 1]->start << std::endl;
	std::cout << "edges[count]->end = " << edges[count - 1]->end << std::endl;
	std::cout << "edges[count]->weight = " << edges[count - 1]->weight << std::endl;*/

	/*for (int i = 0; i < size_edges; i++)
	{
	edges[i] = &(edges_temp[i]);
	}*/

	// 边
	//EData *edges[] = {
	//	// 起点 终点 权
	//	new EData('0', '1', 12),
	//	new EData('0', '5', 16),
	//	new EData('0', '16', 14),
	//	new EData('1', '2', 10),
	//	new EData('1', '5',  7),
	//	new EData('2', '3',  3),
	//	new EData('2', '4',  5),
	//	new EData('2', '5',  6),
	//	new EData('3', '4',  4),
	//	new EData('4', '5',  2),
	//	new EData('4', '16',  8),
	//	new EData('5', '16',  9),
	//};
	/*int vlen = sizeof(vexs) / sizeof(vexs[0]);
	int elen = sizeof(edges) / sizeof(edges[0]);*/

	int vlen = (int)size_points_on_mesh;
	int elen = (int)size_edges;
	ListUDG* pG;


	// 自定义"图"(输入矩阵队列)
	//pG = new ListUDG();
	// 采用已有的"图"
	pG = new ListUDG(vexs, vlen, edges, elen);

	int no_start_point = 0;
	for (FMesh::VertexIter it = (*mesh).vertices_begin(); it != (*mesh).vertices_end(); ++it)
	{
		/*std::cout << mesh.point(*it) << std::endl;
		std::cout << start_point - mesh.point(*it) << std::endl;
		std::cout << (start_point - mesh.point(*it)).length() << std::endl;*/
		//outdata << mesh.point(*it) << std::endl;
		if ((start_point - (*mesh).point(*it)).length() < 0.2)
		{
			//std::cout << no_start_point << std::endl;
			break;
		}
		else
		{
			no_start_point++;
			//std::cout << no_start_point << "+" << std::endl << std::endl;
		}
	}

	/*for (vertex_iterator it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it)
	{
	if (start_point == mesh.point((*it)))
	{
	break;
	}
	no_start_point++;
	}*/
	// dijkstra算法获取start point到其它各个顶点的最短距离
	pG->dijkstra(no_start_point, prev, dist);



	//(*mesh).request_face_normals();
	//(*mesh).request_vertex_normals();
	//(*mesh).update_normals();
	//std::vector<FMesh::Normal> subsample_points_normals;


	size_t size_subsampled_points = points.size();
	distance.resize(size_subsampled_points);

	for (size_t i = 0; i < size_subsampled_points; i++)
	{
		//if (i == 200)
		//{
		//	(*distance)[i].resize(2);
		//	int number = 0;
		//	for (FMesh::VertexIter t = mesh.vertices_begin(); t != mesh.vertices_end(); ++t)
		//	{
		//		//std::cout << "+" << std::endl;
		//		if ((subsampled_points[i] - mesh.point(*t)).length() < 10e-8)
		//		{
		//			std::cout << subsampled_points[i] << std::endl << mesh.point(*t) << std::endl;
		//			(*distance)[i][1] = i;
		//			(*distance)[i][0] = dist[number];

		//			std::cout << (*distance)[i][0] << std::endl << (*distance)[i][1] << std::endl << number << std::endl << std::endl;
		//		}
		//		number++;
		//	}
		//}
		//std::cout << i << "+" << std::endl;
		/*std::ofstream outdata;
		std::string name_file = "C:\\Users\\sinara\\Desktop\\subsample_points_normals.txt";
		outdata.open(name_file, std::ios::out);
		outdata.clear();*/



		distance[i].resize(2);
		int number = 0;
		for (FMesh::VertexIter it = (*mesh).vertices_begin(); it != (*mesh).vertices_end(); ++it)
		{
			//std::cout << "+" << std::endl;
			/*if (number == 127)
			{
			std::cout << subsampled_points[i] << std::endl << mesh.point(*t) << std::endl << (subsampled_points[i] - mesh.point(*t)) << std::endl;
			std::cout << (subsampled_points[i] - mesh.point(*t)).length() << std::endl;
			auto temp = (subsampled_points[i] - mesh.point(*t));
			}*/

			if ((points[i] - (*mesh).point(*it)).length() < 0.2)
			{
				//subsample_points_normals.push_back(((*mesh).normal(*it)));
				//outdata << (mesh.normal(*it)).data()[0] << " " << (mesh.normal(*it)).data()[1] << " " << (mesh.normal(*it)).data()[2] << std::endl;
				//std::cout << subsampled_points[i] << std::endl << mesh.point(*t) << std::endl;
				distance[i][1] = i;
				distance[i][0] = dist[number];
				//std::cout << subsampled_points[i] << std::endl << mesh.point(*it) << std::endl << (subsampled_points[i] - mesh.point(*it)) << std::endl;
				//std::cout << (subsampled_points[i] - (*mesh).point(*it)).length() << std::endl;
				//std::cout << (*distance)[i][0] << std::endl << (*distance)[i][1] << std::endl << number << std::endl << std::endl;
			}
			number++;
		}
	}
	//OutputNormals(subsample_points_normals);
	/*for (int i = 0; i < size_subsampled_points; i++)
	{
	(*distance)[i].resize(2);
	int number = 0;
	for (vertex_iterator it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it)
	{
	if (mesh.point((*it)) == subsampled_points[i])
	{
	(*distance)[i][1] = i;
	(*distance)[i][0] = dist[number];
	}
	number++;
	}
	}*/

	/*for (size_t i = 0; i < size_subsampled_points; ++i)
	{
	std::cout << (*distance)[i][0] << "  " << (*distance)[i][1] << std::endl;
	}*/
	sortrows(distance, 0);
	/*for (size_t i = 0; i < size_subsampled_points; ++i)
	{
	std::cout << distance[i][0] << "  " << distance[i][1] << std::endl;
	}*/
	//std::cout << (*distance).size() << std::endl;
	/*std::cout << (*distance)[0][1] << std::endl << (*distance)[0][0] << std::endl << std::endl << std::endl;
	std::cout << (*distance)[1][1] << std::endl << (*distance)[1][0] << std::endl << std::endl << std::endl;
	std::cout << (*distance)[2][1] << std::endl << (*distance)[2][0] << std::endl << std::endl << std::endl;
	std::cout << (*distance)[3][1] << std::endl << (*distance)[3][0] << std::endl << std::endl << std::endl;*/
	//std::vector<Point, double> distance;
	//for (int i = 0; i < size_subsampled_points; i++)
	//{
	//distance.push_back(subsampled_points[temp_dist[i][1]], temp_dist[i][0]);
	//}

	// floyd算法获取各个顶点之间的最短距离
	//pG->floyd(path, floy);
	//Visualizer(points_on_mesh, subsampled_points, *distance);


	return 0;
}

int DijkstraAlgorithm(FMesh *mesh, std::vector<FMesh::Point> &points, FMesh::Point &start_point, std::vector<int> &sequence_number)
{
	/*size_t size_points_on_mesh = mesh.number_of_vertices();
	size_t size_edges = mesh.edges().size();*/
	size_t size_points_on_mesh = (*mesh).n_vertices();
	size_t size_edges = (*mesh).n_edges();
	//FMesh  m;

	int prev[MAX] = { 0 };
	double dist[MAX] = { 0 };

	// 顶点
	//char vexs[] = { '0', '1', '2', '3', '4', '5', '16' };
	int *vexs = new int[size_points_on_mesh];
	for (size_t i = 0; i < size_points_on_mesh; i++)
	{
		vexs[i] = i;
		//std::cout << vexs[i] << std::endl;
	}

	//weight

	//EData* edges_temp = new EData[size_edges];
	EData **edges;
	edges = (EData **)malloc(size_edges * sizeof(EData *));
	/*for (edge_iterator it = mesh.edges_begin(); it != mesh.edges_end(); ++it)
	{
	vertex_descriptor s = mesh.vertex((*it), 0);
	vertex_descriptor t = mesh.vertex((*it), 1);
	edges[count] = (EData *)malloc(sizeof(EData));
	edges[count]->start = (int)s;
	edges[count]->end = (int)t;
	edges[count]->weight = sqrt((mesh.point(s) - mesh.point(t)).squared_length());
	count++;
	}*/
	int count = 0;
	FMesh::VertexHandle s, t;
	for (FMesh::EdgeIter it = (*mesh).edges_begin(); it != (*mesh).edges_end(); ++it)
	{
		s = (*mesh).to_vertex_handle((*mesh).halfedge_handle(*it, 0));
		t = (*mesh).from_vertex_handle((*mesh).halfedge_handle(*it, 0));
		edges[count] = (EData *)malloc(sizeof(EData));
		edges[count]->start = s.idx();
		edges[count]->end = t.idx();
		edges[count]->weight = sqrt((*mesh).calc_edge_sqr_length((*it)));
		count++;
	}
	/*std::cout << mesh.point(s) << std::endl;
	std::cout << mesh.point(t) << std::endl;
	std::cout << "count =" << count << std::endl;
	std::cout << "edges[count]->start = " << edges[count - 1]->start << std::endl;
	std::cout << "edges[count]->end = " << edges[count - 1]->end << std::endl;
	std::cout << "edges[count]->weight = " << edges[count - 1]->weight << std::endl;*/

	/*for (int i = 0; i < size_edges; i++)
	{
	edges[i] = &(edges_temp[i]);
	}*/

	// 边
	//EData *edges[] = {
	//	// 起点 终点 权
	//	new EData('0', '1', 12),
	//	new EData('0', '5', 16),
	//	new EData('0', '16', 14),
	//	new EData('1', '2', 10),
	//	new EData('1', '5',  7),
	//	new EData('2', '3',  3),
	//	new EData('2', '4',  5),
	//	new EData('2', '5',  6),
	//	new EData('3', '4',  4),
	//	new EData('4', '5',  2),
	//	new EData('4', '16',  8),
	//	new EData('5', '16',  9),
	//};
	/*int vlen = sizeof(vexs) / sizeof(vexs[0]);
	int elen = sizeof(edges) / sizeof(edges[0]);*/

	int vlen = (int)size_points_on_mesh;
	int elen = (int)size_edges;
	ListUDG* pG;


	// 自定义"图"(输入矩阵队列)
	//pG = new ListUDG();
	// 采用已有的"图"
	pG = new ListUDG(vexs, vlen, edges, elen);

	int no_start_point = 0;
	for (FMesh::VertexIter it = (*mesh).vertices_begin(); it != (*mesh).vertices_end(); ++it)
	{
		/*std::cout << mesh.point(*it) << std::endl;
		std::cout << start_point - mesh.point(*it) << std::endl;
		std::cout << (start_point - mesh.point(*it)).length() << std::endl;*/
		//outdata << mesh.point(*it) << std::endl;
		if ((start_point - (*mesh).point(*it)).length() < 0.2)
		{
			//std::cout << no_start_point << std::endl;
			break;
		}
		else
		{
			no_start_point++;
			//std::cout << no_start_point << "+" << std::endl << std::endl;
		}
	}

	/*for (vertex_iterator it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it)
	{
	if (start_point == mesh.point((*it)))
	{
	break;
	}
	no_start_point++;
	}*/
	// dijkstra算法获取start point到其它各个顶点的最短距离
	pG->dijkstra(no_start_point, prev, dist);



	//(*mesh).request_face_normals();
	//(*mesh).request_vertex_normals();
	//(*mesh).update_normals();
	//std::vector<FMesh::Normal> subsample_points_normals;


	size_t size_subsampled_points = points.size();
	std::vector<std::vector<double>> distance;
	distance.resize(size_subsampled_points);

	for (size_t i = 0; i < size_subsampled_points; i++)
	{
		//if (i == 200)
		//{
		//	(*distance)[i].resize(2);
		//	int number = 0;
		//	for (FMesh::VertexIter t = mesh.vertices_begin(); t != mesh.vertices_end(); ++t)
		//	{
		//		//std::cout << "+" << std::endl;
		//		if ((subsampled_points[i] - mesh.point(*t)).length() < 10e-8)
		//		{
		//			std::cout << subsampled_points[i] << std::endl << mesh.point(*t) << std::endl;
		//			(*distance)[i][1] = i;
		//			(*distance)[i][0] = dist[number];

		//			std::cout << (*distance)[i][0] << std::endl << (*distance)[i][1] << std::endl << number << std::endl << std::endl;
		//		}
		//		number++;
		//	}
		//}
		//std::cout << i << "+" << std::endl;
		/*std::ofstream outdata;
		std::string name_file = "C:\\Users\\sinara\\Desktop\\subsample_points_normals.txt";
		outdata.open(name_file, std::ios::out);
		outdata.clear();*/



		distance[i].resize(2);
		int number = 0;
		for (FMesh::VertexIter it = (*mesh).vertices_begin(); it != (*mesh).vertices_end(); ++it)
		{
			//std::cout << "+" << std::endl;
			/*if (number == 127)
			{
			std::cout << subsampled_points[i] << std::endl << mesh.point(*t) << std::endl << (subsampled_points[i] - mesh.point(*t)) << std::endl;
			std::cout << (subsampled_points[i] - mesh.point(*t)).length() << std::endl;
			auto temp = (subsampled_points[i] - mesh.point(*t));
			}*/

			if ((points[i] - (*mesh).point(*it)).length() < 0.2)
			{
				//subsample_points_normals.push_back(((*mesh).normal(*it)));
				//outdata << (mesh.normal(*it)).data()[0] << " " << (mesh.normal(*it)).data()[1] << " " << (mesh.normal(*it)).data()[2] << std::endl;
				//std::cout << subsampled_points[i] << std::endl << mesh.point(*t) << std::endl;
				distance[i][1] = i;
				distance[i][0] = dist[number];
				//std::cout << subsampled_points[i] << std::endl << mesh.point(*it) << std::endl << (subsampled_points[i] - mesh.point(*it)) << std::endl;
				//std::cout << (subsampled_points[i] - (*mesh).point(*it)).length() << std::endl;
				//std::cout << (*distance)[i][0] << std::endl << (*distance)[i][1] << std::endl << number << std::endl << std::endl;
			}
			number++;
		}
	}
	//OutputNormals(subsample_points_normals);
	/*for (int i = 0; i < size_subsampled_points; i++)
	{
	(*distance)[i].resize(2);
	int number = 0;
	for (vertex_iterator it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it)
	{
	if (mesh.point((*it)) == subsampled_points[i])
	{
	(*distance)[i][1] = i;
	(*distance)[i][0] = dist[number];
	}
	number++;
	}
	}*/

	/*for (size_t i = 0; i < size_subsampled_points; ++i)
	{
	std::cout << (*distance)[i][0] << "  " << (*distance)[i][1] << std::endl;
	}*/
	sortrows(distance, 0);
	for (size_t i = 0; i < size_subsampled_points; i++)
	{
		sequence_number.push_back(distance[i][1]);
	}


	/*for (size_t i = 0; i < size_subsampled_points; ++i)
	{
	std::cout << distance[i][0] << "  " << distance[i][1] << std::endl;
	}*/
	//std::cout << (*distance).size() << std::endl;
	/*std::cout << (*distance)[0][1] << std::endl << (*distance)[0][0] << std::endl << std::endl << std::endl;
	std::cout << (*distance)[1][1] << std::endl << (*distance)[1][0] << std::endl << std::endl << std::endl;
	std::cout << (*distance)[2][1] << std::endl << (*distance)[2][0] << std::endl << std::endl << std::endl;
	std::cout << (*distance)[3][1] << std::endl << (*distance)[3][0] << std::endl << std::endl << std::endl;*/
	//std::vector<Point, double> distance;
	//for (int i = 0; i < size_subsampled_points; i++)
	//{
	//distance.push_back(subsampled_points[temp_dist[i][1]], temp_dist[i][0]);
	//}

	// floyd算法获取各个顶点之间的最短距离
	//pG->floyd(path, floy);
	//Visualizer(points_on_mesh, subsampled_points, *distance);


	return 0;
}
#pragma endregion
//================================dijkstra=====================================


//commentted block

//std::vector<FMesh::Point> ReadSubsampledPoints()
//{
//	std::vector<FMesh::Point> subsampled_points;
//	std::string name_file = "C:\\Users\\sinara\\Desktop\\subsampled_points.txt";
//	char a[100];
//	double x = 0, y = 0, z = 0;
//	int count = 0;
//
//	std::ifstream in(name_file, std::ifstream::in);
//	do
//	{
//		in.getline(a, 100, '\n');
//
//		if (std::sscanf(a, "%lf%lf%lf", &x, &y, &z) == 3)
//		{
//			/*if (count == 0)
//			{
//			x0 = x;
//			y0 = y;
//			z0 = z;
//			}
//			points_on_lines.push_back(Point_3(x - x0, y - y0, z - z0));*/
//			subsampled_points.push_back(FMesh::Point(x, y, z));
//		}
//		count++;
//	} while (!in.eof());
//
//	return subsampled_points;
//}
//
//
//int OutputAsTXT(std::vector<std::vector<float>> distance, std::string name_file)
//{
//	std::ofstream outdata;
//
//	outdata.open(name_file, std::ios::out);
//	outdata.clear();
//
//	const size_t size = distance.size();
//	for (std::size_t i = 0; i < size; ++i)
//	{
//		outdata << std::fixed << std::setprecision(0) << distance[i][1] << std::endl;
//		//outdata << points_on_mesh[i].x() << " " << points_on_mesh[i].y() << " " << points_on_mesh[i].z() << std::endl;
//	}
//	return 0;
//}
//
//int main(int argc, char *argv[])
//{
//	FMesh mesh;
//	FMesh *mesh_test = &mesh;
//	std::string load_file;
//	std::getline(std::cin, load_file);
//	if (!OpenMesh::IO::read_mesh(mesh, load_file))
//	{
//		std::cerr << "read error\n";
//		exit(1);
//	}
//
//
//	std::vector<FMesh::Point> subsampled_points = ReadSubsampledPoints();
//
//	std::vector<std::vector<float>> distance;
//
//	for (int i = 0; i < subsampled_points.size(); i++)
//	{
//		DijkstraAlgorithm(mesh_test, subsampled_points, subsampled_points[i], distance);
//		std::string name_output = "C:\\Users\\sinara\\Desktop\\distance\\";
//		name_output += std::to_string(i);
//		name_output += ".txt";
//		//std::cout << name_output << std::endl;
//		OutputAsTXT(distance, name_output);
//	}
//
//	return 0;
//}




//=============================pcl subsampling=================================
#pragma region
int SubsamplingPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, 
	               pcl::PointCloud<pcl::PointXYZ>::Ptr &subsampled_points, pcl::PointCloud<pcl::Normal>::Ptr &subsampled_points_normals)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> sample;
	sample.setInputCloud(cloud);
	sample.setLeafSize(120, 120, 120);
	sample.filter(*cloud_filtered);
	//visualizePointCloud(cloud, cloud_filtered, "cloud_filtered", xy);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	int K = 1;

	size_t size_cloud_filtered = cloud_filtered->size();
	std::vector<int> index(K);
	std::vector<float> d(K);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < size_cloud_filtered; i++)
	{
		if (kdtree.nearestKSearch(cloud_filtered->points[i], K, index, d) > 0)
		{
			subsampled_points->push_back(pcl::PointXYZ(cloud->points[index[K - 1]].x, cloud->points[index[K - 1]].y, cloud->points[index[K - 1]].z));
			subsampled_points_normals->push_back(pcl::Normal(normals->points[index[K - 1]].normal_x, normals->points[index[K - 1]].normal_y, normals->points[index[K - 1]].normal_z));
		}
	}
	//visualizePointCloud(subsampled_points, subsampled_points_normals, "subsampled_points_and_normals", xy);
	//visualizePointCloud(cloud, subsampled_points, "subsampled_points", xy);

	return 0;
}

int SubsamplingPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &subsampled_points, double edge_length)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> sample;
	sample.setInputCloud(cloud);
	//sample.setLeafSize(120, 120, 120);	//WIP. size of voxel grid. 120 mm. Should be determined by the bounding box of mesh.
	double para = 3.5;
	sample.setLeafSize(para*edge_length, para*edge_length, para*edge_length);
	sample.filter(*cloud_filtered);
	//visualizePointCloud(cloud, cloud_filtered, "cloud_filtered", xy);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	int K = 1;

	size_t size_cloud_filtered = cloud_filtered->size();
	std::vector<int> index(K);
	std::vector<float> d(K);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < size_cloud_filtered; i++)
	{
		if (kdtree.nearestKSearch(cloud_filtered->points[i], K, index, d) > 0)
		{
			subsampled_points->push_back(pcl::PointXYZ(cloud->points[index[K - 1]].x, cloud->points[index[K - 1]].y, cloud->points[index[K - 1]].z));
			//subsampled_points_normals->push_back(pcl::Normal(normals->points[index[K - 1]].normal_x, normals->points[index[K - 1]].normal_y, normals->points[index[K - 1]].normal_z));
		}
	}
	//visualizePointCloud(subsampled_points, subsampled_points_normals, "subsampled_points_and_normals", xy);
	//visualizePointCloud(cloud, subsampled_points, "subsampled_points", xy);

	return 0;
}

int SubsamplingPCL(FMesh *mesh, std::vector<FMesh::Point> &subsampled_points, double edge_length)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_on_mesh(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled_points_PCL(new pcl::PointCloud<pcl::PointXYZ>);
	OpenMeshMesh2PCLCloud(mesh, points_on_mesh);
	SubsamplingPCL(points_on_mesh, subsampled_points_PCL, edge_length);
	PCL2OpenMeshVectorPoint(subsampled_points_PCL, subsampled_points);

	return 0;
}

int SubsamplingPCL(FMesh *mesh, std::vector<FMesh::Point> &subsampled_points, std::vector<FMesh::Normal> &subsampled_points_normals)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_on_mesh(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr points_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled_points_PCL(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr points_normals_PCL(new pcl::PointCloud<pcl::Normal>);


	OpenMeshMesh2PCLCloudNormal(mesh, points_on_mesh, points_normals);
	//OpenMeshMesh2PCLCloud(mesh, points_on_mesh);
	SubsamplingPCL(points_on_mesh, points_normals, subsampled_points_PCL, points_normals_PCL);
	PCL2OpenMeshVectorPoint(subsampled_points_PCL, subsampled_points);
	PCL2OpenMeshVectorNormal(points_normals_PCL, subsampled_points_normals);
	//ComputeNormals(mesh, );


	return 0;
}
#pragma endregion
//=============================pcl subsampling=================================




//==============================iterative_ransac===============================
#pragma region
PatchType::PatchType() :plane(0), cylinder(0), cone(0), coefficients_plane(new pcl::ModelCoefficients()),
coefficients_cylinder(new pcl::ModelCoefficients()), coefficients_cone(new pcl::ModelCoefficients()),
cloud_plane(new pcl::PointCloud<pcl::PointXYZ>), cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>), cloud_cone(new pcl::PointCloud<pcl::PointXYZ>), 
normal_plane(new pcl::PointCloud<pcl::Normal>), normal_cylinder(new pcl::PointCloud<pcl::Normal>), normal_cone(new pcl::PointCloud<pcl::Normal>),
plane2(0), cylinder2(0), cone2(0), coefficients_plane2(new pcl::ModelCoefficients()),
coefficients_cylinder2(new pcl::ModelCoefficients()), coefficients_cone2(new pcl::ModelCoefficients())
{
	//coefficients_plane = NULL;
	//coefficients_cylinder = NULL;
	//coefficients_cone = NULL;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BuildPCLCloudStructure(std::vector<FMesh::Point> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	int size_cloud = points.size();
	cloud->width = size_cloud;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(size_cloud);
	for (size_t k = 0; k < size_cloud; ++k)
	{
		cloud->points[k].x = points[k].data()[0];
		cloud->points[k].y = points[k].data()[1];
		cloud->points[k].z = points[k].data()[2];
	}

	return cloud;
	
}

pcl::PointCloud<pcl::Normal>::Ptr BuildPCLNormalStructure(std::vector<FMesh::Normal> normals)
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	int size_cloud = normals.size();

	cloud_normals->width = size_cloud;
	cloud_normals->height = 1;
	cloud_normals->is_dense = false;
	cloud_normals->points.resize(size_cloud);
	for (size_t k = 0; k < size_cloud; ++k)
	{
		cloud_normals->points[k].normal_x = normals[k].data()[0];
		cloud_normals->points[k].normal_y = normals[k].data()[1];
		cloud_normals->points[k].normal_z = normals[k].data()[2];
	}

	return cloud_normals;

}

void setSegmentationParametersForPlane(pcl::SACSegmentation<pcl::PointXYZ>& seg_plane)
{
	// Create the segmentation object
	//pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg_plane.setOptimizeCoefficients(true);
	// Mandatory
	seg_plane.setModelType(pcl::SACMODEL_PLANE);
	seg_plane.setMethodType(PLANE_METHOD_TYPE);
	seg_plane.setMaxIterations(PLANE_MAX_NUM_ITER);
	seg_plane.setDistanceThreshold(PLANE_TOL);
}

void setSegmentationParametersForCylinder(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>& seg_cyl)
{
	// Set all the parameters for cylinder segmentation object
	seg_cyl.setOptimizeCoefficients(true);
	seg_cyl.setModelType(pcl::SACMODEL_CYLINDER);
	seg_cyl.setMethodType(CYL_METHOD_TYPE);
	seg_cyl.setNormalDistanceWeight(CYL_WEIGHT_NORMAL_DISTANCE);
	seg_cyl.setMaxIterations(CYL_MAX_NUM_ITER);
	seg_cyl.setDistanceThreshold(CYL_TOL);
	seg_cyl.setRadiusLimits(CYL_MIN_RADIUS_LIMIT, CYL_MAX_RADIUS_LIMIT);
}

void setSegmentationParametersForCone(pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>& seg_cone)
{
	// Set all the parameters for cone segmentation object
	seg_cone.setOptimizeCoefficients(true);
	seg_cone.setModelType(pcl::SACMODEL_CONE);
	seg_cone.setMethodType(CONE_METHOD_TYPE);
	seg_cone.setMinMaxOpeningAngle(CONE_MIN_OPENING_ANGLE / 180.0  * M_PI, CONE_MAX_OPENING_ANGLE / 180.0 * M_PI); //it is in radiants; min=5degree, max=80degree
	seg_cone.setNormalDistanceWeight(CONE_WEIGHT_NORMAL_DISTANCE);
	seg_cone.setMaxIterations(CONE_MAX_NUM_ITER);
	seg_cone.setDistanceThreshold(CONE_TOL);
	seg_cone.setRadiusLimits(CONE_MIN_RADIUS_LIMIT, CONE_MAX_RADIUS_LIMIT);
}

//1 = points can be recognized as at least one type; 
//0 = points cannot be recognized as any type
bool Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, PatchType &patchtype_temp, bool second_run)
{
	size_t size_cloud = cloud->size();
	bool flag = 0;

	if (!second_run)
	{
		//plane
		pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients());
		setSegmentationParametersForPlane(seg_plane);
		seg_plane.setInputCloud(cloud);
		seg_plane.segment(*inliers_plane, *coefficients_plane);
		if (((inliers_plane->indices.size()) / size_cloud) > PERCENTAGE)
		{
			flag = 1;
			patchtype_temp.plane = 1;
			*(patchtype_temp.coefficients_plane) = *coefficients_plane;
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers_plane);
			extract.setNegative(false);
			extract.filter(*(patchtype_temp.cloud_plane));

			pcl::ExtractIndices<pcl::Normal> extract_normals;
			extract_normals.setInputCloud(normals);
			extract_normals.setIndices(inliers_plane);
			extract_normals.setNegative(false);
			extract_normals.filter(*(patchtype_temp.normal_plane));
		}

		//cylinder
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinder;
		pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients());
		setSegmentationParametersForCylinder(seg_cylinder);
		seg_cylinder.setInputCloud(cloud);
		seg_cylinder.setInputNormals(normals);
		seg_cylinder.segment(*inliers_cylinder, *coefficients_cylinder);
		if (((inliers_cylinder->indices.size()) / size_cloud) > PERCENTAGE)
		{
			flag = 1;
			patchtype_temp.cylinder = 1;
			*(patchtype_temp.coefficients_cylinder) = *coefficients_cylinder;
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers_cylinder);
			extract.setNegative(false);
			extract.filter(*(patchtype_temp.cloud_cylinder));

			pcl::ExtractIndices<pcl::Normal> extract_normals;
			extract_normals.setInputCloud(normals);
			extract_normals.setIndices(inliers_cylinder);
			extract_normals.setNegative(false);
			extract_normals.filter(*(patchtype_temp.normal_cylinder));
		}

		//cone
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
		pcl::PointIndices::Ptr inliers_cone(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients_cone(new pcl::ModelCoefficients());
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cone(new pcl::PointCloud<pcl::PointXYZ>);
		setSegmentationParametersForCone(seg_cone);
		seg_cone.setInputCloud(cloud);
		seg_cone.setInputNormals(normals);
		seg_cone.segment(*inliers_cone, *coefficients_cone);
		if (((inliers_cone->indices.size()) / size_cloud) > PERCENTAGE)
		{
			flag = 1;
			patchtype_temp.cone = 1;
			*(patchtype_temp.coefficients_cone) = *coefficients_cone;
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers_cone);
			extract.setNegative(false);
			extract.filter(*(patchtype_temp.cloud_cone));

			pcl::ExtractIndices<pcl::Normal> extract_normals;
			extract_normals.setInputCloud(normals);
			extract_normals.setIndices(inliers_cone);
			extract_normals.setNegative(false);
			extract_normals.filter(*(patchtype_temp.normal_cone));
		}

		return flag;
	}

	else
	{
		//plane
		pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients());
		setSegmentationParametersForPlane(seg_plane);
		seg_plane.setInputCloud(cloud);
		seg_plane.segment(*inliers_plane, *coefficients_plane);
		if (((inliers_plane->indices.size()) / size_cloud) > PERCENTAGE)
		{
			flag = 1;
			patchtype_temp.plane2 = 1;
			*(patchtype_temp.coefficients_plane2) = *coefficients_plane;
			//pcl::ExtractIndices<pcl::PointXYZ> extract;
			//extract.setInputCloud(cloud);
			//extract.setIndices(inliers_plane);
			//extract.setNegative(false);
			//extract.filter(*(patchtype_temp.cloud_plane));
			//
			//pcl::ExtractIndices<pcl::Normal> extract_normals;
			//extract_normals.setInputCloud(normals);
			//extract_normals.setIndices(inliers_plane);
			//extract_normals.setNegative(false);
			//extract_normals.filter(*(patchtype_temp.normal_plane));
		}

		//cylinder
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinder;
		pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients());
		setSegmentationParametersForCylinder(seg_cylinder);
		seg_cylinder.setInputCloud(cloud);
		seg_cylinder.setInputNormals(normals);
		seg_cylinder.segment(*inliers_cylinder, *coefficients_cylinder);
		if (((inliers_cylinder->indices.size()) / size_cloud) > PERCENTAGE)
		{
			flag = 1;
			patchtype_temp.cylinder2 = 1;
			*(patchtype_temp.coefficients_cylinder2) = *coefficients_cylinder;
			//pcl::ExtractIndices<pcl::PointXYZ> extract;
			//extract.setInputCloud(cloud);
			//extract.setIndices(inliers_cylinder);
			//extract.setNegative(false);
			//extract.filter(*(patchtype_temp.cloud_cylinder));
			//
			//pcl::ExtractIndices<pcl::Normal> extract_normals;
			//extract_normals.setInputCloud(normals);
			//extract_normals.setIndices(inliers_cylinder);
			//extract_normals.setNegative(false);
			//extract_normals.filter(*(patchtype_temp.normal_cylinder));
		}

		//cone
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
		pcl::PointIndices::Ptr inliers_cone(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients_cone(new pcl::ModelCoefficients());
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cone(new pcl::PointCloud<pcl::PointXYZ>);
		setSegmentationParametersForCone(seg_cone);
		seg_cone.setInputCloud(cloud);
		seg_cone.setInputNormals(normals);
		seg_cone.segment(*inliers_cone, *coefficients_cone);
		if (((inliers_cone->indices.size()) / size_cloud) > PERCENTAGE)
		{
			flag = 1;
			patchtype_temp.cone2 = 1;
			*(patchtype_temp.coefficients_cone2) = *coefficients_cone;
			//pcl::ExtractIndices<pcl::PointXYZ> extract;
			//extract.setInputCloud(cloud);
			//extract.setIndices(inliers_cone);
			//extract.setNegative(false);
			//extract.filter(*(patchtype_temp.cloud_cone));
			//
			//pcl::ExtractIndices<pcl::Normal> extract_normals;
			//extract_normals.setInputCloud(normals);
			//extract_normals.setIndices(inliers_cone);
			//extract_normals.setNegative(false);
			//extract_normals.filter(*(patchtype_temp.normal_cone));
		}

		return flag;
	}

}

int OpenMeshMesh2PCLCloudNormal(FMesh *mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
	(*mesh).request_face_normals();
	(*mesh).request_vertex_normals();
	(*mesh).update_normals();
	for (FMesh::VertexIter it = (*mesh).vertices_begin(); it != (*mesh).vertices_end(); ++it)
	{
		cloud->push_back(pcl::PointXYZ((*mesh).point(*it).data()[0], (*mesh).point(*it).data()[1], (*mesh).point(*it).data()[2]));
		normals->push_back(pcl::Normal((*mesh).normal(*it).data()[0], (*mesh).normal(*it).data()[1], (*mesh).normal(*it).data()[2]));
	}

	return 0;
}

int OpenMeshMesh2PCLCloud(FMesh *mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	(*mesh).request_face_normals();
	(*mesh).request_vertex_normals();
	(*mesh).update_normals();
	for (FMesh::VertexIter it = (*mesh).vertices_begin(); it != (*mesh).vertices_end(); ++it)
	{
		cloud->push_back(pcl::PointXYZ((*mesh).point(*it).data()[0], (*mesh).point(*it).data()[1], (*mesh).point(*it).data()[2]));
		//normals->push_back(pcl::Normal(mesh.normal(*it).data()[0], mesh.normal(*it).data()[1], mesh.normal(*it).data()[2]));
	}

	return 0;
}

int OpenMeshVectorPoint2PCLCloud(std::vector<FMesh::Point> &points_openmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	size_t size_cloud = points_openmesh.size();
	for (size_t i = 0; i < size_cloud; i++)
	{
		cloud->push_back(pcl::PointXYZ(points_openmesh[0].data()[0], points_openmesh[0].data()[1], points_openmesh[0].data()[2]));
	}
	return 0;
}

int PCL2OpenMeshVectorPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<FMesh::Point> &points_openmesh)
{
	size_t size_cloud = cloud->size();
	for (size_t i = 0; i < size_cloud; i++)
	{
		points_openmesh.push_back(FMesh::Point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
	}

	return 0;
}

int PCL2OpenMeshVectorNormal(pcl::PointCloud<pcl::Normal>::Ptr &normal, std::vector<FMesh::Normal> &normal_openmesh)
{
	size_t size_cloud = normal->size();
	for (size_t i = 0; i < size_cloud; i++)
	{
		normal_openmesh.push_back(FMesh::Point(normal->points[0].normal_x, normal->points[0].normal_y, normal->points[0].normal_z));
	}

	return 0;
}

int DeleteCandidatePointsFromPoints(std::vector<FMesh::Point> &points, std::vector<FMesh::Point> candidate_points)
{
	int size_candidate_points = candidate_points.size();
	for (int i = 0; i < size_candidate_points; i++)
	{
		int size = points.size();
		for (int j = 0; j < size; j++)
		{
			if (candidate_points[i] == points[j])	//WIP. check the distance.
			{
				points.erase(points.begin() + j);
			}
		}
	}
	return 0;
}

void GetOMeshPointNNormal(FMesh *mesh, std::vector<FMesh::VertexIter> sample_it, std::vector<FMesh::Point>& OpenMeshPointList, std::vector<FMesh::Normal>& OpenMeshNormalList)
{
	OpenMeshPointList.clear();
	OpenMeshNormalList.clear();
	for (int i = 0; i < sample_it.size(); i++)
	{
		FMesh::Point subsample = mesh->point(*(sample_it[i]));
		FMesh::Normal subsampleN = mesh->normal(*(sample_it[i]));
		OpenMeshPointList.push_back(subsample);
		OpenMeshNormalList.push_back(subsampleN);
	}
}


//int IterativeRansac(FMesh *mesh, std::vector<FMesh::Point> subsampled_points_openmesh, std::vector<FMesh::Normal> subsampled_normal_openmesh)
int IterativeRansac(Tooth* surface, float mean_edge_length)
{
	/*FMesh mesh;
	std::string load_file;
	std::getline(std::cin, load_file);
	if (!OpenMesh::IO::read_mesh(mesh, load_file))
	{
		std::cerr << "read error\n";
		exit(1);
	}*/

	//for PCL display and normal calculation
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//OpenMeshMesh2PCLCloudNormal(mesh, cloud, normals);

	////Construct the subsample pool
	//Get the leftmost points by checking the "rollover" number.
	FMesh *mesh = surface->GetOMesh();
	FMesh::Point subsample;
	FMesh::Normal subsampleN;
	FMesh::VertexIter subsampleIt;
	//std::vector<FMesh::Point> subsampled_points_openmesh;
	//std::vector<FMesh::Normal> subsampled_normal_openmesh;
	std::vector<FMesh::VertexIter>	subsample_it_openmesh;
	int leftmost = 10e6, rollnum, leftmostIDX;
	for (int nodenum = 0; nodenum < surface->tpNode.size(); nodenum++)	//get the left most point according to rollover number
	{
		rollnum = surface->tpNode[nodenum].rollover;
		if (rollnum < leftmost)
		{
			//startpt = upperteeth.GetTooth(0)->tpNode[nodenum].node_it;
			leftmost = rollnum;
			leftmostIDX = nodenum;
		}
		//subsample = mesh->point(*(surface->tpNode[nodenum].node_it));
		//subsampleN = mesh->normal(*(surface->tpNode[nodenum].node_it));
		subsampleIt = surface->tpNode[nodenum].node_it;
		//subsampled_points_openmesh.push_back(subsample);
		//subsampled_normal_openmesh.push_back(subsampleN);
		subsample_it_openmesh.push_back(subsampleIt);
	}
	//put the left most point in the first position of subsample list.
	//subsample = subsampled_points_openmesh[leftmostIDX];
	//subsampleN = subsampled_normal_openmesh[leftmostIDX];
	subsampleIt = subsample_it_openmesh[leftmostIDX];
	//subsampled_points_openmesh.erase(subsampled_points_openmesh.begin() + leftmostIDX);
	//subsampled_points_openmesh.insert(subsampled_points_openmesh.begin(), subsample);
	//subsampled_normal_openmesh.erase(subsampled_normal_openmesh.begin() + leftmostIDX);
	//subsampled_normal_openmesh.insert(subsampled_normal_openmesh.begin(), subsampleN);
	subsample_it_openmesh.erase(subsample_it_openmesh.begin() + leftmostIDX);
	subsample_it_openmesh.insert(subsample_it_openmesh.begin(), subsampleIt);


	//Get points and normal from OpenMesh iterator
	/*std::vector<FMesh::Point> subsampled_points_openmesh;
	std::vector<FMesh::Normal> subsampled_normal_openmesh;
	for (int i = 0; i < sample_it.size(); i++)
	{
		FMesh::Point subsample = mesh->point(*(sample_it[i]));
		FMesh::Normal subsampleN = mesh->normal(*(sample_it[i]));
		subsampled_points_openmesh.push_back(subsample);
		subsampled_normal_openmesh.push_back(subsampleN);
	}*/
	//FMesh *mesh = surface->GetOMesh();
	std::vector<FMesh::VertexIter> remain_it = subsample_it_openmesh;
	//std::vector<FMesh::Point> remain_pt_OM;
	//std::vector<FMesh::Normal> remain_nor_OM;
	std::vector<FMesh::Point> subsampled_points_openmesh;
	std::vector<FMesh::Normal> subsampled_normal_openmesh;	
	//std::vector<bool> remain_pool_checked(remain_it.size(), 0);	//MARK LIST to mark whether a sample point has been included in a RANSAC solution.
	//GetOMeshPointNNormal(mesh, remain_it, subsampled_points_openmesh, subsampled_normal_openmesh);	
	
	////////for debug input file//////
	//readParameterFile("C:\\Users\\RZ.IEZ060\\Dropbox\\Dental source code\\Dental_test\\input extract indices.txt");	//C:\Users\RZ.IEZ060\Dropbox\Dental source code\Dental_test //"D:\\Development\\Surface_approximation\\input extract indices.txt"
	//readParameterFile("C:\\ITF\\Surface_approximation\\input extract indices.txt");
	//readParameterFile("../input extract indices.txt");
	//////for final input file location when release//////
	std::string inputparameter = MASTER_FOLDER + "/Input Parameters.txt";
	readParameterFile(inputparameter);


	//pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled_points(new pcl::PointCloud<pcl::PointXYZ>);
	//OpenMeshVectorPoint2PCLCloud(subsampled_points_openmesh, subsampled_points);
	//pcl::PointCloud<pcl::Normal>::Ptr subsampled_points_normals(new pcl::PointCloud<pcl::Normal>);
	//SubsamplingPCL(cloud, normals, subsampled_points, subsampled_points_normals);
	//std::vector<FMesh::Point> subsampled_points_openmesh;
	//PCL2VectorOpenMeshPoint(subsampled_points, subsampled_points_openmesh);
	
	/*size_t size_subsampled_points = subsampled_points_openmesh.size();
	if (size_subsampled_points < 1)
	{
		std::cout << "No subsampled points" << std::endl;
		return -1;
	}*/

	////build the distance sequence list////
	//std::vector<int> sequence_number;	//sorted by the distance: closest to farthest	
	FMesh::VertexIter P0 = subsample_it_openmesh[0];
	FMesh::Point start_point;	//subsampled_points_openmesh[0];	//Initialization. Get the start point for the FIRST patch. the left most point of entire surface.
	//DijkstraAlgorithm(mesh, subsampled_points_openmesh, start_point, sequence_number);
	//int size_subsampled_points_openmesh = subsampled_points_openmesh.size();
	std::vector<PatchType> patchtype;
	PatchType patchtype_last;
	std::vector<FMesh::VertexIter> previous_patch;

	//////Iterative RANSAC//////
	//Initialize for flood fill
	for (FMesh::VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		mesh->data(*v_it).patch = 0;
	}


	while (remain_it.size() > 0)
	{
		int poolsize = remain_it.size();	//this line is for debug only
		//Build the distance sequence list.
		GetOMeshPointNNormal(mesh, remain_it, subsampled_points_openmesh, subsampled_normal_openmesh);	//For Dijkstra. Not for RANSAC.	
		std::vector<int> dist_list;	//distance sequence list. Show the point list according to the distance to the start_point.
		dist_list.clear();
		start_point = mesh->point(*P0);
		DijkstraAlgorithm(mesh, subsampled_points_openmesh, start_point, dist_list);
		bool succeeded_once = false;
		
		//Use the distance list to build a subsample list from the remaining pool. The ITERATIVE RANSAC is run in this loop.
		FMesh::VertexIter P_end_last;
		for (int i = 0; i < dist_list.size(); i++)
		{
			int ith_far = dist_list[i];	//i-th furthest point from the start point.
			//if (remain_pool_checked[ith_far] == TRUE)	//this point has been included in previous RANSAC
			//	continue;

			//Flood-fill
			FMesh::VertexIter P_end = remain_it[ith_far];
			if (  (mesh->data(*P_end).RANSAC) > 0 && i<(dist_list.size()-1) )	//if has been included in the previous RANSAC, then ignore. (i<size-1) to force terminate the loop. Not good solution. Should've saved the biggest iter_patch during iteration. WIP.
				continue;
			int topoIDX = (mesh->data(*P_end)).TopoNode_idx;
			if (topoIDX < 0)
				return -1;	//error. topoIDX should >=0.
			surface->Roll(topoIDX, 1);

			//pick the left of P_end to form a sub patch for RANSAC
			std::vector<FMesh::VertexIter> iter_patch;
			for (int j = 0; j < remain_it.size(); j++)
			{
				if (mesh->data(*(remain_it[j])).patch == 1)	//marked as 1 in the flood fill
					iter_patch.push_back(remain_it[j]);
			}
			int iterpatchsize = iter_patch.size();	//for debug only. to see the size.

			//RANSAC to check this iter_patch
			GetOMeshPointNNormal(mesh, iter_patch, subsampled_points_openmesh, subsampled_normal_openmesh);	
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = BuildPCLCloudStructure(subsampled_points_openmesh);
			pcl::PointCloud<pcl::Normal>::Ptr normals = BuildPCLNormalStructure(subsampled_normal_openmesh);
			PatchType patchtype_temp;
			if (Ransac(cloud, normals, patchtype_temp, 0) > 0 )   //1 = points can be recognized as at least one type; 0 = points cannot be recognized as any type
			{
				patchtype_last = patchtype_temp;
				previous_patch = iter_patch;
				P_end_last = P_end;
				succeeded_once = true;	//succeeded once in the current patch.

				//Mark flag
				for (FMesh::VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
				{
					mesh->data(*v_it).RANSAC = 0;
				}
				for (int k = 0; k < iter_patch.size(); k++)
				{
					mesh->data(*(iter_patch[k])).RANSAC = 1;
				}

				//if all points in the pool are included in the RANSAC
				int sz = remain_it.size();	//for debug
				int sz2 = iter_patch.size();
				int sz3 = dist_list.size();
				if (iter_patch.size() == remain_it.size() || i == (remain_it.size()-1) )	//sometimes the last two points conflict to each other (cannot include each other), then the iter_patch never reach remain_it, however it can always pass RANSAC, which causes infinite loop. Therefore need to check if "i=size-1".
				//if (i == remain_it.size()-1)	//has checked all points in the remain list. NOTE: iter_patch size may be smaller or larger than i, depending on the generatrix direction! E.g. i=58, remain_it.size()=iter_patch.size()=60. Then this check will fail, but all nodes have been marked as RANSAC, which make the next loop infinite. If iter_patch.size() == remain_it.size(), then iter_patch.size() == i+1 == remain_it.size().
				{
					//update results to vertex
					patchtype.push_back(patchtype_last);
					int patchidx = patchtype.size();
					for (int i = 0; i < iter_patch.size(); i++)
					{
						FMesh::VertexIter it = iter_patch[i];
						int topoIDX = mesh->data(*it).TopoNode_idx;
						if (patchtype_last.plane == 1)
						{
							surface->tpNode[topoIDX].primType = 1;
							surface->tpNode[topoIDX].patch_idx = patchidx;
							mesh->data(*it).RANSAC_TYPE = 1;
							continue;
						}
						else if (patchtype_last.cylinder == 1)
						{
							surface->tpNode[topoIDX].primType = 2;
							surface->tpNode[topoIDX].patch_idx = patchidx;
							mesh->data(*it).RANSAC_TYPE = 2;
							continue;
						}
						else if (patchtype_last.cone == 1)
						{
							surface->tpNode[topoIDX].primType = 3;
							surface->tpNode[topoIDX].patch_idx = patchidx;
							mesh->data(*it).RANSAC_TYPE = 3;
							continue;
						}
					}
					//update boundary PCA info
					int nodenum = mesh->data(*P0).TopoNode_idx;
					ON_3dPoint pa = surface->tpNode[nodenum].best_fitted_line_points[0];
					(*(patchtype.end() - 1)).BoundaryLines[0] = pa;	//start point of PCA on the patch's start node
					pa = surface->tpNode[nodenum].best_fitted_line_points[1];
					(*(patchtype.end() - 1)).BoundaryLines[1] = pa;
					nodenum = mesh->data(*P_end_last).TopoNode_idx;
					pa = surface->tpNode[nodenum].best_fitted_line_points[0];
					(*(patchtype.end() - 1)).BoundaryLines[2] = pa;	//start point of PCA on the patch's end node
					pa = surface->tpNode[nodenum].best_fitted_line_points[1];
					(*(patchtype.end() - 1)).BoundaryLines[3] = pa;

					if (iter_patch.size() == remain_it.size())
						remain_it.clear();	//If get in by (sz-1), it's possible the very last sample point is not included.
					else //the last several points are not included due to bad generatrix lines
					{
						P0 = P_end_last;
						//delete processed vertex from the pool.
						int m = 0;
						while (m < remain_it.size())
						{
							FMesh::VertexIter it = remain_it[m];
							if (mesh->data(*it).RANSAC > 0)
							{
								remain_it.erase(remain_it.begin() + m);
								continue;
							}
							m++;
						}
					}
					break;
				}			
			}	

			else 
			{
				//not form a patch. CAUTION! only one point cannot pass the RANSAC process! CASE 2: the very last single point is left over. WIP
				int nn = remain_it.size();
				if (remain_it.size() == 1)	//the very last point, cannot pass RANSAC. OR <2? Two points can form any shape. WIP.
				{
					remain_it.clear();
				}				
				else 
				{
					int patchsize = previous_patch.size();
					if (patchsize > 1 && succeeded_once)	//if size=1, then alway fail RANSAC. CAUTION: even patchsize=5 can fail! Then crashed! Therefore added the succeeded_once. If not, then continue looking for patches.
					{						
						//update results.
						patchtype.push_back(patchtype_last);
						int patchidx = patchtype.size();
						for (int i = 0; i < patchsize; i++)
						{
							FMesh::VertexIter it = previous_patch[i];
							int topoIDX = mesh->data(*it).TopoNode_idx;
							if (patchtype_last.plane == 1)
							{
								surface->tpNode[topoIDX].primType = 1;
								surface->tpNode[topoIDX].patch_idx = patchidx;
								mesh->data(*it).RANSAC_TYPE = 1;
								continue;
							}
							else if (patchtype_last.cylinder == 1)
							{
								surface->tpNode[topoIDX].primType = 2;
								surface->tpNode[topoIDX].patch_idx = patchidx;
								mesh->data(*it).RANSAC_TYPE = 2;
								continue;
							}
							else if (patchtype_last.cone == 1)
							{
								surface->tpNode[topoIDX].primType = 3;
								surface->tpNode[topoIDX].patch_idx = patchidx;
								mesh->data(*it).RANSAC_TYPE = 3;
								continue;
							}
						}
						//update boundary PCA info
						int nodenum = mesh->data(*P0).TopoNode_idx;
						ON_3dPoint pa = surface->tpNode[nodenum].best_fitted_line_points[0];
						(*(patchtype.end() - 1)).BoundaryLines[0] = pa;	//start point of PCA on the patch's start node
						pa = surface->tpNode[nodenum].best_fitted_line_points[1];
						(*(patchtype.end() - 1)).BoundaryLines[1] = pa;
						nodenum = mesh->data(*P_end_last).TopoNode_idx;
						pa = surface->tpNode[nodenum].best_fitted_line_points[0];
						(*(patchtype.end() - 1)).BoundaryLines[2] = pa;	//start point of PCA on the patch's end node
						pa = surface->tpNode[nodenum].best_fitted_line_points[1];
						(*(patchtype.end() - 1)).BoundaryLines[3] = pa;

						P0 = P_end_last;

						//delete processed vertex from the pool.
						int m = 0;
						while (m < remain_it.size())
						{
							FMesh::VertexIter it = remain_it[m];
							if (mesh->data(*it).RANSAC > 0)
							{
								remain_it.erase(remain_it.begin() + m);
								continue;
							}
							m++;
						}
						break;
					}
				}
			}
		}
		//lack of termination condition to force forming a patch. If already iterate all nodes in the pool, but cannot get into the terminal condition.

	}

	//eliminate the sample points around the boundary area between two adjacent patches.
	DeleteBoundaryPoints(patchtype, mean_edge_length);
	Second_Ransac(patchtype);

	surface->Patch = patchtype;
	//reset flood-fill flag
	for (FMesh::VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		mesh->data(*v_it).patch = 0;
	}

	std::string output_file = OUTPUT_PATH + "/" + FILE_NAME + ".txt";

	OutputPatchTypeOnTXT(patchtype, output_file);

	////Sinara's template
	//while (subsampled_points_openmesh.size() != 0)
	//{
	//	for (int i = 1; i < size_subsampled_points_openmesh; i++)
	//	{
	//		//generatrix; 
	//		//input: the point = subsampled_points_openmesh[i]
	//		//output: candidate_points(FMesh::Point>) on the left side and theirs candidate_points_normals 
	//
	//		//update the subsample points. e.g. subsampled_points_openmesh[sequence_number[i]];
	//		std::vector<FMesh::Point> candidate_points = subsampled_points_openmesh;
	//		std::vector<FMesh::Normal> candidate_points_normals = subsampled_normal_openmesh;	//WIP. get from subsampled_normal_openmesh
	//
	//
	//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = BuildPCLCloudStructure(candidate_points);
	//		pcl::PointCloud<pcl::Normal>::Ptr normals = BuildPCLNormalStructure(candidate_points_normals);
	//		PatchType patchtype_temp;
	//		if (Ransac(cloud, normals, patchtype_temp))   //1 = points can be recognized as at least one type; 0 = points cannot be recognized as any type
	//		{
	//			patchtype_last = patchtype_temp;
	//			if (i == size_subsampled_points - 1)
	//			{
	//				//visualizePointCloud(subsampled_points, cloud, "patch", xy);
	//				patchtype.push_back(patchtype_last);
	//			}
	//			//WIP. update the subsample list for RANSAC: size_subsampled_points_openmesh
	//
	//			continue;
	//		}
	//		else
	//		{
	//			patchtype.push_back(patchtype_last);
	//			//visualizePointCloud(subsampled_points, cloud, "patch", xy);
	//			start_point = subsampled_points_openmesh[i];
	//			DeleteCandidatePointsFromPoints(subsampled_points_openmesh, candidate_points);	//WIP. need to handle normal as well! Can use a flag list.
	//			size_subsampled_points_openmesh = subsampled_points_openmesh.size();
	//			DijkstraAlgorithm(mesh, subsampled_points_openmesh, start_point, sequence_number);
	//			break;
	//		}
	//	}
	//}

	
	//single patch test by ZR
	/*GetOMeshPointNNormal(mesh, remain_it, subsampled_points_openmesh, subsampled_normal_openmesh);
	std::vector<FMesh::Point> candidate_points = subsampled_points_openmesh;
	std::vector<FMesh::Normal> candidate_points_normals = subsampled_normal_openmesh;	//WIP. get from subsampled_normal_openmesh		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = BuildPCLCloudStructure(candidate_points);
	pcl::PointCloud<pcl::Normal>::Ptr normals = BuildPCLNormalStructure(candidate_points_normals);
	PatchType patchtype_temp;
	if (Ransac(cloud, normals, patchtype_temp))   //1 = points can be recognized as at least one type; 0 = points cannot be recognized as any type
	{
		patchtype_last = patchtype_temp;
		patchtype.push_back(patchtype_last);
	}
	auto testt = (patchtype_temp.coefficients_cylinder)->values;	//for debug. watching values.
	*/

	//PCL Visualization by Sinara
	/*//time_t end = clock();
	//int j = 0; // start
	//std::vector<int> number = ReadTXTNumber("C:\\Users\\sinara\\Desktop\\distance\\0.txt");
	//pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled_point_cloud = BuildPointCloudStructure(subsampled_points, 0, size_subsampled_points - 1, number);
	//for (int i = 40; i < size_subsampled_points; i++)
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = BuildPointCloudStructure(subsampled_points, j, i, number);            //need to be changed when adding generatrix 
	//	pcl::PointCloud<pcl::Normal>::Ptr normals = BuildPointNormalStructure(subsample_points_normals, j, i, number);    //need to be changed when adding generatrix
	//	PatchType patchtype_temp;
	//	if (IteraticeRansac(cloud, normals, &patchtype_temp))   //1 = points can be recognized as at least one type; 0 = points cannot be recognized as any type
	//	{
	//		patchtype_last = patchtype_temp;
	//		if (i == size_subsampled_points - 1)
	//		{
	//			visualizePointCloud(subsampled_point_cloud, cloud, "patch", xy);
	//		}
	//		continue;
	//	}
	//	else
	//	{
	//		patchtype.push_back(patchtype_last);
	//		visualizePointCloud(subsampled_point_cloud, cloud, "patch", xy);
	//		j = i;
	//		//number = ReadTXTNumber("C:\\Users\\sinara\\Desktop\\distance\\"+std::to_string(i)+".txt");
	//		//subsampled_point_cloud = BuildPointCloudStructure(subsampled_points, 0, size_subsampled_points - 1, number);
	//		i += 40;
	//	}
	//	
	//}	
	//std::cout << (double)(end - start) / CLOCKS_PER_SEC * 1000.0 << std::endl;
	*/

	//std::system("pause");
	return 0;
}

int Second_Ransac(std::vector<PatchType> &patchtype)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normal;
	
	for (int i = 0; i < patchtype.size(); i++)
	{
		if (patchtype[i].plane)	//Plane is preferred among all.
		{
			cloud = patchtype[i].cloud_plane;
			normal = patchtype[i].normal_plane;
		}
		else if (patchtype[i].cone)	//Cone is preferred over Cylinder.			
		{
			cloud = patchtype[i].cloud_cone;
			normal = patchtype[i].normal_cone;
		}
		else if (patchtype[i].cylinder)
		{
			cloud = patchtype[i].cloud_cylinder;
			normal = patchtype[i].normal_cylinder;
		}
		else
		{
			return -1;
		}

		Ransac(cloud, normal, patchtype[i], 1);
	}

	return 0;
}


int DeleteBoundaryPoints(std::vector<PatchType> &patchtype, float mean_edge_length)
{
	if (patchtype.size() < 2)
		return 0;
	float zone_rad = 6 * mean_edge_length ;	//6 on the end boundary side. 2 on the start boundary side. 3:1 ratio
	//delete end boundary points
	for (int i = 0; i < (patchtype.size() - 1); i++)
	{
		ON_3dVector ref_ori, tempa, proja, temp_seg;
		double dist;
		ON_3dPoint ref_seg_a, ref_seg_b;	//end points of the segment.
		ref_seg_a = patchtype[i].BoundaryLines[2];
		ref_seg_b = patchtype[i].BoundaryLines[3];
		ref_ori = ref_seg_b - ref_seg_a;
		ref_ori.Unitize();

		if (patchtype[i].plane)
		{
			/*for (int j = 0; j < (patchtype[i].cloud_plane)->size(); j++)
			{
				auto pt = (patchtype[i].cloud_plane)->points[j];
				tempa.x = pt.x;
				tempa.y = pt.y;
				tempa.z = pt.z;
				temp_seg = tempa - ref_seg_a;
				proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
				dist = (temp_seg - proja).Length();
				if (dist < zone_rad)
				{
					patchtype[i].EndBoundaryPoints.Append(tempa);
				}
			}*/

			int j = 0;
			while (j<(patchtype[i].cloud_plane)->size())
			{
				auto pt = (patchtype[i].cloud_plane)->points[j];
				tempa.x = pt.x;
				tempa.y = pt.y;
				tempa.z = pt.z;
				temp_seg = tempa - ref_seg_a;
				proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
				dist = (temp_seg - proja).Length();
				if (dist < zone_rad)
				{
					patchtype[i].EndBoundaryPoints.Append(tempa);
					//delete points from the original point cloud
					(patchtype[i].cloud_plane)->points.erase((patchtype[i].cloud_plane)->points.begin()+j);
					(patchtype[i].cloud_plane)->width--;
					(patchtype[i].cloud_plane)->points.resize((patchtype[i].cloud_plane)->width);

					(patchtype[i].normal_plane)->points.erase((patchtype[i].normal_plane)->points.begin() + j);
					(patchtype[i].normal_plane)->width--;
					(patchtype[i].normal_plane)->points.resize((patchtype[i].normal_plane)->width);
					continue;
				}
				j++;
			}


		}
		else
		{
			if (patchtype[i].cylinder && (!patchtype[i].cone))	//cylinder
			{
				/*for (int j = 0; j < (patchtype[i].cloud_cylinder)->size(); j++)
				{
					auto pt = (patchtype[i].cloud_cylinder)->points[j];
					tempa.x = pt.x;
					tempa.y = pt.y;
					tempa.z = pt.z;
					temp_seg = tempa - ref_seg_a;
					proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
					dist = (temp_seg - proja).Length();
					if (dist < zone_rad)
					{
						patchtype[i].EndBoundaryPoints.Append(tempa);
					}
				}*/

				int j = 0;
				while (j < (patchtype[i].cloud_cylinder)->size())
				{
					auto pt = (patchtype[i].cloud_cylinder)->points[j];
					tempa.x = pt.x;
					tempa.y = pt.y;
					tempa.z = pt.z;
					temp_seg = tempa - ref_seg_a;
					proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
					dist = (temp_seg - proja).Length();
					if (dist < zone_rad)
					{
						patchtype[i].EndBoundaryPoints.Append(tempa);
						//delete points from the original point cloud
						(patchtype[i].cloud_cylinder)->points.erase((patchtype[i].cloud_cylinder)->points.begin() + j);
						(patchtype[i].cloud_cylinder)->width--;
						(patchtype[i].cloud_cylinder)->points.resize((patchtype[i].cloud_cylinder)->width);

						(patchtype[i].normal_cylinder)->points.erase((patchtype[i].normal_cylinder)->points.begin() + j);
						(patchtype[i].normal_cylinder)->width--;
						(patchtype[i].normal_cylinder)->points.resize((patchtype[i].normal_cylinder)->width);
						continue;
					}
					j++;
				}

			}
			else
			{
				if (patchtype[i].cone)	//cone
				{
					/*for (int j = 0; j < (patchtype[i].cloud_cone)->size(); j++)
					{
						auto pt = (patchtype[i].cloud_cone)->points[j];
						tempa.x = pt.x;
						tempa.y = pt.y;
						tempa.z = pt.z;
						temp_seg = tempa - ref_seg_a;
						proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
						dist = (temp_seg - proja).Length();
						if (dist < zone_rad)
						{
							patchtype[i].EndBoundaryPoints.Append(tempa);
						}
					}*/

					int j = 0;
					while (j < (patchtype[i].cloud_cone)->size())
					{
						auto pt = (patchtype[i].cloud_cone)->points[j];
						tempa.x = pt.x;
						tempa.y = pt.y;
						tempa.z = pt.z;
						temp_seg = tempa - ref_seg_a;
						proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
						dist = (temp_seg - proja).Length();
						if (dist < zone_rad)
						{
							patchtype[i].EndBoundaryPoints.Append(tempa);
							//delete points from the original point cloud
							(patchtype[i].cloud_cone)->points.erase((patchtype[i].cloud_cone)->points.begin() + j);
							(patchtype[i].cloud_cone)->width--;
							(patchtype[i].cloud_cone)->points.resize((patchtype[i].cloud_cone)->width);

							(patchtype[i].normal_cone)->points.erase((patchtype[i].normal_cone)->points.begin() + j);
							(patchtype[i].normal_cone)->width--;
							(patchtype[i].normal_cone)->points.resize((patchtype[i].normal_cone)->width);
							continue;
						}
						j++;
					}
				}
			}
		}
	}
	//delete start boundary points
	zone_rad = 2 * mean_edge_length;
	for (int i = 1; i < patchtype.size(); i++)
	{
		ON_3dVector ref_ori, tempa, proja, temp_seg;
		double dist;
		ON_3dPoint ref_seg_a, ref_seg_b;	//end points of the segment.
		ref_seg_a = patchtype[i].BoundaryLines[0];
		ref_seg_b = patchtype[i].BoundaryLines[1];
		ref_ori = ref_seg_b - ref_seg_a;
		ref_ori.Unitize();

		if (patchtype[i].plane)
		{

			int j = 0;
			while (j < (patchtype[i].cloud_plane)->size())
			{
				auto pt = (patchtype[i].cloud_plane)->points[j];
				tempa.x = pt.x;
				tempa.y = pt.y;
				tempa.z = pt.z;
				temp_seg = tempa - ref_seg_a;
				proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
				dist = (temp_seg - proja).Length();
				if (dist < zone_rad)
				{
					patchtype[i].StartBoundaryPoints.Append(tempa);
					//delete points from the original point cloud
					(patchtype[i].cloud_plane)->points.erase((patchtype[i].cloud_plane)->points.begin() + j);
					(patchtype[i].cloud_plane)->width--;
					(patchtype[i].cloud_plane)->points.resize((patchtype[i].cloud_plane)->width);

					(patchtype[i].normal_plane)->points.erase((patchtype[i].normal_plane)->points.begin() + j);
					(patchtype[i].normal_plane)->width--;
					(patchtype[i].normal_plane)->points.resize((patchtype[i].normal_plane)->width);
					continue;
				}
				j++;
			}
		}
		else
		{
			if (patchtype[i].cylinder && (!patchtype[i].cone))	//cylinder
			{

				int j = 0;
				while (j < (patchtype[i].cloud_cylinder)->size())
				{
					auto pt = (patchtype[i].cloud_cylinder)->points[j];
					tempa.x = pt.x;
					tempa.y = pt.y;
					tempa.z = pt.z;
					temp_seg = tempa - ref_seg_a;
					proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
					dist = (temp_seg - proja).Length();
					if (dist < zone_rad)
					{
						patchtype[i].StartBoundaryPoints.Append(tempa);
						//delete points from the original point cloud
						(patchtype[i].cloud_cylinder)->points.erase((patchtype[i].cloud_cylinder)->points.begin() + j);
						(patchtype[i].cloud_cylinder)->width--;
						(patchtype[i].cloud_cylinder)->points.resize((patchtype[i].cloud_cylinder)->width);

						(patchtype[i].normal_cylinder)->points.erase((patchtype[i].normal_cylinder)->points.begin() + j);
						(patchtype[i].normal_cylinder)->width--;
						(patchtype[i].normal_cylinder)->points.resize((patchtype[i].normal_cylinder)->width);
						continue;
					}
					j++;
				}
			}
			else
			{
				if (patchtype[i].cone)	//cone
				{
					int j = 0;
					while (j < (patchtype[i].cloud_cone)->size())
					{
						auto pt = (patchtype[i].cloud_cone)->points[j];
						tempa.x = pt.x;
						tempa.y = pt.y;
						tempa.z = pt.z;
						temp_seg = tempa - ref_seg_a;
						proja = ON_DotProduct(temp_seg, ref_ori) * ref_ori;
						dist = (temp_seg - proja).Length();
						if (dist < zone_rad)
						{
							patchtype[i].StartBoundaryPoints.Append(tempa);
							//delete points from the original point cloud
							(patchtype[i].cloud_cone)->points.erase((patchtype[i].cloud_cone)->points.begin() + j);
							(patchtype[i].cloud_cone)->width--;
							(patchtype[i].cloud_cone)->points.resize((patchtype[i].cloud_cone)->width);

							(patchtype[i].normal_cone)->points.erase((patchtype[i].normal_cone)->points.begin() + j);
							(patchtype[i].normal_cone)->width--;
							(patchtype[i].normal_cone)->points.resize((patchtype[i].normal_cone)->width);
							continue;
						}
						j++;
					}
				}

			}
		}
	}

	return 0;
}


int OutputPatchTypeOnTXT(std::vector<PatchType> patchtype, std::string output_file)
{
	std::ofstream outdata;
	std::string name_file = "../patchtype.txt";	//"../testfile.txt"	//for debug
	//outdata.open(name_file, ios::app);	//ios::out
	name_file = output_file;	//for final release 
	

	outdata.open(name_file);
	outdata << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	outdata.clear();
	int size_patch = patchtype.size();
	
	outdata << PLANE_TOL << " " << CYL_TOL << " " << CONE_TOL << std::endl;
	
	string sum;

	for (int i = 0; i < size_patch; i++)
	{
		if (patchtype[i].plane)
		{
			sum += "P ";
		}
		else
		{
			//outdata << " ";
			if (patchtype[i].cylinder && (!patchtype[i].cone))	//prefer cone over cylinder.
			{
				sum += "C ";
			}
			else
			{
				//outdata << " ";
				if (patchtype[i].cone)
				{
					sum += "N ";
				}
			}
		}
	}
	sum.erase(sum.end() - 1);
	outdata << sum << std::endl;

	int size_cloud;
	for (int i = 0; i < size_patch; i++)
	{
		if (patchtype[i].plane)
		{
			outdata << "P" << std::endl;
			//boundary PCA points
			outdata << patchtype[i].BoundaryLines[0].x << " " << patchtype[i].BoundaryLines[0].y << " " << patchtype[i].BoundaryLines[0].z << std::endl;
			outdata << patchtype[i].BoundaryLines[1].x << " " << patchtype[i].BoundaryLines[1].y << " " << patchtype[i].BoundaryLines[1].z << std::endl;
			outdata << patchtype[i].BoundaryLines[2].x << " " << patchtype[i].BoundaryLines[2].y << " " << patchtype[i].BoundaryLines[2].z << std::endl;
			outdata << patchtype[i].BoundaryLines[3].x << " " << patchtype[i].BoundaryLines[3].y << " " << patchtype[i].BoundaryLines[3].z << std::endl;
			//patch points
			size_cloud = (patchtype[i].cloud_plane)->size();
			outdata << size_cloud << std::endl;
			outdata << (patchtype[i].coefficients_plane)->values[0] << " " <<
				(patchtype[i].coefficients_plane)->values[1] << " " <<
				(patchtype[i].coefficients_plane)->values[2] << " " <<
				(patchtype[i].coefficients_plane)->values[3] << std::endl;

			//2nd RANSAC
			if (patchtype[i].plane2)
			{
				outdata << (patchtype[i].coefficients_plane2)->values[0] << " " <<
					(patchtype[i].coefficients_plane2)->values[1] << " " <<
					(patchtype[i].coefficients_plane2)->values[2] << " " <<
					(patchtype[i].coefficients_plane2)->values[3] << std::endl;
			}
			else
			{
				outdata << "NULL. Or different type." << std::endl;
			}

			for (int j = 0; j < size_cloud; j++)
			{
				outdata << (patchtype[i].cloud_plane)->points[j].x << " " << (patchtype[i].cloud_plane)->points[j].y << " " <<
					(patchtype[i].cloud_plane)->points[j].z << std::endl;
			}
		}
		else
		{
			if (patchtype[i].cylinder && (!patchtype[i].cone))	//prefer cone over cylinder.
			{
				outdata << "C" << std::endl;
				//boundary PCA points
				outdata << patchtype[i].BoundaryLines[0].x << " " << patchtype[i].BoundaryLines[0].y << " " << patchtype[i].BoundaryLines[0].z << std::endl;
				outdata << patchtype[i].BoundaryLines[1].x << " " << patchtype[i].BoundaryLines[1].y << " " << patchtype[i].BoundaryLines[1].z << std::endl;
				outdata << patchtype[i].BoundaryLines[2].x << " " << patchtype[i].BoundaryLines[2].y << " " << patchtype[i].BoundaryLines[2].z << std::endl;
				outdata << patchtype[i].BoundaryLines[3].x << " " << patchtype[i].BoundaryLines[3].y << " " << patchtype[i].BoundaryLines[3].z << std::endl;

				size_cloud = (patchtype[i].cloud_cylinder)->size();
				outdata << size_cloud << std::endl;
				outdata << (patchtype[i].coefficients_cylinder)->values[0] << " " <<
					(patchtype[i].coefficients_cylinder)->values[1] << " " <<
					(patchtype[i].coefficients_cylinder)->values[2] << " " <<
					(patchtype[i].coefficients_cylinder)->values[3] << " " <<
					(patchtype[i].coefficients_cylinder)->values[4] << " " <<
					(patchtype[i].coefficients_cylinder)->values[5] << " " <<
					(patchtype[i].coefficients_cylinder)->values[6] << std::endl;

				//2nd RANSAC
				if (patchtype[i].cylinder2 && (!patchtype[i].cone2))
				{
					outdata << (patchtype[i].coefficients_cylinder2)->values[0] << " " <<
						(patchtype[i].coefficients_cylinder2)->values[1] << " " <<
						(patchtype[i].coefficients_cylinder2)->values[2] << " " <<
						(patchtype[i].coefficients_cylinder2)->values[3] << " " <<
						(patchtype[i].coefficients_cylinder2)->values[4] << " " <<
						(patchtype[i].coefficients_cylinder2)->values[5] << " " <<
						(patchtype[i].coefficients_cylinder2)->values[6] << std::endl;
				}
				else
				{
					outdata << "NULL. Or different type." << std::endl;
				}

				for (int j = 0; j < size_cloud; j++)
				{
					outdata << (patchtype[i].cloud_cylinder)->points[j].x
						<< " " << (patchtype[i].cloud_cylinder)->points[j].y
						<< " " << (patchtype[i].cloud_cylinder)->points[j].z << std::endl;
				}
			}
			else
			{
				if (patchtype[i].cone)
				{
					outdata << "N" << std::endl;
					//boundary PCA points
					outdata << patchtype[i].BoundaryLines[0].x << " " << patchtype[i].BoundaryLines[0].y << " " << patchtype[i].BoundaryLines[0].z << std::endl;
					outdata << patchtype[i].BoundaryLines[1].x << " " << patchtype[i].BoundaryLines[1].y << " " << patchtype[i].BoundaryLines[1].z << std::endl;
					outdata << patchtype[i].BoundaryLines[2].x << " " << patchtype[i].BoundaryLines[2].y << " " << patchtype[i].BoundaryLines[2].z << std::endl;
					outdata << patchtype[i].BoundaryLines[3].x << " " << patchtype[i].BoundaryLines[3].y << " " << patchtype[i].BoundaryLines[3].z << std::endl;

					size_cloud = (patchtype[i].cloud_cone)->size();
					outdata << size_cloud << std::endl;
					outdata << (patchtype[i].coefficients_cone)->values[0] << " " <<
						(patchtype[i].coefficients_cone)->values[1] << " " <<
						(patchtype[i].coefficients_cone)->values[2] << " " <<
						(patchtype[i].coefficients_cone)->values[3] << " " <<
						(patchtype[i].coefficients_cone)->values[4] << " " <<
						(patchtype[i].coefficients_cone)->values[5] << " " <<
						(patchtype[i].coefficients_cone)->values[6] << std::endl;

					//2nd RANSAC
					if (patchtype[i].cone2)
					{
						outdata << (patchtype[i].coefficients_cone2)->values[0] << " " <<
							(patchtype[i].coefficients_cone2)->values[1] << " " <<
							(patchtype[i].coefficients_cone2)->values[2] << " " <<
							(patchtype[i].coefficients_cone2)->values[3] << " " <<
							(patchtype[i].coefficients_cone2)->values[4] << " " <<
							(patchtype[i].coefficients_cone2)->values[5] << " " <<
							(patchtype[i].coefficients_cone2)->values[6] << std::endl;
					}
					else
					{
						outdata << "NULL. Or different type." << std::endl;
					}

					for (int j = 0; j < size_cloud; j++)
					{
						outdata << (patchtype[i].cloud_cone)->points[j].x
							<< " " << (patchtype[i].cloud_cone)->points[j].y
							<< " " << (patchtype[i].cloud_cone)->points[j].z << std::endl;
					}
				}
			}
		}
		//boundary points
		int st_sz = patchtype[i].StartBoundaryPoints.Count();
		int	end_sz = patchtype[i].EndBoundaryPoints.Count();
		int bry_sz = st_sz + end_sz;
		outdata << "BOUNDARY" << std::endl;
		if (bry_sz)
		{
			outdata << bry_sz << std::endl;
		}
		else
		{
			outdata << "0" << std::endl;
		}
		if (st_sz > 0)
		{
			for (int j = 0; j < st_sz; j++)
			{
				ON_3dPoint pt;
				pt = patchtype[i].StartBoundaryPoints[j];
				outdata << pt.x << " " << pt.y << " " << pt.z << std::endl;
			}
		}
		if (end_sz > 0)
		{
			for (int j = 0; j < end_sz; j++)
			{
				ON_3dPoint pt;
				pt = patchtype[i].EndBoundaryPoints[j];
				outdata << pt.x << " " << pt.y << " " << pt.z << std::endl;
			}
		}
	}
	outdata.close();

	return 0;
}
#pragma endregion
//==============================iterative_ransac===============================




//=====================Fitting 3D Points(on one plane) To a Line=====================
#pragma region
Eigen::Matrix3f buildTransformMatrixFromAxis(Eigen::Vector3f x_axis, Eigen::Vector3f y_axis, Eigen::Vector3f z_axis)
{
	using namespace Eigen;

	Matrix3f transform_matrix;
	transform_matrix << x_axis, y_axis, z_axis;
	return transform_matrix;
}

Eigen::Matrix3f getTransformMatrixForAlignmentWithNormalToPlane(pcl::PointXYZ &origin, pcl::PointXYZ &point_on_x_axis, std::vector<double> plane_param)
{
	Eigen::Vector3f x_axis;
	Eigen::Vector3f y_axis;
	Eigen::Vector3f z_axis;

	//z axis definition	
	z_axis[0] = plane_param[0] ;
	z_axis[1] = plane_param[1] ;
	z_axis[2] = plane_param[2] ;
	double norm_z = z_axis.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		z_axis[i] = z_axis[i] / norm_z;
	}

	//x axis
	x_axis[0] = point_on_x_axis.x - origin.x;
	x_axis[1] = point_on_x_axis.y - origin.y;
	x_axis[2] = point_on_x_axis.z - origin.z;
	//std::cout << x_axis << std::endl;
	double norm_x = x_axis.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		x_axis[i] = x_axis[i] / norm_x;
	}

	//y axis definition
	y_axis = z_axis.cross(x_axis);
	double norm_y = y_axis.norm();
	for (std::size_t i = 0; i < 3; ++i)
	{
		y_axis[i] = y_axis[i] / norm_y;
	}
	/*std::cerr << "x_axis = (" << x_axis[0] << "," << x_axis[1] << "," << x_axis[2] << ")" << std::endl;
	std::cerr << "y_axis = (" << y_axis[0] << "," << y_axis[1] << "," << y_axis[2] << ")" << std::endl;
	std::cerr << "z_axis = (" << z_axis[0] << "," << z_axis[1] << "," << z_axis[2] << ")" << std::endl;*/

	//Get the transformation matrix referred to the new coordinate system

	Eigen::Matrix3f trasf_matrix = buildTransformMatrixFromAxis(x_axis, y_axis, z_axis);
	Eigen::Matrix3f trasf_matrix_inv = trasf_matrix.inverse();

	return trasf_matrix_inv;
	//return trasf_matrix;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud, Eigen::Matrix3f transform_matrix, pcl::PointXYZ &origin)
{
	const int num_nodes = patch_cloud->points.size();
	Eigen::Vector3f point, transformed_point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector3f origin_point_eigen(origin.x, origin.y, origin.z);

	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(patch_cloud->at(i).getArray3fMap());
		/*std::cout << point << std::endl;
		std::cout << appex << std::endl;
		std::cout << transform_matrix << std::endl;
		std::cout << point - appex << std::endl;*/
		transformed_point = transform_matrix*(point - origin_point_eigen);
		
		//std::cout << transformed_point << std::endl;
		(*transformed_points).push_back(pcl::PointXYZ(transformed_point[0], transformed_point[1], transformed_point[2]));
	}

	return transformed_points;
}

//http://mathworld.wolfram.com/LeastSquaresFitting.html
int LinearRegressionVerticalLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points, double &a, double &b)
{
	double xsum = 0, ysum = 0, x2sum = 0, y2sum = 0, xysum = 0, B = 0, number3 = 0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
	int number = (*transformed_points).size();
	for (int i = 0; i < number; i++)
	{
		xsum = xsum + (*transformed_points)[i].x;                        //calculate sigma(xi)
		ysum = ysum + (*transformed_points)[i].y;                        //calculate sigma(yi)
		x2sum = x2sum + pow((*transformed_points)[i].x, 2);                //calculate sigma(x^2i)
		//y2sum = y2sum + pow((*transformed_points)[i].y, 2);
		xysum = xysum + (*transformed_points)[i].x * (*transformed_points)[i].y;                    //calculate sigma(xi*yi)

	}
	//number3 = pow(number, 3);
	//B = (y2sum - number3 * pow(ysum, 2) - x2sum + number3 * pow(xsum, 2)) / (number3 * ysum * xsum - xysum) / 2;
	//a = -B / number3;
	//b = ysum / number - a*xsum / number;
	a = (number * xysum - xsum * ysum) / (number * x2sum - xsum * xsum);            //calculate slope
	b = (x2sum * ysum - xsum * xysum) / (x2sum * number - xsum * xsum);            //calculate intercept

	return 0;
}

//http://mathworld.wolfram.com/LeastSquaresFittingPerpendicularOffsets.html
int LinearRegressionPerpendicularLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points, double &a, double &b)
{
	double xsum = 0, ysum = 0, x2sum = 0, y2sum = 0, xysum = 0, B = 0, number3 = 0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
	int number = (*transformed_points).size();
	for (int i = 0; i < number; i++)
	{
		xsum = xsum + (*transformed_points)[i].x;                        //calculate sigma(xi)
		ysum = ysum + (*transformed_points)[i].y;                        //calculate sigma(yi)
		x2sum = x2sum + pow((*transformed_points)[i].x, 2);                //calculate sigma(x^2i)
		y2sum = y2sum + pow((*transformed_points)[i].y, 2); 
		xysum = xysum + (*transformed_points)[i].x * (*transformed_points)[i].y;                    //calculate sigma(xi*yi)
		
	}
	number3 = pow(number, 3);
	B = (y2sum - number3 * pow(ysum, 2) - x2sum + number3 * pow(xsum, 2)) / (number3 * ysum * xsum - xysum) / 2;
	a = -B / number3;
	b = ysum / number - a*xsum / number;
	//a = (number * xysum - xsum * ysum) / (number * x2sum - xsum * xsum);            //calculate slope
	//b = (x2sum * ysum - xsum * xysum) / (x2sum * number - xsum * xsum);            //calculate intercept

	return 0;
}

//PCA 
int LinearRegressionPCA(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points, double &a, double &b)
{
	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = (*transformed_points).size();
	if (num_atoms == 0)
	{
		std::cout << "No points to fit." << std::endl;
		return -1;
	}
	Eigen::Matrix< Eigen::Vector2d::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(num_atoms, 2);
	for (size_t i = 0; i < num_atoms; ++i)
		centers.row(i) = Eigen::Vector2d((*transformed_points)[i].x, (*transformed_points)[i].y);

	Eigen::Vector2d centroid, axis;
	//centroid质心
	centroid = centers.colwise().mean();

	//transpose转置阵
	Eigen::MatrixXd centered = centers.rowwise() - centroid.transpose();


	//adjoint共轭矩阵																	  
	Eigen::MatrixXd cov = centered.adjoint() * centered;


	//SelfAdjointEigenSolver:Computes eigenvalues and eigenvectors of selfadjoint matrices
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	/*std::cout << "centers:" << std::endl << centers << std::endl;
	std::cout << "centroid:" << std::endl << centroid << std::endl;
	std::cout << "centroid.transpose():" << std::endl << centroid.transpose() << std::endl;
	std::cout << "centered:" << std::endl << centered << std::endl;
	std::cout << "centered.adjoint():" << std::endl << centered.adjoint() << std::endl;
	std::cout << "cov:" << std::endl << cov << std::endl;
	std::cout << "eig.eigenvectors():" << std::endl << eig.eigenvectors() << std::endl;
	std::cout << "eig.eigenvectors().col(2):" << std::endl << eig.eigenvectors().col(2) << std::endl;*/

	//eigenvectors特征向量
	axis = eig.eigenvectors().col(1).normalized();
	//std::cout << "centroid:" << std::endl << centroid << std::endl;
	//std::cout << "axis:" << std::endl << axis << std::endl;
	a = axis[1] / axis[0];
	b = centroid[1] - centroid[0] * axis[1] / axis[0];

	return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectPointsOntoLine(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points, double a, double b, double &error)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_points(new pcl::PointCloud<pcl::PointXYZ>);
	int size = (*transformed_points).size();
	double sum = 0;
	for (int i = 0; i < size; i++)
	{
		double x0 = (*transformed_points)[i].x, y0 = (*transformed_points)[i].y;
		double x = (x0 + a*y0 - a*b) / (1 + a*a);
		double y = a*x + b;
		(*projected_points).push_back(pcl::PointXYZ(x, y, 0));
		sum += sqrt(pow(x - x0, 2) + pow(y - y0, 2));
	}
	error = sum;
	return projected_points;
}

int FindHeadAndTail(pcl::PointCloud<pcl::PointXYZ>::Ptr projected_points, pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_head_tail)
{
	int size = (*projected_points).size();
	double x0 = (*projected_points)[0].x, y0 = (*projected_points)[0].y;
	double x = (*projected_points)[1].x - x0;
	int head_no = 0, tail_no = 0;
	double head_dist = 0, tail_dist = 0;

	for (int i = 1; i < size; i++)
	{
		double xn = (*projected_points)[i].x, yn = (*projected_points)[i].y;
		double dist = sqrt(pow(xn - x0, 2) + pow(yn - y0, 2));;
		if ((xn - x0)*x >= 0)
		{
			if (dist > tail_dist)
			{
				tail_dist = dist;
				tail_no = i;
			}
		}
		else
		{
			if (dist > head_dist)
			{
				head_dist = dist;
				head_no = i;
			}
		}
	}
	(*transformed_head_tail).push_back((*projected_points)[head_no]);
	(*transformed_head_tail).push_back((*projected_points)[tail_no]);
	return 0;
}

//flag_head_tail == 0: head and tail are the projection of the first two points in points_list
//flag_head_tail == 1: head and tail are the projection of the head and tail points in points_list
//fitting_mode == 0: 2d PCA
//fitting_mode == 1: Linear Regression(Perpendicular least squares fitting)
//fitting_mode == 2: Linear Regression(Vertical least squares fitting)

//int Fitting3DPointsToLine2D(ON_3dPointArray &points_opennurbs, ON_3dVector normal_plane, double &error, ON_3dPoint &head, ON_3dPoint &tail, bool flag_head_tail, int fitting_mode)
//	                      /* pcl::PointCloud<pcl::PointXYZ>::Ptr &points_list, std::vector<double> plane_normal, double &error,
//	                      pcl::PointXYZ &head, pcl::PointXYZ &tail, )*/
//{
//	std::vector<double> plane_normal;
//	plane_normal.push_back(normal_plane.x);
//	plane_normal.push_back(normal_plane.y);
//	plane_normal.push_back(normal_plane.z);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr points_list(new pcl::PointCloud<pcl::PointXYZ>);
//	int size_points_opennurbs = points_opennurbs.Count();
//	for (int i = 0; i < size_points_opennurbs; i++)
//	{
//		points_list->push_back(pcl::PointXYZ(points_opennurbs[i].x, points_opennurbs[i].y, points_opennurbs[i].z));
//	}
//
//	//visualizePointCloud(points_list, "points_list", xy);
//	Eigen::Matrix3f transform_matrix1 = getTransformMatrixForAlignmentWithNormalToPlane((*points_list).points[0], (*points_list).points[1], plane_normal);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points = transformCloudByMatrix(points_list, transform_matrix1, (*points_list).points[0]);
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points = transformPlanarPatchPoints(points_list, plane_normal);
//	
//	//visualizePointCloud(transformed_points, "transformed_points", xy);
//	/*visualizePointCloud(points_list, transformed_points, "transformed_points", xy);
//	std::cout << sqrt(pow((*points_list)[0].x - (*points_list)[1].x,2) + pow((*points_list)[0].y - (*points_list)[1].y,2) +pow((*points_list)[0].z - (*points_list)[1].z,2)) << std::endl;
//	std::cout << sqrt(pow((*transformed_points)[0].x - (*transformed_points)[1].x, 2) + pow((*transformed_points)[0].y - (*transformed_points)[1].y, 2) + pow((*transformed_points)[0].z - (*transformed_points)[1].z,2)) << std::endl;*/
//	double a, b;
//	if (fitting_mode == 0)
//	{
//		LinearRegressionPCA(transformed_points, a, b);
//	}
//	else
//	{
//		if (fitting_mode == 1)
//		{
//			LinearRegressionPerpendicularLeastSquares(transformed_points, a, b);
//		}
//		else
//		{
//			LinearRegressionVerticalLeastSquares(transformed_points, a, b);		
//		}
//	}
//	
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
//	//(*line).push_back(pcl::PointXYZ(0, b, 0));
//	//(*line).push_back(pcl::PointXYZ(1000, 1000*a+b, 0));
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_points = ProjectPointsOntoLine(transformed_points, a, b, error);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_head_tail(new pcl::PointCloud<pcl::PointXYZ>);
//	if (flag_head_tail == 0)
//	{
//		(*transformed_head_tail).push_back(projected_points->points[0]);
//		(*transformed_head_tail).push_back(projected_points->points[1]);
//	}
//	else
//	{
//		FindHeadAndTail(projected_points, transformed_head_tail);
//	}
//	
//	//visualizePointCloud(transformed_points, transformed_head_tail, "transformed_head_tail", xy);
//
//
//
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr coordinate_system((new pcl::PointCloud<pcl::PointXYZ>));
//	(*coordinate_system).push_back(pcl::PointXYZ(0, 0, 0));
//	(*coordinate_system).push_back(pcl::PointXYZ(1, 0, 0));   //x
//	(*coordinate_system).push_back(pcl::PointXYZ(0, 0, 1));   //z
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_coordinate_system = transformCloudByMatrix(coordinate_system, transform_matrix1, (*points_list).points[0]);
//	//visualizePointCloud(points_list, coordinate_system, "coordinate_system", xy);
//	//visualizePointCloud(transformed_points, transformed_coordinate_system, "transformed_coordinate_system", xy);
//	//std::vector<double> transformed_coordinate_normal;
//	std::vector<double> normal_z;
//	normal_z.push_back((*transformed_coordinate_system)[2].x - (*transformed_coordinate_system)[0].x);
//	normal_z.push_back((*transformed_coordinate_system)[2].y - (*transformed_coordinate_system)[0].y);
//	normal_z.push_back((*transformed_coordinate_system)[2].z - (*transformed_coordinate_system)[0].z);
//
//
//
//
//
//	Eigen::Matrix3f transform_matrix2 = getTransformMatrixForAlignmentWithNormalToPlane((*transformed_coordinate_system)[0], (*transformed_coordinate_system)[1], normal_z);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_transformed_points = transformCloudByMatrix(transformed_points, transform_matrix2, (*transformed_coordinate_system)[0]);
//	//visualizePointCloud(points_list, transformed_transformed_points, "transformed_transformed_points", xy);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr head_tail = transformCloudByMatrix(transformed_head_tail, transform_matrix2, (*transformed_coordinate_system)[0]);
//
//	//visualizePointCloud(points_list, head_tail, "head_tail", xy);
//	head.x = (*head_tail)[0].x;
//	head.y = (*head_tail)[0].y;
//	head.z = (*head_tail)[0].z;
//	tail.x = (*head_tail)[1].x;
//	tail.y = (*head_tail)[1].y;
//	tail.z = (*head_tail)[1].z;
//	//head = (*head_tail)[0];
//	//tail = (*head_tail)[1];
//
//
//	return 0;
//}

int Fitting3DPointsToLine2D(ON_3dPointArray &points_opennurbs, ON_3dVector normal, double &error, ON_3dPoint &head, ON_3dPoint &tail, bool flag_head_tail, int fitting_mode)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_list = OpennurbsArray2PointCloud(points_opennurbs);
	std::vector<double> plane_normal = OpennurbsVector2VectorDouble(normal);
	pcl::PointXYZ head_pcl = OpennurbsPoint2PCLPoint(head);
	pcl::PointXYZ tail_pcl = OpennurbsPoint2PCLPoint(tail);
	Fitting3DPointsToLine2D(points_list, plane_normal, error, head_pcl, tail_pcl, flag_head_tail, fitting_mode);
	head.x = head_pcl.x;
	head.y = head_pcl.y;
	head.z = head_pcl.z;

	tail.x = tail_pcl.x;
	tail.y = tail_pcl.y;
	tail.z = tail_pcl.z;

	return 0;
}

int Fitting3DPointsToLine2D(pcl::PointCloud<pcl::PointXYZ>::Ptr &points_list, std::vector<double> plane_normal, double &error,
	pcl::PointXYZ &head, pcl::PointXYZ &tail, bool flag_head_tail, int fitting_mode)
{

	Eigen::Matrix3f transform_matrix1 = getTransformMatrixForAlignmentWithNormalToPlane((*points_list).points[0], (*points_list).points[1], plane_normal);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points = transformCloudByMatrix(points_list, transform_matrix1, (*points_list).points[0]);
	
	//visualizePointCloud(transformed_points, points_list, "transformed_points", xy);
	//std::cout << sqrt(pow((*points_list)[0].x - (*points_list)[1].x,2) + pow((*points_list)[0].y - (*points_list)[1].y,2) +pow((*points_list)[0].z - (*points_list)[1].z,2)) << std::endl;
	//std::cout << sqrt(pow((*transformed_points)[0].x - (*transformed_points)[1].x, 2) + pow((*transformed_points)[0].y - (*transformed_points)[1].y, 2) + pow((*transformed_points)[0].z - (*transformed_points)[1].z,2)) << std::endl;
	double a, b;
	if (fitting_mode == 0)
	{
		LinearRegressionPCA(transformed_points, a, b);
	}
	else
	{
		if (fitting_mode == 1)
		{
			LinearRegressionPerpendicularLeastSquares(transformed_points, a, b);
		}
		else
		{
			LinearRegressionVerticalLeastSquares(transformed_points, a, b);
		}
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
	//(*line).push_back(pcl::PointXYZ(0, b, 0));
	//(*line).push_back(pcl::PointXYZ(1000, 1000*a+b, 0));
	//visualizePointCloud(transformed_points, line, "line", xy);

	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_points = ProjectPointsOntoLine(transformed_points, a, b, error);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_head_tail(new pcl::PointCloud<pcl::PointXYZ>);
	if (flag_head_tail == 0)
	{
		(*transformed_head_tail).push_back(projected_points->points[0]);
		(*transformed_head_tail).push_back(projected_points->points[1]);
	}
	else
	{
		FindHeadAndTail(projected_points, transformed_head_tail);
	}

	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr coordinate_system((new pcl::PointCloud<pcl::PointXYZ>));
	//(*coordinate_system).push_back(pcl::PointXYZ(0, 0, 0));
	//(*coordinate_system).push_back(pcl::PointXYZ(1, 0, 0));   //x
	//(*coordinate_system).push_back(pcl::PointXYZ(0, 0, 1));   //z
	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_coordinate_system = transformCloudByMatrix(coordinate_system, transform_matrix1, (*points_list).points[0]);
	//
	//
	////std::vector<double> transformed_coordinate_normal;
	//std::vector<double> normal;
	//normal.push_back((*transformed_coordinate_system)[2].x - (*transformed_coordinate_system)[0].x);
	//normal.push_back((*transformed_coordinate_system)[2].y - (*transformed_coordinate_system)[0].y);
	//normal.push_back((*transformed_coordinate_system)[2].z - (*transformed_coordinate_system)[0].z);


	//Eigen::Matrix3f transform_matrix2 = getTransformMatrixForAlignmentWithNormalToPlane((*transformed_coordinate_system)[0], (*transformed_coordinate_system)[1], normal);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_transformed_points = transformCloudByMatrix(transformed_points, transform_matrix2, (*transformed_coordinate_system)[0]);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr head_tail = transformCloudByMatrix(transformed_head_tail, transform_matrix2, (*transformed_coordinate_system)[0]);


	pcl::PointCloud<pcl::PointXYZ>::Ptr head_tail(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector3f point, temp;
	for (std::size_t i = 0; i < transformed_head_tail->size(); ++i)
	{
		point = Eigen::Vector3f(transformed_head_tail->at(i).getArray3fMap());

		temp = transform_matrix1.transpose()*point;

		//std::cout << transformed_point << std::endl;
		(*head_tail).push_back(pcl::PointXYZ(temp[0] + (*points_list).points[0].x, temp[1] + (*points_list).points[0].y, temp[2] + (*points_list).points[0].z));
	}
	
	head = (*head_tail)[0];
	tail = (*head_tail)[1];

	/*visualizePointCloud(points_list, "points_list", xy);*/
	//visualizePointCloud(transformed_points, "transformed_points", xy);
	//visualizePointCloud(points_list, coordinate_system, "coordinate_system", xy);
	//visualizePointCloud(transformed_points, transformed_coordinate_system, "transformed_coordinate_system", xy);
	//visualizePointCloud(points_list, "points_list", xy);
	//visualizePointCloud(points_list, transformed_transformed_points, "transformed_transformed_points", xy);
	//visualizePointCloud(points_list, head_tail, "head_tail", xy);
	//visualizePointCloud(transformed_points, transformed_head_tail, "transformed_head_tail", xy);
	
	return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OpennurbsArray2PointCloud(ON_3dPointArray &points_opennurbs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_list(new pcl::PointCloud<pcl::PointXYZ>);
	int size = points_opennurbs.Count();

	for (int i = 0; i < size; i++)
	{
		points_list->push_back(pcl::PointXYZ(points_opennurbs[i].x, points_opennurbs[i].y, points_opennurbs[i].z));
	}
	return points_list;
}

std::vector<double> OpennurbsVector2VectorDouble(ON_3dVector &normal)
{
	//int size = points_opennurbs.SizeOfArray();

	//for (int i = 0; i < 3; i++)
	//{
	std::vector<double> plane_normal;

	plane_normal.push_back(normal.x);
	plane_normal.push_back(normal.y);
	plane_normal.push_back(normal.z);
	//}
	return plane_normal;
}

pcl::PointXYZ OpennurbsPoint2PCLPoint(ON_3dPoint &point_opennurbs)
{
	pcl::PointXYZ point;

	point.x = point_opennurbs.x;
	point.y = point_opennurbs.y;
	point.z = point_opennurbs.z;
	return point;
}


#pragma endregion
//=====================Fitting 3D Points(on one plane) To a Line=====================


//int main()
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr points_list(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<double> plane_normal;
//	std::string load_file;
//	std::getline(std::cin, load_file);
//	ReadTXTFileAsPointCloud(load_file.c_str(), points_list, plane_normal);
//	double error;
//	pcl::PointXYZ head, tail;
//	bool flag = 1;
//	int mode = 0;
//
//	Fitting3DPointsToLine2D(points_list, plane_normal, error, head, tail, flag, mode);
//
//	return 0;
//}










//
////============================PCL visualizer================================
//#pragma region
//boost::shared_ptr<pcl::visualization::PCLVisualizer> visCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string window_label, camera_position camera_pos)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_label));
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPointCloud(cloud_to_visualize, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud");
//	//viewer->addCoordinateSystem(1, "global");
//	viewer->addCoordinateSystem(500, cloud_to_visualize->points[0].x, cloud_to_visualize->points[0].y, cloud_to_visualize->points[0].z, xy);
//
//	// Set camera position according to different input.
//	switch (camera_pos)
//	{
//	case xy:
//		viewer->setCameraPosition(
//			0, 0, 1,//double pos_x, double pos_y, double pos_z,
//			0, 0, -1,//double view_x, double view_y, double view_z,
//			1, 0, 0);//double up_x, double up_y, double up_z, int viewport = 0
//	case yz:
//		viewer->setCameraPosition(
//			1, 0, 0,//double pos_x, double pos_y, double pos_z,                                    
//			-1, 0, 0,//double view_x, double view_y, double view_z,
//			0, 1, 0);//double up_x, double up_y, double up_z, int viewport = 0
//	case xz:
//		viewer->setCameraPosition(
//			0, -1, 0,//double pos_x, double pos_y, double pos_z,                                    
//			0, 1, 0,//double view_x, double view_y, double view_z,
//			0, 0, 1);//double up_x, double up_y, double up_z, int viewport = 0
//	}
//
//	return (viewer);
//}
//
//void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string viewer_window_label, camera_position camera_pos)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//
//	// Make use of the 
//	viewer = visCloud(cloud, viewer_window_label, camera_pos);
//
//	// Reset camera according to the input data. Zoom out so that all data points can be viewed.
//	viewer->resetCamera();
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//	viewer->close();
//
//}
//
//void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string label_viewer_window, camera_position camera_pos)
//{
//	// Window setup
//	pcl::visualization::PCLVisualizer viewer(label_viewer_window);
//	viewer.setBackgroundColor(0, 0, 0);
//	//viewer.addCoordinateSystem(1);
//	viewer.addCoordinateSystem(500, cloud1->points[0].x, cloud1->points[0].y, cloud1->points[0].z, xy);
//	viewer.setCameraPosition(0, 0, 1,//double pos_x, double pos_y, double pos_z,                                    
//		0, 0, 0,//double view_x, double view_y, double view_z,
//		0, 1, 0);//double up_x, double up_y, double up_z, int viewport = 0);
//
//	// Add the first cloud: size 2, white(default)
//	viewer.addPointCloud(cloud1, "cloud1");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");
//	// Define color "red"
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 255, 0);
//	// Add the second cloud: size 3, red (defined)
//	viewer.addPointCloud<pcl::PointXYZ>(cloud2, red, "cloud2");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud2");
//
//	//Auto recenters the view.
//	viewer.resetCamera();
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//	viewer.close();
//}
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, std::string window_label, camera_position camera_pos)
//{
//	// --------------------------------------------------------
//	// -----Open 3D viewer and add point cloud and normals-----
//	// --------------------------------------------------------
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_label));
//	viewer->setBackgroundColor(0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
//	viewer->addPointCloud(cloud, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 5, "normals");
//	//viewer->addCoordinateSystem(500.0, "global");
//	//viewer->initCameraParameters();
//	viewer->addCoordinateSystem(500.0, cloud->at(0).x, cloud->at(0).y, cloud->at(0).z, xy);
//
//	// Set camera position according to different input.
//	switch (camera_pos)
//	{
//	case xy:
//		viewer->setCameraPosition(
//			0, 0, 1,//double pos_x, double pos_y, double pos_z,
//			0, 0, -1,//double view_x, double view_y, double view_z,
//			1, 0, 0);//double up_x, double up_y, double up_z, int viewport = 0
//	case yz:
//		viewer->setCameraPosition(
//			1, 0, 0,//double pos_x, double pos_y, double pos_z,                                    
//			-1, 0, 0,//double view_x, double view_y, double view_z,
//			0, 1, 0);//double up_x, double up_y, double up_z, int viewport = 0
//	case xz:
//		viewer->setCameraPosition(
//			0, -1, 0,//double pos_x, double pos_y, double pos_z,                                    
//			0, 1, 0,//double view_x, double view_y, double view_z,
//			0, 0, 1);//double up_x, double up_y, double up_z, int viewport = 0
//	}
//
//	return (viewer);
//}
//
//void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::string viewer_window_label, camera_position camera_pos)
//{
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//
//	// Make use of the 
//	viewer = normalsVis(cloud, cloud_normals, viewer_window_label, camera_pos);
//
//	// Reset camera according to the input data. Zoom out so that all data points can be viewed.
//	viewer->resetCamera();
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//	viewer->close();
//
//}
//
////============================PCL visualizer================================
//#pragma endregion