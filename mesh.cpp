#include "mesh.h"
#include <cstring>
#include <iostream>
#include <strstream>
#include <fstream>
#include <cmath>
#include <float.h>
#define _USE_MATH_DEFINES
//#include <math.h>

using namespace std;

/////////////////////////////////////////
// helping inline functions
inline double Cot(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
	Vector3d v1 = p1 - p2;
	Vector3d v2 = p3 - p2;

	v1 /= v1.L2Norm();
	v2 /= v2.L2Norm();
	double tmp = v1.Dot(v2);
	return 1.0 / tan(acos(tmp));
}

inline double Area(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
	Vector3d v1 = p2 - p1;
	Vector3d v2 = p3 - p1;
	return v1.Cross(v2).L2Norm() / 2.0;
}


/////////////////////////////////////////
// implementation of OneRingHEdge class
OneRingHEdge::OneRingHEdge(const Vertex * v) {
	if (v == NULL) start = next = NULL;
	else start = next = v->HalfEdge();
}

HEdge * OneRingHEdge::NextHEdge() {
	HEdge *ret = next;
	if (next && next->Prev()->Twin() != start)
		next = next->Prev()->Twin();
	else
		next = NULL;
	return ret;
}

/////////////////////////////////////////
// implementation of Mesh class
//
// function AddFace
// it's only for loading obj model, you do not need to understand it
void Mesh::AddFace(int v1, int v2, int v3) {
	int i;
	HEdge *he[3], *bhe[3];
	Vertex *v[3];
	Face *f;

	// obtain objects
	for (i = 0; i < 3; i++) he[i] = new HEdge();
	for (i = 0; i < 3; i++) bhe[i] = new HEdge(true);
	v[0] = vList[v1];
	v[1] = vList[v2];
	v[2] = vList[v3];
	f = new Face();

	// connect prev-next pointers
	SetPrevNext(he[0], he[1]);
	SetPrevNext(he[1], he[2]);
	SetPrevNext(he[2], he[0]);
	SetPrevNext(bhe[0], bhe[1]);
	SetPrevNext(bhe[1], bhe[2]);
	SetPrevNext(bhe[2], bhe[0]);

	// connect twin pointers
	SetTwin(he[0], bhe[0]);
	SetTwin(he[1], bhe[2]);
	SetTwin(he[2], bhe[1]);

	// connect start pointers for bhe
	bhe[0]->SetStart(v[1]);
	bhe[1]->SetStart(v[0]);
	bhe[2]->SetStart(v[2]);
	for (i = 0; i < 3; i++) he[i]->SetStart(v[i]);

	// connect start pointers
	// connect face-hedge pointers
	for (i = 0; i < 3; i++) {
		v[i]->SetHalfEdge(he[i]);
		v[i]->adjHEdges.push_back(he[i]);
		SetFace(f, he[i]);
	}
	v[0]->adjHEdges.push_back(bhe[1]);
	v[1]->adjHEdges.push_back(bhe[0]);
	v[2]->adjHEdges.push_back(bhe[2]);

	// merge boundary if in need
	for (i = 0; i < 3; i++) {
		Vertex *start = bhe[i]->Start();
		Vertex *end = bhe[i]->End();
		for (size_t j = 0; j < end->adjHEdges.size(); j++) {
			HEdge *curr = end->adjHEdges[j];
			if (curr->IsBoundary() && curr->End() == start) {
				SetPrevNext(bhe[i]->Prev(), curr->Next());
				SetPrevNext(curr->Prev(), bhe[i]->Next());
				SetTwin(bhe[i]->Twin(), curr->Twin());
				bhe[i]->SetStart(NULL);	// mark as unused
				curr->SetStart(NULL);	// mark as unused
				break;
			}
		}
	}

	// finally add hedges and faces to list
	for (i = 0; i < 3; i++) heList.push_back(he[i]);
	for (i = 0; i < 3; i++) bheList.push_back(bhe[i]);
	fList.push_back(f);
}

// function LoadObjFile
// it's only for loading obj model, you do not need to understand it
bool Mesh::LoadObjFile(const char *filename) {
	if (filename == NULL || strlen(filename) == 0) return false;
	ifstream ifs(filename);
	if (ifs.fail()) return false;

	Clear();

	char buf[1024], type[1024];
	do {
		ifs.getline(buf, 1024);
		istrstream iss(buf);
		iss >> type;

		// vertex
		if (strcmp(type, "v") == 0) {
			double x, y, z;
			iss >> x >> y >> z;
			AddVertex(new Vertex(x, y, z));
		}
		// face
		else if (strcmp(type, "f") == 0) {
			int index[3];
			iss >> index[0] >> index[1] >> index[2];
			AddFace(index[0] - 1, index[1] - 1, index[2] - 1);
		}
	} while (!ifs.eof());
	ifs.close();

	size_t i;
	Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i = 0; i < vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());

	Vector3d tot;
	for (i = 0; i < vList.size(); i++) tot += vList[i]->Position();
	Vector3d avg = tot / vList.size();
	for (i = 0; i < vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

	HEdgeList list;
	for (i = 0; i < bheList.size(); i++)
	if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i = 0; i < vList.size(); i++)
	{
		vList[i]->adjHEdges.clear();
		vList[i]->SetIndex((int)i);
		vList[i]->SetFlag(0);
	}

	return true;
}

void Mesh::mesh2halfedge(ON_Mesh* mesh)
{
	int nodeindex[3];

	for (int vi = 0; vi < mesh->VertexCount(); vi++)
	{
		ON_3dPoint v = mesh->m_V[vi];
		AddVertex(new Vertex(v.x, v.y, v.z));
	}

	for (int fi = 0; fi < mesh->FaceCount(); fi++)
	{
		const ON_MeshFace& f = mesh->m_F[fi];

		nodeindex[0] = f.vi[0];
		nodeindex[1] = f.vi[1];
		nodeindex[2] = f.vi[2];

		AddFace(nodeindex[0], nodeindex[1], nodeindex[2]);
	}

	size_t i;
	//Vector3d box = this->MaxCoord() - this->MinCoord();
	//for (i = 0; i < vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());

	//Vector3d tot;
	//for (i = 0; i < vList.size(); i++) tot += vList[i]->Position();
	//Vector3d avg = tot / vList.size();
	//for (i = 0; i < vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

	HEdgeList list;
	for (i = 0; i < bheList.size(); i++)
	if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i = 0; i < vList.size(); i++)
	{
		vList[i]->adjHEdges.clear();
		vList[i]->SetIndex((int)i);
		vList[i]->SetFlag(0);
	}
}

void Mesh::DisplayMeshInfo()
{
	// Find number of vertices, faces, non-boundary edges and boundary edges
	int num_vertices = vList.size();
	int num_faces = fList.size();
	int num_hedges = heList.size() + bheList.size(); // including twin edges
	int num_bedges = 0.5*bheList.size();

	// Find number of loop
	int num_loop = 0;
	if (num_bedges != 0)
	{
		num_loop = CountBoundaryLoops();
	}

	// Find number of connected components
	int num_connectedComp = CountConnectedComponents();

	// Find number of genus by Euler formula x(M) = 2(c-g) - b = v-e+f
	int num_genus = num_connectedComp - 0.5*(num_vertices - 0.5*num_hedges + num_faces + num_loop);

	printf("number of vertices is %d\n", num_vertices);
	printf("number of faces is %d\n", num_faces);
	printf("number of half edges is %d\n", num_hedges);
	//printf("number of boundaries is %d\n", num_bedges);
	printf("number of boundary loops is %d\n", num_loop);
	printf("number of connected components is %d\n", num_connectedComp);
	printf("number of genus is %d\n", num_genus);
}

// Count number of boundary loops
int Mesh::CountBoundaryLoops()
{
	int num_loop = 0;
	HEdgeList tmpbheList = bheList;
	while (tmpbheList.size() != 0)
	{
		HEdgeList Q;
		HEdge* start_e = tmpbheList[0];
		HEdge* next_e = start_e->Next();
		num_loop++;
		tmpbheList.erase(tmpbheList.begin());
		while (next_e != start_e)
		{
			for (vector<HEdge*>::iterator itr = tmpbheList.begin(); itr != tmpbheList.end(); ++itr)
			{
				if (*itr == next_e)
				{
					tmpbheList.erase(itr);
					break;
				}
			}
			next_e = next_e->Next();
			for (vector<HEdge*>::iterator itr = tmpbheList.begin(); itr != tmpbheList.end(); ++itr)
			{
				if (*itr == next_e)
				{
					tmpbheList.erase(itr);
					break;
				}
			}
		}
	}
	return num_loop;
}

// Count number of connected Components
int Mesh::CountConnectedComponents()
{
	// Use Breadth first search algorithm
	int count_vertices = 0;
	int num_connectedComp = 0;
	while (count_vertices != vList.size())
	{
		++num_connectedComp;
		Vertex* v = NULL;
		for (int i = 0; i < vList.size(); i++)
		{
			v = vList[i];
			if (!v->Flag()) break;
		}

		VertexList Q;
		Q.push_back(v);
		v->SetFlag(1);

		while (Q.size() != 0)
		{
			++count_vertices;
			Vertex* tmp_v = Q[0];
			Q.erase(Q.begin());
			OneRingVertex ring(tmp_v);
			Vertex* curr = NULL;
			while (curr = ring.NextVertex())
			{
				if (!curr->Flag())
				{
					Q.push_back(curr);
					curr->SetFlag(1);
				}
			}

		}
	}

	for (int i = 0; i < vList.size(); ++i)
	{
		Vertex* v = vList[i];
		v->SetFlag(0);
	}
	return num_connectedComp;
}


void Mesh::Scaling(ON_3dPoint centerp, double factor)
{
	Vector3d cp(centerp.x, centerp.y, centerp.z);
	for (int i = 0; i < vList.size(); i++)
	{
		Vector3d newpos = cp + (vList[i]->Position() - cp)*factor;
		vList[i]->SetPosition(newpos);
	}
}

// Delete a vertex that are already marked.
void Mesh::DeleteVertex()
{
	for (vector<Vertex*>::iterator itr = vList.begin(); itr != vList.end(); ++itr)
	{
		Vertex* v = *itr;
		if (v->Flag())
		{
			OneRingHEdge ring(v);
			//HEdgeList eToDel;
			//FaceList fToDel;
			HEdge* currE = NULL;
			Face* currF = NULL;
			while (currE = ring.NextHEdge())
			{
				if (currE->IsValid())
				{
					//eToDel.push_back(currE);
					//eToDel.push_back(currE->Twin());
					currE->SetValid(false);
					currE->Twin()->SetValid(false);
					if (!currE->IsBoundary())
					{
						currF = currE->LeftFace();
						if (currF->IsValid())
						{
							//fToDel.push_back(currF);
							currF->SetValid(false);
						}
					}


				}

			}
			// Delete vertex
			vList.erase(itr);
			// Delete half-edges
			HEdgeList totEdge = heList;
			for (int i = 0; i < bheList.size(); ++i)
			{
				totEdge.push_back(bheList[i]);
			}
			for (vector<HEdge*>::iterator itr = totEdge.begin(); itr != totEdge.end();)
			{
				HEdge* e = *itr;
				if (!e->IsValid())
				{
					itr = totEdge.erase(itr);
				}
				else
					itr++;
			}
			// Delete faces
			for (vector<Face*>::iterator itr = fList.begin(); itr != fList.end();)
			{
				Face* f = *itr;
				if (!f->IsValid())
				{
					itr = fList.erase(itr);
				}
				else
					itr++;
			}

			break;
		}
	}
}

void Mesh::ComputeVertexNormals()
{
	int num_vertices = vList.size();
	int count = 0;
	while (count != num_vertices - 1)
	{
		Vertex* v = vList[count];
		Vector3d t1, t2;
		int val = v->Valence();
		double step = 2 * M_PI / (double)val;
		//VertexList tmp_vlist;

		if (!v->IsBoundary())
		{
			double angle = 0.0;
			OneRingVertex ring(v);
			Vertex* curr = NULL;
			while (curr = ring.NextVertex())
			{
				t1 += cos(angle)*curr->Position();
				t2 += sin(angle)*curr->Position();
				angle += step;
			}

		}
		// edited
		else
		{
			//v->SetNormal(Vector3d(0.0,0.0,0.0));

			HEdge* bhe = NULL;
			HEdge* he = NULL;
			OneRingHEdge ring(v);
			while (he = ring.NextHEdge())
			{
				if (he->IsBoundary())
				{
					bhe = he;
				}
			}

			if (bhe->End() == NULL | bhe->Prev()->Start() == NULL)
			{
				v->SetNormal(Vector3d(0.0,0.0,0.0));
				continue;
			}

			Vector3d p_ed = bhe->End()->Position();
			Vector3d p_st = bhe->Prev()->Start()->Position();
			t1 = p_st - p_ed;
			if (val == 2)
			{
				t2 = p_st - p_ed + 2.0*v->Position();
			}
			else if (val == 3)
			{
				Vector3d p_1 = bhe->Twin()->Prev()->End()->Position();
				t2 = p_1 - v->Position();
			}
			else
			{
				double Theta = M_PI / (double)(val - 1);
				double alpha = 2 * cos(Theta) - 2;
				t2 = sin(Theta)*(p_st + p_ed);
				HEdge* he2 = bhe->Prev()->Twin()->Prev()->Twin();
				int t = 1;
				while (he2 != bhe)
				{
					t2 += alpha*sin(t*Theta)*he2->End()->Position();
					he2 = he2->Prev()->Twin();
					t++;
				}
			}
			
		}

		Vector3d normal = t1.Cross(t2);
		normal /= normal.L2Norm();
		v->SetNormal(normal);
		count = count + 1;
		//printf("Vertex %d normal is %f %f %f\n", count, v->Normal().X(), v->Normal().Y(), v->Normal().Z());
	}

}

void Mesh::UmbrellaSmooth()
{
	vector<Vector3d> nlist;
	// Only smooth inner vertices.
	for (int i = 0; i < vList.size(); i++) {
		qDebug("node index: %d.\n", i);
		//printf("node index: %d.\n", i);

		//if (i == 8559)
		//{
		//	qDebug("bug exists here.\n");
		//}
		
		Vertex *u = vList[i];
		OneRingVertex ring(u);
		int num = u->Valence();

		if (num == 0) // since num cannot be 0 for correct topology case
		{
			nlist.push_back(vList[i]->Position());
			continue;
		}
		if (vList[i]->IsBoundary()) {
			nlist.push_back(vList[i]->Position());
			continue;
		}
		Vertex *v = NULL;
		Vector3d newPosition;
		double val = 0.0;
		while (v = ring.NextVertex())
		{
			val += 1.0;
			newPosition += v->Position();
		}

		newPosition /= val;
		if (_isnan(newPosition.L2Norm())) newPosition = u->Position();
		//if (newPosition.L2Norm()>1e5) newPosition = u->Position();

		nlist.push_back(newPosition);
	}

	for (int i = 0; i < vList.size(); i++)
		vList[i]->SetPosition(nlist[i]);
		
}

void Mesh::UmbrellaSmoothCotangent()
{
	int n = vList.size();
	// Only smooth inner vertices. 
	for (int i = 0; i < n; i++)
	{
		if (vList[i]->IsBoundary()) continue;

		Vertex* u = vList[i];
		OneRingHEdge ring(u);
		HEdge* he = NULL;
		HEdgeList helist;
		double val = u->Valence();
		vector<double> wij;
		double sum_w = 0.0;
		while (he = ring.NextHEdge())
		{
			Vertex *v = he->End();
			Vertex *vNext = he->Next()->End();
			Vertex *vPrev = he->Twin()->Prev()->Start();
			double cotA = Cot(v->Position(), vNext->Position(), u->Position());
			double cotB = Cot(u->Position(), vPrev->Position(), v->Position());
			wij.push_back(cotA + cotB);
			helist.push_back(he);
			sum_w += cotA + cotB;
		}

		double ratio = 1.0; // can be set to other values.
		double X = (1 - ratio)*u->Position().X();
		double Y = (1 - ratio)*u->Position().Y();
		double Z = (1 - ratio)*u->Position().Z();

		for (int j = 0; j < val; ++j)
		{
			X += wij[j] / sum_w*helist[j]->End()->Position().X();
			Y += wij[j] / sum_w*helist[j]->End()->Position().Y();
			Z += wij[j] / sum_w*helist[j]->End()->Position().Z();
		}
		//printf("Vertex %d old position is %f %f %f\n", i, vList[i]->Position().X(), vList[i]->Position().Y(), vList[i]->Position().Z());
		vList[i]->SetPosition(Vector3d(X, Y, Z));
		//printf("Vertex %d new position is %f %f %f\n", i, X, Y, Z);
	}

}

//void Mesh::ImplicitUmbrellaSmooth()
//{
//	/*************************/
//	/* insert your code here */
//	/*************************/
//	int n = vList.size();
//	Matrix* L = new Matrix(n, n);
//	for (int i = 0; i < n; i++)
//	{
//		Vertex *u = vList[i];
//		OneRingHEdge ring(u);
//		HEdge *he = NULL;
//		double val = u->Valence();
//		while ((he = ring.NextHEdge()) != NULL)
//			L->AddElement(i, he->End()->Index(), -1.0 / val);
//		L->AddElement(i, i, 2.0);
//	}
//	L->SortMatrix();
//
//	// Use BCG to solve Lx(i+1) = x(i).
//	double* b = new double[n];
//	double* x[3];
//	for (int i = 0; i < 3; i++)
//	{
//		x[i] = new double[n];
//		for (int j = 0; j < n; j++)
//		{
//			x[i][j] = b[j] = vList[j]->Position()[i];
//		}
//		L->BCG(b, x[i]);
//	}
//
//	for (int i = 0; i < n; i++)
//	{
//		double X = x[0][i];
//		double Y = x[1][i];
//		double Z = x[2][i];
//		vList[i]->SetPosition(Vector3d(X, Y, Z));
//	}
//
//	delete[] b;
//	delete[] x[0];
//	delete[] x[1];
//	delete[] x[2];
//	delete L;
//}
//
//void Mesh::ImplicitUmbrellaSmoothCotangent()
//{
//	/*************************/
//	/* insert your code here */
//	/*************************/
//	//Matrix* t1 = new Matrix(2,2);
//	//Matrix* t2 = new Matrix(2,2);
//	//Matrix* Lt = new Matrix(2, 2);
//	//t1->AddElement(0,0,1);
//	//t1->AddElement(1,0,1);
//	//t1->AddElement(1,1,1);
//	//t2->AddElement(0,1,1);
//	//t2->AddElement(1,1,1);
//	//t2->AddElement(0,0,1);
//	//t2->AddElement(1,0,2);
//	//t1->SortMatrix();
//	//t2->SortMatrix();
//	//double* t2 = new double[2];
//	//double* res = new double[2];
//	//res[0] = 2;
//	//res[1] = 1;
//	//t2[0] = 2;
//	//t2[1] = 1;
//	//t1->BCG(t2, res);
//	//cout<<res[0]<<" "<<res[1]<<endl;
//	//t1->MultiplyMatrix(t2, Lt);
//
//	int n = vList.size();
//	Matrix* L = new Matrix(n, n);
//	//Matrix* Lt = new Matrix(n, n);
//	//Matrix* Mul = new Matrix(n, n);
//	// Only smooth inner vertices. 
//	for (int i = 0; i < n; i++)
//	{
//		if (vList[i]->IsBoundary()) continue;
//
//		Vertex* u = vList[i];
//		OneRingHEdge ring(u);
//		HEdge* he = NULL;
//		HEdgeList helist;
//		double val = u->Valence();
//		vector<double> wij;
//		double sum_w = 0.0;
//		double ratio = 1.0;
//		while (he = ring.NextHEdge())
//		{
//			Vertex *v = he->End();
//			Vertex *vNext = he->Next()->End();
//			Vertex *vPrev = he->Twin()->Prev()->Start();
//			double cotA = Cot(v->Position(), vNext->Position(), u->Position());
//			double cotB = Cot(u->Position(), vPrev->Position(), v->Position());
//			wij.push_back(cotA + cotB);
//			helist.push_back(he);
//			sum_w += cotA + cotB;
//		}
//		for (int j = 0; j < val; j++)
//		{
//			L->AddElement(i, helist[j]->End()->Index(), -ratio*wij[j] / sum_w);
//			//Lt->AddElement(helist[j]->End()->Index(), i, -wij[j]/sum_w);
//		}
//		L->AddElement(i, i, 1.0 + ratio*1.0);
//		//Lt->AddElement(i, i, 1.0+ratio*1.0);
//	}
//	L->SortMatrix();
//	//Lt->SortMatrix();
//	//L->MultiplyMatrix(Lt, Mul);
//	//double* d = new double[n];
//	//for (int i = 0;i<n;i++) 
//	//    d[i] = L[i][]
//
//	// Use BCG to solve Lx(i+1) = x(i).
//	double* b = new double[n];
//	double* x[3];
//	for (int i = 0; i < 3; i++)
//	{
//		x[i] = new double[n];
//		for (int j = 0; j < n; j++)
//		{
//			x[i][j] = b[j] = vList[j]->Position()[i];
//		}
//		//L->BCG_ATA(b, x[i]);
//		L->BCG(b, x[i]);
//		//L->BCG_New(b, x[i]);
//	}
//
//	for (int i = 0; i < n; i++)
//	{
//		double X = x[0][i];
//		double Y = x[1][i];
//		double Z = x[2][i];
//		vList[i]->SetPosition(Vector3d(X, Y, Z));
//	}
//
//	delete[] b;
//	delete[] x[0];
//	delete[] x[1];
//	delete[] x[2];
//	delete L;
//
//}

//void Mesh::ComputeVertexCurvatures()
//{
//	/*************************/
//	/* insert your code here */
//	/*************************/
//	size_t i, j;
//	vector<double> curvatureList;
//	double meanCurvature = 0.0;
//
//	for (i = 0, j = 0; i < vList.size(); i++)
//	{
//		if (vList[i]->IsBoundary())
//		{
//			curvatureList.push_back(0.0);
//			vList[i]->SetColor(Vector3d(0, 0, 0));
//			continue;
//		}
//		j++;
//		Vertex *u = vList[i];
//		OneRingHEdge ring(u);
//		HEdge *he = NULL;
//		Vector3d curvatureNormal;
//		double area = 0.0;
//		while ((he = ring.NextHEdge()) != NULL)
//		{
//			Vertex *v = he->End();
//			Vertex *vNext = he->Next()->End();
//			Vertex *vPrev = he->Twin()->Prev()->Start();
//			double cotA = Cot(v->Position(), vNext->Position(), u->Position());
//			double cotB = Cot(u->Position(), vPrev->Position(), v->Position());
//			curvatureNormal += (cotA + cotB) * (v->Position() - u->Position());
//			area += Area(u->Position(), v->Position(), vNext->Position());
//		}
//		curvatureNormal /= -4.0 * area;
//		double cur = curvatureNormal.L2Norm();
//		meanCurvature += cur;
//		curvatureList.push_back(cur);
//		//printf("Index %d, curvature is %f\n", i, cur);
//	}
//	meanCurvature /= (double)j;
//
//	// Color ramp.
//	for (i = 0; i < vList.size(); i++) {
//		if (vList[i]->IsBoundary())
//			continue;
//
//		Vertex *u = vList[i];
//		double diff = curvatureList[i] - meanCurvature;
//		if (diff >= 0) u->SetColor(Vector3d(diff, 1, 0));
//		else u->SetColor(Vector3d(0, 1, -diff));
//	}
//}

