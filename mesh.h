#ifndef __MESH_H__
#define __MESH_H__

#include <cstdlib>
#include <vector>
#include "vector3.h"
#include "Geometry.h"

// classes
class HEdge;
class Vertex;
class Face;
class Mesh;
class OneRingHEdge;
class OneRingVertex;
class FaceEnumeration;

// types
typedef std::vector<HEdge*> HEdgeList;
typedef std::vector<Vertex*> VertexList;
typedef std::vector<Face*> FaceList;

// other helper functions
inline void SetPrevNext(HEdge *e1, HEdge *e2);
inline void SetTwin(HEdge *e1, HEdge *e2);
inline void SetFace(Face *f, HEdge *e);

////////// class HEdge //////////
class HEdge {
private:
	HEdge *twin, *prev, *next;	// twin/previous/next half edges
	Vertex *start;				// start vertex
	Face *face;					// left face
	bool boundary;				// flag for boundary edge

	bool valid;

	// it's for count boundary loop use, 
	// you can use it freely for marking in boundary loop counting 
	// and connected component counting
	bool flag;

public:
	/////////////////////////////////////
	// constructor
	HEdge(bool b = false) {
		boundary = b;
		twin = prev = next = NULL;
		start = NULL;
		face = NULL;
		flag = false;
		valid = true;
	}
	/////////////////////////////////////
	// access functions
	HEdge*  Twin() const { return twin; }
	HEdge*  Prev() const { return prev; }
	HEdge*  Next() const { return next; }
	Vertex* Start() const { return start; }
	Vertex* End() const { return next->start; } // for convenience
	Face*   LeftFace() const { return face; }
	bool    Flag() const { return flag; }

	HEdge*  SetTwin(HEdge* e) { return twin = e; }
	HEdge*  SetPrev(HEdge* e) { return prev = e; }
	HEdge*  SetNext(HEdge* e) { return next = e; }
	Vertex* SetStart(Vertex* v) { return start = v; }
	Face*   SetFace(Face* f) { return face = f; }
	bool    SetFlag(bool b) { return flag = b; }
	bool    SetValid(bool b) { return valid = b; }
	bool    IsBoundary() const { return boundary; }
	bool    IsValid() const { return valid; }
};

////////// class OneRingHEdge //////////
// this class is use for access the neighbor HALF EDGES
// of a given vertex, please see Vertex::IsBoundary() for its usage
class OneRingHEdge {
private:
	HEdge *start, *next;
public:
	OneRingHEdge(const Vertex * v);	// constructor
	HEdge * NextHEdge();			// iterator
};

////////// class OneRingVertex //////////
// this class is use for access the neighbor VERTICES 
// of a given vertex, please see Vertex::Valence() for its usage
class OneRingVertex {
private:
	OneRingHEdge ring;
public:
	OneRingVertex(const Vertex * v) : ring(v) { }	// constructor
	Vertex * NextVertex() { HEdge *he = ring.NextHEdge(); return (he) ? he->End() : NULL; } // iterator
};

////////// class Vertex //////////
class Vertex {
private:
	Vector3d position;	// position (x,y,z) in space
	Vector3d normal;	// normal vector for smooth shading rendering
	Vector3d color;		// color value for curvature displaying
	HEdge *he;			// one of half edge starts with this vertex
	int index;			// index in the Mesh::vList, DO NOT UPDATE IT
	int flag;           // 0 for unselected, 1 for selected
	bool valid;
public:
	vector<HEdge*> adjHEdges; // for reading object only, do not use it in other place

	// constructors
	Vertex() : he(NULL), flag(0), valid(true) { }
	Vertex(const Vector3d & v) : he(NULL), position(v), flag(0), valid(true) { }
	Vertex(double x, double y, double z) : he(NULL), position(x, y, z), flag(0), valid(true) { }

	// access functions
	const Vector3d & Position() const { return position; }
	const Vector3d & Normal() const { return normal; }
	const Vector3d & Color() const { return color; }
	HEdge * HalfEdge() const { return he; }
	int Index() const { return index; }
	int Flag() const { return flag; }
	const Vector3d & SetPosition(const Vector3d & p) { return position = p; }
	const Vector3d & SetNormal(const Vector3d & n) { return normal = n; }
	const Vector3d & SetColor(const Vector3d & c) { return color = c; }
	HEdge * SetHalfEdge(HEdge * he) { return Vertex::he = he; }
	int SetIndex(int index) { return Vertex::index = index; }

	int SetFlag(int value) { return Vertex::flag = value; }

	bool IsValid() const { return valid; }
	bool SetValid(bool b) { return valid = b; }

	// check for boundary vertex
	bool IsBoundary() const {
		OneRingHEdge ring(this);
		HEdge *curr = NULL;
		while (curr = ring.NextHEdge())
		{
			if (curr->IsBoundary()) 
				return true;
		}

		return false;
	}

	// compute the valence (# of neighbor vertices)
	int Valence() const {
		int count = 0;
		OneRingVertex ring(this);
		Vertex *curr = NULL;
		while (curr = ring.NextVertex())
		{
			count++;

			if (count >= 10) // in case of error, normally there will be only max. 6 valence
			{
				count = 0;
				break;
			}
		}

		return count;
	}
};

////////// class Face //////////
class Face {
private:
	HEdge * he;
	bool valid;
public:
	// constructor
	Face() : he(NULL), valid(true) { }

	// access function
	HEdge * HalfEdge() const { return he; }
	HEdge * SetHalfEdge(HEdge * he) { return Face::he = he; }

	// check for boundary face
	bool IsBoundary() {
		HEdge *curr = he;
		do {
			if (curr->Twin()->IsBoundary()) return true;
			curr = curr->Next();
		} while (curr != he);
		return false;
	}
	bool SetValid(bool b) { return valid = b; }
	bool IsValid() const { return valid; }
};

////////// class Mesh //////////
class Mesh {
public:
	HEdgeList heList;		// list of NON-boundary half edges
	HEdgeList bheList;		// list of boundary half egdes
	VertexList vList;		// list of vertices
	FaceList fList;			// list of faces

	// constructor & destructors
	Mesh() { }
	~Mesh() { Clear(); }

	// access functions
	const HEdgeList Edges() const { return heList; }
	const HEdgeList BoundaryEdges() const { return bheList; }
	const VertexList Vertices() const { return vList; }
	const FaceList Faces() const { return fList; }

	// functions for loading obj files,
	// you DO NOT need to understand and use them
	void AddVertex(Vertex *v) { vList.push_back(v); }
	void AddFace(int v1, int v2, int v3);
	void Clear() {
		size_t i;
		for (i = 0; i < heList.size(); i++) delete heList[i];
		for (i = 0; i < bheList.size(); i++) delete bheList[i];
		for (i = 0; i < vList.size(); i++) delete vList[i];
		for (i = 0; i < fList.size(); i++) delete fList[i];
		heList.clear();
		bheList.clear();
		vList.clear();
		fList.clear();
	}

	// Use for delete a vertex.
	void DeleteVertex();

	Vector3d MinCoord() const {
		Vector3d minCoord;
		for (size_t i = 0; i < vList.size(); i++)
			minCoord = minCoord.Min((vList[i])->Position());
		return minCoord;
	}

	Vector3d MaxCoord() const {
		Vector3d maxCoord;
		for (size_t i = 0; i < vList.size(); i++)
			maxCoord = maxCoord.Max(vList[i]->Position());
		return maxCoord;
	}

	bool LoadObjFile(const char * filename);

	void mesh2halfedge(ON_Mesh* mesh);

	/************************************************************************/
	/* please implement the following functions */
	void DisplayMeshInfo();
	void ComputeVertexNormals();
	//void ComputeVertexCurvatures();
	void UmbrellaSmooth();
	//void ImplicitUmbrellaSmooth();
	void UmbrellaSmoothCotangent();
	//void ImplicitUmbrellaSmoothCotangent();
	/************************************************************************/

	// additional helper functions
	// implement them if in needed
	int CountBoundaryLoops();
	int CountConnectedComponents();

	void Scaling(ON_3dPoint centerp, double factor); //make scaling according to the given center point and factor
};

// other helper functions
inline void SetPrevNext(HEdge *e1, HEdge *e2) { e1->SetNext(e2); e2->SetPrev(e1); }
inline void SetTwin(HEdge *e1, HEdge *e2) { e1->SetTwin(e2); e2->SetTwin(e1); }
inline void SetFace(Face *f, HEdge *e) { f->SetHalfEdge(e); e->SetFace(f); }

#endif // __MESH_H__