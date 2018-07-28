#pragma once
#include "Geometry.h"
#include "ANN.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include "../GUI/ProgressWidget.h"
#include "MCTable.h"
#include "mpVector.h"
#include "PLYWriter.h"
#include "mesh.h"
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>

#define kEpsilon	1e-8
#define MAXDIS		6.5		//maximum thickness of jigblock
#define MINDIS		3.1		//minimum distance from jigblock front face to tooth
#define OVERHANGDIS	1.5		//maximum thickness for jigblock slot overhang feature
#define JIGOFFSET	1 //distance from jigblock bottom face to tooth bottom plane
#define JIGWIDTH	5	//jigblock width
#define JIGOVERHEIGHT	0.8	//distance from top face of the bracket slot to the top plane of the jig block
#define JIGOCCLUHEIGHT  0.5 //jig occlusal height for dual projection

// a 2D node tree constructed by ANN lib.
class NodeTree
{
public:
	NodeTree(ON_3dPoint ori, ON_3dVector nor, ON_3dVector x);
	~NodeTree();
	void SetMesh(ON_Mesh *meshin){ m_pMesh = meshin; };

	//Project all nodes that locate in the front view to the defined plane
	//projected 2D node coords will be stored in the datapts.
	//normal is the projection direction.
	void NodeProjectToPlane();
	void ConstructKDTree();
	
	//search the nearest k nodes, and return the node topology index, do remember to declare int topindex[k]
	//ahead of this function.
	bool ANNSearch(ON_3dPoint querypnt, int k, int *topindex);

private:
	ON_Mesh *m_pMesh;
	ANNpointArray datapts;
	int nodenum;
	vector<int> nodetopindex; //store the corresponding datapnt topology index in the mesh structure.
	ANNkd_tree *kdtree;
	ON_3dPoint origin;
	ON_3dVector normal;
	ON_3dVector x_axis;
	ON_3dVector y_axis;
};


class JigFace;
class JigTriangle
{
public:
	//the order of the three vertices respect the right-hand law, pointing towards the normal.
	JigTriangle(int v1, int v2, int v3);
	~JigTriangle(){};
	ON_3dPoint GetVertex(int vi); //vi=0,1,2
	int	GetNodei(int i){ return nodei[i]; };
	ON_3dVector GetNormal(){ return normal; };
	void SetNormal(ON_3dVector nor){ normal = nor; };
	void CalNormal(bool reversed=false);
	void SetFace(JigFace *f){ m_face = f; };
	bool CheckTopology(); //return false if any nodei=-1;

private:
	int nodei[3];
	ON_3dVector normal;
	JigFace *m_face;
};

class JigFace : public FEntity
{
public:
	JigFace(){};
	/*			1
			------------->xaxis
			|			|
		0	|	origin	|height	 2
	left   \|/yaxis		|  right
			-------------
				3  width
	*/
	JigFace(ON_3dPoint origin, ON_3dVector nor, ON_3dVector xdir, double height, double width, int density); //creating a flat jigface by setting its properties.
	
	//when using this function, make sure that the leftbnd and rightbnd comply with the
	//specified directions shown on top. otherwise the normal of this face would be opposite.
	JigFace(ON_3dPointArray &leftbnd, ON_3dPointArray &rightbnd, int density); //creating the jigface by setting the left and right boundary
	~JigFace(){};
	JigFace & operator =(const JigFace &jf);
	void MeshTriangle();
	ON_3dPoint GetNode(int nodei){ return nodelist[nodei]; };

	//get the 4 vertices of the rectangle
	ON_3dPointArray GetNodes(bool isCL, double upper, double lower, double left, double right, int &xn, int &yn);
	ON_3dPointArray GetBoundary(int bi); //bi=boundary index(0-3), point array are in clockwise order
	void SetNodeList(ON_3dPointArray a){ nodelist = a; };
	const ON_3dPointArray& GetNodeList(){ return nodelist; };
	ON_3dVector GetNormal(int tri_index){ return trianglelist[tri_index].GetNormal(); };
//	vector<JigTriangle> GetTriangleList(){ return trianglelist; };
	int GetFaceNum(){ return trianglelist.size(); };
	//project the face along given direction to the tooth and its bracket, the resultant
	//node coordinates will change.
	void RayProjection(ON_3dVector dir,const double maxdistance, Tooth *tooth_in);

	void RayProjection_f(ON_3dVector dir, double maxdistance, Tooth *tooth_in, double& min_dis);

	void Translate(ON_3dVector v);

	//Dig a hole on the surface, return the boundary point list, true to return clockwise, false to return CCL.
	ON_3dPointArray DigAHole(bool isCL, double upper, double lower, double left, double right);

	// output old data structure
	void append_JigFace(std::ostream &output);
	void append_JigFace(QDataStream &output);
	void append_JigFace(FILE* fp);

	bool ExportSTL_binary(FILE* fout, bool isBinary = true);
	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
private: 
	bool RayTriangleIntersect(const ON_3dPoint &orig, const ON_3dVector &dir, // ray
		const ON_3dPoint &v0, const ON_3dPoint &v1, const ON_3dPoint &v2, // triangle
		float &t, // intersection point P = orig + t*dir
		float &u, // weight of v0
		float &v); // weight of v1
	bool RayMeshIntersect(ON_3dPoint &orig, ON_3dVector &dir,
		const ON_MeshTopology* T_mesh);

private: //members
	ON_3dPointArray nodelist;
	vector<JigTriangle> trianglelist;
	LaGenMatInt nodeindex;	
	LaGenMatInt nodeflag; // used for hole digging
	GLuint	tempList, m_uListFlat, m_uListSmooth, m_uListMesh, m_uListPoints;
};

class PntBlock : public FEntity // a class for LDNI point in/out data structure
{
public:
	PntBlock(){};
	~PntBlock(){};
	PntBlock(int upperorlower, Tooth *tooth_in, double height, double width, double depth, double resol, double occluheight = JIGOCCLUHEIGHT);
	void ZProjection(double & min_dis); // project the plane along zaxis to create samplepnts, part of the construction functions
	void YProjection(double maxdis); // project the bottom plane along -yaxis to create occlusal feature.
	void ZShifting(double distance); //modify material to shift the front face along zaxis
	void OffsetBnd(); // change the outer most boundary value to zero to facilitate mesh construction
	void MakeHole(bool isbracket, int upperorlower, double zwidth);

	/////////////////////////////// Accessing methods ///////////////////////////////////////////
	bool GetPnt(int xi, int yi, int zi, ON_3dPoint &coord); // coord: output coordinates if return true
	void SetPnt(int xi, int yi, int zi, bool value);
	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	InitDisplayList();

public: //public members for direct usage from outside calling
	double samplewidth;
	ON_3dPoint origin; //origin is on the upper-left corner.
	ON_3dVector xaxis, yaxis, zaxis;
	int xn, yn, zn;

private:
	vector<LaGenMatInt> samplepnts; // 0 1 represent in/out status	
	Tooth *m_tooth;	
	GLuint m_uListPoints;
};

class MeshBlock : public FEntity // a class for LDNI point mesh reconstruction
{
public:
	typedef unsigned int PointIdx;
	typedef FMesh::VertexHandle VertexHandle;
	typedef ON_4fPoint FPoint;
	MeshBlock(){ isNormal = false; };
	~MeshBlock();
	void MeshReconstruct(PntBlock *pb); //reconstruct 2-manifold mesh structure from PntBlock

	ON_3dPoint Coord(double px, double py, double pz); //get the coord w.r.t the frame, pz along -z
	int GetFaceNum(); // get face number;
	//output
	bool ExportSTL_binary(FILE* fout, bool isBinary = true);
	// for test only
	bool ExportPLY(const char*filename);
	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
private:


	bool isNormal; // if the face normal is already computed
	GLuint	tempList, m_uListSmooth, m_uListMesh;

	// Open mesh structure
	FMesh* omesh;

public:
	// size and pos info
	ON_3dPoint origin;
	ON_3dVector xdir, ydir, zdir;
	double xlength, ylength, zlength;
};

class JigBlock : public FEntity  // this class is to represent block mesh with 6 faces
{
public:
	JigBlock(){};
	JigBlock(int upperorlower, Tooth *toothin, double height, double width, int density);
	JigBlock(ON_3dPointArray leftbnd, ON_3dPointArray rightbnd, double resol); //to generate connector bar
	~JigBlock();
	ON_3dPointArray DigAHole(int face_index, bool isCL, double upper, double lower, double left, double right);
	JigFace* GetFace(int face_i){ return m_faces[face_i]; };

	int GetFaceNum(); // get face number
	//output old data structure, obseleted functions
	void append_JigBlock(std::ostream &output);
	void append_JigBlock(QDataStream &output);
	void append_JigBlock(FILE* fp);

	bool ExportSTL_binary(FILE* fout, bool isBinary = true);
	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
private:
	JigFace *m_faces[6]; //sequence: front, rear, up, down, left, right
	Tooth *m_tooth;
	double mindis2tooth;
};


class Jig : public FEntity
{
public:
	Jig(int n);
	~Jig();
//	void CreateJig(Teeth* teethin, const vector<int> &selected);	
//	void ConnectBlocks(double upper = 0.1, double lower = 0.5, double left = 0, double right = 0.5);
//	void ConnectBlocks_curvy(int upperorlower, double upper = 0.3, double lower = 0.7, double width = 0.3);
	
	void CreateJig2(Teeth* teethin, const vector<int> &selected, double jigwidth = JIGWIDTH, double occluheight = JIGOCCLUHEIGHT, double res = 0.1);
	void ConnectmBlocks_curvy(int upperorlower, double upper = 0.3, double lower = 0.7, double width = 0.3);
	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
	//bool ExportSTL(std::ostream &output); // C++ method
	//bool ExportSTL(QDataStream &output); // Qt method

//	bool ExportSTL(const char* filename);
	bool ExportSTL_binary(const char* filename, bool isBinary = true);
	
	// for progress bar
	ProgressWidget* progresswidget;
	ProgressWidget* progresswidgetExport;
private:
	//vector<JigBlock*> m_blocks;
	//vector<JigFace*> m_connectors;
	
	// new data structure
	vector<MeshBlock*> mblocks; 
	vector<JigBlock*> mconnectors;
	vector<int> i_selected;
	double resol;
};

// some public useful functions:
mpVector LinearInterp(mp4Vector p1, mp4Vector p2, float value);

