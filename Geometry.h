#pragma once

#define UPPER			0x0001
#define LOWER			0x0000

#define INCISOR			0x0001
#define LEFT			0x0002
#define RIGHT			0x0000
#define BRACKET_HEIGHT	3
#define BRACKET_SLOT	0.64


#include "../fkernel/DataStructure/FBody.h"
#include "FEntity.h"
#include "../fkernel/FKernelDefines.h"
#include"../external/lapackpp/lapackpp.h"
#include <string>
#include "ANN.h"
#include <cmath>
#include "../server3/Graph.cpp"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

//added by ZR
#include "../server3/fitting_3D_points_to_line.h"	//for PCA function. Added by ZR.
//#include "../server3/dijkstra.h"	//dijkstra. Added by ZR. Cannot include here. The djkstra.h need to include Geometry.h in order to use MyTraits. Besides, Geometry.cpp and mainwindow.cpp both include dijkstra.h.
#include <fstream>
#include <iomanip>

#include "io.h"
//#include "iterative_ransac.h"
class PatchType;
//added by ZR end

struct MyTraits : OpenMesh::DefaultTraits	//Set Point and Normal to be double precision.
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
	VertexTraits
	{
		bool patch;	//flag on Vertices. Initialized as 0. 1 for flood-fill painting.
		bool RANSAC;	//for iterative RANSAC process flag. 0 for non-processed. 1 for processed.
		int TopoNode_idx;	//The TopoNode index. -1 if not an TopoNode. For iterative RANSAC.
		int RANSAC_TYPE;	//RANSAC type. For final RANSAC result. 0 for initialization. 1 for Plane. 2 for Cylinder. 3 for Cone.
		//bool iso;	//Default=0. Whether the vertex is isotropic after PCA evaluation. If in PCA (max_error - min_error) / min_error < 1%, then this vertex is marked as an isotropic vertex (which is on a plane/sphere).
		//int Patch_idx;	//i-th patch after RANSAC. Count from 1. 0 for NULL. 
	};

};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  FMesh;	//Use customized traits.

//typedef OpenMesh::TriMesh_ArrayKernelT<>  FMesh;	//Use standard traits.


FMesh* Mesh2OMesh(ON_Mesh* p_mesh);

using namespace std;
using namespace FT;

class Teeth;
class Tooth;
class Mesh;
//added by ZR for architecture project
class Planar;
class TopoNode;	//nodes in the OpenMesh structure.
class IntersectCurve;	//intersection curve on TopoNode.
//added by ZR end

struct MeshTriangle
{
	int nodeindex[3];
	ON_3dVector fnormal;
};

class MeshFace : public FEntity // a FV mesh structure with an open boundary
{
public:
	MeshFace(){};
	~MeshFace(){};

	////////////////////////////////computing methods//////////////////////////////////////////
	//extract the bottom face of meshin w.r.t the negtive direction of refdir
	void ExtractBottom(ON_Mesh *meshin, ON_3dVector refdir);

	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
private:
	ON_3dPointArray nodelist;
	vector<MeshTriangle> facelist;
	vector<int> boundary;
	GLuint	tempList, m_uListSmooth, m_uListMesh;
};

class NodeTree3D
{
public:
	NodeTree3D(ON_Mesh *mesh_in);
	~NodeTree3D();
	//search the nearest point to the query point
	ON_3dPointArray ANNSearch(ON_3dPoint querypnt, int k);
private:
	int nodenum;
	ON_Mesh *m_pMesh;
	ANNpointArray datapts;
	ANNkd_tree *kdtree;
};
//Define and manipulate curves
class Curve : public FEntity
{
public:
	Curve(const ON_3dPointArray &pa);
	Curve();
	~Curve() {};
	/////////////////////////////// Accessing methods ///////////////////////////////////////////

	// Important Note: t should be strictly within the range [0,1].
	ON_3dPoint PointAt(double t);
	ON_3dVector TangentAt(double t);
	double CurvatureAt(double t);
	ON_3dVector NormalAt(double t);
	double GetParameter(int ti){ return m_T[ti]; };
	void SetCurveName(QString fname){ filename = fname; };
	QString GetCurveName(){ return filename; }
	void SetTeeth(Teeth *t);//this function should be called every time when teeth are attached to this curve
	Teeth* GetTeeth(){ return m_Teeth; }
	vector<ON_3dPoint> GetKnotPoints();
	ON_3dVector GetXaxis(){ return xaxis; }
	ON_3dVector GetYaxis(){ return yaxis; }
	ON_3dVector GetZaxis(){ return zaxis; }
	ON_3dVector GetOrigin(){ return origin; }
	double FindParameter(double par_in, double dis);  //find the corresponding parameter on the curve to the input parameter at the distance dis.
	/////////////////////////////// Manipulating methods //////////////////////////////////////
	void Rotate(double r_angle, char axis);
	void Rotate(double r_angle, ON_3dVector axis, ON_3dPoint p);
	void Translate(double x, double y, double z);
	void Translate(ON_3dVector v);
	void Align(ON_3dVector de_x, ON_3dVector de_z);
	void Reverse();
	bool ModifyCurve(int idx, ON_3dPoint mp);
	bool ModifyCurve(int idx, ON_2dPoint mp);  //used for manual adjustment of control knots in x-y plane
	bool IsCCL(ON_3dVector axis);	//check whether the curve is CCL w.r.p to given axis
	Curve & operator=(Curve & c);
	void ExpandCurve(double Xscale=1, double Yscale=1); //expand the curve to the desired scale.
	void MakePlanar(Teeth *teethin);		//find a suitable plane and project/update all knots (reference point) along each long axis onto that plane.
	/////////////////////////////// Display methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	InitDisplayList();
	virtual void	DrawPoints();
	virtual void	CalBoundBox();
	virtual void	DrawFlatShade();
	virtual void	DrawMesh();

public:
	QString filename;
	bool clipplane_on;
	double clipPos;	//t parameter on the curve for clipping plane.
	bool clipDir;	//true to view from the tangent direction.
	

//display parameters:
	int disp_option;  //0: no cross-section; 1: round cross-section; 2: square cross-section
	double width;	//for round cross-section, width=diameter
private:
//	ON_NurbsCurve m_pCurve;
	vector<ON_BezierCurve> m_Curve;		//cubic b-spline
	vector<double> m_T; //parameters for control knots, from 0 to 1
	ON_3dVector xaxis;
	ON_3dVector yaxis;
	ON_3dVector zaxis;
	ON_3dPoint origin;
	Teeth* m_Teeth;
	GLuint	m_uListSmooth;
};

//Define and manipulate bracket
class Bracket : public FEntity
{
public:
	Bracket();
	~Bracket();

	/////////////////////////////// Accessing methods ///////////////////////////////////////////
	void	SetMesh(ON_Mesh* pMesh){ m_pMesh = pMesh; }
	void	SetMesh(FEntity* in);
	ON_Mesh*	GetMesh(){ return m_pMesh; }
	void	SetArchWirePar(double par){ archwire_par = par; };
	double  GetArchWirePar(){ return archwire_par; };
	void	SetArchWireParIni(double par){ archwire_par_initial = par; };
	double	GetArchWireParIni(){ return archwire_par_initial; };
	ON_Xform	GetXform(){ return m_xForm; };

	/////////////////////////////// Manipulating methods ///////////////////////////////////////////
	void Transform(ON_Xform &matr);
	void Translate(ON_3dPoint des);	//translate the origin to des.
	void Translate(double x, double y, double z);
	void Align(ON_3dVector de_x, ON_3dVector de_z); //align to the designated frame of the same origin.
	void Rotate(double r_angle, ON_3dVector axis, ON_3dPoint origin);
	/////////////////////////////// Public methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawSmoothShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
	void			InitCustomDisplayList();  //displaylist for customized bottom
	virtual void	CalBoundBox();
	virtual void	AddDisplayControl();
	void			CalAngle(ON_3dVector longaxis);	//for LCS display

	/////////////////////////////// Computing methods ///////////////////////////////////////////
	// calculate the local coordinate system by detecting slot features, the origin will be
	// the center of the bottom surface of the slots, xaxis will be the surface normal, yaxis
	// will be the slot direction, zaxis is the cross product.
	bool	CalLCS(double featurelength, int numofslot);
	void	CalNormals();

	/////////////////////////////// Customization methods ///////////////////////////////////////////
	FMesh* ExtractBracketBottom();  //the bottom face will be stored in bottomface;
	void FaceProjection(FMesh *meshin); //project the enlarged bottom face to tooth;
	void Customize();  //customize the bracket base
	bool isCustomized(){ return is_customized; };
	bool ExportCustomizedBracket(const char* filename, bool isBinary /*= true*/);

	bool RayTriangleIntersect(const ON_3dPoint &orig, const ON_3dVector &dir, // ray
		const ON_3dPoint &v0, const ON_3dPoint &v1, const ON_3dPoint &v2, // triangle
		float &t, // intersection point P = orig + t*dir
		float &u, // weight of v0
		float &v); // weight of v1

	void AddOMesh(){ m_oMesh = Mesh2OMesh(m_pMesh); }

public:
	bool localCS_on;
	ON_3dVector xaxis;
	ON_3dVector yaxis;
	ON_3dVector zaxis;
	ON_3dPoint origin;
	Tooth *m_tooth;
	//bool isModified = false; // if its archwire par is changed, marked as true

protected:
	ON_Mesh*		m_pMesh;
	
private:
	GLuint			tempList, m_uListSmooth, m_uListMesh, m_CListSmooth;
	vector<int> candidate_edges;
	ON_3dVector		rotAngle;	//for display
	double	archwire_par;		//record the bracket position on the archwire. initial value is 0;
	double	archwire_par_initial; // record the bracket position on the archwire after initialization.
	
	//for customize bracket
	bool is_customized; //whether the customization has been done, if yes, displaylist needs to update.
	bool is_Cdisplist;  //whether the display list for customized bottom has been added.
	FMesh *topface, *bottomface, *sideface;
	//FMesh *CustmizedBracket;
	FMesh*			m_oMesh;
	
};

//Define and manipulate tooth
class Tooth : public FEntity
{
public:  //functions
	Tooth();
	~Tooth();
	/////////////////////////////// Computing methods /////////////////////////////////////////
	void CalLongAxis(ON_3dVector ref);
	void CalCentroid();	//calculate centroid point
	void CalRefPnt(int upper_or_lower);	//refpnt is defined as the highest point for lower teeth (or lowest point for upper teeth) on the longaxis projected from the mesh vertices
	ON_3dPoint GetBottomPnt(int upper_or_lower); // get the lowest position along long axis, top position for lower teeth
	void CalNormals();		//calculate facet and vertex normal value if m_FN and m_N is empty
//	void CalBCylinder();
	void CalVoxelMap(int density=50);		//calculate the data structure for collision detection
	void CalSamplePnts(double angle, ON_3dPoint refp);
	bool IsPointInside(ON_3dPoint p);
	ON_3dPoint NearestPoint(ON_3dPoint q);  //find q's nearest point on tooth.
	double Distance(Tooth *t2); //compute the shortest distance to another tooth
	double OcclusalDistance(ON_3dPoint p, double thresh); //calculate the occlusal distance of a given point, if distance>=thresh, return thresh


	/////////////////////////////// Manipulating methods ////////////////////////////////////////
	void Transform(ON_Xform &matr, bool bracketmove=false);  //transform the tooth by a matrix
	void Rotate(double r_angle, char axis, bool bracketmove = false); //rotate about refpnt, moving frame
	void Rotate(double r_angle, ON_3dVector axis, ON_3dPoint origin, bool bracketmove = false);
	void Translate(double dx, double dy, double dz, bool bracketmove = false); //translation
	void Translate(ON_3dVector delta, bool bracketmove = false);//translation
	void Translate(double dis, char axis, bool bracketmove = false);
	void Align(ON_3dVector de_x, ON_3dVector de_z); //align to the designated frame of the same origin.
	void AlignToCurve(Curve &incurve, double par); //align tooth to the given curve at the specified parameter, tooth angulation will be preserved.
	void Restore(); //restore to its original position.

	void SetViewPar(double p){ view_par = p; };
	void ResetViewPar(){ view_par = 1; };
	ON_Xform InterpTransform(double par);  //interpolate the rigid body transformation from 0 (initial position) to 1 (current position), return a 4*4 matrix
	void SetStaging(double par);  //set the tooth back to a previous stage, par from 0 to 1, the display position will not be changed.
	void ResetStaging(); //reset the tooth to the final position.
	void GetStagingRange(double &mvmt, double &inc, double &torq); //calculate the movement range of the tooth from begining to the final stage, mvmt refers to translation, inc refers to inclination, torq refers to torque. mm/deg
	void CalStagingStep(double mvmt = 0.2, double ang = 1, double rot = 3); //calculate the maximum staging step under certain requirement.

	/////////////////////////////// Accessing methods ///////////////////////////////////////////
	void			SetMesh(ON_Mesh* pMesh){ m_pMesh = pMesh; }
	void			SetMesh(FEntity* in);
	void			SetTree();
	NodeTree3D*		GetTree(){ return m_tree; };
	ON_Mesh*		GetMesh(){ return m_pMesh; }
	void			SetbMeshed(bool bval){ m_bMeshed = bval; }
	bool			IsMeshed(){ return m_bMeshed; }
	void			SetType(int t){ type = t; }
	void			SetType(int t, int p){ T_type = t, T_pos = p; };
	void			AddBracket(FEntity* in);
	void			RemoveBracket();
	Bracket*		GetBracket(){ return m_bracket; }
	ON_Xform		GetXform(){ return m_xForm; };
	void			SetPairTooth(Tooth *t){ paired_tooth = t; };
	Tooth*			GetPairTooth(){ return paired_tooth; };

	/////////////////////////////// Public methods ///////////////////////////////////////////
	void			Update();
	virtual void	DrawPoints();
	virtual void	DrawSmoothShade();
	virtual void	DrawFlatShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
	virtual void	CalBoundBox();
	virtual void	AddDisplayControl();
	virtual void	DrawOriginal();
	void			InitOcclusalDisp();
	void	CalAngle(ON_3dVector a);
	void	SetLongAxis(ON_3dVector l){ longaxis = l; };
	void	HideBracket();
	void	ShowBracket();
	void	HideTooth();   //hide this tooth for display, it's still there.
	void	ShowTooth(bool bracket_display = true);  //re-display this tooth.
//  void	DrawBCylinder();
//  void	SetOriPos(ON_3dPoint a){ oriPos = a; };	//store the original position when the tooth is just loaded.
//  void	SetOriAxi(ON_3dVector a){ oriLongAxis = a; };
//  void	SetOriFrame(ON_3dVector xax, ON_3dVector yax, ON_3dVector zax);
//	void	IsInitial(bool a){ isInitial = a; };	//Initialize the trigger for display
//	void	SetRotAngle(double ang, char axis);	//Set rot angle for tooth display manipulation
//	void	SetRotMatrix(double angle, ON_3dVector vector, ON_3dVector point);	//angle in radian value

	///////////////////////Added by ZR for Architecture project/////////////////////////
	void	CalRefVector(ON_3dVector vn, ON_3dVector axi);	//Calculate a reference vector for the mesh. vn is the normal of a given vertex, axi is the given axis (e.g., Z-axi in the WCS).
	void	CalRefVector();
	void	AddOMesh(){ m_oMesh = Mesh2OMesh(m_pMesh); };
	FMesh*	GetOMesh(){ return m_oMesh; };
	void	AlignBestFitLine();	//Globally align the best fit intersection lines (single segment). Use dot product to check the orientation of adjacent lines. If <0 then flip the next one.
	void	AlignBestFitLineSeg();	//Align the best fit intersetction lines (multiple segment).
	int		Roll(int ni, int roll_type);	//ni is node index. Use flood-fill to compute how many nodes on the mesh has been rolled on a given TopoNode. Type 0: include all triangle points. Type 1: ignore triangle points which is on the right hand of intersection lines (caution of precision error).
	std::vector<PatchType>	Patch;	//RANSAC results
	int		FixPlanarRegion();	//USE THIS BEFORE RANSAC! Set the generatrix line of planar nodes in alignment with its closest non-planar node.


	///////////////////////Added by ZR end/////////////////////////////
	

public:  //functional members
	string filename;
	ON_3dPoint refpnt;		//reference point, also serves as the origin for local coordinate system
	ON_3dPoint centroid;
	ON_3dVector longaxis;	//z axis in local coordinate system
	ON_3dVector xaxis;		//x axis in local coordinate system
	ON_3dVector yaxis;		//y axis in local coordinate system
//	ON_Cylinder bcylinder;	//bounding cylinder centered at longaxis
	Bracket *m_bracket;     //user imported bracket
	vector<int>		sample_pnts;	//sample some sparsely distributed points on the mesh to check collision.
	double	arch_par;  //tooth alignment parameter on arch curve.
	double  staging_step;  //rule based maximum allowed staging step (0-1). use CalStagingStep to acquire.
	bool locked; //if the tooth is locked, it won't move and its refpnt on curve is also fixed.

	////////added by ZR for Architecture project//////
	ON_3dVector refvec;
	vector<TopoNode> tpNode;	//store the list of topo node for display and intersection calculation.
	bool tpNodeDisplay;	//trigger for toponode display
	bool is_rolled;
	////////added by ZR end//////

	// display trigger
	bool refpnt_on;		//trigger for display refpnt
	bool longaxis_on;	//trigger for display longaxis	
//	bool bcylinder_on;	//trigger for bcylinder display
	bool localCS_on;	//trigger for local coordinate system display
	bool occlusal_on;	//display occlusal diagram;
	bool m_bocclusal;	//initial occlusal display list.

	//tooth type
	int type;
	int T_type; //1-7, cent, lat, can, pre1, pre2, mol1, mol2.
	int T_pos;  //upperleft: 11; upperright:01; lowerleft:10; lowerright:00

	// color bar
	double maxOccluDist;
	double minOccluDist;


protected:	//data structure
	ON_Mesh*		m_pMesh;			///mesh representation
	bool			m_bMeshed;			///if false, m_pMesh needs to be updated
private:
	GLuint			tempList, m_uListFlat, m_uListSmooth, m_uListMesh, m_uListPoints, m_occlusalSmooth;
	ON_3dVector		rotAngle;	//for cylinder display
	//ON_3dPoint		oriPos;	//for tooth translation. Original position when the tooth is loaded in.
	//ON_3dVector		oriLongAxis;
	//ON_3dVector		oriYAxis;
	//ON_3dVector		oriXAxis;
	double			view_par;	//0-1, 0 represents original position, 1 represents current position.
	NodeTree3D*		m_tree;	//kd-tree structure
	ON_Xform		m_xForm_f;  //this xform stores the final tooth position info when m_xForm changes back to a previous stage.

	//voxel map data structure, this data structure will keep static regardless of the transformation.
	int voxel_density;
	LaGenMatDouble voxel_top;	
	LaGenMatDouble voxel_bot;
	double xlim[2];
	double ylim[2];
	ON_3dVector xdir;
	ON_3dVector ydir; //xdir and ydir are vectors along xlim and ylim
	ON_3dVector zdir; //zdir is the initialized longaxis

	//occlusal map
	Tooth *paired_tooth;

	//added by ZR for architecture project
	FMesh*	m_oMesh;
	//added by ZR end
};


//Define and manipulate teeth
class Teeth
{
public:
	Teeth();
	~Teeth();
	const vector<Tooth*>& GetTeethlist(){ return teethlist; };
	void AddTooth(FEntity*, string, int);
	bool RemoveTooth(int ti);		//ti starts from 0, return false when ti is larger than teethlist.size;
	
	void DisplayAppliance(bool bracket_display, bool archwire_display);
	void HideTeeth();
	void ShowTeeth(bool bracket_display, bool archwire_display);
	bool Highlight(int ti);  //highlight display, trigger=0x0001: highlight, trigger=0x0002: refpnt on, trigger=0x0004: longaxis on
	void ClearHighlight();
	void UpdateAllViews(bool refcur_on = false);
	void OcclusalDisplayOn();
	void OcclusalDisplayOff();
	void OcclusalDisplayUpdate(int toothi); // update occlusal display when tooth position is adjusted.
	
	Tooth* GetTooth(int tooth_index);
	Curve* GetCurve() { return refcurve; };
	Curve* GetArchwire(){ return archwire; };
	int Size(){ return teethlist.size(); };
	bool IsCCL();			//check whether the teeth are arranged counter-clockwise

	bool RankUp(int ti);		//rank up the selected tooth in the teethlist, false if ti is 1 or out of range.
	bool RankDown(int ti);		//rank down the selected tooth, false if ti is out of range..

	void GetIncisor(int &incisor_l, int &incisor_r);

	/////////////////////////////// Computing methods /////////////////////////////////////////

	//check whether two objects collide with each other, t1 and t2 are two tooth index for collision 
	//detection, return true if collision happens. this function will automatically check the rough
	//distance of the two teeth, if close enough: use mesh-mesh detection, 
	//otherwise, use bcylinder-bcylinder detection
	bool CheckCollision(int t1, int t2, double gap = 0);

	double CheckDistance(int t1, int t2);

	//check collision between tooth and its corresponding bracket
	bool CheckCollision(int ti);

	//sort the teeth in CCL sequence.
	void BubbleSort();
	//calculate reference curve, if return false, the memory has already been allocated,
	//no need to add into display entity.
	bool CalRefCurve();

	//calculate tooth coordinate system individually. user should notice that CalRefCurve() should
	//be called prior to calling this function.
	bool CalToothCS();

	//attach all teeth to a given curve, the orientation of each tooth will not be changed, make
	//sure that the teeth is arranged counter-clockwise, otherwise false will return.
	//the index of both incisors should be as-well input, default values are given.
	//the output_par will store the calculated parameters on the curve for each tooth refpnt.
	//error code: -1£º not CCL fashion
	//			  -2:  incisor index exceeds maximum number of teeth
	int AttachToCurve(Curve &incurve, vector<double> &output_par);

	//in addition to attach to curve, this function also adjust tooth orientation by Andrew's 6 keys
	int AttachToCurve_Andrew(Curve &incurve, vector<double> &output_par);

	//align all teeth to the modified refcurve, preserve the angulations, return the remaining length (+/-) of the curve
	double AlignToRefCurve();
	double AlignToRefCurve2(double gap = 0);
	void   UpdateTeethAlign(int toothi); //update teeth alignment once any tooth is manually transformed, to avoid collision.
	void   AlignTeeth(int t1, int t2, bool is_t1fixed = false, bool is_t2fixed = false, double gap = 0);

	void IPR(double value);  //do IPR for the front 8 teeth to fit the curve, value must be positive.
	void CalStaging(bool collision = false); //calculate intermediate staging for aligner design.
	LaGenMatInt CalCollisionMat(int t1, int t2); //calculate the 01 collision matrix of two adjacent teeth.

	/////////////////////////////// Manipulating methods /////////////////////////////////////////
	void Translate(double dx, double dy, double dz);
	void Translate(ON_3dVector v);
	void Rotate(double rot_angle, ON_3dVector axis, ON_3dPoint origin);
	void InterConfigView(double par); //view teeth configuration in between initial and final configuration, par=0-1

	//Bracket manipulation
	double AddBracket(int toothi, FEntity* in);	 //return the parameter of archwire that the bracket is on
	bool AlignBracketToTooth(int toothi, double height = BRACKET_HEIGHT);
	bool AlignBracketOnWire(int toothi, double &par_i); //place bracket on archwire and align to each tooth.
	bool MoveBracketOnWire(int toothi, double par); //manually adjust the bracket positioning along archwire.
	void FlipBracket(int toothi);
	void UpdateBracket(int toothi); //update bracket after the chosen tooth is transformed.


	void RestoreTeeth(); //restore teeth alignment, if bracket is attached, it will be transformed accordingly.

	//Archwire manipulation
	bool SetArchWire(Curve *inwire, double width, bool is_square = true, double height = BRACKET_HEIGHT);
	void ResetArchWire(){ archwire = NULL; };
	bool UpdateArchWire(int toothi); //when a new bracket is added, update the position of the archwire together with
									 //all attached brackets so that no bracket collides with its tooth. return false if the archwire no longer fits.
	bool UpdateArchWire();		//if false, this archwire does not fit.

	void SetToothType(int leftcenter, int rightcenter);
	bool PairTooth(Teeth *pairteeth);  //pair two groups of teeth, return true if succeed
	bool OverBite(int leftorright, double &OB); //calculate overbite distance of center tooth.
	bool OverJet(int leftorright, double &OJ); //calculate overjet distance.
	bool Angulation(int leftorright, double &AG); //calculate center teeth angulation, AG is degree unit.
	void LockTooth(int toothi){ teethlist[toothi]->locked = true; };
	void UnlockTooth(int toothi){ teethlist[toothi]->locked = false; };
	//	void UpdateToothType(); //execute when some tooth is removed.
public:
	int upperorlower; //UPPER=1, LOWER=0
	ON_3dPoint origin; //fixed once assigned value
	bool is_typeset;

	LaGenMatDouble staging; //stores the stage parameter for each tooth, row index is tooth index, column index is stage index. (0-1)
	bool is_staging;

private:
	vector<Tooth*> teethlist;	
//	vector<NodeTree3D*> treelist; //use for memory release purpose only.
	Curve *refcurve;
	Curve *archwire;
	bool is_sorted;	
	bool is_hide; //trigger for teeth hiding
	ON_3dVector ref_axis; //teeth are sequenced CCL w.r.t. ref_axis 

};


// some public useful functions:
double TriangleArea(ON_2dPoint v1, ON_2dPoint v2, ON_2dPoint v3);
// Barycentrix interpolation, par[3] equals to the partition area; return false if p is outside triangle v1-v2-v3
bool TriangleInterp(ON_2dPoint v1, ON_2dPoint v2, ON_2dPoint v3, ON_2dPoint p, double *par);

vector<int> MeshSampling(ON_Mesh *mesh_in, ON_3dPoint ref, double angle, ON_3dVector refz);  //return the vertex index in mesh, i.e. mesh_in->Vertex(i)

//return the vertex index of the input mesh that is closest to the input ray
int RayMeshIntersection(ON_3dPoint ray_ori, ON_3dVector ray_dir, const ON_Mesh* mesh_in);


/*! \brief Convert RGB to HSV color space

Converts a given set of RGB values `r', `g', `b' into HSV
coordinates. The input RGB values are in the range [0, 1], and the
output HSV values are in the ranges h = [0, 360], and s, v = [0,
1], respectively.

\param fR Red component, used as input, range: [0, 1]
\param fG Green component, used as input, range: [0, 1]
\param fB Blue component, used as input, range: [0, 1]
\param fH Hue component, used as output, range: [0, 360]
\param fS Hue component, used as output, range: [0, 1]
\param fV Hue component, used as output, range: [0, 1]

*/
void RGBtoHSV(float& fR, float& fG, float fB, float& fH, float& fS, float& fV);

/*! \brief Convert HSV to RGB color space

Converts a given set of HSV values `h', `s', `v' into RGB
coordinates. The output RGB values are in the range [0, 1], and
the input HSV values are in the ranges h = [0, 360], and s, v =
[0, 1], respectively.

\param fR Red component, used as output, range: [0, 1]
\param fG Green component, used as output, range: [0, 1]
\param fB Blue component, used as output, range: [0, 1]
\param fH Hue component, used as input, range: [0, 360]
\param fS Hue component, used as input, range: [0, 1]
\param fV Hue component, used as input, range: [0, 1]

*/
void HSVtoRGB(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV);

// find a feasible path conencting the starting point and end point of a given 01 matrix;
// return false if no path exists;
// define row_out and col_out prior to the calling of this function;
// the size of row_out and col_out should be equal.
bool FindPath(LaGenMatInt mat_in, vector<int> &row_out, vector<int> &col_out);

//merge two incremental vectors into a longer one, and return the index vector w.r.t v1.
vector<int> MergeVector(vector<int> v1, vector<int> v2);

//expand the input vector according to the input index vector
void ExpandVector(vector<int> &vin, vector<int> vindex);

// convert ON_Mesh to OpenMesh
FMesh* Mesh2OMesh(ON_Mesh* p_mesh);

// export STL main part, do not forget to add header for binary STL
bool ExportOMesh(FILE* fout, FMesh* _mesh, bool isBinary = true);

//std::vector<FMesh::VertexHandle> ExtractBnd(FMesh *meshin); //extract the ordered boundary from the given mesh data
//FMesh* FlipSurface(FMesh *meshin);
std::vector<FMesh::VertexHandle> ExtractBnd(FMesh *meshin); //extract the ordered boundary from the given mesh data

void MeshProjection(FMesh* meshin, FMesh* des, ON_3dVector dir, ON_3dVector xaxis, int res); //project meshin to des along dir

FMesh* DeepCopy(FMesh* meshin, bool flipnormal = false); //copy the mesh


//added by ZR for architecture project
class Planar : public FEntity
{
public:
	Planar(){};
	Planar(ON_3dVector p, ON_3dVector nor){ pOri = p; pNor = nor; };
	~Planar(){};
	void SetOrigin(ON_3dVector p){ pOri = p; };
	void SetNormal(ON_3dVector nor){ pNor = nor; };
	ON_3dVector GetOrigin(){ return pOri; };
	ON_3dVector GetNormal(){ return pNor; };
	double Distance(ON_3dVector p);	//calculate the distance of a given vector to the plane. Negtive value means the point is below the plane (defined by normal);

private:
	ON_3dVector pOri;
	ON_3dVector pNor;

};

class TopoNode
{
public:	
	TopoNode(){ huntting_done = false; rollover = NULL; is_trimmed = false; primType = 0; patch_idx = 0; };
	TopoNode(ON_3dPoint p, ON_3dVector n){ point = p; normal = n; huntting_done = false; };
	~TopoNode(){};
	
	//////Functions//////
	//ON_3dPointArray* GetIntersectCurve(int i){ return (intersectCurve[i]->curve); };
	void GeneratrixHunt();	//Need to perform intersection beforehand! Brutal hunting for the best fit line. Need to perform intersection on all directions first.
	void GeneratrixHunt(int fine_search_int_num);	//Hunt for the best_fit_lines after the refine_search;
	int GeneratrixHuntGolden(double t_interval, double t_error, ON_3dVector nodeRefVec, FMesh* mesh, FMesh::VertexIter p0_v_it);	//Including intersection! Golden ratio hunt for minimum error fitting line. (Terminate condition: (angel, error), Starting vector, vertex iter in the openmesh.)
	double IntersectAndEva(Planar* int_plane, FMesh* omesh, FMesh::VertexIter p0_v_it);	//intersect. Return fitting error.
	void Roll(FMesh* omesh);	//Evaluate how many vertices has been roll over up to this node. Use plane to evaluate whether the points is above or below the plane. Not suitable for curvey surface which intersects with a plane for more than ONE time.
	void TrimSeg(double tol_seg);	//trim and align the intersection segments (NEED TO FIX the normal and adjust the segment direction to form a right-hand CS!). delete the one which is on the other side of panel. The criteria is distance = (dist_a + dist_b)/2, where the dist_a and dist_b is the distance from end points of other segments to the reference segments.
	void GeneratrixHuntSeg(double RANSAC_plane_tol, bool planar_filter);	//modified based on GeneratrixHunt(). PCA evaluate all segments. Use tolerance which is defined by PLANE_TOL to detect planar/isotropic nodes. If number_of_direction(whose error is smaller than tol) / total_intersect_number > 30/180, then this node is marked as isotropic node by setting primType = 1.
	void SortSeg(int cn);	//Sort the segments on the direction specified by cn (curve number) according to PCA. (ALSO align the PCA according to plane normal and node normal.


	//////Intersection Members//////	
	vector<Curve*> interCurve;	//bezier curve list. Each curve is a list of ON_3dPoint. There should be a member in "Curve" that denotes the Least Square Fitting line and error.
	vector<IntersectCurve*> intersectCurve;	//basic intersection curve.

	//////Geometric Members//////
	ON_3dPoint point;	//node coordinate
	ON_3dVector normal;	//node normal
	ON_3dVector plane_nor;	//normal of intersection plane.
	ON_3dVector best_plane_nor;	//normal of best fitting intersection plane.
	ON_3dVector best_fitted_line_points[2];	//end points of best fitting lines. [start point, end point]. Obtained in the Tooth::AlignBestFitLine function.
	int ind;	//index in the OpenMesh structure
	FMesh::VertexIter node_it;	//iterator in the OpenMesh structure
	double interval;	//intersection interval. In Radiant.
	int primType;	//mark the node which primitive types it belongs according to RANSAC results. 0 for NULL. 1 for Plane, 2 for Cylinder, 3 for Cone.
	int patch_idx;	//belong to which patch. 0 for NULL as initilization. Count from 1.
	int best_fit_line_index;	//index of the best fitting straight line in the "intersectCurve".
	double best_fit_error;	//error of the best fit line.
	bool huntting_done;	//It's possible there is no proper fitting after hunting.
	bool is_trimmed;	//whether the segments are trimmed, which means the segments that are far apart from the reference segments are deleted.

	int rollover;	//Number of vertices that have been roll over up to this node.
	vector<FMesh::VertexHandle> roll_vh;	//openmesh vertex handler of rolled-over node
	ON_3dPointArray roll_node;	//rolled nodes.
	vector<FMesh::HalfedgeHandle> roll_heh;	//for flood-fill display check

	//ON_3dPoint next_node[4];	//next node and next next node in a neighboring facet according to OpenMesh structure. Up to 4 nodes (forming two triangular facets). 0&1 for a facet, 2&3 for the other facet.
	vector<ON_3dPoint> next_node;	//for display test only

};

class IntersectCurve
{
public:
	IntersectCurve(){ is_intersect = false; is_intersect_through = false; is_PCA_done = false; };
	IntersectCurve(ON_3dPointArray c){ curve = c; };
	~IntersectCurve(){};

	//void SetUpCurve(ON_3dPointArray c){ curve = c; };	//oddly cannot use "SetCurve" as name.

	ON_3dPointArray curve;
	vector<ON_3dPointArray> curves_seg_raw;	//raw data of multiple intersection curves segments along one direction. Intersect including void/concave surfaces.
	vector<ON_3dPointArray> curves_seg_trim;	//trimmed multiple intersection curves segments along one direction. Intersect including void/concave surfaces.
	//vector<vector<FMesh::HalfedgeHandle>> curves_seg_he;	//The half edge handle of each intersection points.
	vector<vector<FMesh::FaceHandle>> curves_seg_fh_raw;	//The face handle of each intersection triangles. The number is -1 than the intersection points, because the last one is on the boundary halfedge which don't have triangles!
	vector<vector<FMesh::FaceHandle>> curves_seg_fh_trim;	//after trimmed.

	Planar curveplane;	//the plane where this curve is on.
	float error;	//error in PCA fitting.
	bool is_intersect, is_intersect_through, is_PCA_done;	//"through" for multiple segment intersection
	ON_3dPoint fitted_line[2];	//PCA fitting line. [start point, end point]
	//Geometric Members
	//function that evaluates straightness



};


int EdgeIntersectPlane(ON_3dVector pa, ON_3dVector pb, Planar* p, ON_3dPoint& intpt);	//pa, pb are the end points of an edge. p is the plane. Output the intersection point to intpt.
int PlaneIntersectMesh(FMesh* mesh, FMesh::VertexIter p0_it, Planar* p, ON_3dPointArray& intcurve);	//only one intersection segment. Stop the intersection upon reaching boundary.
int PlaneIntersectMeshThrough(FMesh* mesh, FMesh::VertexIter p0_it, Planar* p, vector<ON_3dPointArray>& intcurve_seg, vector<vector<FMesh::FaceHandle>>& curves_seg_fh);	//multiple intersection segments. Intersect through the whole surface, cross voids/concave boundaries.


//added by ZR end.