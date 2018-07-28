#include "Geometry.h"
#include <GL/gl.h>
#include <GL/GLU.h>
#include <glut.h>

#define GL_GLEXT_LEGACY
#define GL_GLEXT_PROTOTYPES
#include <glext.h>
#include <qopenglext.h>
#include <QtOpenGL>
#include "mesh.h"

#include "../server3/dijkstra.h"	//added by ZR for architecture project.
#include "../server3/iterative_ransac.h"	//added by ZR for architecture project.

Teeth::Teeth()
{
	teethlist.clear();
	refcurve = NULL;
	archwire = NULL;
	is_sorted = false;
	is_typeset = false;
	is_hide = false;
	ref_axis = ON_3dVector(0, 0, 1);
	origin = ON_3dPoint(0, 0, 0);
	is_staging = false;
	//default index
}

Teeth::~Teeth()
{
	//for (int i = 0; i < treelist.size(); i++)
	//{
	//	delete treelist[i];
	//}
	annClose();
	//int n = teethlist.size();
	//for (int i = 0; i < n;i++)
	//{
	//	delete teethlist[i];
	//}
	//if (refcurve_init != NULL)
	//	delete refcurve_init;
	//if (refcurve != NULL)
	//	delete refcurve;
}

void Teeth::AddTooth(FEntity* in, string name, int upperorlower)
{
	int n = teethlist.size();
	Tooth *t = new Tooth;
	t->SetMesh(in);
	t->CalNormals();
	t->SetTree();
	teethlist.push_back(t);
//	treelist.push_back(t->GetTree());
	t->filename = name;
	t->CalLongAxis(ref_axis);
	t->CalRefPnt(upperorlower);
	t->CalVoxelMap(30);
	/*ON_3dPoint refp;
	if (upperorlower == UPPER)
	refp = t->refpnt + t->longaxis * 4;
	else
	refp = t->refpnt - t->longaxis * 4;*/
	t->CalSamplePnts(PI / 10, t->centroid);
}

bool Teeth::RemoveTooth(int ti)
{
	if (ti >= teethlist.size())
		return false;
	else
	{
		teethlist.erase(teethlist.begin() + ti);
		/*CalRefCurve();
		refcurve->Update();*/
//		AlignToRefCurve2();
//		CalRefCurve();
		UpdateAllViews(true);
		return true;
	}
}

void Teeth::DisplayAppliance(bool bracket_display, bool archwire_display)
{
	if (is_hide)
		return;
	if (bracket_display)
	{
		int n = teethlist.size();
		for (int i = 0; i < n; i++)
			teethlist[i]->ShowBracket();
	}
	else
	{
		int n = teethlist.size();
		for (int i = 0; i < n; i++)
			teethlist[i]->HideBracket();
	}
	if (archwire != NULL)
	{
		if (archwire_display)
			archwire->m_isHide = false;
		else
			archwire->m_isHide = true;
	}
}

void Teeth::HideTeeth()
{
	int n = teethlist.size();
	for (int i = 0; i < n; i++)
		teethlist[i]->HideTooth();
	DisplayAppliance(false,false);
	is_hide = true;
}

void Teeth::ShowTeeth(bool bracket_display, bool archwire_display)
{
	is_hide = false;
	int n = teethlist.size();
	for (int i = 0; i < n; i++)
		teethlist[i]->ShowTooth(bracket_display);
	DisplayAppliance(bracket_display, archwire_display);
}

bool Teeth::Highlight(int ti)
{
	if (ti >= teethlist.size())
		return false;
	else
	{
		for (int i = 0; i < teethlist.size(); i++)
			teethlist[i]->highlight_on = false;	
	}
	teethlist[ti]->highlight_on = true;
	return true;
}

void Teeth::ClearHighlight()
{
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->refpnt_on = false;
		teethlist[i]->longaxis_on = false;
		teethlist[i]->highlight_on = false;
		teethlist[i]->localCS_on = false;
	}
}

Tooth* Teeth::GetTooth(int tooth_index)
{
	int n = teethlist.size();
	if (tooth_index < n)
		return teethlist[tooth_index];
	else
		return NULL;
}

bool Teeth::IsCCL()
{
	if (is_sorted)
		return true;
	int n = teethlist.size();
	if (n <= 1) return false;
	ON_3dPoint center(0, 0, 0);
	for (int i = 0; i < n; i++)
	{
		center = center + teethlist[i]->centroid;
	}
	center = center / n;
	for (int i = 0; i < n - 1; i++)
	{
		double d = ON_CrossProduct(teethlist[i]->centroid - center, teethlist[i + 1]->centroid - center)*teethlist[i + 1]->longaxis;
		if (d < 0)
			return false;
	}
	is_sorted = true;
	return true;
}

bool Teeth::RankUp(int ti)
{
	int n = teethlist.size();
	if (ti == 0 || ti >= n)
		return false;
	else
	{
		Tooth * temp= teethlist[ti];
		teethlist[ti] = teethlist[ti - 1];
		teethlist[ti - 1] = temp;
		is_sorted = false;
		return true;
	}
}

bool Teeth::RankDown(int ti)
{
	int n = teethlist.size();
	if (ti >= n-1)
		return false;
	else
	{
		Tooth * temp = teethlist[ti];
		teethlist[ti] = teethlist[ti + 1];
		teethlist[ti + 1] = temp;
		is_sorted = false;
		return true;
	}
}

void Teeth::GetIncisor(int &incisor_l, int &incisor_r)
{
	incisor_l = 0;
	incisor_r = 0;
	int n = teethlist.size();
	for (int i = 0; i < n; i++)
	{
		if (teethlist[i]->T_type == 1)
		{
			if (teethlist[i]->T_pos & LEFT)
				incisor_l = i;
			else
				incisor_r = i;
		}
	}
}

bool Teeth::CheckCollision(int t1, int t2, double gap)
{
	int n = teethlist.size();
	if (t1 >= n || t2 >= n)
		return false;
	Tooth *tooth1 = teethlist[t1], *tooth2 = teethlist[t2];
	// is t1 collide with t2
	int np = tooth1->sample_pnts.size();
	for (int i = 0; i < np; i++)
	{
		ON_3dPoint p = tooth1->GetMesh()->Vertex(tooth1->sample_pnts[i]);
		ON_3dPoint p2 = tooth2->NearestPoint(p);
		double d = (p2 - p).Length();
		if (tooth2->IsPointInside(p))
			d=-d;
		if (d < gap)
			return true;
	}

	// is t2 collide with t1
	np = tooth2->sample_pnts.size();
	for (int i = 0; i < np; i++)
	{
		ON_3dPoint p = tooth2->GetMesh()->Vertex(tooth2->sample_pnts[i]);
		ON_3dPoint p1 = tooth1->NearestPoint(p);
		double d = (p1 - p).Length();
		if (tooth1->IsPointInside(p))
			d = -d;
		if (d < gap)
			return true;
	}
	return false;

	//ON_3dPoint q = (tooth1->centroid + tooth1->refpnt) / 2, p;
	//double dis;
	//for (int i = 0; i < 5; i++)
	//{
	//	p = tooth2->NearestPoint(q);
	//	q = p;
	//	p = tooth1->NearestPoint(q);
	//	dis = (p - q).Length();
	//	q = p;
	//}
	//if (dis < 0.5)
	//	return true;
	//return false;
}

bool Teeth::CheckCollision(int ti)
{
	if (ti >= teethlist.size())
		return false;
	Tooth *mtooth = teethlist[ti];
	if (mtooth->m_bracket == NULL)
		return false;
	int nv = mtooth->m_bracket->GetMesh()->VertexCount();
	for (int i = 0; i < nv; i++)
	{
		ON_3dPoint p = mtooth->m_bracket->GetMesh()->Vertex(i);
		if (mtooth->IsPointInside(p))
			return true;
	}
	return false;
}

double Teeth::CheckDistance(int t1, int t2)
{
	double d1 = teethlist[t1]->Distance(teethlist[t2]);
	double d2 = teethlist[t2]->Distance(teethlist[t1]);
	return min(d1, d2);
}

void Teeth::BubbleSort()
{
	if (is_sorted)
		return;
	int n = teethlist.size();
	if (n <= 1) return;
	ON_3dPoint center(0, 0, 0);
	for (int i = 0; i < n; i++)
	{
		center = center + teethlist[i]->centroid;
	}
	center = center / n;
	//PCA
	LaGenMatDouble M = LaGenMatDouble::zeros(3, 3), VR(3, 3);
	LaGenMatDouble pm(3, 1), pm_t(1, 3);
	for (int i = 0; i < n; i++)
	{
		ON_3dVector p = teethlist[i]->centroid - center;
		pm(0, 0) = p.x;
		pm(1, 0) = p.y;
		pm(2, 0) = p.z;
		pm_t(0, 0) = p.x;
		pm_t(0, 1) = p.y;
		pm_t(0, 2) = p.z;
		Blas_Mat_Mat_Mult(pm, pm_t, M, false, false, 1, 1);
	}
	LaVectorDouble eig_r(3), eig_i(3);
	LaEigSolve(M, eig_r, eig_i, VR);
	double e1 = eig_r(0);
	double e2 = eig_r(1);
	double e3 = eig_r(2);
	double smallest = abs(e1);
	int index = 0;
	if (abs(e2) < smallest)
	{
		smallest = abs(e2);
		index = 1;
	}
	if (abs(e3) < smallest)
	{
		smallest = abs(e3);
		index = 2;
	}
	ON_3dVector zzz(VR(0, index), VR(1, index), VR(2, index));
	if (zzz * ref_axis < 0)
		zzz = -zzz;
	//PCA
	ON_3dVector axis = teethlist[0]->centroid - center, v1, v2;
	axis = axis - zzz*(axis*zzz);
	axis.Unitize();
	for (int j = n - 1; j > 0; j--)
	{
		for (int i = 0; i < j; i++)
		{
			v1 = teethlist[i]->centroid - center, v2 = teethlist[i + 1]->centroid - center;
			v1 = v1 - zzz*(v1*zzz);
			v2 = v2 - zzz*(v2*zzz);
			v1.Unitize();
			v2.Unitize();
			double a1 = acos(v1*axis*0.999), a2=acos(v2*axis*0.999);
			if (ON_CrossProduct(axis, v1)*zzz < 0)
				a1 = -a1;
			if (ON_CrossProduct(axis, v2)*zzz < 0)
				a2 = -a2;
			if (a2 < a1)
			{
				Tooth* temp = teethlist[i];
				teethlist[i] = teethlist[i + 1];
				teethlist[i + 1] = temp;
			}
		}
	}
	v1 = teethlist[0]->centroid - center;
	v2 = teethlist[n - 1]->centroid - center;
	v1 = v1 - zzz*(v1*zzz);
	v2 = v2 - zzz*(v2*zzz);
	v1.Unitize();
	v2.Unitize();
	double amax = acos(v1*v2);
	int starti = 0;
	for (int i = 0; i < n - 1; i++)
	{
		v1 = teethlist[i]->centroid - center;
		v2 = teethlist[i + 1]->centroid - center;
		v1 = v1 - zzz*(v1*zzz);
		v2 = v2 - zzz*(v2*zzz);
		v1.Unitize();
		v2.Unitize();
		if (acos(v1*v2) > amax)
		{
			amax = acos(v1*v2);
			starti = i + 1;
		}
	}
	for (int i = 0; i < starti; i++)
	{
		teethlist.push_back(teethlist[0]);
		teethlist.erase(teethlist.begin());
	}
	is_sorted = true;
}

bool Teeth::CalRefCurve()
{
	BubbleSort();
	bool is_new = false;
	int n = teethlist.size();
	ON_3dPointArray pointlist(n);
	for (int i = 0; i < n; i++)
	{
		pointlist.Insert(i, teethlist[i]->refpnt);
	}
	if (refcurve == NULL)
	{
		refcurve = new Curve(pointlist);
		is_new = true;
	}		
	else
	{
		Curve temp(pointlist);
		*refcurve = temp;
	}
	refcurve->MakePlanar(this);
	refcurve->SetTeeth(this);  //also update tooth::arch_par on the curve for every tooth.
	//nominate the 7th and 8th teeth as the centers
	if (n == 14)
	{
		SetToothType(6, 7);
	}
	//calculate origin for the first time
	if (is_new)
	{
		origin = (pointlist[0] + pointlist[n - 1]) / 2;
		CalToothCS();
		refcurve->m_isHide = false;
	}
	return is_new;
}

bool Teeth::CalToothCS()
{
	if (refcurve == NULL)
		return false;
	int n = teethlist.size();
	for (int i = 0; i < n; i++)
	{
		ON_3dVector tempaxis = refcurve->TangentAt(refcurve->GetParameter(i));
		teethlist[i]->yaxis = ON_DECL::ON_CrossProduct(teethlist[i]->longaxis, tempaxis);
		teethlist[i]->yaxis.Unitize();
		teethlist[i]->xaxis = ON_DECL::ON_CrossProduct(teethlist[i]->yaxis, teethlist[i]->longaxis);
		//teethlist[i]->SetOriFrame(teethlist[i]->xaxis, teethlist[i]->yaxis, teethlist[i]->longaxis);
		//teethlist[i]->SetOriPos(teethlist[i]->refpnt);
		teethlist[i]->localCS_on = true;
	}
	return true;
}


int Teeth::AttachToCurve(Curve &incurve, vector<double> &par)
{
	BubbleSort();
	int n = teethlist.size();
	int incisor_r, incisor_l;
	GetIncisor(incisor_l, incisor_r);
	if (incisor_l==0||incisor_r==0)
		return -2;
	if (!incurve.IsCCL(ref_axis))
		incurve.Reverse();
	//for (int i = 0; i < n; i++)
	//	teethlist[i]->CalVoxelMap(30);
	double par_l = 0.4, par_r = 0.6, deltapar = 0.05, par_i;
	ON_3dPoint p_l, p_r, p_i;
	Tooth *t_l = teethlist[incisor_l], *t_r = teethlist[incisor_r], *t_i;
	int moving_dir = 1; //1:move towards each other  0: move away
	par.resize(n);

	while (abs(deltapar) > 0.002)		//arrange the two incisors
	{
		p_l = incurve.PointAt(par_l);
		p_r = incurve.PointAt(par_r);
		t_l->Translate(p_l - t_l->refpnt);
		t_r->Translate(p_r - t_r->refpnt);
		if (!CheckCollision(incisor_l, incisor_r))	//no collision
		{
			if (!moving_dir)
			{
				deltapar /= -2;
				moving_dir = 1;
			}
		}
		else     //collision
		{
			if (moving_dir)
			{
				deltapar /= -2;
				moving_dir = 0;
			}
		}
		par_l += deltapar;
		par_r -= deltapar;
	}
	par[incisor_l] = par_l;
	par[incisor_r] = par_r;

	for (int i = incisor_l - 1; i >= 0; i--)	//arrange the left teeth
	{
		moving_dir = 1;
		t_i = teethlist[i];
		deltapar = 0.05;
		par_i = par[i + 1];
		while (abs(deltapar) > 0.002)
		{
			p_i = incurve.PointAt(par_i);
			t_i->Translate(p_i - t_i->refpnt);
			if (!CheckCollision(i, i + 1))
			{
				if (!moving_dir)
				{
					deltapar /= -2;
					moving_dir = 1;
				}
			}
			else
			{
				if (moving_dir)
				{
					deltapar /= -2;
					moving_dir = 0;
				}
			}
			par_i += deltapar;
		}
		par[i] = par_i;
	}

	for (int i = incisor_r + 1; i < n; i++)		//arrange the right teeth
	{
		moving_dir = 0;
		t_i = teethlist[i];
		deltapar = 0.05;
		par_i = par[i - 1];
		while (abs(deltapar) > 0.002)
		{
			p_i = incurve.PointAt(par_i);
			t_i->Translate(p_i - t_i->refpnt);
			if (!CheckCollision(i, i - 1))
			{
				if (moving_dir)
				{
					deltapar /= -2;
					moving_dir = 0;
				}
			}
			else
			{
				if (!moving_dir)
				{
					deltapar /= -2;
					moving_dir = 1;
				}
			}
			par_i -= deltapar;
		}
		par[i] = par_i;
	}
	CalRefCurve();
//	CalToothCS();
	incurve.SetTeeth(this);
	return 1;
}


int Teeth::AttachToCurve_Andrew(Curve &incurve, vector<double> &par)
{
	double opt_ang[7] = { 0, 0, 0, 3, 7, 12, 20 };
	double coe;
	if (upperorlower == UPPER)
		coe = 1;
	else
		coe = -1;
	BubbleSort();
	int n = teethlist.size();
	int incisor_r, incisor_l;
	GetIncisor(incisor_l, incisor_r);
	if (incisor_l == 0 || incisor_r == 0)
		return -2;
	if (!incurve.IsCCL(ref_axis))
		incurve.Reverse();
	//for (int i = 0; i < n; i++)
	//	teethlist[i]->CalVoxelMap();
	double par_l = 0.4, par_r = 0.6, deltapar = 0.05, par_i;
	ON_3dPoint p_l, p_r, p_i;
	ON_3dVector x_l, x_r, z_l, z_r, x_i, z_i;
	Tooth *t_l = teethlist[incisor_l], *t_r = teethlist[incisor_r], *t_i;
	int moving_dir = 1; //1:move towards each other  0: move away
	par.resize(n);
	while (abs(deltapar) > 0.002)		//arrange the two incisors
	{
		p_l = incurve.PointAt(par_l);
		x_l = incurve.TangentAt(par_l);
		z_l = incurve.GetZaxis() + coe*sin(opt_ang[6] * PI / 180)*ON_DECL::ON_CrossProduct(incurve.GetZaxis(), x_l);
		z_l.Unitize();
		p_r = incurve.PointAt(par_r);
		x_r = incurve.TangentAt(par_r);
		z_r = incurve.GetZaxis() + coe*sin(opt_ang[6] * PI / 180)*ON_DECL::ON_CrossProduct(incurve.GetZaxis(), x_r);
		z_r.Unitize();
		t_l->Translate(p_l - t_l->refpnt);
		t_l->Align(x_l, z_l);
//		t_l->CalVoxelMap(30);
		t_r->Translate(p_r - t_r->refpnt);
		t_r->Align(x_r, z_r);
		t_r->CalVoxelMap(30);
		if (!CheckCollision(incisor_l, incisor_r))	//no collision
		{
			if (!moving_dir)
			{
				deltapar /= -2;
				moving_dir = 1;
			}
		}
		else     //collision
		{
			if (moving_dir)
			{
				deltapar /= -2;
				moving_dir = 0;
			}
		}
		par_l += deltapar;
		par_r -= deltapar;
	}
	par[incisor_l] = par_l;
	par[incisor_r] = par_r;

	for (int i = incisor_l - 1; i >= 0; i--)	//arrange the left teeth
	{
		moving_dir = 1;
		t_i = teethlist[i];
		deltapar = 0.05;
		par_i = par[i + 1] - 0.1;
		while (abs(deltapar) > 0.002)
		{
			p_i = incurve.PointAt(par_i);
			x_i = incurve.TangentAt(par_i);
			z_i = incurve.GetZaxis() + coe*sin(opt_ang[i] * PI / 180)*ON_DECL::ON_CrossProduct(incurve.GetZaxis(), x_i);
			z_i.Unitize();
			t_i->Translate(p_i - t_i->refpnt);
			t_i->Align(x_i, z_i);
//			t_i->CalVoxelMap(30);
			if (!CheckCollision(i, i + 1))
			{
				if (!moving_dir)
				{
					deltapar /= -2;
					moving_dir = 1;
				}
			}
			else
			{
				if (moving_dir)
				{
					deltapar /= -2;
					moving_dir = 0;
				}
			}
			par_i += deltapar;
		}
		par[i] = par_i;
	}

	for (int i = incisor_r + 1; i < n; i++)		//arrange the right teeth
	{
		moving_dir = 0;
		t_i = teethlist[i];
		deltapar = 0.05;
		par_i = par[i - 1] + 0.1;
		while (abs(deltapar) > 0.002)
		{
			p_i = incurve.PointAt(par_i);
			x_i = incurve.TangentAt(par_i);
			z_i = incurve.GetZaxis() + coe*sin(opt_ang[n-i-1] * PI / 180)*ON_DECL::ON_CrossProduct(incurve.GetZaxis(), x_i);
			z_i.Unitize();
			t_i->Translate(p_i - t_i->refpnt);
			t_i->Align(x_i, z_i);
//			t_i->CalVoxelMap(30);
			if (!CheckCollision(i, i - 1))
			{
				if (moving_dir)
				{
					deltapar /= -2;
					moving_dir = 0;
				}
			}
			else
			{
				if (!moving_dir)
				{
					deltapar /= -2;
					moving_dir = 1;
				}
			}
			par_i -= deltapar;
		}
		par[i] = par_i;
	}
	CalRefCurve();
	//	CalToothCS();
	incurve.SetTeeth(this);
	return 1;
}

double Teeth::AlignToRefCurve()
{
	int n = teethlist.size(), center_l, center_r;
	vector<double> par(n);
	for (int i = 0; i < n; i++)
	{
		par[i] = refcurve->GetParameter(i);
	}
	int moving_dir = 1;
	GetIncisor(center_l, center_r);
	double deltapar = 0.05;
	while (abs(deltapar)>0.002)
	{
		teethlist[center_l]->AlignToCurve(*refcurve, par[center_l]);
		teethlist[center_r]->AlignToCurve(*refcurve, par[center_r]);
		if (!CheckCollision(center_l, center_r))
		{
			if (!moving_dir)
			{
				deltapar /= -2;
				moving_dir = 1;
			}
		}
		else     //collision
		{
			if (moving_dir)
			{
				deltapar /= -2;
				moving_dir = 0;
			}
		}
		par[center_l] += deltapar;
		par[center_r] -= deltapar;
	}

	for (int i = center_l - 1; i >= 0; i--)
	{
		moving_dir = 1;
		deltapar = 0.05;
		while (abs(deltapar) > 0.002)
		{
			teethlist[i]->AlignToCurve(*refcurve, par[i]);
			if (!CheckCollision(i, i + 1))
			{
				if (!moving_dir)
				{
					deltapar /= -2;
					moving_dir = 1;
				}
			}
			else
			{
				if (moving_dir)
				{
					deltapar /= -2;
					moving_dir = 0;
				}
			}
			par[i] += deltapar;
		}
	}
	for (int i = center_r + 1; i < n; i++)
	{
		moving_dir = 0;
		deltapar = 0.05;
		while (abs(deltapar) > 0.002)
		{
			teethlist[i]->AlignToCurve(*refcurve, par[i]);
			if (!CheckCollision(i, i - 1))
			{
				if (moving_dir)
				{
					deltapar /= -2;
					moving_dir = 0;
				}
			}
			else
			{
				if (!moving_dir)
				{
					deltapar /= -2;
					moving_dir = 1;
				}
			}
			par[i] -= deltapar;
		}
	}
	double ipr_length = 0;
	if (par[0] < 0)
		ipr_length += (refcurve->PointAt(par[0]) - refcurve->PointAt(0)).Length();
	if (par[n - 1]>1)
		ipr_length += (refcurve->PointAt(par[n - 1]) - refcurve->PointAt(1)).Length();
	CalRefCurve();
	return ipr_length;
}

double Teeth::AlignToRefCurve2(double gap)
{
	int n = teethlist.size(), center_l, center_r;
//	vector<double> par(n);
	GetIncisor(center_l, center_r);
	AlignTeeth(center_l, center_r, false, false, gap);
	/*double par_m = (teethlist[center_l]->arch_par + teethlist[center_r]->arch_par) / 2;
	par[center_l] = par_m - 0.05, par[center_r] = par_m + 0.05;
	teethlist[center_l]->AlignToCurve(*refcurve, par[center_l]);
	teethlist[center_r]->AlignToCurve(*refcurve, par[center_r]);
	double dis = CheckDistance(center_l, center_r) / 2;
	par[center_l] = refcurve->FindParameter(par[center_l], dis);
	par[center_r] = refcurve->FindParameter(par[center_r], -dis);
	teethlist[center_l]->AlignToCurve(*refcurve, par[center_l]);
	teethlist[center_r]->AlignToCurve(*refcurve, par[center_r]);
	dis = (CheckDistance(center_l, center_r) - gap) / 2;
	par[center_l] = refcurve->FindParameter(par[center_l], dis);
	par[center_r] = refcurve->FindParameter(par[center_r], -dis);
	teethlist[center_l]->AlignToCurve(*refcurve, par[center_l]);
	teethlist[center_r]->AlignToCurve(*refcurve, par[center_r]);*/

	for (int i = center_l - 1; i >= 0; i--)
	{
		AlignTeeth(i, i + 1, false, true, gap);
		/*teethlist[i]->AlignToCurve(*refcurve, par[i + 1] - 0.1);
		dis = CheckDistance(i, i + 1);
		par[i] = refcurve->FindParameter(par[i + 1] - 0.1, dis);
		teethlist[i]->AlignToCurve(*refcurve, par[i]);
		dis = CheckDistance(i, i + 1) - gap;
		par[i] = refcurve->FindParameter(par[i], dis);
		teethlist[i]->AlignToCurve(*refcurve, par[i]);*/
	}

	for (int i = center_r + 1; i < n; i++)
	{
		AlignTeeth(i - 1, i, true, false, gap);
		/*teethlist[i]->AlignToCurve(*refcurve, par[i - 1] + 0.1);
		dis = CheckDistance(i, i - 1);
		par[i] = refcurve->FindParameter(par[i - 1] + 0.1, -dis);
		teethlist[i]->AlignToCurve(*refcurve, par[i]);
		dis = CheckDistance(i, i - 1) - gap;
		par[i] = refcurve->FindParameter(par[i], -dis);
		teethlist[i]->AlignToCurve(*refcurve, par[i]);*/
	}
	double ipr_length = 0;
	if (teethlist[0]->arch_par < 0)
		ipr_length += (refcurve->PointAt(teethlist[0]->arch_par) - refcurve->PointAt(0)).Length();
	if (teethlist[n-1]->arch_par > 1)
		ipr_length += (refcurve->PointAt(teethlist[n - 1]->arch_par) - refcurve->PointAt(1)).Length();
	CalRefCurve();
	return ipr_length;
}

void Teeth::UpdateTeethAlign(int toothi)
{
	if (toothi == -1)
		return;

	int center_l, center_r;
	GetIncisor(center_l, center_r);
	double par = teethlist[toothi]->arch_par, par_i;
	if (toothi <= center_l)
	{
		for (int i = toothi - 1; i >= 0; i--)
		{
			/*teethlist[i]->AlignToCurve(*refcurve, par - 0.1);
			double dis = CheckDistance(i, i + 1);
			par_i = refcurve->FindParameter(par - 0.1, dis);
			teethlist[i]->AlignToCurve(*refcurve, par_i);
			dis = CheckDistance(i, i + 1);
			par_i = refcurve->FindParameter(par_i, dis);
			teethlist[i]->AlignToCurve(*refcurve, par_i);
			par = par_i;*/
			AlignTeeth(i, i + 1, false, true);
		}
	}
	else
	{
		for (int i = toothi + 1; i < teethlist.size(); i++)
		{
			/*teethlist[i]->AlignToCurve(*refcurve, par + 0.1);
			double dis = CheckDistance(i, i - 1);
			par_i = refcurve->FindParameter(par + 0.1, -dis);
			teethlist[i]->AlignToCurve(*refcurve, par_i);
			dis = CheckDistance(i, i - 1);
			par_i = refcurve->FindParameter(par_i, -dis);
			teethlist[i]->AlignToCurve(*refcurve, par_i);
			par = par_i;*/
			AlignTeeth(i-1, i, true, false);
		}
	}
	CalRefCurve();
}

void Teeth::AlignTeeth(int t1, int t2, bool is_t1fixed /*= false*/, bool is_t2fixed /*= false*/, double gap /*=0*/)
{
	if (teethlist[t1]->locked)
		is_t1fixed = true;
	if (teethlist[t2]->locked)
		is_t2fixed = true;
	if (is_t1fixed && is_t2fixed)
		return;
	double par1 = teethlist[t1]->arch_par, par2 = teethlist[t2]->arch_par;
	if (!is_t1fixed && !is_t2fixed)
	{
		double dis = CheckDistance(t1, t2) - gap;
		double min_dis = abs(dis);
		double best_par1 = par1, best_par2 = par2;
		int i = 0;
		while (abs(dis) > 0.05 && i < 5)
		{
			par1 = refcurve->FindParameter(par1, dis / 6);
			par2 = refcurve->FindParameter(par2, -dis / 6);
			teethlist[t1]->AlignToCurve(*refcurve, par1);
			teethlist[t2]->AlignToCurve(*refcurve, par2);
			dis = CheckDistance(t1, t2) - gap;
			if (abs(dis) < min_dis)
			{
				min_dis = abs(dis);
				best_par1 = par1;
				best_par2 = par2;
			}
			i++;
		}
		teethlist[t1]->AlignToCurve(*refcurve, best_par1);
		teethlist[t2]->AlignToCurve(*refcurve, best_par2);
		//if (dis - gap < 0)
		//{
		//	double deltapar = -0.001;
		//	while (CheckCollision(t1, t2, gap))
		//	{
		//		par1 += deltapar;
		//		par2 -= deltapar;
		//		teethlist[t1]->AlignToCurve(*refcurve, par1);
		//		teethlist[t2]->AlignToCurve(*refcurve, par2);
		//	}
		//}
		//else
		//{
		//	double deltapar = 0.001;
		//	while (!CheckCollision(t1, t2, gap))
		//	{
		//		par1 += deltapar;
		//		par2 -= deltapar;
		//		teethlist[t1]->AlignToCurve(*refcurve, par1);
		//		teethlist[t2]->AlignToCurve(*refcurve, par2);
		//	}
		//}
		return;
	}
	if (is_t1fixed) //left tooth fixed
	{
		par2 = par1 + 0.1;
		double best_par2 = par2;
		teethlist[t2]->AlignToCurve(*refcurve, par2);
		double dis = CheckDistance(t1, t2) - gap;		
		double min_dis = abs(dis);
		int i = 0;
		while (abs(dis) > 0.05 && i < 5)
		{
			par2 = refcurve->FindParameter(par2, -dis/2);
			teethlist[t2]->AlignToCurve(*refcurve, par2);
			dis = CheckDistance(t1, t2) - gap;
			if (abs(dis) < min_dis)
			{
				min_dis = abs(dis);
				best_par2 = par2;
			}
			i++;
		}
		teethlist[t2]->AlignToCurve(*refcurve, best_par2);
		//if (dis - gap < 0)
		//{
		//	double deltapar = -0.001;
		//	while (CheckCollision(t1, t2, gap))
		//	{
		//		par2 -= deltapar;
		//		teethlist[t2]->AlignToCurve(*refcurve, par2);
		//	}
		//}
		//else
		//{
		//	double deltapar = 0.001;
		//	while (!CheckCollision(t1, t2, gap))
		//	{
		//		par2 -= deltapar;
		//		teethlist[t2]->AlignToCurve(*refcurve, par2);
		//	}
		//}
		return;
	}
	if (is_t2fixed) //right tooth fixed
	{
		par1 = par2 - 0.1;
		double best_par1 = par1;
		teethlist[t1]->AlignToCurve(*refcurve, par1);
		double dis = CheckDistance(t1, t2) - gap;
		double min_dis = abs(dis);
		int i = 0;
		while (abs(dis) > 0.05 && i < 5)
		{
			par1 = refcurve->FindParameter(par1, dis/2);
			teethlist[t1]->AlignToCurve(*refcurve, par1);
			dis = CheckDistance(t1, t2) - gap;
			if (abs(dis) < min_dis)
			{
				min_dis = abs(dis);
				best_par1 = par1;
			}
			i++;
		}
		teethlist[t1]->AlignToCurve(*refcurve, best_par1);
		//if (dis - gap < 0)
		//{
		//	double deltapar = -0.001;
		//	while (CheckCollision(t1, t2, gap))
		//	{
		//		par1 += deltapar;
		//		teethlist[t1]->AlignToCurve(*refcurve, par1);
		//	}
		//}
		//else
		//{
		//	double deltapar = 0.001;
		//	while (!CheckCollision(t1, t2, gap))
		//	{
		//		par1 += deltapar;
		//		teethlist[t1]->AlignToCurve(*refcurve, par1);
		//	}
		//}
		return;
	}
}

void Teeth::IPR(double value)
{
	int n = teethlist.size(), center_l, center_r;
	vector<double> par(n);
	for (int i = 0; i < n; i++)
	{
		par[i] = refcurve->GetParameter(i);
	}
	GetIncisor(center_l, center_r);
	double dis = value / 2;
	for (int i = center_l; i >= 0; i--)
	{
		par[i] = refcurve->FindParameter(par[i], dis);
		teethlist[i]->AlignToCurve(*refcurve, par[i]);
		if (teethlist[i]->T_type <= 3)
			dis += value;
	}
	dis = value / 2;
	for (int i = center_r; i < n; i++)
	{
		par[i] = refcurve->FindParameter(par[i], -dis);
		teethlist[i]->AlignToCurve(*refcurve, par[i]);
		if (teethlist[i]->T_type <= 3)
			dis += value;
	}
	CalRefCurve();
}

void Teeth::CalStaging(bool collision)
{
	int n = teethlist.size();

	if (n == 0)
		return;

	vector<int> staging_num(n);
	for (int i = 0; i < n; i++)
	{
		teethlist[i]->CalStagingStep();
		staging_num[i] = ceil(1 / teethlist[i]->staging_step) + 1;
	}

	if (!collision)
	{		
		vector<int>::iterator colnum = max_element(staging_num.begin(), staging_num.end());
		staging = LaGenMatDouble::ones(n, *colnum);
		for (int i = 0; i < n; i++)
		{
			double st = 0, spd = teethlist[i]->staging_step;
			for (int j = 0; j < staging_num[i] - 1; j++)
				staging(i, j) = st + j*spd;
		}
		is_staging = true;
	}
	else
	{
		vector<vector<int>> allstaging(n);
		FindPath(CalCollisionMat(0, 1), allstaging[0], allstaging[1]);
		for (int i = 2; i < n; i++)
		{
			vector<int> s1;
			FindPath(CalCollisionMat(i - 1, i), s1, allstaging[i]);
			vector<int> mergeindex = MergeVector(s1, allstaging[i - 1]);
			ExpandVector(allstaging[i], mergeindex);
			mergeindex = MergeVector(allstaging[i - 1], s1);
			for (int j = 0; j < i; j++)
				ExpandVector(allstaging[j], mergeindex);
		}
		staging = LaGenMatDouble::ones(n, allstaging[0].size());
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < allstaging[i].size(); j++)
			{
				staging(i, j) = min(allstaging[i][j] * teethlist[i]->staging_step, 1.0);
			}
		}
		is_staging = true;
	}

}

LaGenMatInt Teeth::CalCollisionMat(int t1, int t2)
{
	teethlist[t1]->CalStagingStep();
	teethlist[t2]->CalStagingStep();
	int staging_num1 = ceil(1 / teethlist[t1]->staging_step) + 1, staging_num2 = ceil(1 / teethlist[t2]->staging_step) + 1;
	LaGenMatInt matout = LaGenMatInt::ones(staging_num1, staging_num2);
	double spd1 = teethlist[t1]->staging_step, spd2 = teethlist[t2]->staging_step;
	for (int i = 0; i < staging_num1; i++)
	{
		for (int j = 0; j < staging_num2; j++)
		{
			double staging1 = min(i*spd1, 1.0), staging2 = min(j*spd2, 1.0);
			teethlist[t1]->SetStaging(staging1);
			teethlist[t2]->SetStaging(staging2);
			if (CheckCollision(t1, t2))
				matout(i, j) = 0;
		}
	}
	matout(0, 0) = 1;
	matout(staging_num1 - 1, staging_num2 - 1) = 1;
	teethlist[t1]->ResetStaging();
	teethlist[t2]->ResetStaging();
	return matout;
}

void Teeth::Translate(double dx, double dy, double dz)
{
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->Translate(dx, dy, dz);
	}
	if (refcurve != NULL)
		refcurve->Translate(dx, dy, dz);
}

void Teeth::Translate(ON_3dVector v)
{
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->Translate(v);
	}
	if (refcurve != NULL)
		refcurve->Translate(v);
}

void Teeth::Rotate(double rot_angle, ON_3dVector axis, ON_3dPoint origin)
{
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->Rotate(rot_angle, axis, origin);
	}
	if (refcurve!=NULL)
		refcurve->Rotate(rot_angle, axis, origin);
}

void Teeth::InterConfigView(double par)
{
	if (!is_staging)
	{
		for (int i = 0; i < teethlist.size(); i++)
			teethlist[i]->SetViewPar(par);
	}
	else
	{
		int m = staging.cols() - 1;
		int lp = floor(m*par);
		if (lp == m)
			lp--;
		for (int i = 0; i < teethlist.size(); i++)
		{
			double vp = staging(i, lp) + (staging(i,lp+1)-staging(i,lp))*(m*par-lp);
			teethlist[i]->SetViewPar(vp);
		}
	}
}

double Teeth::AddBracket(int toothi, FEntity* in)
{
	teethlist[toothi]->AddBracket(in);
	if (!archwire)
	{
		AlignBracketToTooth(toothi, BRACKET_HEIGHT);
		return 0;
	}
	else
	{
		double par;
		AlignBracketOnWire(toothi, par);
		teethlist[toothi]->m_bracket->SetArchWirePar(par);
		teethlist[toothi]->m_bracket->SetArchWireParIni(par);
		return par;
	}
}

bool Teeth::AlignBracketToTooth(int toothi, double height)
{
	teethlist[toothi]->m_bracket->CalLCS(1, 2);
	
	ON_3dVector gz = refcurve->GetZaxis();
	ON_3dVector gx = -teethlist[toothi]->yaxis;
	gx = ON_CrossProduct(ON_CrossProduct(gz, gx), gz);
	gx.Unitize();
	teethlist[toothi]->m_bracket->Align(gx, gz);
	ON_3dPoint des = teethlist[toothi]->refpnt;
	if (upperorlower == UPPER)
		des = des + gz*height;
	else
		des = des - gz*height;
	teethlist[toothi]->m_bracket->Translate(des);
	double dd = 0.1;
	while (CheckCollision(toothi))
	{
		des = des + gx*dd;
		teethlist[toothi]->m_bracket->Translate(des);
	}
	dd = -0.05;
	while (!CheckCollision(toothi))
	{
		des = des + gx*dd;
		teethlist[toothi]->m_bracket->Translate(des);
	}
	return true;
	
	//else
	//	return false;
}

bool Teeth::AlignBracketOnWire(int toothi, double &par_i)
{
	if (archwire == NULL)
	{
		return false;
	}
	if (teethlist[toothi]->m_bracket->CalLCS(1, 2))
	{
		ON_3dPoint sp = teethlist[toothi]->refpnt - (teethlist[toothi]->refpnt - archwire->GetOrigin())*archwire->GetZaxis()*archwire->GetZaxis();
		ON_3dVector sv = -teethlist[toothi]->yaxis, vi;
		sv = sv - archwire->GetZaxis()*(archwire->GetZaxis()*sv);
		sv.Unitize();
		double par_l = 0, par_u = 1;
		for (int i = 0; i < 10; i++)
		{
			par_i = (par_l + par_u) / 2;
			vi = archwire->PointAt(par_i) - sp;
			if (ON_CrossProduct(sv, vi)*archwire->GetZaxis() > 0)   //move left
				par_u = par_i;
			else
				par_l = par_i;
		}
		ON_3dVector gx = ON_CrossProduct(archwire->TangentAt(par_i), archwire->GetZaxis());
		teethlist[toothi]->m_bracket->Align(gx, archwire->GetZaxis());
		ON_3dPoint des = archwire->PointAt(par_i) - archwire->width*0.5*gx;
		teethlist[toothi]->m_bracket->Translate(des);
		return true;
	}
	return false;
}

bool Teeth::MoveBracketOnWire(int toothi, double par)
{
	if (archwire == NULL)
		return false;
	ON_3dVector gx = ON_CrossProduct(archwire->TangentAt(par), archwire->GetZaxis());
	teethlist[toothi]->m_bracket->Align(gx, archwire->GetZaxis());
	ON_3dPoint des = archwire->PointAt(par) - archwire->width*0.5*gx;
	teethlist[toothi]->m_bracket->Translate(des);
	teethlist[toothi]->m_bracket->SetArchWirePar(par);
	return true;
}

void Teeth::FlipBracket(int toothi)
{
	if (teethlist[toothi]->m_bracket)
	{
		teethlist[toothi]->m_bracket->Rotate(PI, teethlist[toothi]->m_bracket->xaxis, teethlist[toothi]->m_bracket->origin);
		teethlist[toothi]->m_bracket->zaxis = -teethlist[toothi]->m_bracket->zaxis;
		teethlist[toothi]->m_bracket->yaxis = -teethlist[toothi]->m_bracket->yaxis;
		if (archwire == NULL)
		{
			ON_3dVector gx = teethlist[toothi]->m_bracket->xaxis;
			double dx = 0.05;
			if (CheckCollision(toothi))
			{
				while (CheckCollision(toothi))
				{
					teethlist[toothi]->m_bracket->Translate(teethlist[toothi]->m_bracket->origin + gx*dx);
				}
			}
			else
			{
				while (!CheckCollision(toothi))
				{
					teethlist[toothi]->m_bracket->Translate(teethlist[toothi]->m_bracket->origin - gx*dx);
				}
			}
		}
		teethlist[toothi]->m_bracket->Update();
	}
}

void Teeth::UpdateBracket(int toothi)
{
	if (teethlist[toothi]->m_bracket)
	{
		teethlist[toothi]->CalVoxelMap(100);
		ON_3dVector gx = teethlist[toothi]->m_bracket->xaxis;
		double dx = 0.05;
		if (CheckCollision(toothi))
		{
			while (CheckCollision(toothi))
			{
				teethlist[toothi]->m_bracket->Translate(teethlist[toothi]->m_bracket->origin + gx*dx);
			}
		}
		else
		{
			while (!CheckCollision(toothi))
			{
				teethlist[toothi]->m_bracket->Translate(teethlist[toothi]->m_bracket->origin - gx*dx);
			}
		}
		teethlist[toothi]->m_bracket->Update();
	}
}

void Teeth::RestoreTeeth()
{
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->Restore();
	}
}

bool Teeth::SetArchWire(Curve *inwire, double width, bool is_square, double height)
{
	if (refcurve == NULL)
		return false;
	archwire = inwire;
	if (is_square)
		archwire->disp_option = 2;
	else
		archwire->disp_option = 1;
	archwire->width = width;
	archwire->Align(refcurve->GetXaxis(), refcurve->GetZaxis());
	ON_3dVector v = refcurve->PointAt(0.5) - archwire->PointAt(0.5);
	archwire->Translate(v);
	if (upperorlower == UPPER)
		archwire->Translate(archwire->GetZaxis()*height);
	else
		archwire->Translate(-archwire->GetZaxis()*height);
	archwire->Translate(-archwire->GetYaxis() * 1);
	archwire->SetTeeth(this);
	archwire->Update();
	return true;
}

bool Teeth::UpdateArchWire(int toothi)
{
	if (teethlist[toothi]->m_bracket == NULL)
		return true;
	double dy = 0.1;
	if (CheckCollision(toothi))
	{
		for (int i = 0; i < 10; i++)
		{
			archwire->Translate(-dy*archwire->GetYaxis());
			if (!CheckCollision(toothi))
				break;
		}
		archwire->Update();
	}
	else
		return true;
	if (CheckCollision(toothi))
		return false;
	return true;
}

bool Teeth::UpdateArchWire()
{
	//restore archwire first
	archwire->Align(refcurve->GetXaxis(), refcurve->GetZaxis());
	ON_3dVector v = refcurve->PointAt(0.5) - archwire->PointAt(0.5);
	archwire->Translate(v - v*archwire->GetZaxis()*archwire->GetZaxis());
	archwire->Translate(-archwire->GetYaxis() * 1);
	//restore archwire first

	int n = teethlist.size();
	bool is_ok = true;
	for (int i = 0; i < n; i++)
	{
		if (!UpdateArchWire(i))
			is_ok = false;
	}
	return is_ok;
}

void Teeth::SetToothType(int leftcenter, int rightcenter)
{
	if (is_typeset)
		return;
	for (int i = leftcenter; i >= 0; i--)
		teethlist[i]->SetType(leftcenter - i + 1, LEFT|upperorlower);
	for (int i = rightcenter; i < teethlist.size(); i++)
		teethlist[i]->SetType(i - rightcenter + 1, RIGHT|upperorlower);
	is_typeset = true;
}

bool Teeth::PairTooth(Teeth *pairteeth)
{
	if (!pairteeth)
		return false;
	if (!is_typeset || !pairteeth->is_typeset)
		return false;
	BubbleSort();
	pairteeth->BubbleSort();
	vector<Tooth*> pairlist = pairteeth->GetTeethlist();
	int this_center, pair_center;
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->SetPairTooth(NULL);
		if (teethlist[i]->T_type == 1)
			this_center = i;
	}
	for (int i = 0; i < pairlist.size(); i++)
	{
		pairlist[i]->SetPairTooth(NULL);
		if (pairlist[i]->T_type == 1)
			pair_center = i;
	}
	int i = 0;
	while (this_center-i>=0 && pair_center-i>=0)
	{
		teethlist[this_center - i]->SetPairTooth(pairlist[pair_center - i]);
		pairlist[pair_center - i]->SetPairTooth(teethlist[this_center - i]);
		i++;
	}
	i = 1;
	while (this_center + i < teethlist.size() && pair_center + i < pairlist.size())
	{
		teethlist[this_center + i]->SetPairTooth(pairlist[pair_center + i]);
		pairlist[pair_center + i]->SetPairTooth(teethlist[this_center + i]);
		i++;
	}
	return true;
}

bool Teeth::OverBite(int leftorright, double &OB)
{
	if (!is_typeset)
		return false;
	//CalRefCurve();
	int incisor_l, incisor_r, incisor;
	GetIncisor(incisor_l, incisor_r);
	if (leftorright == LEFT)
		incisor = incisor_l;
	else
		incisor = incisor_r;
	Tooth *upper_t, *lower_t;
	if (!teethlist[incisor]->GetPairTooth())
		return false;
	ON_3dVector dir = refcurve->GetZaxis();
	if (upperorlower == UPPER)
	{
		upper_t = teethlist[incisor];
		lower_t = teethlist[incisor]->GetPairTooth();
	}
	else
	{
		upper_t = teethlist[incisor]->GetPairTooth();
		lower_t = teethlist[incisor];
	}
	double min_d = upper_t->centroid*dir, max_d = lower_t->centroid*dir, dis;
	int n = upper_t->GetMesh()->VertexCount();
	for (int i = 0; i < n; i++)
	{
		dis = upper_t->GetMesh()->Vertex(i) * dir;
		if (dis < min_d)
			min_d = dis;
	}
	n = lower_t->GetMesh()->VertexCount();
	for (int i = 0; i < n; i++)
	{
		dis = lower_t->GetMesh()->Vertex(i) * dir;
		if (dis > max_d)
			max_d = dis;
	}
	OB = max_d - min_d;
	return true;
}

bool Teeth::OverJet(int leftorright, double &OJ)
{
	if (!is_typeset)
		return false;
	//CalRefCurve();
	int incisor_l, incisor_r, incisor;
	GetIncisor(incisor_l, incisor_r);
	if (leftorright == LEFT)
		incisor = incisor_l;
	else
		incisor = incisor_r;
	Tooth *upper_t, *lower_t;
	if (!teethlist[incisor]->GetPairTooth())
		return false;
	ON_3dVector tang = refcurve->TangentAt(refcurve->GetParameter(incisor));
	ON_3dVector dir = ON_CrossProduct(tang, refcurve->GetZaxis());
	dir.Unitize();
	if (upperorlower == UPPER)
	{
		upper_t = teethlist[incisor];
		lower_t = teethlist[incisor]->GetPairTooth();
	}
	else
	{
		upper_t = teethlist[incisor]->GetPairTooth();
		lower_t = teethlist[incisor];
	}
	double max_u = upper_t->centroid*dir, max_l = lower_t->centroid*dir, dis;
	int n = upper_t->GetMesh()->VertexCount();
	for (int i = 0; i < n; i++)
	{
		dis = upper_t->GetMesh()->Vertex(i) * dir;
		if (dis > max_u)
			max_u = dis;
	}
	n = lower_t->GetMesh()->VertexCount();
	for (int i = 0; i < n; i++)
	{
		dis = lower_t->GetMesh()->Vertex(i) * dir;
		if (dis > max_l)
			max_l = dis;
	}
	OJ = max_u - max_l;
	return true;
}

bool Teeth::Angulation(int leftorright, double &AG)
{
	if (!is_typeset)
		return false;
	int incisor_l, incisor_r, incisor;
	GetIncisor(incisor_l, incisor_r);
	if (leftorright == LEFT)
		incisor = incisor_l;
	else
		incisor = incisor_r;
	Tooth *upper_t, *lower_t;
	if (!teethlist[incisor]->GetPairTooth())
		return false;
	if (upperorlower == UPPER)
	{
		upper_t = teethlist[incisor];
		lower_t = teethlist[incisor]->GetPairTooth();
	}
	else
	{
		upper_t = teethlist[incisor]->GetPairTooth();
		lower_t = teethlist[incisor];
	}
	AG = acos(upper_t->longaxis*(-lower_t->longaxis)) * 180 / PI;
	return true;
}

void Teeth::UpdateAllViews(bool refcur_on)
{
	//the clip plane display
	
	if (refcurve != NULL)
	{
		refcurve->highlight_on = refcur_on;
		refcurve->Update();
	}
	for (int i = 0; i < teethlist.size(); i++)
	{
		teethlist[i]->Update();
	}	
}

void Teeth::OcclusalDisplayOn()
{
	for (int i = 0; i < teethlist.size(); i++)
		teethlist[i]->m_bocclusal = false;
}

void Teeth::OcclusalDisplayOff()
{
	for (int i = 0; i < teethlist.size(); i++)
		teethlist[i]->occlusal_on = false;
}

void Teeth::OcclusalDisplayUpdate(int toothi)
{
	int center_l, center_r;
	GetIncisor(center_l, center_r);
	if (toothi <= center_l)
	{
		for (int i = 0; i <= toothi; i++)
		{
			if (teethlist[i]->occlusal_on)
			{
				teethlist[i]->m_bocclusal = false;
				teethlist[i]->GetPairTooth()->m_bocclusal = false;
			}
		}
	}
	else
	{
		for (int i = toothi; i < teethlist.size(); i++)
		{
			if (teethlist[i]->occlusal_on)
			{
				teethlist[i]->m_bocclusal = false;
				teethlist[i]->GetPairTooth()->m_bocclusal = false;
			}
		}
	}
	UpdateAllViews();
}

Tooth::Tooth()
{
	m_pMesh = NULL;
	m_bMeshed = false;
	m_bracket = NULL;
	paired_tooth = NULL;
	m_red = 0.9;
	m_green = 1;
	m_blue = 1;
	refpnt_on = false;
	longaxis_on = false;
	localCS_on = false;
	voxel_density = 50;
	type = 0;
	view_par = 1;
	m_xForm.Identity();
	m_xForm_f.Identity();
	m_bocclusal = true;
	occlusal_on = false;
	locked = false;
}

Tooth::~Tooth()
{
	if (m_pMesh)
		m_pMesh->Destroy();
	if (m_tree)
		delete m_tree;
}

void Tooth::CalLongAxis(ON_3dVector ref)
{
	CalCentroid();
	LaGenMatDouble M = LaGenMatDouble::zeros(3, 3), VR(3,3);
	LaGenMatDouble pm(3, 1), pm_t(1, 3);
	const ON_MeshTopology *T_mesh = &(m_pMesh->Topology());
	int n = T_mesh->TopVertexCount();
	int i = 0;
	while (i < n)
	{
		ON_3dVector p = T_mesh->TopVertexPoint(i) - centroid;
		pm(0, 0) = p.x;
		pm(1, 0) = p.y;
		pm(2, 0) = p.z;
		pm_t(0, 0) = p.x;
		pm_t(0, 1) = p.y;
		pm_t(0, 2) = p.z;	
		Blas_Mat_Mat_Mult(pm, pm_t, M, false, false, 1, 1);
		i++;
	}
//	M.scale(1 / n);
	LaVectorDouble eig_r(3), eig_i(3);
	LaEigSolve(M, eig_r, eig_i, VR);
	double e1 = eig_r(0);
	double e2 = eig_r(1);
	double e3 = eig_r(2);
	double largest = abs(e1);
	int index = 0;
	if (abs(e2) > largest)
	{
		largest = abs(e2);
		index = 1;
	}
	if (abs(e3) > largest)
	{
		largest = abs(e3);
		index = 2;
	}
	longaxis.x = VR(0, index);
	longaxis.y = VR(1, index);
	longaxis.z = VR(2, index);
	if (longaxis * ref <0)
		longaxis = -longaxis;	//make it upwards

}

void Tooth::CalCentroid()
{
	const ON_MeshTopology *T_mesh = &(m_pMesh->Topology());
	int n = T_mesh->TopVertexCount();
	ON_3dVector cen(0,0,0);
	for (int i = 0; i < n; i++)
	{
		cen += T_mesh->TopVertexPoint(i);
	}
	centroid = cen / n;
}

void Tooth::CalRefPnt(int upper_or_lower)
{
	ON_3dVector dir = longaxis;
	double Dmax = 0, d;
	if (upper_or_lower == UPPER)
		dir = -dir;
	const ON_MeshTopology *tmesh = &(m_pMesh->Topology());
	int n = tmesh->TopVertexCount();
	for (int i = 0; i < n; i++)
	{
		d = (tmesh->TopVertexPoint(i) - centroid)*dir;
		if (d>Dmax)
			Dmax = d;
	}
	refpnt = centroid + dir*Dmax;
}

ON_3dPoint Tooth::GetBottomPnt(int upper_or_lower)
{
	ON_3dPoint p;
	ON_3dVector dir = longaxis;
	double Dmax = 0, d;
	if (upper_or_lower == UPPER)
		dir = -dir;
	int n = m_pMesh->VertexCount();
	for (int i = 0; i < n; i++)
	{
		d = (m_pMesh->Vertex(i) - centroid)*dir;
		if (d > Dmax)
			Dmax = d;
	}
	p = centroid + dir*Dmax;
	return p;
}

void Tooth::CalNormals()
{
	if (!m_pMesh->HasFaceNormals())
	{
		m_pMesh->ComputeFaceNormals();
		m_pMesh->UnitizeFaceNormals();
	}
	if (!m_pMesh->HasVertexNormals())
	{
		m_pMesh->ComputeVertexNormals();
		m_pMesh->UnitizeVertexNormals();
	}
}

//void Tooth::CalBCylinder()
//{
//	double hmax = 0, hmin = 0, rmax = 0, h, r;
//	const ON_MeshTopology *tmesh = &(m_pMesh->Topology());
//	int n = tmesh->TopVertexCount();
//	ON_3dVector v;
//	for (int i = 0; i < n; i++)
//	{
//		v = tmesh->TopVertexPoint(i) - centroid;
//		h = v * longaxis;
//		if (h > hmax)
//			hmax = h;
//		if (h < hmin)
//			hmin = h;
//		r = sqrt(v.Length()*v.Length() - h*h);
//		if (r > rmax)
//			rmax = r;
//	}
//	bcylinder.circle = ON_Circle(ON_Plane(centroid, longaxis), centroid, rmax);
//	bcylinder.height[0] = hmin;
//	bcylinder.height[1] = hmax;
//}

void Tooth::CalVoxelMap(int density)
{
	/*voxel_density = density;
	voxel_top = LaGenMatDouble::zeros(voxel_density, voxel_density);
	voxel_bot = LaGenMatDouble::zeros(voxel_density, voxel_density);
	int n = m_pMesh->VertexCount();
	xlim[0] = centroid.x;
	xlim[1] = centroid.x;
	ylim[0] = centroid.y;
	ylim[1] = centroid.y;
	double x, y, dx, dy;
	for (int i = 0; i < n; i++)
	{
		x = m_pMesh->Vertex(i).x;
		y = m_pMesh->Vertex(i).y;
		if (x < xlim[0]) xlim[0] = x;
		if (x > xlim[1]) xlim[1] = x;
		if (y < ylim[0]) ylim[0] = y;
		if (y > ylim[1]) ylim[1] = y;
	}
	dx = (xlim[1] - xlim[0]) / (voxel_density-1);
	dy = (ylim[1] - ylim[0]) / (voxel_density-1);
	int rowi, coli;
	double zvalue;
	for (int i = 0; i < n; i++)
	{
		zvalue = m_pMesh->Vertex(i).z;
		rowi = round((m_pMesh->Vertex(i).x - xlim[0]) / dx);
		coli = round((m_pMesh->Vertex(i).y - ylim[0]) / dy);
		if (voxel_top(rowi, coli) == 0 && voxel_bot(rowi, coli) == 0)
		{
			voxel_top(rowi, coli) = zvalue;
			voxel_bot(rowi, coli) = zvalue;
		}
		else
		{
			if (zvalue > voxel_top(rowi, coli))	voxel_top(rowi, coli) = zvalue;
			if (zvalue < voxel_bot(rowi, coli))	voxel_bot(rowi, coli) = zvalue;
		}
	}*/

	zdir = longaxis;
	xdir = ON_CrossProduct(ON_3dVector(0,1,0),zdir);
	xdir.Unitize();
	ydir = ON_CrossProduct(zdir, xdir);
	voxel_density = density;
	double zlevel = centroid*zdir;
	voxel_top = LaGenMatDouble::ones(voxel_density, voxel_density);
	voxel_bot = LaGenMatDouble::ones(voxel_density, voxel_density);
	voxel_top.scale(zlevel);
	voxel_bot.scale(zlevel);
	int n = m_pMesh->VertexCount();
	xlim[0] = centroid*xdir;
	xlim[1] = centroid*xdir;
	ylim[0] = centroid*ydir;
	ylim[1] = centroid*ydir;
	double x, y, dx, dy;
	for (int i = 0; i < n; i++)
	{
		x = m_pMesh->Vertex(i)*xdir;
		y = m_pMesh->Vertex(i)*ydir;
		if (x < xlim[0]) xlim[0] = x;
		if (x > xlim[1]) xlim[1] = x;
		if (y < ylim[0]) ylim[0] = y;
		if (y > ylim[1]) ylim[1] = y;
	}
	dx = (xlim[1] - xlim[0]) / (voxel_density - 1);
	dy = (ylim[1] - ylim[0]) / (voxel_density - 1);
	int rowi, coli;
	double zvalue, height[3];;
	ON_3dPoint v[3];
	ON_2dPoint v2d[3];
	int vx[3], vy[3];
	for (int fi = 0; fi < m_pMesh->FaceCount(); fi++)
	{
		const ON_MeshFace& f = m_pMesh->m_F[fi];
		for (int i = 0; i < 3; i++)
		{
			v[i] = m_pMesh->Vertex(f.vi[i]);
			v2d[i].x = v[i] * xdir;
			v2d[i].y = v[i] * ydir;
			vx[i] = round((v2d[i].x - xlim[0]) / dx);
			vy[i] = round((v2d[i].y - ylim[0]) / dy);
		}
		int vx_max = vx[0], vx_min = vx[0], vy_max = vy[0], vy_min = vy[0];
		for (int i = 0; i < 3; i++)
		{
			height[i] = v[i] * zdir;
			if (vx[i] > vx_max)
				vx_max = vx[i];
			if (vx[i] < vx_min)
				vx_min = vx[i];
			if (vy[i] > vy_max)
				vy_max = vy[i];
			if (vy[i] < vy_min)
				vy_min = vy[i];
		}
		double par[3];
		for (int i = vx_min; i <= vx_max; i++)
		{
			for (int j = vy_min; j <= vy_max; j++)
			{
				ON_2dPoint pp(dx*i + xlim[0], dy*j + ylim[0]);
				if (TriangleInterp(v2d[0], v2d[1], v2d[2], pp, par))
				{
					zvalue = height[0] * par[0] + height[1] * par[1] + height[2] * par[2];
					//if (voxel_top(i, j) == 0 && voxel_bot(i, j) == 0)
					//{
					//	voxel_top(i, j) = zvalue;
					//	voxel_bot(i, j) = zvalue;
					//}
					//else
					//{
					if (zvalue > voxel_top(i, j))	voxel_top(i, j) = zvalue;
					if (zvalue < voxel_bot(i, j))	voxel_bot(i, j) = zvalue;
				//	}
				}
			}
		}
	}
}

void Tooth::CalSamplePnts(double angle, ON_3dPoint refp)
{
	sample_pnts = MeshSampling(m_pMesh, refp, angle,longaxis);
}

bool Tooth::IsPointInside(ON_3dPoint p)
{
/*
	ON_Xform inverse_form = this->GetXform().Inverse();
	p.Transform(inverse_form);
	double dx, dy;
	dx = (xlim[1] - xlim[0]) / (voxel_density - 1);
	dy = (ylim[1] - ylim[0]) / (voxel_density - 1);
	int rowi = round((p.x - xlim[0]) / dx);
	int coli = round((p.y - ylim[0]) / dy);
	if (rowi >= 0 && rowi < voxel_density)
	{
		if (coli >= 0 && coli < voxel_density)
		{
			if (p.z>voxel_bot(rowi, coli) && p.z < voxel_top(rowi, coli))
				return true;
		}
	}
	return false;*/

	ON_Xform inverse_form = this->GetXform().Inverse();
	p.Transform(inverse_form);
	double dx, dy;
	dx = (xlim[1] - xlim[0]) / (voxel_density - 1);
	dy = (ylim[1] - ylim[0]) / (voxel_density - 1);
	double px = p*xdir, py = p*ydir;
	if (px >= xlim[0] && px <= xlim[1])
	{
		if (py >= ylim[0] && py <= ylim[1])
		{
			int rowi = round((px - xlim[0]) / dx), coli = round((py - ylim[0]) / dy);
			if (p*zdir>voxel_bot(rowi, coli) && p*zdir < voxel_top(rowi, coli))
				return true;
		}
	}
	return false;
}

ON_3dPoint Tooth::NearestPoint(ON_3dPoint q)
{
	ON_Xform inverse_form = this->GetXform().Inverse();
	q.Transform(inverse_form);
	ON_3dPointArray pp = m_tree->ANNSearch(q, 1);
	pp[0].Transform(m_xForm);
	return pp[0];
}

double Tooth::Distance(Tooth *t2)
{
	int pn = t2->sample_pnts.size();
	ON_3dPoint p2 = t2->GetMesh()->Vertex(t2->sample_pnts[0]), p = NearestPoint(p2);
	double dmin = (p - p2).Length(), d;
	if (IsPointInside(p2))
		dmin = -dmin;
	for (int i = 1; i < pn; i++)
	{
		p2 = t2->GetMesh()->Vertex(t2->sample_pnts[i]);
		p = NearestPoint(p2);
		d = (p - p2).Length();
		if (IsPointInside(p2))
			d = -d;
		if (d < dmin)
			dmin = d;
	}
	return dmin;
}

double Tooth::OcclusalDistance(ON_3dPoint p, double thre)
{
	ON_Xform inverse_form = this->GetXform().Inverse();
	p.Transform(inverse_form);
	double distance = thre;
	double px = p*xdir, py = p*ydir;
	double dx = (xlim[1] - xlim[0]) / (voxel_density - 1), dy = (ylim[1] - ylim[0]) / (voxel_density - 1);
	if (px >= xlim[0] && px <= xlim[1])
	{
		if (py >= ylim[0] && py <= ylim[1])
		{
			int row_l = floor((px - xlim[0]) / dx), col_l = floor((py - ylim[0]) / dy);
			int row_u = ceil((px - xlim[0]) / dx), col_u = ceil((py - ylim[0]) / dy);
			double u = (px - xlim[0] - row_l*dx) / dx, v = (py - ylim[0] - col_l*dy) / dy;
			double height0;
			if (T_pos & UPPER) //this is upper tooth
			{
				height0 = (1 - u)*(voxel_bot(row_l, col_l)*(1 - v) + voxel_bot(row_l, col_u)*v) + u*(voxel_bot(row_u, col_l)*(1 - v) + voxel_bot(row_u, col_u)*v);
				distance = height0 - p*zdir;
			}
			else //this is lower tooth
			{
				height0 = (1 - u)*(voxel_top(row_l, col_l)*(1 - v) + voxel_top(row_l, col_u)*v) + u*(voxel_top(row_u, col_l)*(1 - v) + voxel_top(row_u, col_u)*v);
				distance = p*zdir - height0;
			}
		}
	}
	return distance > thre ? thre : distance;
}

void Tooth::Transform(ON_Xform &matr, bool bracketmove)
{
	m_pMesh->Transform(matr);
	refpnt.Transform(matr);
	centroid.Transform(matr);
	xaxis.Transform(matr);
	yaxis.Transform(matr);
	longaxis.Transform(matr);
	m_xForm = matr*m_xForm;
	if (m_bracket && bracketmove)
		m_bracket->Transform(matr);
	m_bBox = false;
}

void Tooth::Rotate(double r_angle, char axis, bool bracketmove)
{
	ON_3dVector r_axis;
	switch (axis)
	{
	case 'x':
		r_axis = xaxis;
		yaxis.Rotate(r_angle, r_axis);	//angle is in radians
		longaxis.Rotate(r_angle, r_axis);
		break;
	case 'y':
		r_axis = yaxis;
		xaxis.Rotate(r_angle, r_axis);
		longaxis.Rotate(r_angle, r_axis);
		break;
	case 'z':
		r_axis = longaxis;
		xaxis.Rotate(r_angle, r_axis);
		yaxis.Rotate(r_angle, r_axis);
		break;
	default:
		return;
	}
	m_pMesh->Rotate(r_angle, r_axis, centroid);
//	centroid.Rotate(r_angle, r_axis, refpnt);
	refpnt.Rotate(r_angle, r_axis, centroid);
	//update xForm
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Rotation(r_angle, r_axis, centroid);
	m_xForm = temp_form*m_xForm;
	if (m_bracket && bracketmove)
		m_bracket->Rotate(r_angle, r_axis, centroid);
	m_bBox = false;
}

void Tooth::Rotate(double r_angle, ON_3dVector axis, ON_3dPoint origin, bool bracketmove)
{
	xaxis.Rotate(r_angle, axis);
	yaxis.Rotate(r_angle, axis);
	longaxis.Rotate(r_angle, axis);
	m_pMesh->Rotate(r_angle, axis, origin);
	centroid.Rotate(r_angle, axis, origin);
	refpnt.Rotate(r_angle, axis, origin);
	//update xForm
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Rotation(r_angle, axis, origin);
	m_xForm = temp_form*m_xForm;
	if (m_bracket && bracketmove)
		m_bracket->Rotate(r_angle, axis, origin);
	m_bBox = false;
}

void Tooth::Translate(double dx, double dy, double dz, bool bracketmove)
{
	ON_3dVector t(dx, dy, dz);
	m_pMesh->Translate(t);
	refpnt = refpnt + t;
	centroid = centroid + t;
	//update xform
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Translation(t);
	m_xForm = temp_form*m_xForm;
	if (m_bracket && bracketmove)
		m_bracket->Translate(dx, dy, dz);
	m_bBox = false;
}

void Tooth::Translate(ON_3dVector delta, bool bracketmove)
{
	m_pMesh->Translate(delta);
	refpnt = refpnt + delta;
	centroid = centroid + delta;
	//update xform
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Translation(delta);
	m_xForm = temp_form*m_xForm;
	if (m_bracket && bracketmove)
		m_bracket->Translate(delta.x, delta.y, delta.z);
	m_bBox = false;
}

void Tooth::Translate(double dis, char axis, bool bracketmove /*= false*/)
{
	ON_3dVector r_axis;
	switch (axis)
	{
	case 'x':
		r_axis = xaxis;
		Translate(dis * r_axis, bracketmove);
		break;
	case 'y':
		r_axis = yaxis;
		Translate(dis * r_axis, bracketmove);
		break;
	case 'z':
		r_axis = longaxis;
		Translate(dis * r_axis, bracketmove);
		break;
	default:
		return;
	}
}

void Tooth::Align(ON_3dVector de_x, ON_3dVector de_z)
{
	double theta = acos(float(de_z*longaxis));
	if (abs(theta) < 0.0001)
	{
		theta = acos(float(de_x*xaxis));
		if (abs(theta) >= 0.0001)
		{
			ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
			Rotate(theta, tempaxis, refpnt);
		}
	}
	else
	{
		ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(longaxis, de_z);
		Rotate(theta, tempaxis, refpnt);
		theta = acos(float(de_x*xaxis));
		tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
		Rotate(theta, tempaxis, refpnt);
	}
}

void Tooth::AlignToCurve(Curve &incurve, double par)
{
	ON_3dPoint des = incurve.PointAt(par);
	ON_3dVector tang = incurve.TangentAt(par), ori_tang = xaxis - xaxis*incurve.GetZaxis()*incurve.GetZaxis();
	double angle = acos(tang*ori_tang);
	if (ON_CrossProduct(ori_tang, tang)*incurve.GetZaxis() < 0)
		angle = -angle;
	Translate(des - refpnt);
	Rotate(angle, incurve.GetZaxis(), refpnt);
	arch_par = par;
}

void Tooth::Restore()
{
	/*Align(oriXAxis, oriLongAxis);
	Translate(oriPos - refpnt);*/
	ON_Xform inverse_form = this->GetXform().Inverse();
	Transform(inverse_form, true);
}

ON_Xform Tooth::InterpTransform(double par)
{
	//ON_3dVector z0, z1, x0, x1, y0, y1, p0, p1;
	//if (m_xForm_f.IsIdentity())
	//{
	//	z0.Set(0, 0, 1), z1.Set(m_xForm[0][2], m_xForm[1][2], m_xForm[2][2]);
	//	x0.Set(1, 0, 0), x1.Set(m_xForm[0][0], m_xForm[1][0], m_xForm[2][0]);
	//	y0.Set(0, 1, 0);
	//	p0.Set(0, 0, 0), p1.Set(m_xForm[0][3], m_xForm[1][3], m_xForm[2][3]);
	//}
	//else
	//{
	//	z0.Set(0, 0, 1), z1.Set(m_xForm_f[0][2], m_xForm_f[1][2], m_xForm_f[2][2]);
	//	x0.Set(1, 0, 0), x1.Set(m_xForm_f[0][0], m_xForm_f[1][0], m_xForm_f[2][0]);
	//	y0.Set(0, 1, 0);
	//	p0.Set(0, 0, 0), p1.Set(m_xForm_f[0][3], m_xForm_f[1][3], m_xForm_f[2][3]);
	//}
	//p0 = p0*(1 - par) + p1*par; 
	//double theta = acos(float(z0*z1));
	//if (abs(theta) < 0.0001)
	//{
	//	theta = acos(float(x0*x1));
	//	if (abs(theta) >= 0.0001)
	//	{
	//		ON_3dVector tempaxis = ON_CrossProduct(x0, x1);
	//		theta *= par;
	//		x0.Rotate(theta, tempaxis);
	//		y0.Rotate(theta, tempaxis);
	//		z0.Rotate(theta, tempaxis);
	//	}
	//}
	//else
	//{
	//	ON_3dVector tempaxis = ON_CrossProduct(z0, z1);
	//	theta *= par;
	//	x0.Rotate(theta, tempaxis);
	//	y0.Rotate(theta, tempaxis);
	//	z0.Rotate(theta, tempaxis);
	//	theta = acos(float(x0*x1));
	//	theta *= par;
	//	tempaxis = ON_CrossProduct(x0, x1);
	//	x0.Rotate(theta, tempaxis);
	//	y0.Rotate(theta, tempaxis);
	//	z0.Rotate(theta, tempaxis);
	//}
	//return ON_Xform(p0, x0, y0, z0);

	if (par == 1)
	{
		if (m_xForm_f.IsIdentity())
			return m_xForm;
		else
			return m_xForm_f;
	}
	ON_3dVector z0, z1, x0, x1;
	ON_3dPoint p0, p1;
	ON_Xform interform(1), inverse_form, temp_form;
	if (m_xForm_f.IsIdentity())
	{
		inverse_form = this->GetXform().Inverse();
		z0 = longaxis, z1 = longaxis;
		z0.Transform(inverse_form);
		x0 = xaxis, x1 = xaxis;
		x0.Transform(inverse_form);
		p0 = centroid, p1 = centroid;
		p0.Transform(inverse_form);
	}
	else
	{
		inverse_form = m_xForm;
		inverse_form.Inverse();
		z0 = longaxis;
		z0.Transform(inverse_form);
		z1 = z0;
		z1.Transform(m_xForm_f);
		x0 = xaxis;
		x0.Transform(inverse_form);
		x1 = x0;
		x1.Transform(m_xForm_f);
		p0 = centroid;
		p0.Transform(inverse_form);
		p1 = p0;
		p1.Transform(m_xForm_f);
	}	
	double theta = acos(float(z0*z1));
	if (abs(theta) < 0.0001)
	{
		theta = acos(float(x0*x1));
		if (abs(theta) >= 0.0001)
		{
			ON_3dVector tempaxis = ON_CrossProduct(x0, x1);
			theta *= par;
			interform.Rotation(theta, tempaxis, p0);		
		}
		temp_form.Identity();
		temp_form.Translation((p1 - p0)*par);
		interform = temp_form*interform;
	}
	else
	{
		ON_3dVector tempaxis = ON_CrossProduct(z0, z1);
		tempaxis.Unitize();
		interform.Rotation(theta * par, tempaxis, p0);
		x0.Rotate(theta, tempaxis);
		z0.Rotate(theta * par, tempaxis);
		theta = acos(float(x0*x1));
		if (abs(theta) >= 0.0001)
		{
			theta *= par;
			if (ON_CrossProduct(x0, x1)*z1 > 0)
				tempaxis = z0;
			else
				tempaxis = -z0;
			temp_form.Identity();
			temp_form.Rotation(theta, tempaxis, p0);
			interform = temp_form*interform;
		}		
		temp_form.Identity();
		temp_form.Translation((p1 - p0)*par);
		interform = temp_form*interform;
	}
	return interform;
}

void Tooth::SetStaging(double par)
{
	if (m_xForm_f.IsIdentity())
		m_xForm_f = m_xForm;
	ON_Xform inverse_form = this->GetXform().Inverse();
	ON_Xform staging_form = InterpTransform(par);
	Transform(staging_form*inverse_form);
}

void Tooth::ResetStaging()
{
	ON_Xform inverse_form = this->GetXform().Inverse();
	if (!m_xForm_f.IsIdentity())
		Transform(m_xForm_f*inverse_form);
	else
		Transform(m_xForm*inverse_form);
	m_xForm_f.Identity();
}

void Tooth::GetStagingRange(double &mvmt, double &inc, double &torq)
{
	ON_3dVector z0, z1, x0, x1;
	ON_3dPoint p0, p1;
	ON_Xform interform(1), inverse_form, temp_form;
	if (m_xForm_f.IsIdentity())
	{
		inverse_form = this->GetXform().Inverse();
		z0 = longaxis, z1 = longaxis;
		z0.Transform(inverse_form);
		x0 = xaxis, x1 = xaxis;
		x0.Transform(inverse_form);
		p0 = centroid, p1 = centroid;
		p0.Transform(inverse_form);
	}
	else
	{
		inverse_form = m_xForm;
		inverse_form.Inverse();
		z0 = longaxis;
		z0.Transform(inverse_form);
		z1 = z0;
		z1.Transform(m_xForm_f);
		x0 = xaxis;
		x0.Transform(inverse_form);
		x1 = x0;
		x1.Transform(m_xForm_f);
		p0 = centroid;
		p0.Transform(inverse_form);
		p1 = p0;
		p1.Transform(m_xForm_f);
	}
	mvmt = (p1 - p0).Length();
	inc = acos(float(z0*z1));
	if (abs(inc) < 0.0001)
		torq = acos(float(x0*x1));
	else
	{
		ON_3dVector tempaxis = ON_CrossProduct(z0, z1);
		x0.Rotate(inc, tempaxis);
		torq = acos(float(x0 * x1));
	}
	inc = inc * 180 / PI;
	torq = torq * 180 / PI;
}

void Tooth::CalStagingStep(double mvmt /*= 0.2*/, double ang /*= 1*/, double rot /*= 3*/)
{
	double total_mvmt, total_inc, total_rot;
	GetStagingRange(total_mvmt, total_inc, total_rot);
	staging_step = 1;
	if (total_mvmt != 0)
		staging_step = min(mvmt / total_mvmt, staging_step);
	if (total_inc != 0)
		staging_step = min(ang / total_inc, staging_step);
	if (total_rot != 0)
		staging_step = min(rot / total_rot, staging_step);
}

void Tooth::SetMesh(FEntity* in)
{
	m_pMesh = new ON_Mesh;
	*m_pMesh = *((FBody*)in)->GetMesh();
}

void Tooth::SetTree()
{
	m_tree = new NodeTree3D(m_pMesh);
}

void Tooth::AddBracket(FEntity* in)
{
	//if (m_bracket)
	//{
	//	delete	m_bracket;
	//	m_bracket = NULL;
	//}		
	m_bracket = new Bracket;
	m_bracket->SetMesh(in);
	m_bracket->CalNormals();
	m_bracket->InitDisplayList();
	m_bracket->m_tooth = this;
}

void Tooth::RemoveBracket()
{
	/*if (m_bracket)
	{
		delete m_bracket;
		m_bracket = NULL;
	}*/
	m_bracket = NULL;
}

void Tooth::Update()
{
	FT::FEntity::Update();
	if (!m_bMeshed)
	{
		//put remesh code here
	}
	if (!m_bocclusal)
	{
		InitOcclusalDisp();
	}
}

void Tooth::DrawPoints()
{
	FEntity::DrawPoints();
	if (!m_pMesh)	return;
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	
	if (highlight_on == true)
	{
		GLfloat	highlightcolor[] = { 0.8, 0.2, 0.2 };
		glColor3fv(highlightcolor);
	}
	//glDisable(GL_LIGHTING);

	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPushMatrix();
	/*if (oriXAxis.Length()==0)
	{
		ON_3dVector vv;
		vv = view_par*(refpnt - oriPos);
		glTranslatef(vv.x, vv.y, vv.z);

	}
	else
	{
		DoMapping();

	}*/
	GLdouble mapping[16];
	int n = 0;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			mapping[n] = m_xForm.m_xform[i][j];
			n += 1;
		}
	}
	//glMultMatrixd(*m_xForm.m_xform);
	glMultMatrixd(mapping);

	glCallList(m_uListPoints);
	glPopMatrix();	
	AddDisplayControl();
}

void Tooth::DrawSmoothShade()
{
	if (m_isHide)  return;
	if (!m_pMesh)	return;
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	if (highlight_on == true)
	{
		glColor3f(0.8, 0.2, 0.2);
		
	}
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPushMatrix();
	GLdouble mapping[16];
	ON_Xform i_form = InterpTransform(view_par);
	int n = 0;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			mapping[n] = i_form.m_xform[i][j];
			n += 1;
		}
	}
	//glMultMatrixd(*m_xForm.m_xform);
	glMultMatrixd(mapping);
	if (!occlusal_on || highlight_on)
		glCallList(m_uListSmooth);
	else
		glCallList(m_occlusalSmooth);
	glPopMatrix();
	if (highlight_on)
		AddDisplayControl();
}

void Tooth::DrawFlatShade()
{
	if (!m_pMesh)	return;
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	if (highlight_on == true)
	{
		GLfloat	highlightcolor[] = { 0.8, 0.2, 0.2 };
		glColor3fv(highlightcolor);
	}
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPushMatrix();
	GLdouble mapping[16];
	int n = 0;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			mapping[n] = m_xForm.m_xform[i][j];
			n += 1;
		}
	}
	//glMultMatrixd(*m_xForm.m_xform);
	glMultMatrixd(mapping);
	glCallList(m_uListFlat);
	glPopMatrix();
	AddDisplayControl();
}

void Tooth::DrawMesh()
{
	if (!m_pMesh)	return;
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	if (highlight_on == true)
	{
		GLfloat	highlightcolor[] = { 0.8, 0.2, 0.2 };
		glColor3fv(highlightcolor);
	}
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0, 1.0);

	glEnable(GL_DEPTH_TEST);
	glPushMatrix();
	GLdouble mapping[16];
	int n = 0;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			mapping[n] = m_xForm.m_xform[i][j];
			n += 1;
		}
	}
	//glMultMatrixd(*m_xForm.m_xform);
	glMultMatrixd(mapping);
	glCallList(m_uListSmooth);
	glPopMatrix();

	glPolygonOffset(0.0f, 0.0f);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor3f(0.0f, 0.0f, 0.0f);
	glPushMatrix();

	glMultMatrixd(mapping);
	glCallList(m_uListMesh);
	glPopMatrix();

	AddDisplayControl();
}

void Tooth::InitDisplayList()
{
	if (NULL == m_pMesh)	return;
	m_bDispList = true;
	if (glIsList(tempList))
		glDeleteLists(tempList, 6);
	tempList = glGenLists(6);
	F3dPoint v[4];
	F3fVector n; 
	int face_count = m_pMesh->FaceCount();
	glNewList(tempList, GL_COMPILE);
	glBegin(GL_TRIANGLES);
	for (int fi = 0; fi < face_count; fi++)
	{
		const ON_MeshFace& f = m_pMesh->m_F[fi];
		v[0] = m_pMesh->m_V[f.vi[0]];
		v[1] = m_pMesh->m_V[f.vi[1]];
		v[2] = m_pMesh->m_V[f.vi[2]];
		n = m_pMesh->m_FN[fi];
		glNormal3d(n.x, n.y, n.z);	//face normal
		for (int i = 0; i < 3; i++)
		{
			glVertex3d(v[i].x, v[i].y, v[i].z);
		}
	}
	glEnd();
	glEndList();

	//drawflatshade list
	m_uListFlat = tempList + 1;
	glNewList(m_uListFlat, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glCallList(tempList);
	glEndList();

	//draw smooth shade list
	m_uListSmooth = tempList + 2;
	glNewList(m_uListSmooth, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	ON_GL(*m_pMesh);
	glEndList();

	//draw mesh list
	m_uListMesh = tempList + 3;
	glNewList(m_uListMesh, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);         // Draw Mesh
	glCallList(tempList);

	//added by ZR for vertex normal vector display
	int nvsize = m_pMesh->m_N.Count();	//added by ZR for normal vector display
	float nl=10;	//normal vector length
	F3fVector nv; //vertex
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	//glLineWidth(5);
	for (int i = 0; i < nvsize; i++)
	{
		n = m_pMesh->m_N[i];
		nv = m_pMesh->m_V[i];
		glVertex3d(nv.x, nv.y, nv.z);	//vertex
		glVertex3d( (nv+nl*n).x, (nv+nl*n).y, (nv+nl*n).z );	//end of normal vector on a vertex		
	}
	glEnd();
	glColor3f(m_red, m_green, m_blue);
	//added by ZR end

	glEndList();

	//draw point list
	int nsize = m_pMesh->m_V.Count();
	m_uListPoints = tempList + 4;
	glNewList(m_uListPoints, GL_COMPILE);
	glPointSize(3.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < nsize; i++)
	{
		n = m_pMesh->m_N[i];
		glNormal3d(n.x, n.y, n.z);	//face normal
		glVertex3d(m_pMesh->m_V[i].x, m_pMesh->m_V[i].y, m_pMesh->m_V[i].z);
	}
	glEnd();
	glEndList();
	
}

void Tooth::CalBoundBox()
{
	if (!m_pMesh)	return;
	double minPnt[3], maxPnt[3];
	if (m_pMesh->GetBBox(minPnt, maxPnt))
	{
		m_entBox.SetMinPoint(F3dPoint(minPnt));
		m_entBox.SetMaxPoint(F3dPoint(maxPnt));
	}
	m_bBox = true;
}

void Tooth::AddDisplayControl()
{
	// added by ZR for Architecture project

	float nl = 100;	//normal length
	int i = 0;	//index of vertex/facet
	F3fVector n;	//normal of vertex

	//display according to the index of facet
	/*
	F3dPoint v[2];	//vertex of facet
	const ON_MeshFace& ff = m_pMesh->m_F[i];
	v[0] = m_pMesh->m_V[ff.vi[0]];	//current vertex
	v[1] = m_pMesh->m_V[ff.vi[1]];
	n = m_pMesh->m_N[ff.vi[0]];
	glDisable(GL_LIGHTING);
	glColor3f(0, 1, 1);
	glPointSize(10);
	glBegin(GL_POINTS);
	glNormal3f(n.x, n.y, n.z);
	glVertex3f(v[0].x, v[0].y, v[0].z);
	glEnd();
	glLineWidth(5);
	glBegin(GL_LINES);
	glVertex3f(v[0].x, v[0].y, v[0].z);
	glVertex3f((v[0] + nl*n).x, (v[0] + nl*n).y, (v[0] + nl*n).z);	//normal on the vertex
	glColor3f(0, 1, 0);
	glVertex3f(v[0].x, v[0].y, v[0].z);	//first edge on the vertex
	glVertex3f(v[1].x, v[1].y, v[1].z);
	glEnd();
	*/

	//display according to the index of vertex
	F3fVector pv;	//vertex
	n = m_pMesh->m_N[i];
	pv = m_pMesh->m_V[i];
	//display normal of the first vertex
	/*glDisable(GL_LIGHTING);
	glColor3f(0, 1, 1);
	glPointSize(10);
	glBegin(GL_POINTS);
	glNormal3f(n.x, n.y, n.z);
	glVertex3f(pv.x, pv.y, pv.z);
	glEnd();
	glLineWidth(5);
	glBegin(GL_LINES);
	glVertex3f(pv.x, pv.y, pv.z);
	glVertex3f((pv + nl*n).x, (pv + nl*n).y, (pv + nl*n).z);
	glEnd();*/

	//display reference vector
	/*glDisable(GL_LIGHTING);
	glColor3f(0, 1, 0);
	glLineWidth(5);
	glBegin(GL_LINES);
	glVertex3f(pv.x, pv.y, pv.z);
	glVertex3f((pv + nl*refvec).x, (pv + nl*refvec).y, (pv + nl*refvec).z);
	glEnd();
	glColor3f(m_red, m_green, m_blue);
	glEnable(GL_LIGHTING);*/

	//display topoNode
	if (tpNodeDisplay)
	{
		//for rolling and flood-fill check
		if (1)	//is_rolled
		{
			double xx, yy, zz;
			FMesh::VertexIter v_it, v_end(m_oMesh->vertices_end());
			glPointSize(10);
			glDisable(GL_LIGHTING);
			glColor3f(1, 0, 0);
			for (v_it = m_oMesh->vertices_begin(); v_it != v_end; ++v_it)
			{
				if ( (m_oMesh->data(*v_it).patch) >0)
				{ 
					auto ptemp = m_oMesh->point(*v_it);
					xx = ptemp.values_[0];
					yy = ptemp.values_[1];
					zz = ptemp.values_[2];				
					glBegin(GL_POINTS);
					glVertex3f(xx, yy, zz);
					glEnd();
				}
			}

			if (0)	//half-edge display check
			{
				FMesh::HalfedgeHandle eh;
				glEnable(GL_POLYGON_OFFSET_LINE);
				glPolygonOffset(1, 5);
				glLineWidth(3);
				glColor3f(1, 0, 1);
				for (int ii = 0; ii < tpNode.size(); ii++)
				{
					for (int he = 0; he < tpNode[ii].roll_heh.size(); he++)
					{
						eh = tpNode[ii].roll_heh[he];
						auto vi_f = m_oMesh->from_vertex_handle(eh);
						ON_3dPoint pa = m_oMesh->point(vi_f).data();
						auto vi_t = m_oMesh->to_vertex_handle(eh);
						ON_3dPoint pb = m_oMesh->point(vi_t).data();
						glBegin(GL_LINES);
						glVertex3f(pa.x, pa.y, pa.z);
						glVertex3f(pb.x, pb.y, pb.z);
						glEnd();
					}
				}
			}

		}

		if (0) //rolling index check
		{
			ON_3dPoint pt;
			glPointSize(20);
			glDisable(GL_LIGHTING);
			float clr;
			int total_node = m_oMesh->n_vertices();
			float rp;	//rolling percentage
			float ri;	//rolling index

			for (int ii = 0; ii < tpNode.size(); ii++)
			{
				ri = tpNode[ii].rollover;
				rp = (float)ri / (float)total_node;

				//color for visual check
			/*	auto aa = 1 * (1 - rp);				
				glColor3f(std::pow(aa,1), std::sqrt((0.25-(aa-0.5)*(aa-0.5))), std::pow(rp,1));*/
				if (rp <= 0.33)
				{
					glColor3f(1, 0, 0);
				}
				else if (rp >= 0.33 && rp <= 0.66)
				{
					glColor3f(0, 1, 0);
				}
				else
				{
					glColor3f(0, 0, 1);
				}

				pt = tpNode[ii].point;
				glBegin(GL_POINTS);
				glVertex3f(pt.x, pt.y, pt.z);
				glEnd();

			}
		}

		//RANSAC check
		#pragma region
		if (1)
		{
			glPointSize(20);
			for (int ii = 0; ii < tpNode.size(); ii++)
			{
				int ransactype = tpNode[ii].primType;
				int ransacidx = tpNode[ii].patch_idx;	//i-th patch
				ransactype = ransacidx;
							
				if (ransactype == 1)	//plane
				{
					glColor3f(1, 0, 0);
				}
				else if (ransactype == 2)	//cylinder
				{
					glColor3f(0, 1, 0);
				}
				else if (ransactype == 3)	//cone
				{
					glColor3f(0, 0, 1);
				}
				else if (ransactype == 4)
				{
					glColor3f(1, 0, 1);
				}
				else if (ransactype == 5)
				{
					glColor3f(0.8, 0.8, 0.2);
				}
				else if (ransactype == 0)	//not included
				{
					glColor3f(0.3, 0.8, 0.3);
				}

				ON_3dPoint pt = tpNode[ii].point;
				glBegin(GL_POINTS);
				glVertex3f(pt.x, pt.y, pt.z);
				glEnd();
			}

			//patch boundary
			int sz = Patch.size();
			if (sz > 1)
			{
				glDisable(GL_LIGHTING);
				glLineWidth(5);
				glEnable(GL_POLYGON_OFFSET_LINE);
				glPolygonOffset(1, 6);
				//End boundary
				for (int i = 0; i < sz-1; i++)
				{
					//Patch boundary
					ON_3dPoint pa, pb;
					pa = Patch[i].BoundaryLines[2];
					pb = Patch[i].BoundaryLines[3];
					glColor3f(1, 0, 0);
					glBegin(GL_LINES);
					glVertex3f(pa.x, pa.y, pa.z);
					glVertex3f(pb.x, pb.y, pb.z);
					glEnd();
					glColor3f(0.8, 0.2, 0.8);
					glBegin(GL_POINTS);
					glVertex3f(pa.x, pa.y, pa.z);
					glEnd();
					//sample points on transition region
					int brysz = Patch[i].EndBoundaryPoints.Count();
					if (brysz < 1)
						continue;
					for (int j = 0; j < brysz; j++)
					{
						glColor3f(0, 0, 0);
						pa = Patch[i].EndBoundaryPoints[j];
						glBegin(GL_POINTS);
						glVertex3f(pa.x, pa.y, pa.z);
						glEnd();
					}				
				}
				//Start boundary
				for (int i = 1; i < sz; i++)
				{
					//Patch boundary
					ON_3dPoint pa, pb;
					pa = Patch[i].BoundaryLines[0];
					pb = Patch[i].BoundaryLines[1];
					glColor3f(1, 0, 0);
					glBegin(GL_LINES);
					glVertex3f(pa.x, pa.y, pa.z);
					glVertex3f(pb.x, pb.y, pb.z);
					glEnd();
					glColor3f(0.8, 0.2, 0.8);
					glBegin(GL_POINTS);
					glVertex3f(pa.x, pa.y, pa.z);
					glEnd();
					//sample points on transition region
					int brysz = Patch[i].StartBoundaryPoints.Count();
					if (brysz < 1)
						continue;
					for (int j = 0; j < brysz; j++)
					{
						glColor3f(0, 0, 0);
						pa = Patch[i].StartBoundaryPoints[j];
						glBegin(GL_POINTS);
						glVertex3f(pa.x, pa.y, pa.z);
						glEnd();
					}
				}
				glDisable(GL_POLYGON_OFFSET_LINE);
			}

		}
		#pragma endregion
		

		//Planar check
		#pragma region
		if (0)
		{
			for (int i = 0; i < tpNode.size(); i++)
			{
				if (tpNode[i].primType == 1)
				{
					glPointSize(20);
					glColor3f(1, 1, 1);
					glBegin(GL_POINTS);
					glVertex3f(tpNode[i].point.x, tpNode[i].point.y, tpNode[i].point.z);
					glEnd();
				}
			}

		}		
		#pragma endregion

		glPointSize(20);
		for (int i = 0; i < tpNode.size(); i++)
		{
			////display the node.
			//glDisable(GL_LIGHTING);
			//glBegin(GL_POINTS);
			//glColor3f(1, 0, 0);
			//glVertex3f(tpNode[i].point.x, tpNode[i].point.y, tpNode[i].point.z);
			////display the initial intersection facet. for test only
			///*if (tpNode[i].next_node.size())
			//{
			//glColor3f(0, 1, 0);
			//glVertex3f(tpNode[i].next_node[0].x, tpNode[i].next_node[0].y, tpNode[i].next_node[0].z);
			//glColor3f(0, 0, 1);
			//glVertex3f(tpNode[i].next_node[1].x, tpNode[i].next_node[1].y, tpNode[i].next_node[1].z);
			//}*/
			//glEnd();

			//display the point normal
			/*glColor3f(0, 1, 1);
			glDisable(GL_LIGHTING);
			glLineWidth(5);
			glBegin(GL_LINES);
			glVertex3f(tpNode[i].point.x, tpNode[i].point.y, tpNode[i].point.z);
			glVertex3f((tpNode[i].point + nl*tpNode[i].normal).x, (tpNode[i].point + nl*tpNode[i].normal).y, (tpNode[i].point + nl*tpNode[i].normal).z);
			glEnd();*/

			//plane normal
			/*glBegin(GL_LINES);
			glColor3f(0, 1, 0);
			glVertex3f(tpNode[i].point.x, tpNode[i].point.y, tpNode[i].point.z);
			glVertex3f((tpNode[i].point + nl*tpNode[i].plane_nor).x, (tpNode[i].point + nl*tpNode[i].plane_nor).y, (tpNode[i].point + nl*tpNode[i].plane_nor).z);
			glEnd();*/

			//rolling direction
			if (tpNode[i].huntting_done)
			{
				glDisable(GL_LIGHTING);
				glLineWidth(5);
				glColor3f(0, 0, 1);
				glBegin(GL_LINES);
				glVertex3f(tpNode[i].point.x, tpNode[i].point.y, tpNode[i].point.z);
				glVertex3f((tpNode[i].point + 0.5*nl*tpNode[i].best_plane_nor).x, (tpNode[i].point + 0.5*nl*tpNode[i].best_plane_nor).y, (tpNode[i].point + 0.5*nl*tpNode[i].best_plane_nor).z);
				glEnd();
			}

			for (int int_num = 0; int_num < tpNode[i].intersectCurve.size(); int_num++)
			{

				//////display of intersection curve, if any//////
				if ((tpNode[i].intersectCurve[int_num])->is_intersect)
				{
					ON_3dPoint pt;
					bool endpointdisp = 0;

					//display intersection points
					/*glEnable(GL_POLYGON_OFFSET_POINT);
					glPolygonOffset(1, 6);
					glPointSize(8);
					glBegin(GL_POINTS);
					glColor3f(1, 0, 0);//glColor3f(0, 1, 0.5);//glColor3f(0.87, 0.19, 0.39);
					for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curve.Count(); cp++)
					{
						pt = (tpNode[i].intersectCurve[int_num])->curve[cp];
						glVertex3f(pt.x, pt.y, pt.z);
					}
					glEnd();*/

					//display intersection curve
					glEnable(GL_POLYGON_OFFSET_LINE);
					glPolygonOffset(1, 5);
					glLineWidth(3);
					//glBegin(GL_LINE_STRIP);
					if (int_num == tpNode[i].best_fit_line_index)
					{
						glBegin(GL_LINE_STRIP);
						glColor3f(0, 0, 1);	//best fit line
						for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curve.Count(); cp++)
						{
							pt = (tpNode[i].intersectCurve[int_num])->curve[cp];
							glVertex3f(pt.x, pt.y, pt.z);
						}
						glEnd();
						////end point of single line intersection
						//if (endpointdisp)
						//{
						//glBegin(GL_POINTS);
						//glColor3f(0.2, 0.8, 0.2);	//green
						//glPointSize(1);
						//pt = *((tpNode[i].intersectCurve[int_num])->curve.First());
						//glVertex3f(pt.x, pt.y, pt.z);
						//glEnd();
						//}

						//display the multiple intersection segments. test 20180304
						if ( (tpNode[i].intersectCurve[int_num])->is_intersect_through)
						{
							////raw segment
							//for (int seg = 0; seg < (tpNode[i].intersectCurve[int_num])->curves_seg_raw.size(); seg++)
							//{
							//	glLineStipple(2, 0x00FF); //(4,0xAAAA)
							//	glEnable(GL_LINE_STIPPLE);
							//	glBegin(GL_LINE_STRIP);
							//	glColor3f(1, 0, 0);	//best fit line
							//	for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curves_seg_raw[seg].Count(); cp++)
							//	{
							//		pt = (tpNode[i].intersectCurve[int_num])->curves_seg_raw[seg][cp];
							//		glVertex3f(pt.x, pt.y, pt.z);
							//	}
							//	glEnd();
							//	glDisable(GL_LINE_STIPPLE);
							//}

							//trimmed segment
							for (int seg = 0; seg < (tpNode[i].intersectCurve[int_num])->curves_seg_trim.size(); seg++)
							{
								glBegin(GL_LINE_STRIP);
								glColor3f(0, 0, 1);	//best fit line
								for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curves_seg_trim[seg].Count(); cp++)
								{
									pt = (tpNode[i].intersectCurve[int_num])->curves_seg_trim[seg][cp];
									glVertex3f(pt.x, pt.y, pt.z);
								}
								glEnd();
								//starting points of each segments
								if (endpointdisp)
								{
									glBegin(GL_POINTS);
									glColor3f(0.8, 0.8, 0.2);	//yellow
									glPointSize(1);
									pt = (tpNode[i].intersectCurve[int_num])->curves_seg_trim[seg][0];
									glVertex3f(pt.x, pt.y, pt.z);
									glEnd();
								}
							}
							if (endpointdisp)
							{
								//First point of all segments.
								glBegin(GL_POINTS);
								glColor3f(0.8, 0.2, 0.8);	//purple
								glPointSize(3);
								pt = (tpNode[i].intersectCurve[int_num])->curves_seg_trim[0][0];
								glVertex3f(pt.x, pt.y, pt.z);
								glEnd();
								//PCA
								if ((tpNode[i].intersectCurve[int_num])->is_PCA_done)
								{
									glBegin(GL_POINTS);
									glPointSize(3);
									glColor3f(0, 1, 0);	//start point as green								
									pt = (tpNode[i].intersectCurve[int_num])->fitted_line[0];
									glVertex3f(pt.x, pt.y, pt.z);
									glColor3f(0, 0, 1);	//end point as blue
									pt = (tpNode[i].intersectCurve[int_num])->fitted_line[1];
									glVertex3f(pt.x, pt.y, pt.z);
									glEnd();
								}
							}
						}

					}
					/*else //Other intersection lines
					{
						glBegin(GL_LINE_STRIP);
						glColor3f(0, 1, 0);	//glColor3f(0.2, 0.8, 0.2); //glColor3f(0, 0.5, 1);	
						for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curve.Count(); cp++)
						{
							pt = (tpNode[i].intersectCurve[int_num])->curve[cp];
							glVertex3f(pt.x, pt.y, pt.z);
						}
						glEnd();

						if ((tpNode[i].intersectCurve[int_num])->is_intersect_through)
						{
							//raw segment
							for (int seg = 0; seg < (tpNode[i].intersectCurve[int_num])->curves_seg_raw.size(); seg++)
							{
								glLineStipple(2, 0x00FF); //(4,0xAAAA)
								glEnable(GL_LINE_STIPPLE);
								glBegin(GL_LINE_STRIP);
								glColor3f(0, 1, 0);	//best fit line
								for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curves_seg_raw[seg].Count(); cp++)
								{
									pt = (tpNode[i].intersectCurve[int_num])->curves_seg_raw[seg][cp];
									glVertex3f(pt.x, pt.y, pt.z);
								}
								glEnd();
								glDisable(GL_LINE_STIPPLE);
							}

							//trimmed segment
							for (int seg = 0; seg < (tpNode[i].intersectCurve[int_num])->curves_seg_trim.size(); seg++)
							{
								glBegin(GL_LINE_STRIP);
								glColor3f(0, 1, 0);	//best fit line
								for (int cp = 0; cp < (tpNode[i].intersectCurve[int_num])->curves_seg_trim[seg].Count(); cp++)
								{
									pt = (tpNode[i].intersectCurve[int_num])->curves_seg_trim[seg][cp];
									glVertex3f(pt.x, pt.y, pt.z);
								}
								glEnd();
								//starting points of each segments
								if (endpointdisp)
								{
									glBegin(GL_POINTS);
									glColor3f(0.8, 0.8, 0.2);
									glPointSize(1);
									pt = (tpNode[i].intersectCurve[int_num])->curves_seg_trim[seg][0];
									glVertex3f(pt.x, pt.y, pt.z);
									glEnd();
								}
							}

						}
					}*/						
				}
			}

			if (int clip_plane_display = 0)
			{
				for (int int_num = 0; int_num < tpNode[i].intersectCurve.size(); int_num++)
				{
					//////clipping plane/////
					ON_3dVector n, pos;
					//n = tpNode[i].plane_nor;
					n = (tpNode[i].intersectCurve[int_num])->curveplane.GetNormal();
					n.Unitize();
					pos = tpNode[i].point;
					//plane normal
					/*glBegin(GL_LINES);
					glColor3f(0, 1, 0);
					glVertex3f(tpNode[i].point.x, tpNode[i].point.y, tpNode[i].point.z);
					glVertex3f((tpNode[i].point + nl*n).x, (tpNode[i].point + nl*n).y, (tpNode[i].point + nl*n).z);
					glEnd();*/

					////show clipping plane and clipping effect////
					//float xx = n.x, yy = n.y, zz = n.z, aa;
					////if (!clipDir)
					//if (1)
					//{
					//	xx = -xx, yy = -yy, zz = -zz;
					//}
					//aa = 0 - xx*pos.x - yy*pos.y - zz*pos.z;
					//GLdouble eqn[4] = { xx, yy, zz, aa };	//Ax+By+Cz+D=0, ABC is normal.
					//glClipPlane(GL_CLIP_PLANE0, eqn);
					//glEnable(GL_CLIP_PLANE0);

					//////visual of clipping plane//////
					ON_3dVector globalZ, up, side, upl, upr, lowl, lowr;	//up direction and 4 points of the rectangular.
					globalZ.Set(0, 0, 1);
					ON_3dVector tempt = ON_CrossProduct(n, globalZ);
					//up = ON_CrossProduct(tempt, n);
					up = tpNode[i].normal;
					up.Unitize();
					side = ON_CrossProduct(up, n);
					side.Unitize();
					int height = 1350 / 2, width = 3000 / 2;	//35,20
					float offset = 0.05;	//for display
					upl = pos + up*height - side*width + offset*n;
					upr = pos + up*height + side*width + offset*n;
					lowl = pos - up*height - side*width + offset*n;
					lowr = pos - up*height + side*width + offset*n;
					//define appearance.
					if (m_iSelect == 0)
						glColor4f(0.3, 0.3, 0.3, 0.5);	//glColor4f(0.2, 0.2, 0.8, 0.2);
					glEnable(GL_LIGHTING);
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

					glEnable(GL_POLYGON_OFFSET_FILL);
					//glPolygonOffset(1.0, 1.0);
					glEnable(GL_DEPTH_TEST);
					glLineWidth(3);

					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glBegin(GL_QUADS);
					glVertex3f(upl.x, upl.y, upl.z);
					glVertex3f(lowl.x, lowl.y, lowl.z);
					glVertex3f(lowr.x, lowr.y, lowr.z);
					glVertex3f(upr.x, upr.y, upr.z);
					glEnd();

					glColor4f(0.1, 0.3, 0.3, 0.8);
					glBegin(GL_LINE_STRIP);
					glVertex3f(upl.x, upl.y, upl.z);
					glVertex3f(lowl.x, lowl.y, lowl.z);
					glVertex3f(lowr.x, lowr.y, lowr.z);
					glVertex3f(upr.x, upr.y, upr.z);
					glVertex3f(upl.x, upl.y, upl.z);
					glEnd();
				}
			}
		}
		glEnable(GL_LIGHTING);
		glColor3f(m_red, m_green, m_blue);		
	}

	// added by ZR for Architecture project end

	//This commented block is for the original Dental project.
	/* 
	//test sample points display
	//int n = sample_pnts.size();
	//glDisable(GL_LIGHTING);
	//glPointSize(10.0);
	//glBegin(GL_POINTS);
	//for (int i = 0; i < n; i++)
	//{
	//	glVertex3d(m_pMesh->Vertex(sample_pnts[i]).x, m_pMesh->Vertex(sample_pnts[i]).y, m_pMesh->Vertex(sample_pnts[i]).z);
	//}
	//glEnd();
	//glPointSize(1);


	//display reference points

	glDisable(GL_LIGHTING);
	glPointSize(15.0);
	glBegin(GL_POINTS);
	glVertex3d(refpnt.x, refpnt.y, refpnt.z);
	glEnd();
	glPointSize(1);


	//display long axis

	glDisable(GL_LIGHTING);
	glLineWidth(2);
	int halflength = 15;
	glBegin(GL_LINES);
	glVertex3d(centroid.x - halflength*longaxis.x, centroid.y - halflength*longaxis.y, centroid.z - halflength*longaxis.z);
	glVertex3d(centroid.x + halflength*longaxis.x, centroid.y + halflength*longaxis.y, centroid.z + halflength*longaxis.z);
	glEnd();
	glLineWidth(1);

	//draw local coordinate system
	GLfloat length = 5;
	GLUquadric* cone1 = gluNewQuadric();
	GLUquadric* cone2 = gluNewQuadric();

	//draw x-axis
	glColor3f(1, 0, 0);
	CalAngle(xaxis);
	glPushMatrix();
	glTranslatef(centroid.x, centroid.y, centroid.z);
	glRotatef(rotAngle.x, 0, 0, 1);
	glRotatef(rotAngle.y, 0, 1, 0);

	glTranslatef(0, 0, length);
	gluCylinder(cone1, length / 10, 0, length / 3, 10, 3);
	glTranslatef(0, 0, -length);
	gluCylinder(cone2, 0, length / 20, length, 10, 3);
	glPopMatrix();

	//draw y-axis
	glColor3f(0, 1, 0);
	CalAngle(yaxis);
	glPushMatrix();
	glTranslatef(centroid.x, centroid.y, centroid.z);
	glRotatef(rotAngle.x, 0, 0, 1);
	glRotatef(rotAngle.y, 0, 1, 0);

	glTranslatef(0, 0, length);
	gluCylinder(cone1, length / 10, 0, length / 3, 10, 3);
	glTranslatef(0, 0, -length);
	gluCylinder(cone2, 0, length / 20, length, 10, 3);
	glPopMatrix();

	//draw z-axis
	glColor3f(0, 0, 1);
	CalAngle(longaxis);
	glPushMatrix();
	glTranslatef(centroid.x, centroid.y, centroid.z);
	glRotatef(rotAngle.x, 0, 0, 1);
	glRotatef(rotAngle.y, 0, 1, 0);

	glTranslatef(0, 0, length);
	gluCylinder(cone1, length / 10, 0, length / 3, 10, 3);
	glTranslatef(0, 0, -length);
	gluCylinder(cone2, 0, length / 20, length, 10, 3);
	glPopMatrix();
	*/
}

void Tooth::DrawOriginal()
{
	if (m_isHide)  return;
	if (!m_pMesh)	return;
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	if (highlight_on == true)
		glColor3f(0.8, 0.2, 0.2);
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPushMatrix();
	//GLdouble mapping[16];
	//ON_Xform i_form = InterpTransform(0);
	//int n = 0;
	//for (int j = 0; j < 4; j++)
	//{
	//	for (int i = 0; i < 4; i++)
	//	{
	//		mapping[n] = i_form.m_xform[i][j];
	//		n += 1;
	//	}
	//}
	////glMultMatrixd(*m_xForm.m_xform);
	//glMultMatrixd(mapping);
//	if (!occlusal_on || highlight_on)
	glCallList(m_uListSmooth);
	//else
	//glCallList(m_occlusalSmooth);
	glPopMatrix();

	if (highlight_on)
		AddDisplayControl();
}

void Tooth::InitOcclusalDisp()
{
	double occlu_dist = 0.5;
	if (glIsList(m_occlusalSmooth))
		glDeleteLists(m_occlusalSmooth, 1);
	m_occlusalSmooth = tempList + 5;
	glNewList(m_occlusalSmooth, GL_COMPILE);	
	if (paired_tooth == NULL)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glCallList(m_uListSmooth);
	}
	else
	{
		ON_3dPoint v[3];
		ON_3fVector n;
		double dist[3], thres=2;
		float m_redRev = m_red, m_greenRev = m_green, m_blueRev = m_blue;
		int face_count = m_pMesh->FaceCount();
		int vertex_count = m_pMesh->VertexCount();
		vector<float> allDists;
		ON_Xform inverseform = this->GetXform().Inverse();
		ON_Mesh display_mesh = *m_pMesh;
		display_mesh.Transform(inverseform);

		minOccluDist = occlu_dist;
		maxOccluDist = 0;
		for (int vi = 0; vi < vertex_count; ++vi)
		{
			double distance = paired_tooth->OcclusalDistance(m_pMesh->m_V[vi], occlu_dist);
			allDists.push_back(distance);
			if (distance<minOccluDist)
			{
				minOccluDist = distance;
			}
			if (distance>maxOccluDist)
			{
				maxOccluDist = distance;
			}
		}

		glBegin(GL_TRIANGLES);
		for (int fi = 0; fi < face_count; fi++)
		{
			const ON_MeshFace& f = display_mesh.m_F[fi];
			v[0] = display_mesh.m_V[f.vi[0]];
			v[1] = display_mesh.m_V[f.vi[1]];
			v[2] = display_mesh.m_V[f.vi[2]];

			dist[0] = allDists[f.vi[0]];
			dist[1] = allDists[f.vi[1]];
			dist[2] = allDists[f.vi[2]];
			n = display_mesh.m_FN[fi];

			glNormal3d(n.x, n.y, n.z);
			for (int i = 0; i < 3; i++)
			{
				if (dist[i] == occlu_dist)
				{
					glColor3f(m_red, m_green, m_blue);
					glVertex3d(v[i].x, v[i].y, v[i].z);
				}
				else if (dist[i] < 0)
				{
					float fH = 0.0;
					float fS = 1.0;
					float fV = 1.0 - abs(dist[i]) / occlu_dist;
					HSVtoRGB(m_redRev, m_greenRev, m_blueRev, fH, fS, fV);
					glColor3f(m_redRev, m_greenRev, m_blueRev);
					glVertex3d(v[i].x, v[i].y, v[i].z);
				}
				else
				{
					float fH = dist[i] / occlu_dist;
					fH *= 240;
					float fS = 1.0;
					float fV = 1.0;
					HSVtoRGB(m_redRev, m_greenRev, m_blueRev, fH, fS, fV);
					glColor3f(m_redRev, m_greenRev, m_blueRev);
					glVertex3d(v[i].x, v[i].y, v[i].z);
				}

			}
		}
		glEnd();
		//glBegin(GL_TRIANGLES);
		//for (int fi = 0; fi < face_count; fi++)
		//{
		//	const ON_MeshFace& f = display_mesh.m_F[fi];
		//	v[0] = display_mesh.m_V[f.vi[0]];
		//	dist[0] = paired_tooth->OcclusalDistance(m_pMesh->m_V[f.vi[0]], occlu_dist);
		//	v[1] = display_mesh.m_V[f.vi[1]];
		//	dist[1] = paired_tooth->OcclusalDistance(m_pMesh->m_V[f.vi[1]], occlu_dist);
		//	v[2] = display_mesh.m_V[f.vi[2]];
		//	dist[2] = paired_tooth->OcclusalDistance(m_pMesh->m_V[f.vi[2]], occlu_dist);
		//	n = display_mesh.m_FN[fi];

		//	glNormal3d(n.x, n.y, n.z);
		//	for (int i = 0; i < 3; i++)
		//	{
		//		if (dist[i] == occlu_dist)
		//		{
		//			glColor3f(m_red, m_green, m_blue);
		//			glVertex3d(v[i].x, v[i].y, v[i].z);
		//		}
		//		else if (dist[i]<0)
		//		{
		//			float fH = 0.0;
		//			float fS = 1.0;
		//			float fV = 1.0 - abs(dist[i]) / occlu_dist;
		//			HSVtoRGB(m_redRev, m_greenRev, m_blueRev, fH, fS, fV);
		//			glColor3f(m_redRev, m_greenRev, m_blueRev);
		//			glVertex3d(v[i].x, v[i].y, v[i].z);
		//		}
		//		else
		//		{
		//			float fH = dist[i] / occlu_dist;
		//			fH *= 240;
		//			float fS = 1.0;
		//			float fV = 1.0;
		//			HSVtoRGB(m_redRev, m_greenRev, m_blueRev, fH, fS, fV);
		//			//glColor3f(m_red, m_green * dist[i] / occlu_dist, m_blue *  dist[i] / occlu_dist);
		//			glColor3f(m_redRev, m_greenRev, m_blueRev);
		//			glVertex3d(v[i].x, v[i].y, v[i].z);
		//		}
		//		
		//	}
		//}
		//glEnd();
	}	
	glEndList();
	occlusal_on = true;
	m_bocclusal = true;
}

Curve::Curve(const ON_3dPointArray &pa)
{
	m_iEntType = CURVE;
	int n = pa.Count();
	m_T.resize(n);
	m_T[0] = 0;
	double total_length = 0;
	for (int i = 1; i < n; i++)
	{
		total_length += (pa[i] - pa[i - 1]).Length();
		m_T[i] = total_length;
	}
	for (int i = 0; i < n; i++)
	{
		m_T[i] = m_T[i] / m_T[n - 1];
	}

	m_Curve.resize(n - 1);
	vector<ON_3dVector> D_1(n), d(n);
	d[0] = 3 * (pa[1] - pa[0]);
	d[n - 1] = 3 * (pa[n - 1] - pa[n - 2]);
	for (int i = 1; i < n-1; i++)
	{
		d[i] = 3 * (pa[i + 1] - pa[i - 1]);
	}
	//initialize coefficients (thomas algorithm)
	vector<double> c(n - 1, 1), b(n, 4), a(n - 1, 1);
	b[0] = 2;
	b[n - 1] = 2;

	c[0] = c[0] / b[0];
	d[0] = d[0] / b[0];

	for (int i = 1; i < n - 1; i++)
	{
		c[i] = c[i] / (b[i] - a[i - 1] * c[i - 1]);
		d[i] = (d[i] - a[i - 1] * d[i - 1]) / (b[i] - a[i - 1] * c[i - 1]);
	}
	d[n - 1] = (d[n - 1] - a[n - 2] * d[n - 2]) / (b[n - 1] - a[n - 2] * c[n - 2]);

	D_1[n - 1] = d[n - 1];
	for (int i = n-2; i >=0; i--)
	{
		D_1[i] = d[i] - c[i] * d[i + 1];
	}

	for (int i = 0; i < n - 1; i++)
	{
		ON_3dPointArray cv(4);
		cv.Insert(0, pa[i]);
		cv.Insert(1, pa[i]+D_1[i]/3);
		cv.Insert(2, pa[i+1]-D_1[i+1]/3);
		cv.Insert(3, pa[i+1]);
		m_Curve[i] = ON_BezierCurve(cv);
	}
	//
	//if (n > 2)
	//{
	//	int mid = floor(n / 2);
	//	ON_3dVector v = ON_DECL::ON_CrossProduct(pa[0] - pa[n - 1], pa[mid] - (pa[0] + pa[n - 1]) / 2);
	//	if (v.z <= 0)
	//	{
	//		ON_3dPointArray pb;
	//		pb.Reserve(n);
	//		for (int i = 0; i < n; i++)
	//			pb.Insert(i, pa[n - i - 1]);
	//		m_pCurve.CreateClampedUniformNurbs(3, n, n, pb);
	//		std:reverse(m_t.begin(), m_t.end());
	//	}
	//	else
	//		m_pCurve.CreateClampedUniformNurbs(3, n, n, pa);
	//}
	
	origin = (pa[0] + pa[n - 1]) / 2;
	xaxis = pa[n - 1] - pa[0];
	xaxis.Unitize();
	zaxis = ON_DECL::ON_CrossProduct(xaxis, origin - PointAt(0.5));
	zaxis.Unitize();
	yaxis = ON_DECL::ON_CrossProduct(zaxis, xaxis);	
	highlight_on = false;
	clipplane_on = false;
	clipPos = 0;
	clipDir = 1;
	m_Teeth = NULL;
	disp_option = 0;
	width = 2;
}

Curve::Curve()
{
	highlight_on = false;
	clipplane_on = false;
	clipPos = 0;
	clipDir = 1;
	m_Teeth = NULL;
}

void Curve::SetTeeth(Teeth *t)
{
	m_Teeth = t;
	int n = m_T.size();
	for (int i = 0; i < n; i++)
		m_Teeth->GetTooth(i)->arch_par = m_T[i];
}

vector<ON_3dPoint> Curve::GetKnotPoints()
{
	//int n = m_t.size();
	//ON_3dPointArray pa;
	//ON_3dPoint onePt;
	//for (int i = 0; i < n - 1; i++)
	//{
	//	onePt = m_Curve[i].PointAt(0);
	//	pa.Insert(i,onePt);
	//}
	//onePt = m_Curve[n - 2].PointAt(1);
	//pa.Insert(n - 1, onePt);
	//return pa;

	int n = m_T.size();
	vector<ON_3dPoint> pa(n);
	for (int i = 0; i < n; i++)
		pa[i] = PointAt(m_T[i]);
	return pa;
}

ON_3dPoint Curve::PointAt(double t)
{
	int i;
	for (i = 0; i < m_T.size(); i++)
	{
		if (t < m_T[i]) 
			break;
	}
	if (i == 0) //t<0
	{
		double length = (m_Curve[0].PointAt(1) - m_Curve[0].PointAt(0)).Length() * (-t / m_T[1]);
		return m_Curve[0].PointAt(0) - m_Curve[0].TangentAt(0) * length;
	}
	if (i == m_T.size()) i--;		//t>=1
	return m_Curve[i - 1].PointAt((t - m_T[i - 1]) / (m_T[i] - m_T[i - 1]));	
}

ON_3dVector Curve::TangentAt(double t)
{
	//int i;
	//for (i = 0; i < m_T.size(); i++)
	//{
	//	if (t < m_T[i])
	//		break;
	//}
	//if (i==0) //t<0
	//{
	//	ON_3dVector ta = m_Curve[0].TangentAt(t / m_T[1]);
	//	ta.Unitize();
	//	return ta;
	//}
	//if (i == m_T.size()) i--;		//t>=1
	//ON_3dVector ta = m_Curve[i - 1].TangentAt((t - m_T[i - 1]) / (m_T[i] - m_T[i - 1]));
	//ta.Unitize();
	//return ta;
	if (t < 0)
	{
		ON_3dVector ta = m_Curve[0].TangentAt(t / m_T[1]);
		ta.Unitize();
		return ta;
	}
	else
	{
		ON_3dVector ta = PointAt(t + 0.01) - PointAt(t - 0.01);
		ta.Unitize();
		return ta;
	}
}

double Curve::CurvatureAt(double t)
{
	int i;
	for (i = 0; i < m_T.size(); i++)
	{
		if (t < m_T[i])
			break;
	}
	if (i == 0) //t<0
	{
		ON_3dVector ka = m_Curve[0].CurvatureAt(t / m_T[1]);
		double l = ka.Length();
		return l;
	}
	if (i == m_T.size()) i--;		//t>=1
	ON_3dVector ka = m_Curve[i - 1].CurvatureAt((t - m_T[i - 1]) / (m_T[i] - m_T[i - 1]));
	double l = ka.Length();
	return l;
}

ON_3dVector Curve::NormalAt(double t)
{
	int i;
	for (i = 0; i < m_T.size(); i++)
	{
		if (t < m_T[i])
			break;
	}
	if (i == 0) //t<0
	{
		ON_3dVector ka = m_Curve[0].CurvatureAt(t / m_T[1]);
		ka.Unitize();
		return ka;
	}
	if (i == m_T.size()) i--;		//t>=1
	ON_3dVector ka = m_Curve[i - 1].CurvatureAt((t - m_T[i - 1]) / (m_T[i] - m_T[i - 1]));
	ka.Unitize();
	return ka;
}

double Curve::FindParameter(double par_in, double dis)
{
	double par_out, dis_i;
	ON_3dPoint p_in = PointAt(par_in);
	if (dis == 0)
		return par_in;
	if (dis < 0)
	{
		dis = -dis;
		double par_l = 0 < par_in - 0.2 ? 0 : par_in - 0.2, par_u = par_in;
		for (int i = 0; i < 10; i++)
		{
			par_out = (par_l + par_u) / 2;
			dis_i = (p_in - PointAt(par_out)).Length();
			if (dis_i > dis) //too far away
				par_l = par_out;
			else            // too close
				par_u = par_out;
		}
		return par_out;
	}
	else
	{
		double par_l = par_in, par_u = 1 > par_in + 0.2 ? 1 : par_in + 0.2;
		for (int i = 0; i < 10; i++)
		{
			par_out = (par_l + par_u) / 2;
			dis_i = (p_in - PointAt(par_out)).Length();
			if (dis_i > dis) //too far away
				par_u = par_out;
			else            // too close
				par_l = par_out;
		}
		return par_out;
	}
}

void Curve::Rotate(double r_angle, char axis)
{
	switch (axis)
	{
	case 'x':
		Rotate(r_angle, xaxis, origin);
		break;
	case 'y':
		Rotate(r_angle, yaxis, origin);
		break;
	case 'z':
		Rotate(r_angle, zaxis, origin);
		break;
	default:
		return;
	}	
}

void Curve::Rotate(double r_angle, ON_3dVector axis, ON_3dPoint p)
{
	for (int i = 0; i < m_Curve.size(); i++)
		m_Curve[i].Rotate(r_angle, axis, p);
	xaxis.Rotate(r_angle, axis);
	yaxis.Rotate(r_angle, axis);
	zaxis.Rotate(r_angle, axis);
	origin.Rotate(r_angle, axis, p);
	if (m_Teeth)
	{
		int n = m_Teeth->GetTeethlist().size();
		for (int i = 0; i < n; i++)
		{
			if (m_Teeth->GetTooth(i)->m_bracket)
				m_Teeth->GetTooth(i)->m_bracket->Rotate(r_angle, axis, p);
		}
	}
}

void Curve::Translate(double x, double y, double z)
{
	ON_3dVector v(x, y, z);
	for (int i = 0; i < m_Curve.size(); i++)
	{
		m_Curve[i].Translate(v);
	}	
	origin = origin + v;
	if (m_Teeth)
	{
		int n = m_Teeth->GetTeethlist().size();
		for (int i = 0; i < n; i++)
		{
			if (m_Teeth->GetTooth(i)->m_bracket)
				m_Teeth->GetTooth(i)->m_bracket->Translate(x, y, z);
		}
	}
}

void Curve::Translate(ON_3dVector v)
{
	for (int i = 0; i < m_Curve.size(); i++)
	{
		m_Curve[i].Translate(v);
	}
	origin = origin + v;
	if (m_Teeth)
	{
		int n = m_Teeth->GetTeethlist().size();
		for (int i = 0; i < n; i++)
		{
			if (m_Teeth->GetTooth(i)->m_bracket)
				m_Teeth->GetTooth(i)->m_bracket->Translate(v.x, v.y, v.z);
		}
	}
}

void Curve::Align(ON_3dVector de_x, ON_3dVector de_z)
{
	double theta = acos(float(de_z*zaxis));
	if (abs(theta) < 0.0001)
	{
		theta = acos(float(de_x*xaxis));
		if (abs(theta) >= 0.0001)
		{
			ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
			Rotate(theta, tempaxis, origin);
		}
	}
	else
	{
		ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(zaxis, de_z);
		Rotate(theta, tempaxis, origin);
		theta = acos(float(de_x*xaxis));
		tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
		Rotate(theta, tempaxis, origin);
	}
}

void Curve::Reverse()
{
	vector<double> temp_t(m_T.size());
	vector<ON_BezierCurve> temp_curve(m_Curve.size());
	for (int i = 0; i < m_Curve.size(); i++)
	{
		temp_curve[i] = m_Curve[m_Curve.size() - i - 1];
		temp_curve[i].Reverse();
	}
	m_Curve = temp_curve;
	for (int i = 0; i < m_T.size(); i++)
	{
		temp_t[i] = 1 - m_T[m_T.size() - i - 1];
	}
	m_T = temp_t;
	xaxis.Reverse();
	yaxis.Reverse();
	zaxis.Reverse();
}

bool Curve::ModifyCurve(int idx, ON_3dPoint mp)
{
	int n = m_T.size();
	if (idx >= n || idx < 0)	return false;
	vector<ON_3dPoint> pa(n);
	for (int i = 0; i < n - 1; i++)
		pa[i] = m_Curve[i].PointAt(0);
	pa[n - 1] = m_Curve[n - 2].PointAt(1);
	pa[idx] = mp;

	double total_length = 0;
	for (int i = 1; i < n; i++)
	{
		total_length += (pa[i] - pa[i - 1]).Length();
		m_T[i] = total_length;
	}
	for (int i = 0; i < n; i++)
	{
		m_T[i] = m_T[i] / m_T[n - 1];
	}
	vector<ON_3dVector> D_1(n), d(n);
	d[0] = 3 * (pa[1] - pa[0]);
	d[n - 1] = 3 * (pa[n - 1] - pa[n - 2]);
	for (int i = 1; i < n - 1; i++)
	{
		d[i] = 3 * (pa[i + 1] - pa[i - 1]);
	}
	//initialize coefficients (thomas algorithm)
	vector<double> c(n - 1, 1), b(n, 4), a(n - 1, 1);
	b[0] = 2;
	b[n - 1] = 2;

	c[0] = c[0] / b[0];
	d[0] = d[0] / b[0];

	for (int i = 1; i < n - 1; i++)
	{
		c[i] = c[i] / (b[i] - a[i - 1] * c[i - 1]);
		d[i] = (d[i] - a[i - 1] * d[i - 1]) / (b[i] - a[i - 1] * c[i - 1]);
	}
	d[n - 1] = (d[n - 1] - a[n - 2] * d[n - 2]) / (b[n - 1] - a[n - 2] * c[n - 2]);

	D_1[n - 1] = d[n - 1];
	for (int i = n - 2; i >= 0; i--)
	{
		D_1[i] = d[i] - c[i] * d[i + 1];
	}

	for (int i = 0; i < n - 1; i++)
	{
		ON_3dPointArray cv(4);
		cv.Insert(0, pa[i]);
		cv.Insert(1, pa[i] + D_1[i] / 3);
		cv.Insert(2, pa[i + 1] - D_1[i + 1] / 3);
		cv.Insert(3, pa[i + 1]);
		m_Curve[i] = ON_BezierCurve(cv);
	}
}

bool Curve::ModifyCurve(int idx, ON_2dPoint mp)
{
	int n = m_T.size();
	if (idx >= n || idx < 0)	return false;
	vector<ON_3dPoint> pa(n);
	for (int i = 0; i < n - 1; i++)
		pa[i] = m_Curve[i].PointAt(0);
	pa[n - 1] = m_Curve[n - 2].PointAt(1);
	double mpz = pa[idx] * zaxis;
	pa[idx] = origin + mp.x*xaxis + mp.y*yaxis + mpz*zaxis;

	double total_length = 0;
	for (int i = 1; i < n; i++)
	{
		total_length += (pa[i] - pa[i - 1]).Length();
		m_T[i] = total_length;
	}
	for (int i = 0; i < n; i++)
	{
		m_T[i] = m_T[i] / m_T[n - 1];
	}
	vector<ON_3dVector> D_1(n), d(n);
	d[0] = 3 * (pa[1] - pa[0]);
	d[n - 1] = 3 * (pa[n - 1] - pa[n - 2]);
	for (int i = 1; i < n - 1; i++)
	{
		d[i] = 3 * (pa[i + 1] - pa[i - 1]);
	}
	//initialize coefficients (thomas algorithm)
	vector<double> c(n - 1, 1), b(n, 4), a(n - 1, 1);
	b[0] = 2;
	b[n - 1] = 2;

	c[0] = c[0] / b[0];
	d[0] = d[0] / b[0];

	for (int i = 1; i < n - 1; i++)
	{
		c[i] = c[i] / (b[i] - a[i - 1] * c[i - 1]);
		d[i] = (d[i] - a[i - 1] * d[i - 1]) / (b[i] - a[i - 1] * c[i - 1]);
	}
	d[n - 1] = (d[n - 1] - a[n - 2] * d[n - 2]) / (b[n - 1] - a[n - 2] * c[n - 2]);

	D_1[n - 1] = d[n - 1];
	for (int i = n - 2; i >= 0; i--)
	{
		D_1[i] = d[i] - c[i] * d[i + 1];
	}

	for (int i = 0; i < n - 1; i++)
	{
		ON_3dPointArray cv(4);
		cv.Insert(0, pa[i]);
		cv.Insert(1, pa[i] + D_1[i] / 3);
		cv.Insert(2, pa[i + 1] - D_1[i + 1] / 3);
		cv.Insert(3, pa[i + 1]);
		m_Curve[i] = ON_BezierCurve(cv);
	}
}

bool Curve::IsCCL(ON_3dVector axis)
{
	ON_3dVector p = ON_DECL::ON_CrossProduct(PointAt(0) - origin, PointAt(0.5) - origin);
	if (p*axis > 0)
		return true;
	else
		return false;
}

Curve & Curve::operator=(Curve & c)
{
	if (this == &c)
		return *this;
	m_Curve = c.m_Curve;
	m_T = c.m_T;
	m_Teeth = c.m_Teeth;
	origin = c.origin;
	xaxis = c.xaxis;
	yaxis = c.yaxis;
	zaxis = c.zaxis;
	m_uListSmooth = c.m_uListSmooth;
	return *this;
}


void Curve::ExpandCurve(double Xscale/*=1*/, double Yscale/*=1*/)
{
	int n = m_T.size();
	ON_3dPointArray pa(n);
	for (int i = 0; i < n - 1; i++)
		pa.Insert(i, m_Curve[i].PointAt(0));
	pa.Insert(n - 1, m_Curve[n - 2].PointAt(1));
	ON_3dPoint temp_p;
	for (int i = 0; i < n; i++)
	{
		temp_p = ON_3dPoint((pa[i] - origin)*xaxis*Xscale, (pa[i] - origin)*yaxis*Yscale, (pa[i] - origin)*zaxis);
		pa[i] = xaxis*temp_p.x + yaxis*temp_p.y + zaxis*temp_p.z + origin;
	}
	*this = Curve(pa);
}

void Curve::MakePlanar(Teeth *teethin)
{
	int n = m_T.size();
	ON_3dPointArray pa(n);
	for (int i = 0; i < n - 1; i++)
		pa.Insert(i, m_Curve[i].PointAt(0));
	pa.Insert(n - 1, m_Curve[n - 2].PointAt(1));
	ON_3dPoint center = (PointAt(0) + PointAt(1) + PointAt(0.5)) / 3;
	zaxis = ON_CrossProduct(PointAt(0.5) - center, PointAt(0) - center);
	zaxis.Unitize();
	for (int i = 0; i < n; i++)
	{
		double t = (center - pa[i])*zaxis / (teethin->GetTooth(i)->longaxis*zaxis);
		pa[i] = pa[i] + teethin->GetTooth(i)->longaxis*t;
		teethin->GetTooth(i)->refpnt = pa[i];
//		pa[i] += (center - pa[i])*zaxis*zaxis;
	}
		
	*this = Curve(pa);
}

void Curve::Update()
{
	FT::FEntity::Update();
	//if (m_Teeth != NULL)
	//	m_Teeth->UpdateAllViews(false);
}

void Curve::DrawSmoothShade()
{
	if (m_isHide)  return;
	if (highlight_on)
		glColor3f(0.7, 0.7, 0.3);	
	else
		glColor3f(0.7, 0.3, 0.3);	
	if (disp_option == 0)
		glDisable(GL_LIGHTING);
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);
	glCallList(m_uListSmooth);

	glEnable(GL_LIGHTING);
	glColor3f(0, 0, 0);

	if (clipplane_on)
	{
		//clipping plane
		ON_3dVector n, pos;
		n = TangentAt(clipPos);
		n.Unitize();
		pos = PointAt(clipPos);
		float xx = n.x, yy = n.y, zz = n.z, aa;
		if (!clipDir)
		{
			xx = -xx, yy = -yy, zz = -zz;
		}
		aa = 0 - xx*pos.x - yy*pos.y - zz*pos.z;
		GLdouble eqn[4] = { xx, yy, zz, aa };	//Ax+By+Cz+D=0, ABC is normal.
		glClipPlane(GL_CLIP_PLANE0, eqn);
		glEnable(GL_CLIP_PLANE0);

		//visual of clipping plane
		ON_3dVector globalZ, up, side, upl, upr, lowl, lowr;	//up direction and 4 points of the rectangular.
		globalZ.Set(0, 0, 1);
		ON_3dVector tempt = ON_CrossProduct(n, globalZ);
		up = ON_CrossProduct(tempt, n);
		up.Unitize();
		side = ON_CrossProduct(up, n);
		side.Unitize();
		int height = 35 / 2, width = 20 / 2;
		float offset = 0.05;	//for display
		upl = pos + up*height - side*width + offset*n;
		upr = pos + up*height + side*width + offset*n;
		lowl = pos - up*height - side*width + offset*n;
		lowr = pos - up*height + side*width + offset*n;

		//define appearance.
		if (m_iSelect == 0)
			glColor4f(0.2, 0.2, 0.8, 0.2);
		glEnable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_POLYGON_OFFSET_FILL);
		//glPolygonOffset(1.0, 1.0);
		glEnable(GL_DEPTH_TEST);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);
		glVertex3f(upl.x, upl.y, upl.z);
		glVertex3f(lowl.x, lowl.y, lowl.z);
		glVertex3f(lowr.x, lowr.y, lowr.z);
		glVertex3f(upr.x, upr.y, upr.z);
		glEnd();

		////draw the point on curve
		//glDisable(GL_LIGHTING);
		//glColor3f(0.8, 0.1, 0.1);
		//glPointSize(15.0);
		//glBegin(GL_POINTS);
		//glVertex3f(pos.x + offset*n.x, pos.y + offset*n.y, pos.z + offset*n.z);
		//glEnd();
		//glEnable(GL_LIGHTING);
		//			glDisable(GL_BLEND);

	}
	
}

void Curve::DrawFlatShade()
{
	DrawSmoothShade();
}

void Curve::DrawMesh()
{
	DrawSmoothShade();
}

void Curve::DrawPoints()
{
	DrawSmoothShade();
}

void Curve::CalBoundBox()
{
	double pmin[3] = { PointAt(1).x, PointAt(1).y, PointAt(1).z };
	double pmax[3] = { pmin[0], pmin[1], pmin[2] };
	ON_3dPoint p;
	for (double i = 0; i < 1; i += 0.1)
	{
		p = PointAt(i);
		if (p.x < pmin[0])
			pmin[0] = p.x;
		if (p.y < pmin[1])
			pmin[1] = p.y;
		if (p.z < pmin[2])
			pmin[2] = p.z;
		if (p.x > pmax[0])
			pmax[0] = p.x;
		if (p.y > pmax[1])
			pmax[1] = p.y;
		if (p.z > pmax[2])
			pmax[2] = p.z;
	}
	m_entBox.SetMinPoint(ON_3dPoint(pmin));
	m_entBox.SetMaxPoint(ON_3dPoint(pmax));
}

void Curve::InitDisplayList()
{
	if (glIsList(m_uListSmooth))
		glDeleteLists(m_uListSmooth, 1);
	m_uListSmooth = glGenLists(1);

	glNewList(m_uListSmooth, GL_COMPILE);

	if (disp_option == 0)
	{
		glLineWidth(5);
		glBegin(GL_LINE_STRIP);
		double t = 0, ts = 50;
		while (t < 1)
		{
			glVertex3d(PointAt(t).x, PointAt(t).y, PointAt(t).z);
			t += (1 / ts);
		}
		glVertex3d(PointAt(1).x, PointAt(1).y, PointAt(1).z); //for the very last point when t=1
		glEnd();

		//show the control vertices on the curve.
		glPointSize(15);
		glBegin(GL_POINTS);
		ON_3dVector pt;
		for (int n = 0; n < m_T.size(); n++)
		{
			pt = PointAt(m_T[n]);
			glVertex3d(pt.x, pt.y, pt.z);
		}
		glEnd();
		glLineWidth(1);
	}
	if (disp_option == 1)	//round cross-section wire
	{
		ON_3fVector globalZ, up, side;
		globalZ.Set(0, 0, 1);
		ON_3fVector tanv, pos;
		ON_3fVector tempt;
		double t = 0;
		int ts = 50;
		int cirpts = 60;
		double step = 2 * PI / 60;
		ON_3fVector crosslist[51][61];
		ON_3fVector crossnormal[51][61];
		double rad = width / 2;

		for (int crossN = 0; crossN < (ts+1); crossN++)
		{
			tanv = TangentAt(t);
			tanv.Unitize();
			pos = PointAt(t);

			tempt = ON_CrossProduct(tanv, globalZ);
			up = ON_CrossProduct(tempt, tanv);
			up.Unitize();
			side = ON_CrossProduct(tanv, up);
			side.Unitize();
		
			for (int i = 0; i < (cirpts+1); i++)
			{
				tempt = pos + up*cos(i*step)*rad + side*sin(i*step)*rad;
				crosslist[crossN][i] = tempt;
				crossnormal[crossN][i] = tempt - pos;
				crossnormal[crossN][i].Unitize();
			}
			t += (1 / double(ts));
		}

		for (int crossN = 0; crossN < ts ; crossN++)
		{
			glBegin(GL_TRIANGLE_STRIP);
			for (int i = 0; i < cirpts; i++)
			{
				glNormal3f(crossnormal[crossN + 1][i].x, crossnormal[crossN + 1][i].y, crossnormal[crossN + 1][i].z);
				glVertex3f(crosslist[crossN + 1][i].x, crosslist[crossN + 1][i].y, crosslist[crossN + 1][i].z);
				glNormal3f(crossnormal[crossN][i].x, crossnormal[crossN][i].y, crossnormal[crossN][i].z);
				glVertex3f(crosslist[crossN][i].x, crosslist[crossN][i].y, crosslist[crossN][i].z);				
			}
			glEnd();
		}

		//two ends
		glBegin(GL_POLYGON);
		for (int i=cirpts; i > 0; i--)
		{
			tanv = -TangentAt(0);
			tanv.Unitize();
			glNormal3f(tanv.x, tanv.y, tanv.z);
			glVertex3f(crosslist[0][i].x, crosslist[0][i].y, crosslist[0][i].z);
		}
		glEnd();
		glBegin(GL_POLYGON);
		for (int i=0; i < cirpts; i++)
		{
			tanv = TangentAt(1);
			tanv.Unitize();
			glNormal3f(tanv.x, tanv.y, tanv.z);
			glVertex3f(crosslist[ts][i].x, crosslist[ts][i].y, crosslist[ts][i].z);
		}
		glEnd();
	}

	if (disp_option == 2)	//square cross-section wire
	{
		ON_3fVector globalZ, up, side, upl, upr, lowl, lowr;	//up direction and 4 points of the rectangular.
		globalZ.Set(0, 0, 1);
		ON_3fVector tanv, pos;
		ON_3fVector tempt;
		double t = 0;
		int ts = 50;
		//vector<ON_3fVector> crosssec(4);
		//vector<vector<ON_3fVector>> crosslist(ts + 1, vector<ON_3dVector>(4));
		ON_3fVector crosslist[51][5];	//[cross-section number][vertex of square]
		ON_3fVector crossedgenormal[51][4];	//The normal of the up, right, down and left side.

		for (int crossN = 0; crossN < (ts+1); crossN++)
		{
			tanv = TangentAt(t);
			tanv.Unitize();
			pos = PointAt(t);

			tempt = ON_CrossProduct(tanv, globalZ);
			up = ON_CrossProduct(tempt, tanv);
			up.Unitize();
			side = ON_CrossProduct(tanv, up);
			side.Unitize();
			crossedgenormal[crossN][0] = up;
			crossedgenormal[crossN][1] = side;
			crossedgenormal[crossN][2] = -up;
			crossedgenormal[crossN][3] = -side;

			double length = width/2;
			//upl = pos + up*length - side*length; crosssec.push_back(upl);
			//upr = pos + up*length + side*length; crosssec.push_back(upr);
			//lowl = pos - up*length - side*length; crosssec.push_back(lowr);
			//lowr = pos - up*length + side*length; crosssec.push_back(lowl);
			upl = pos + up*length - side*length; crosslist[crossN][0] = upl; crosslist[crossN][4] = upl;
			upr = pos + up*length + side*length; crosslist[crossN][1] = upr;
			lowr = pos - up*length + side*length; crosslist[crossN][2] = lowr;
			lowl = pos - up*length - side*length; crosslist[crossN][3] = lowl;
			t += (1 / double(ts) );
		}

		for (int crossN = 0; crossN < ts; crossN++)
		{			
			glBegin(GL_TRIANGLES);
			for (int i = 0; i < 4; i++)
			{
				//first triangle
				glNormal3f(crossedgenormal[crossN][i].x, crossedgenormal[crossN][i].y, crossedgenormal[crossN][i].z);
				glVertex3f(crosslist[crossN][i].x, crosslist[crossN][i].y, crosslist[crossN][i].z);

				glNormal3f(crossedgenormal[crossN][i].x, crossedgenormal[crossN][i].y, crossedgenormal[crossN][i].z);
				glVertex3f(crosslist[crossN][i + 1].x, crosslist[crossN][i + 1].y, crosslist[crossN][i + 1].z);

				glNormal3f(crossedgenormal[crossN+1][i].x, crossedgenormal[crossN+1][i].y, crossedgenormal[crossN+1][i].z);	
				glVertex3f(crosslist[crossN + 1][i].x, crosslist[crossN + 1][i].y, crosslist[crossN + 1][i].z);

				//second triangle
				glNormal3f(crossedgenormal[crossN][i].x, crossedgenormal[crossN][i].y, crossedgenormal[crossN][i].z);
				glVertex3f(crosslist[crossN][i + 1].x, crosslist[crossN][i + 1].y, crosslist[crossN][i + 1].z);

				glNormal3f(crossedgenormal[crossN + 1][i].x, crossedgenormal[crossN + 1][i].y, crossedgenormal[crossN + 1][i].z);
				glVertex3f(crosslist[crossN + 1][i + 1].x, crosslist[crossN + 1][i + 1].y, crosslist[crossN + 1][i + 1].z);

				glNormal3f(crossedgenormal[crossN + 1][i].x, crossedgenormal[crossN + 1][i].y, crossedgenormal[crossN + 1][i].z);
				glVertex3f(crosslist[crossN+1][i].x, crosslist[crossN+1][i].y, crosslist[crossN+1][i].z);
			}
			glEnd();
		}

		//two ends
		glBegin(GL_QUADS);
		tanv = -TangentAt(0);
		tanv.Unitize();
		glNormal3f(tanv.x, tanv.y, tanv.z);
		for (int i = 3; i >= 0; i--)
		{
			glVertex3f(crosslist[0][i].x, crosslist[0][i].y, crosslist[0][i].z);
		}

		tanv = TangentAt(1);
		tanv.Unitize();
		glNormal3f(tanv.x, tanv.y, tanv.z);
		for (int i = 0; i < 4; i++)
		{
			glVertex3f(crosslist[ts][i].x, crosslist[ts][i].y, crosslist[ts][i].z);
		}
		glEnd();
	}
	glEndList();		
}




//void Tooth::DrawBCylinder()
//{
//	int cslice = 30;
//	double cdia = bcylinder.circle.Diameter()/2;
//	double cheight = bcylinder.Height();
//
//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glBegin(GL_POLYGON);
//	for (int i = 0; i <= cslice; i++)
//	{
//		glNormal3f(0, 0, 1);
//		glVertex3f(	cdia * cos(2 * PI*(double) i/cslice),
//					cdia * sin(2 * PI*(double)i / cslice),
//					0 	);
//	}
//	glEnd();
//
//	glBegin(GL_TRIANGLE_STRIP);
//	for (int i = 0; i <= cslice; i++)
//	{
//		glNormal3f(	-cos(2 * PI*(double)i / cslice),
//					-sin(2 * PI*(double)i / cslice),
//					0 );
//		glVertex3f(	cdia * cos(2 * PI*(double)i / cslice),
//					cdia * sin(2 * PI*(double)i / cslice),
//					0 );
//		glNormal3f(	-cos(2 * PI*(double)i / cslice),
//					-sin(2 * PI*(double)i / cslice),
//					0);
//		glVertex3f(	cdia * cos(2 * PI*(double)i / cslice),
//					cdia * sin(2 * PI*(double)i / cslice),
//					cheight);
//	}
//	glEnd();
//	
//	glBegin(GL_POLYGON);
//	for (int i = 0; i <= cslice; i++)
//	{
//		glNormal3f(0, 0, 1);
//		glVertex3f(	cdia * cos(2 * PI*(double)i / cslice),
//					cdia * sin(2 * PI*(double)i / cslice),
//					cheight );
//	}
//	glEnd();
//}

void Tooth::CalAngle(ON_3dVector longaxis)
{
	//ON_3dVector caxis = bcylinder.circle.plane.Normal();
	ON_3dVector caxis = longaxis;
	
	double az, al;
	al = 180/PI * acos(caxis.z/caxis.Length());

	if ((caxis.x == 0) && (caxis.y == 0))
		az = 0;
	else
	{
		if (caxis.x >= 0)
			az = 180 / PI * asin(caxis.y / sqrt(pow(caxis.x, 2) + pow(caxis.y, 2)));
		else
			az = 180 - (180 / PI * asin(caxis.y / sqrt(pow(caxis.x, 2) + pow(caxis.y, 2))));
	}
	rotAngle.Set(az,al,0);
}

void Tooth::HideBracket()
{
	if (m_bracket)
		m_bracket->m_isHide = true;
}

void Tooth::ShowBracket()
{
	if (m_bracket)
		m_bracket->m_isHide = false;
}

void Tooth::HideTooth()
{
	m_isHide = true;
	HideBracket();
}

void Tooth::ShowTooth(bool bracket_display)
{
	m_isHide = false;
	if (bracket_display)
		ShowBracket();
}

//void Tooth::SetOriFrame(ON_3dVector xax, ON_3dVector yax, ON_3dVector zax)
//{ 
//	oriXAxis = xax;
//	oriYAxis = yax;
//	oriLongAxis = zax;
//	m_bDispList = false;
//}

//void Tooth::DoMapping()
//{
//	GLdouble mappingMatrix[16];
//	ON_3dVector xa, ya, za;	//World's coordinate frame
//	ON_3dVector xb, yb, zb;	//Tooth's initial frame
//	ON_3dVector xc, yc, zc;	//Tooth's final frame (the current frame).
//	xa.Set(1, 0, 0);
//	ya.Set(0, 1, 0);
//	za.Set(0, 0, 1);
//	xb = oriXAxis;
//	yb = oriYAxis;
//	zb = oriLongAxis;
//	//xc = xaxis;
//	//yc = yaxis;
//	//zc = longaxis;
//
//	xc = oriXAxis;
//	yc = oriYAxis;
//	zc = oriLongAxis;
//	ON_3dVector xtemp = oriXAxis;
//	double theta = acos(float(oriLongAxis*longaxis));
//	if (abs(theta) < 0.0001)
//	{
//		theta = acos(float(oriXAxis*xaxis));
//		if (abs(theta) >= 0.0001)
//		{
//			if (ON_DECL::ON_CrossProduct(oriXAxis, xaxis)*longaxis > 0)
//			{
//				xc.Rotate(theta*view_par, zc);
//				yc = ON_DECL::ON_CrossProduct(zc, xc);
//			}
//			else
//			{
//				xc.Rotate(-theta*view_par, zc);
//				yc = ON_DECL::ON_CrossProduct(zc, xc);
//			}
//		}
//	}
//	else
//	{
//		ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(oriLongAxis, longaxis);
//		tempaxis.Unitize();
//		zc = oriLongAxis;
//		zc.Rotate(theta*view_par, tempaxis);
//		xc.Rotate(theta*view_par, tempaxis);
//		yc.Rotate(theta*view_par, tempaxis);
//		xtemp.Rotate(theta, tempaxis);
//		theta = acos(float(xtemp*xaxis));
//		if (ON_DECL::ON_CrossProduct(xtemp, xaxis)*zc > 0)
//		{
//			xc.Rotate(theta*view_par, zc);
//			yc.Rotate(theta*view_par, zc);
//		}
//		else
//		{
//			xc.Rotate(-theta*view_par, zc);
//			yc.Rotate(-theta*view_par, zc);
//		}		
//	}
//
//	ON_3dPoint AB, AC;
//	AB = oriPos;	//vector from World's origin to initial frame's origin.
//	AC = oriPos + view_par*(refpnt - oriPos);	//vector from World's origin to final frame's origin.
//
//	//Trans from A to C
//	glTranslated(AC.x, AC.y, AC.z);
//	//Rot frame A to C
//	mappingMatrix[0] = xa*xc; mappingMatrix[4] = xa*yc; mappingMatrix[8] = xa*zc; mappingMatrix[12] = 0;
//	mappingMatrix[1] = ya*xc; mappingMatrix[5] = ya*yc; mappingMatrix[9] = ya*zc; mappingMatrix[13] = 0;
//	mappingMatrix[2] = za*xc; mappingMatrix[6] = za*yc; mappingMatrix[10] = za*zc; mappingMatrix[14] = 0;
//	mappingMatrix[3] = 0; mappingMatrix[7] = 0; mappingMatrix[11] = 0; mappingMatrix[15] = 1;
//	glMultMatrixd(mappingMatrix);
//	//Rot frame B to A
//	mappingMatrix[0] = xb*xa; mappingMatrix[4] = xb*ya; mappingMatrix[8] = xb*za; mappingMatrix[12] = 0;
//	mappingMatrix[1] = yb*xa; mappingMatrix[5] = yb*ya; mappingMatrix[9] = yb*za; mappingMatrix[13] = 0;
//	mappingMatrix[2] = zb*xa; mappingMatrix[6] = zb*ya; mappingMatrix[10] = zb*za; mappingMatrix[14] = 0;
//	mappingMatrix[3] = 0; mappingMatrix[7] = 0; mappingMatrix[11] = 0; mappingMatrix[15] = 1;
//	glMultMatrixd(mappingMatrix);
//	//Trans from B to A
//	glTranslated(-AB.x, -AB.y, -AB.z);
//}

Bracket::Bracket()
{
	m_iEntType = BRACKET;
	m_pMesh = NULL;
	origin = ON_3dPoint(0, 0, 0);
	xaxis = ON_3dVector(1, 0, 0);
	yaxis = ON_3dVector(0, 1, 0);
	zaxis = ON_3dVector(0, 0, 1);
	localCS_on = true;	
	m_xForm.Identity();
	archwire_par = 0;
	is_customized = false;
}

Bracket::~Bracket()
{
	if (m_pMesh)
		m_pMesh->Destroy();
	if (topface)
		delete topface;
	if (bottomface)
		delete bottomface;
	if (sideface)
		delete sideface;
}

void Bracket::SetMesh(FEntity* in)
{
	m_pMesh = new ON_Mesh;
	*m_pMesh = *((FBody*)in)->GetMesh();
}

void Bracket::Transform(ON_Xform &matr)
{
	m_pMesh->Transform(matr);
	origin.Transform(matr);
	xaxis.Transform(matr);
	yaxis.Transform(matr);
	zaxis.Transform(matr);
	origin.Transform(matr);
	m_xForm = matr*m_xForm;
	m_bBox = false;
}

void Bracket::Translate(ON_3dPoint des)
{
	ON_3dVector v = des - origin;
	m_pMesh->Translate(v);
	origin = des;
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Translation(v);
	m_xForm = temp_form*m_xForm;
	m_bBox = false;
}

void Bracket::Translate(double x, double y, double z)
{
	ON_3dVector v = ON_3dVector(x, y, z);
	m_pMesh->Translate(v);
	origin += v;
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Translation(v);
	m_xForm = temp_form*m_xForm;
	m_bBox = false;
}

void Bracket::Align(ON_3dVector de_x, ON_3dVector de_z)
{
	double theta = acos(float(de_z*zaxis));
	if (abs(theta) < 0.0001)
	{
		theta = acos(float(de_x*xaxis));
		if (abs(theta) >= 0.0001)
		{
			ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
			Rotate(theta, tempaxis, origin);
		}
	}
	else if (theta >= PI-0.0001)
	{
		Rotate(PI, xaxis, origin);
		theta = acos(float(de_x*xaxis));
		ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
		Rotate(theta, tempaxis, origin);
	}
	else
	{
		ON_3dVector tempaxis = ON_DECL::ON_CrossProduct(zaxis, de_z);
		Rotate(theta, tempaxis, origin);
		theta = acos(float(de_x*xaxis));
		tempaxis = ON_DECL::ON_CrossProduct(xaxis, de_x);
		Rotate(theta, tempaxis, origin);
	}
}

void Bracket::Rotate(double r_angle, ON_3dVector axis, ON_3dPoint p)
{
	xaxis.Rotate(r_angle, axis);
	yaxis.Rotate(r_angle, axis);
	zaxis.Rotate(r_angle, axis);
	origin.Rotate(r_angle, axis, p);
	m_pMesh->Rotate(r_angle, axis, p);
	ON_Xform temp_form = ON_Xform(1);
	temp_form.Rotation(r_angle, axis, p);
	m_xForm = temp_form*m_xForm;
	m_bBox = false;
}

void Bracket::Update()
{
	FT::FEntity::Update();
	if (is_customized)
		InitCustomDisplayList();
}

void Bracket::DrawMesh()
{
	if (!m_pMesh)	return;
	if (m_iSelect == 0)
		glColor3f(0.73, 0.43, 0.25);//glColor3f(m_red, m_green, m_blue);
	GLfloat specComp[4] = { 0.73, 0.43, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.3, 0.3, 0.3, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.73, 0.43, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_DEPTH_TEST);
	glPushMatrix();
	//m_xForm.Transpose();
	GLdouble mapping[16];
	int n = 0;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			mapping[n] = m_xForm.m_xform[i][j];
			n += 1;
		}
	}
	//glMultMatrixd(*m_xForm.m_xform);
	glMultMatrixd(mapping);
	glCallList(m_uListSmooth);
	glColor3f(0, 0, 0);

	glCallList(m_uListMesh);
	glPopMatrix();
	glPolygonOffset(0.0f, 0.0f);
	glDisable(GL_POLYGON_OFFSET_FILL);
	AddDisplayControl();
}


void Bracket::DrawSmoothShade()
{
	if (m_isHide)  return;
	if (!m_pMesh)	return;
	if (highlight_on)
		glColor3f(0.8, 0.2, 0.2);//glColor3f(m_red, m_green, m_blue);
	else
		glColor3f(0.7, 0.7, 0.7);
	GLfloat specComp[4] = { 0.7, 0.7, 0.7, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.3, 0.3, 0.3, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.73, 0.43, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_DEPTH_TEST);
	glPushMatrix();	
	//m_xForm.Transpose();
	GLdouble mapping[16];
	int n = 0;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			mapping[n] = m_xForm.m_xform[i][j];
			n += 1;
		}
	}	
	//glMultMatrixd(*m_xForm.m_xform);
	glMultMatrixd(mapping);
	if (is_Cdisplist)
	{
		glColor3f(0.9, 0.1, 0.1);
		glCallList(m_CListSmooth);
	}
	glCallList(m_uListSmooth);
	
	glPopMatrix();
	glPolygonOffset(0.0f, 0.0f);
	glDisable(GL_POLYGON_OFFSET_FILL);

	////for test only
	//const ON_MeshTopology *T_mesh = &(m_pMesh->Topology());
	//for (int i = 0; i < candidate_edges.size(); i++)
	//{
	//	int v1 = T_mesh->m_tope[candidate_edges[i]].m_topvi[0], v2 = T_mesh->m_tope[candidate_edges[i]].m_topvi[1];
	//	glDisable(GL_LIGHTING);
	//	glLineWidth(6);
	//	glColor3f(0.8, 0, 0.2);
	//	glBegin(GL_LINES);
	//	ON_3dPoint ver1 = T_mesh->TopVertexPoint(v1), ver2 = T_mesh->TopVertexPoint(v2);
	//	glVertex3d(ver1.x, ver1.y, ver1.z);
	//	glVertex3d(ver2.x, ver2.y, ver2.z);
	//	glEnd();
	//}
	////for test only
	AddDisplayControl();
	
}

void Bracket::InitDisplayList()
{
	if (NULL == m_pMesh)	return;
	m_bDispList = true;
	if (glIsList(tempList))
		glDeleteLists(tempList, 3);
	tempList = glGenLists(3);
	int fi;
	F3dPoint v[4];
	F3fVector n;
	int face_count = m_pMesh->FaceCount();
	glNewList(tempList, GL_COMPILE);
	glBegin(GL_TRIANGLES);
	for (int fi = 0; fi < face_count; fi++)
	{
		const ON_MeshFace& f = m_pMesh->m_F[fi];
		v[0] = m_pMesh->m_V[f.vi[0]];
		v[1] = m_pMesh->m_V[f.vi[1]];
		v[2] = m_pMesh->m_V[f.vi[2]];
		n = m_pMesh->m_FN[fi];
		glNormal3d(n.x, n.y, n.z);	//face normal
		for (int i = 0; i < 3; i++)
		{
			glVertex3d(v[i].x, v[i].y, v[i].z);
		}
	}
	glEnd();
	glEndList();

	//draw smooth shade list
	F3fVector normal[4];
	m_uListSmooth = tempList + 1;
	glNewList(m_uListSmooth, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	ON_GL(*m_pMesh);
	glEndList();

	//draw mesh list
	m_uListMesh = tempList + 2;
	glNewList(m_uListMesh, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glCallList(tempList);
	glEndList();
}

void Bracket::InitCustomDisplayList()
{
	//is_customized = false;
	if (glIsList(m_CListSmooth))
		glDeleteLists(m_CListSmooth, 1);
	m_CListSmooth = glGenLists(1);
	glNewList(m_CListSmooth, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLES);
	FMesh *oneface = topface;
	ON_Xform inverse_form = this->GetXform().Inverse();
//	inverse_form = ON_Xform(1);
	if (oneface)	
	{
		for (FMesh::FaceIter fi = oneface->faces_begin(); fi != oneface->faces_end(); ++fi)
		{
			FMesh::ConstFaceVertexCCWIter mFVIter;
			ON_3dVector nor = ON_3dVector(oneface->normal(*fi).data());
			nor.Transform(inverse_form);
			glNormal3d(nor.x, nor.y, nor.z);
			mFVIter = oneface->fv_ccwiter(*fi);
			for (int i = 0; i < 3; ++i)
			{
				ON_3dPoint coord = ON_3dPoint(oneface->point(*mFVIter).data());
				coord.Transform(inverse_form);
				glVertex3d(coord.x, coord.y, coord.z);
				mFVIter++;
			}
		}
	}
	oneface = bottomface;
	if (oneface)
	{
		for (FMesh::FaceIter fi = oneface->faces_begin(); fi != oneface->faces_end(); ++fi)
		{
			FMesh::ConstFaceVertexCCWIter mFVIter;
			ON_3dVector nor = ON_3dVector(oneface->normal(*fi).data());
			nor.Transform(inverse_form);
			glNormal3d(nor.x, nor.y, nor.z);
			mFVIter = oneface->fv_ccwiter(*fi);
			for (int i = 0; i < 3; ++i)
			{
				ON_3dPoint coord = ON_3dPoint(oneface->point(*mFVIter).data());
				coord.Transform(inverse_form);
				glVertex3d(coord.x, coord.y, coord.z);
				mFVIter++;
			}
		}
	}
	oneface = sideface;
	if (oneface)
	{
		for (FMesh::FaceIter fi = oneface->faces_begin(); fi != oneface->faces_end(); ++fi)
		{
			FMesh::ConstFaceVertexCCWIter mFVIter;
			ON_3dVector nor = ON_3dVector(oneface->normal(*fi).data());
			nor.Transform(inverse_form);
			glNormal3d(nor.x, nor.y, nor.z);
			mFVIter = oneface->fv_ccwiter(*fi);
			for (int i = 0; i < 3; ++i)
			{
				ON_3dPoint coord = ON_3dPoint(oneface->point(*mFVIter).data());
				coord.Transform(inverse_form);
				glVertex3d(coord.x, coord.y, coord.z);
				mFVIter++;
			}
		}
	}

	glEnd();
	glEndList();
	is_Cdisplist = true;
}

void Bracket::CalBoundBox()
{
	if (!m_pMesh)	return;
	double minPnt[3], maxPnt[3];
	if (m_pMesh->GetBBox(minPnt, maxPnt))
	{
		m_entBox.SetMinPoint(F3dPoint(minPnt));
		m_entBox.SetMaxPoint(F3dPoint(maxPnt));
	}
	m_bBox = true;
}

void Bracket::AddDisplayControl()
{
	
	glEnable(GL_LIGHTING);
	glPointSize(15.0);
	glColor3f(0.8,0.2,0.2);
	glBegin(GL_POINTS);
	glVertex3d(origin.x, origin.y, origin.z);
	glEnd();
	glPointSize(1);	

	if (localCS_on)
	{
		GLfloat length = 3;
		GLUquadric* cone1 = gluNewQuadric();
		GLUquadric* cone2 = gluNewQuadric();

		//draw x-axis. Uses the same flag of "longaxis_on". If the local frame is not initialized, the value of xaxis is zero, and hence doesn't display.
		glColor3f(1, 0, 0);
		CalAngle(xaxis);
		glPushMatrix();
		glTranslatef(origin.x, origin.y, origin.z);
		glRotatef(rotAngle.x, 0, 0, 1);
		glRotatef(rotAngle.y, 0, 1, 0);

		glTranslatef(0, 0, length);
		gluCylinder(cone1, length / 10, 0, length / 3, 10, 3);
		glTranslatef(0, 0, -length);
		gluCylinder(cone2, 0, length / 20, length, 10, 3);
		glPopMatrix();

		//draw y-axis
		glColor3f(0, 1, 0);
		CalAngle(yaxis);
		glPushMatrix();
		glTranslatef(origin.x, origin.y, origin.z);
		glRotatef(rotAngle.x, 0, 0, 1);
		glRotatef(rotAngle.y, 0, 1, 0);

		glTranslatef(0, 0, length);
		gluCylinder(cone1, length / 10, 0, length / 3, 10, 3);
		glTranslatef(0, 0, -length);
		gluCylinder(cone2, 0, length / 20, length, 10, 3);
		glPopMatrix();

		//draw z-axis
		glColor3f(0, 0, 1);
		CalAngle(zaxis);
		glPushMatrix();
		glTranslatef(origin.x, origin.y, origin.z);
		glRotatef(rotAngle.x, 0, 0, 1);
		glRotatef(rotAngle.y, 0, 1, 0);

		glTranslatef(0, 0, length);
		gluCylinder(cone1, length / 10, 0, length / 3, 10, 3);
		glTranslatef(0, 0, -length);
		gluCylinder(cone2, 0, length / 20, length, 10, 3);
		glPopMatrix();	
	}

}

void Bracket::CalAngle(ON_3dVector longaxis)
{
	//ON_3dVector caxis = bcylinder.circle.plane.Normal();
	ON_3dVector caxis = longaxis;

	double az, al;
	al = 180 / PI * acos(caxis.z / caxis.Length());

	if ((caxis.x == 0) & (caxis.y == 0))
		az = 0;
	else
	{
		if (caxis.x >= 0)
			az = 180 / PI * asin(caxis.y / sqrt(pow(caxis.x, 2) + pow(caxis.y, 2)));
		else
			az = 180 - (180 / PI * asin(caxis.y / sqrt(pow(caxis.x, 2) + pow(caxis.y, 2))));
	}
	rotAngle.Set(az, al, 0);
}


bool Bracket::CalLCS(double featurelength, int numofslot)
{
	double minlength = featurelength - 0.2, maxlength = featurelength + 0.2, length;
	candidate_edges.clear();
	const ON_MeshTopology *T_mesh = &(m_pMesh->Topology());
	int n = T_mesh->TopEdgeCount();
	for (int i = 0; i < n; i++)
	{
		int v1, v2, f1, f2, otheredge, othervertex;
		v1 = T_mesh->m_tope[i].m_topvi[0];
		v2 = T_mesh->m_tope[i].m_topvi[1];
		length = (T_mesh->TopVertexPoint(v1) - T_mesh->TopVertexPoint(v2)).Length();
		if (length >= minlength && length <= maxlength)
		{
			f1 = T_mesh->m_tope[i].m_topfi[0];
			f2 = T_mesh->m_tope[i].m_topfi[1];
			if (abs(m_pMesh->m_FN[f1] * m_pMesh->m_FN[f2]) < 0.0001)  //perpendicular
			{
				for (int j = 0; j < 3; j++)
				{
					otheredge = T_mesh->m_topf[f1].m_topei[j];
					if (otheredge != i)
					{
						break;
					}
				}
				othervertex = T_mesh->m_tope[otheredge].m_topvi[0];
				if (othervertex==v1 || othervertex==v2)
					othervertex = T_mesh->m_tope[otheredge].m_topvi[1];
				if ((T_mesh->TopVertexPoint(othervertex) - T_mesh->TopVertexPoint(v1))*m_pMesh->m_FN[f2] > 0)
				{
					candidate_edges.push_back(i);
				}
			}
		}
	}
	
	if (candidate_edges.size() != numofslot * 2)
		return false;
	xaxis = m_pMesh->m_FN[T_mesh->m_tope[candidate_edges[0]].m_topfi[0]];
	zaxis = m_pMesh->m_FN[T_mesh->m_tope[candidate_edges[0]].m_topfi[1]];
	for (int i = 0; i < candidate_edges.size(); i++)
	{
		ON_3dVector temp1 = m_pMesh->m_FN[T_mesh->m_tope[candidate_edges[i]].m_topfi[0]];
		ON_3dVector temp2 = m_pMesh->m_FN[T_mesh->m_tope[candidate_edges[i]].m_topfi[1]];
		if (xaxis*temp1 < 0.98 && xaxis*temp2 < 0.98)
		{
			xaxis = m_pMesh->m_FN[T_mesh->m_tope[candidate_edges[0]].m_topfi[1]];
			zaxis = m_pMesh->m_FN[T_mesh->m_tope[candidate_edges[0]].m_topfi[0]];
			break;
		}
	}
	yaxis = ON_DECL::ON_CrossProduct(zaxis, xaxis);
	yaxis.Unitize();
	origin = ON_3dPoint(0, 0, 0);
	for (int i = 0; i < candidate_edges.size(); i++)
	{
		origin = origin + T_mesh->TopVertexPoint(T_mesh->m_tope[candidate_edges[i]].m_topvi[0]) + T_mesh->TopVertexPoint(T_mesh->m_tope[candidate_edges[i]].m_topvi[1]);
	}
	origin = origin / int(candidate_edges.size() * 2);
	localCS_on = true;
	return true;
}

void Bracket::CalNormals()
{
	if (!m_pMesh->HasFaceNormals())
	{
		m_pMesh->ComputeFaceNormals();
		m_pMesh->UnitizeFaceNormals();
	}
	if (!m_pMesh->HasVertexNormals())
	{
		m_pMesh->ComputeVertexNormals();
		m_pMesh->UnitizeVertexNormals();
	}
}

FMesh* Bracket::ExtractBracketBottom()
{
 	FMesh *topmesh=new FMesh;
	m_oMesh->request_face_normals();
	m_oMesh->update_face_normals();
	OpenMesh::VPropHandleT <bool> vSelected;
	OpenMesh::FPropHandleT <bool> fSelected;
	OpenMesh::VPropHandleT <int> newIdx;
	m_oMesh->add_property(vSelected);
	m_oMesh->add_property(fSelected);
	m_oMesh->add_property(newIdx);

	FMesh::VertexIter mVIter;
	for (mVIter = m_oMesh->vertices_begin(); mVIter != m_oMesh->vertices_end();++mVIter)
	{
		m_oMesh->property(vSelected, *mVIter) = false;
		m_oMesh->property(newIdx, *mVIter) = 0;
	}
	FMesh::FaceIter mFIter;
	for (mFIter = m_oMesh->faces_begin(); mFIter != m_oMesh->faces_end();++mFIter)
	{
		m_oMesh->property(fSelected, *mFIter) = false;
	}
	
	FMesh::Normal testNormal;
	FMesh::FaceHandle startFh;
	
	double tempResult = 0;
	for (mFIter = m_oMesh->faces_begin(); mFIter != m_oMesh->faces_end();++mFIter)
	{
		FMesh::ConstFaceVertexCWIter mFVIter;
		mFVIter = m_oMesh->fv_cwiter(*mFIter);
		ON_3dPoint v0 = ON_3dPoint(m_oMesh->point(*mFVIter).data());
		ON_3dPoint v1 = ON_3dPoint(m_oMesh->point(*(++mFVIter)).data());
		ON_3dPoint v2 = ON_3dPoint(m_oMesh->point(*(++mFVIter)).data());
		float t, u, v;
		ON_3dVector rayAxis = -xaxis;
		if (RayTriangleIntersect(origin, rayAxis, v0, v1, v2, t, u, v))
		{
			//double dotResult = ON_DotProduct(v0 - origin, rayAxis);
			if (t>tempResult)
			{
				tempResult = t;
				startFh = m_oMesh->face_handle(mFIter->idx());;
				testNormal=m_oMesh->normal(*mFIter);
			
			}
		}

	}
	
	FMesh::FaceFaceIter mFFIter;
	m_oMesh->property(fSelected, startFh) = true;

	std::vector<FMesh::VertexHandle> topVHandle;
	int faceCount;
	std::list<FMesh::FaceHandle> faceStack;
	FMesh::Normal avgNormal;
	FMesh::ConstFaceVertexCCWIter topFVccwiter;
	for (topFVccwiter = m_oMesh->fv_ccwiter(startFh); topFVccwiter.is_valid(); ++topFVccwiter)
	{
		topVHandle.push_back(topmesh->add_vertex(m_oMesh->point(*topFVccwiter)));
		m_oMesh->property(vSelected, *topFVccwiter) = true;
		m_oMesh->property(newIdx, *topFVccwiter) = topVHandle.back().idx();
	}
	topmesh->add_face(topVHandle);
	topVHandle.clear();
	//topNormalList.push_back(m_oMesh->normal(*startFIter));
	faceCount = 1;
	avgNormal = m_oMesh->normal(startFh);
	
	while (true)
	{
		for (mFFIter = m_oMesh->ff_iter(startFh); mFFIter.is_valid();++mFFIter)
		{
			if (m_oMesh->property(fSelected,*mFFIter)==false)
			{
				if ((m_oMesh->normal(*mFFIter)|avgNormal)>=0.9)
				{
					//add this face patch into topmesh
					for (topFVccwiter = m_oMesh->fv_ccwiter(*mFFIter); topFVccwiter.is_valid();++topFVccwiter)
					{
						if (!m_oMesh->property(vSelected, *topFVccwiter))
						{
							topVHandle.push_back(topmesh->add_vertex(m_oMesh->point(*topFVccwiter)));
							m_oMesh->property(vSelected, *topFVccwiter) = true;
							m_oMesh->property(newIdx, *topFVccwiter) = topVHandle.back().idx();
						}
						else
						{
							topVHandle.push_back(topmesh->vertex_handle(m_oMesh->property(newIdx, *topFVccwiter)));
						}
						
						
					}
					topmesh->add_face(topVHandle);
					topVHandle.clear();

					//push this patch into faceStack
					m_oMesh->property(fSelected, *mFFIter)=true;
					faceStack.push_back(*mFFIter);
					avgNormal = (faceCount*avgNormal + m_oMesh->normal(*mFFIter)) / (faceCount + 1.0);
					++faceCount;
					//
					
				}
			}
		}
		if (faceStack.empty())
		{
			break;
		}
		startFh = m_oMesh->face_handle(faceStack.front().idx());
		faceStack.pop_front();
		
	}
	

	

	m_oMesh->remove_property(fSelected);
	m_oMesh->remove_property(vSelected);
	m_oMesh->remove_property(newIdx);
	topmesh->request_face_normals();
	topmesh->update_face_normals();
	testNormal = topmesh->normal(topmesh->face_handle(0));

	return topmesh;
}

void Bracket::FaceProjection(FMesh *meshin)
{
	FMesh *toothmesh = Mesh2OMesh(m_tooth->GetMesh());
	MeshProjection(meshin, toothmesh, -xaxis, yaxis, 50);
	delete toothmesh;
}

void Bracket::Customize()
{
	//bottomface = ExtractBracketBottom();
	bottomface = ExtractBracketBottom();
	topface = DeepCopy(bottomface,true);
	vector<FMesh::VertexHandle> bnd = ExtractBnd(bottomface);

	ON_3dPointArray bnd1 = ON_3dPointArray(bnd.size());
	for (int i = 0; i < bnd.size(); ++i)
		bnd1.Insert(i, ON_3dPoint(bottomface->point(bnd[i]).data()));

	FaceProjection(bottomface);

	ON_3dPointArray bnd2 = ON_3dPointArray(bnd.size());
	for (int i = 0; i < bnd.size(); ++i)
		bnd2.Insert(i, ON_3dPoint(bottomface->point(bnd[i]).data()));
	/*
	//water-tight customized bracket
	CustmizedBracket = DeepCopy(topface);
	FMesh::VertexIter btmVIter;
	for (btmVIter = bottomface->vertices_begin(); btmVIter != bottomface->vertices_end();++btmVIter)
	{
		CustmizedBracket->add_vertex(bottomface->point(*btmVIter));
	}

	int numBtm = bottomface->vertices_end()->idx();
	FMesh::FaceIter btmFIter;
	FMesh::FaceVertexIter btmFVIter;
	std::vector<FMesh::VertexHandle> vh;
	vh.resize(3);
	for (btmFIter = bottomface->faces_begin(); btmFIter != bottomface->faces_end();++btmFIter)
	{
		for (btmFVIter = bottomface->fv_iter(*btmFIter); btmFVIter.is_valid();++btmFVIter)
		{
			vh.push_back(CustmizedBracket->vertex_handle(btmFVIter->idx()+numBtm));
		}
		CustmizedBracket->add_face(vh);
		vh.clear();
	}
	
	int i;
	for ( i = 0; i < bnd.size()-1;++i)
	{
		CustmizedBracket->add_face(CustmizedBracket->vertex_handle(bnd[i].idx() + 1), CustmizedBracket->vertex_handle(bnd[i].idx()), CustmizedBracket->vertex_handle(bnd[i].idx() + numBtm));
		CustmizedBracket->add_face(CustmizedBracket->vertex_handle(bnd[i].idx() + numBtm), CustmizedBracket->vertex_handle(bnd[i].idx() + numBtm), CustmizedBracket->vertex_handle(bnd[i].idx() + 1));
	}
	CustmizedBracket->add_face(CustmizedBracket->vertex_handle(bnd.front().idx()), CustmizedBracket->vertex_handle(bnd.back().idx()), CustmizedBracket->vertex_handle(bnd.back().idx() + numBtm));
	CustmizedBracket->add_face(CustmizedBracket->vertex_handle(bnd.back().idx() + numBtm), CustmizedBracket->vertex_handle(bnd.front().idx() + numBtm), CustmizedBracket->vertex_handle(bnd.front().idx()));
	*/

	//sideface construction
	sideface = new FMesh;
	vector<FMesh::VertexHandle> vhandle;
	vhandle.reserve(bnd.size() * 2);
	for (int i = 0; i < bnd.size(); ++i)
	{
		vhandle.push_back(sideface->add_vertex(FMesh::Point(bnd1[i].x, bnd1[i].y, bnd1[i].z)));
		vhandle.push_back(sideface->add_vertex(FMesh::Point(bnd2[i].x, bnd2[i].y, bnd2[i].z)));
	}
	for (int i = 0; i < bnd.size()-1; ++i)
	{
		sideface->add_face(vhandle[i * 2], vhandle[i * 2 + 1], vhandle[i * 2 + 2]);
		sideface->add_face(vhandle[i * 2+1], vhandle[i * 2 + 3], vhandle[i * 2 + 2]);
	}
	sideface->add_face(vhandle[2 * bnd.size() - 2], vhandle[2 * bnd.size() - 1], vhandle[0]);
	sideface->add_face(vhandle[2*bnd.size() - 1], vhandle[1], vhandle[0]);
	sideface->request_face_normals();
	sideface->update_face_normals();
	//sideface construction
	is_customized = true;
}



bool Bracket::ExportCustomizedBracket(const char* filename, bool isBinary /*= true*/)
{
	FILE *fout;
	fout = fopen(filename, "wb");
	if (fout == 0)
		return false;

	if (isBinary)
	{
		char header[128] = "solid                                                                                                ";

		unsigned long numFaces = 0;

		numFaces = numFaces + topface->n_faces() + bottomface->n_faces() + sideface->n_faces();
		fwrite(header, 80, 1, fout);

		// write number of faces
		fwrite(&numFaces, 1, sizeof(int), fout);

		FMesh* oneface = topface;
		if (oneface)
			ExportOMesh(fout, oneface, isBinary);
		oneface = bottomface;
		if (oneface)
			ExportOMesh(fout, oneface, isBinary);
		oneface = sideface;
		if (oneface)
			ExportOMesh(fout, oneface, isBinary);

	}
	else // ASCII
	{
		fprintf(fout, "solid HKUST\r\n");

		FMesh* oneface = topface;
		if (oneface)
			ExportOMesh(fout, oneface, isBinary);
		oneface = bottomface;
		if (oneface)
			ExportOMesh(fout, oneface, isBinary);
		oneface = sideface;
		if (oneface)
			ExportOMesh(fout, oneface, isBinary);

		fprintf(fout, "endsolid HKUST\r\n");
	}

	fclose(fout);

	return true;
}

bool Bracket::RayTriangleIntersect(const ON_3dPoint &orig, const ON_3dVector &dir, /* ray */ const ON_3dPoint &v0, const ON_3dPoint &v1, const ON_3dPoint &v2, /* triangle */ float &t, /* intersection point P = orig + t*dir */ float &u, /* weight of v0 */ float &v)
{
	//implement by Lufeng
	ON_3dVector v0v1 = v1 - v0;
	ON_3dVector v0v2 = v2 - v0;
	ON_3dVector pvec = ON_CrossProduct(dir, v0v2);
	float det = ON_DotProduct(v0v1, pvec);

	// culling
	if (det < 1e-8) return false;

	//// if not culling
	//if (std::abs(det) < kEpsilon) return false;

	float invDet = 1 / det;

	ON_3dVector tvec = orig - v0;
	u = ON_DotProduct(tvec, pvec)*invDet;
	if (u < 0 || u > 1) return false;

	ON_3dVector qvec = ON_CrossProduct(tvec, v0v1);
	v = ON_DotProduct(dir, qvec)*invDet;
	if (v < 0 || u + v > 1) return false;

	t = ON_DotProduct(v0v2, qvec)*invDet;

	return true;
}

NodeTree3D::NodeTree3D(ON_Mesh *mesh_in)
{
	m_pMesh = mesh_in;
	nodenum = m_pMesh->VertexCount();
	datapts = annAllocPts(nodenum, 3);
	for (int i = 0; i < nodenum; i++)
	{
		datapts[i][0] = m_pMesh->Vertex(i).x;
		datapts[i][1] = m_pMesh->Vertex(i).y;
		datapts[i][2] = m_pMesh->Vertex(i).z;
	}
	if (datapts>0)
		kdtree = new ANNkd_tree(datapts, nodenum, 3);
}

NodeTree3D::~NodeTree3D()
{
	annDeallocPts(datapts);
	//if (kdtree)
	//	delete kdtree;
}

ON_3dPointArray NodeTree3D::ANNSearch(ON_3dPoint querypnt, int k)
{
	ANNpoint q = annAllocPt(3);
	q[0] = querypnt.x;
	q[1] = querypnt.y;
	q[2] = querypnt.z;
	ANNidxArray nnIdx= new ANNidx[k];
	ANNdistArray dists = new ANNdist[k];
	kdtree->annkSearch(q, k, nnIdx, dists);
	ON_3dPointArray pp(k);
	for (int i = 0; i < k; i++)
	{
		ON_3dPoint p(datapts[nnIdx[i]][0], datapts[nnIdx[i]][1], datapts[nnIdx[i]][2]);
		pp.Insert(i, p);
	}
	delete[] nnIdx;
	delete[] dists;
	return pp;
}

double TriangleArea(ON_2dPoint v1, ON_2dPoint v2, ON_2dPoint v3)
{
	ON_3dVector v2_v1 = v2 - v1, v3_v1 = v3 - v1;
	double l = v3_v1.Length();
	v3_v1.Unitize();
	return l*(v2_v1 - v3_v1*(v2_v1*v3_v1)).Length() / 2;
}

bool TriangleInterp(ON_2dPoint v1, ON_2dPoint v2, ON_2dPoint v3, ON_2dPoint p, double *par)
{
	double total_area = TriangleArea(v1, v2, v3), area1 = TriangleArea(v2, v3, p), area2 = TriangleArea(v1, v3, p), area3 = TriangleArea(v1, v2, p);
	par[0] = area1 / total_area;
	par[1] = area2 / total_area;
	par[2] = area3 / total_area;
	if (par[0] + par[1] + par[2] > 1.00001)
		return false;
	return true;
}

vector<int> MeshSampling(ON_Mesh *mesh_in, ON_3dPoint refp, double angle, ON_3dVector refz)
{
	/*vector<int> pnts;
	pnts.push_back(0);
	int n = mesh_in->VertexCount();
	ON_3dVector dir0, dir1;
	for (int i = 1; i < n; i++)
	{
		bool is_good = true;
		dir1 = mesh_in->Vertex(i) - refp;
		dir1.Unitize();
		for (int j = pnts.size() - 1; j >= 0; j--)
		{
			dir0 = mesh_in->Vertex(pnts[j]) - refp;	
			dir0.Unitize();			
			if (acos(dir0*dir1) < angle)
			{
				is_good = false;
				break;
			}
		}
		if (is_good)
			pnts.push_back(i);
	}
	return pnts;*/

	vector<int> pnts;
	ON_3dVector refx = ON_CrossProduct(ON_3dVector(0, 1, 0), refz);
	refx.Unitize();
	ON_3dVector refy = ON_CrossProduct(refz, refx);
	for (double theta = -PI / 2 + angle; theta <= 0; theta += angle)
	{		
		for (double alfa = 0; alfa < 2 * PI; alfa += angle)
		{
			ON_3dVector ray_dir = cos(theta)*sin(alfa)*refx + cos(theta)*cos(alfa)*refy + sin(theta)*refz;
			int pti = RayMeshIntersection(refp, ray_dir, mesh_in);
			pnts.push_back(pti);
		}
	}
	return pnts;
}

int RayMeshIntersection(ON_3dPoint ray_ori, ON_3dVector ray_dir, const ON_Mesh* mesh_in)
{
	ray_dir.Unitize();
	const ON_MeshTopology* t_mesh = &(mesh_in->Topology());
	int vi = 0;
	ON_3dVector dir = t_mesh->TopVertexPoint(vi) - ray_ori;
	dir.Unitize();
	double angle = acos(ray_dir * dir);
	double angle2 = angle;
	int vn = t_mesh->m_topv[vi].m_tope_count;
	for (int i = 0; i < vn; i++)
	{
		int ei = t_mesh->m_topv[vi].m_topei[i];
		int vi2 = t_mesh->m_tope[ei].m_topvi[0] == vi ? t_mesh->m_tope[ei].m_topvi[1] : t_mesh->m_tope[ei].m_topvi[0];
		dir = t_mesh->TopVertexPoint(vi2) - ray_ori;
		dir.Unitize();
		if (angle2 > acos(ray_dir*dir))
		{
			angle2 = acos(ray_dir*dir);
			vi = vi2;
		}
	}
	while (angle2 < angle)
	{
		angle = angle2;
		vn = t_mesh->m_topv[vi].m_tope_count;
		for (int i = 0; i < vn; i++)
		{
			int ei = t_mesh->m_topv[vi].m_topei[i];
			int vi2 = t_mesh->m_tope[ei].m_topvi[0] == vi ? t_mesh->m_tope[ei].m_topvi[1] : t_mesh->m_tope[ei].m_topvi[0];
			dir = t_mesh->TopVertexPoint(vi2) - ray_ori;
			dir.Unitize();
			if (angle2 > acos(ray_dir*dir))
			{
				angle2 = acos(ray_dir*dir);
				vi = vi2;
			}
		}
	}
	return *t_mesh->m_topv[vi].m_vi;
}

void RGBtoHSV(float& fR, float& fG, float fB, float& fH, float& fS, float& fV)
{
	float fCMax = max(max(fR, fG), fB);
	float fCMin = min(min(fR, fG), fB);
	float fDelta = fCMax - fCMin;

	if (fDelta > 0) {
		if (fCMax == fR) {
			fH = 60 * (fmod(((fG - fB) / fDelta), 6));
		}
		else if (fCMax == fG) {
			fH = 60 * (((fB - fR) / fDelta) + 2);
		}
		else if (fCMax == fB) {
			fH = 60 * (((fR - fG) / fDelta) + 4);
		}

		if (fCMax > 0) {
			fS = fDelta / fCMax;
		}
		else {
			fS = 0;
		}

		fV = fCMax;
	}
	else {
		fH = 0;
		fS = 0;
		fV = fCMax;
	}

	if (fH < 0) {
		fH = 360 + fH;
	}
}

void HSVtoRGB(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV)
{
	float fC = fV * fS; // Chroma
	float fHPrime = fmod(fH / 60.0, 6);
	float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
	float fM = fV - fC;

	if (0 <= fHPrime && fHPrime < 1) {
		fR = fC;
		fG = fX;
		fB = 0;
	}
	else if (1 <= fHPrime && fHPrime < 2) {
		fR = fX;
		fG = fC;
		fB = 0;
	}
	else if (2 <= fHPrime && fHPrime < 3) {
		fR = 0;
		fG = fC;
		fB = fX;
	}
	else if (3 <= fHPrime && fHPrime < 4) {
		fR = 0;
		fG = fX;
		fB = fC;
	}
	else if (4 <= fHPrime && fHPrime < 5) {
		fR = fX;
		fG = 0;
		fB = fC;
	}
	else if (5 <= fHPrime && fHPrime < 6) {
		fR = fC;
		fG = 0;
		fB = fX;
	}
	else {
		fR = 0;
		fG = 0;
		fB = 0;
	}

	fR += fM;
	fG += fM;
	fB += fM;
}

bool FindPath(LaGenMatInt mat_in, vector<int> &row_out, vector<int> &col_out)
{
	int row = mat_in.rows();
	int col = mat_in.cols();

	// use Dijkstra algorithm for shortest path finding
	Graph g;
	int seq = 0;
	int w1 = 10; // normal weight
	int w2 = 1; // diagnol weight
	int w3 = 10000; // penalty
	int w4 = 100; // collision penalty
	
	for (int i = 0; i < row - 1; ++i)
	{
		for (int j = 0; j < col - 1; ++j)
		{
			if (mat_in(i, j) == 1)
			{
				////-------------collision penalty =  inf.-----------------------
				//if (mat_in(i, j + 1) == 1 && mat_in(i + 1, j) == 1 && mat_in(i + 1, j + 1) == 1)
				//	g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j, w1 }, { (i + 1)*col + j + 1, w2 } });
				//	//g.add_vertex(i*col + j, { { i*col + j + 1, w1 } });
				//else if(mat_in(i, j + 1) == 1 && mat_in(i + 1, j) == 1)
				//	g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j, w1 } });
				//else if(mat_in(i, j + 1) == 1 && mat_in(i + 1, j + 1) == 1)
				//	g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j + 1, w2 } });
				//else if(mat_in(i + 1, j) == 1 && mat_in(i + 1, j + 1) == 1)
				//	g.add_vertex(i*col + j, { { (i + 1)*col + j, w1 }, { (i + 1)*col + j + 1, w2 } });
				//else if(mat_in(i, j + 1) == 1)
				//	g.add_vertex(i*col + j, { { i*col + j + 1, w1 } });
				//else if (mat_in(i + 1, j) == 1)
				//	g.add_vertex(i*col + j, { { (i + 1)*col + j, w1 } });
				//else if(mat_in(i + 1, j + 1) == 1)
				//	g.add_vertex(i*col + j, { { (i + 1)*col + j + 1, w2 } });


				// collision penalty = w4 a considerate large number
				if (mat_in(i, j + 1) == 1 && mat_in(i + 1, j) == 1 && mat_in(i + 1, j + 1) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j, w1 }, { (i + 1)*col + j + 1, w2 } });
				else if (mat_in(i, j + 1) == 1 && mat_in(i + 1, j) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j, w1 }, { (i + 1)*col + j + 1, w4 } });
				else if (mat_in(i, j + 1) == 1 && mat_in(i + 1, j + 1) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j, w4 }, { (i + 1)*col + j + 1, w2 } });
				else if (mat_in(i + 1, j) == 1 && mat_in(i + 1, j + 1) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w4 }, { (i + 1)*col + j, w1 }, { (i + 1)*col + j + 1, w2 } });
				else if(mat_in(i, j + 1) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w1 }, { (i + 1)*col + j, w4 }, { (i + 1)*col + j + 1, w4 } });
				else if (mat_in(i + 1, j) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w4 }, { (i + 1)*col + j, w1 }, { (i + 1)*col + j + 1, w4 } });
				else if(mat_in(i + 1, j + 1) == 1)
					g.add_vertex(i*col + j, { { i*col + j + 1, w4 }, { (i + 1)*col + j, w4 }, { (i + 1)*col + j + 1, w2 } });
				else
					g.add_vertex(i*col + j, { { i*col + j + 1, w4 }, { (i + 1)*col + j, w4 }, { (i + 1)*col + j + 1, w4 } });
			}
			else
				g.add_vertex(i*col + j, { { i*col + j + 1, w4 }, { (i + 1)*col + j, w4 }, { (i + 1)*col + j + 1, w4 } });
		}
	}

	// last col
	for (int i = 0; i < row - 1; ++i)
	{
		if (mat_in(i, col - 1) == 1)
		{
			if (mat_in(i + 1, col - 1) == 1)
				g.add_vertex(i*col + col - 1, { { (i + 1)*col + col - 1, w1 } });
			// add collision penalty
			else
				g.add_vertex(i*col + col - 1, { { (i + 1)*col + col - 1, w4 } });
		}
		// add collision penalty
		else
			g.add_vertex(i*col + col - 1, { { (i + 1)*col + col - 1, w4 } });
		
	}
	
	// last row
	for (int j = 0; j < col - 1; ++j)
	{
		if (mat_in(row - 1, j) == 1)
		{
			if (mat_in(row - 1, j + 1) == 1)
				g.add_vertex((row - 1)*col + j, { { (row - 1)*col + j + 1, w1 } });
			// add collision penalty
			else
				g.add_vertex((row - 1)*col + j, { { (row - 1)*col + j + 1, w4 } });
		}
		// add collision penalty
		else
			g.add_vertex((row - 1)*col + j, { { (row - 1)*col + j + 1, w4 } });
	}

	// connect head to tail
	g.add_vertex(col*row - 1, { { 0, w3 } });

	int init_node = 0;
	int dest_node = col*row - 1;
	vector<int> seqVertex;
	//qDebug() << "As initial node: " << init_node << endl;
	//qDebug() << "As goal node: " << dest_node << endl;

	for (int vertex : g.shortest_path(init_node, dest_node))
	{
		//qDebug() << "Solution path from goal sequence : " << seq << " Node : " << vertex << endl;
		seqVertex.push_back(vertex);
		seq++;
	}
	seqVertex.push_back(init_node);
	//qDebug() << "Solution path from goal sequence : " << seq << " Node : " << init_node << endl;
	if (seq == 0)
		return false;

	int numSeq = seqVertex.size();

	for (int i = numSeq - 1; i >= 0; --i)
	{
		int rowIndex = seqVertex[i] / col;
		int colIndex = seqVertex[i] % col;

		row_out.push_back(rowIndex);
		col_out.push_back(colIndex);
	}

	return true;

}

vector<int> MergeVector(vector<int> v1, vector<int> v2)
{
	vector<int> vout;
	vout.reserve(v1.size());
	int currentvalue = 0;
	int finalvalue = v1[v1.size() - 1];
	int i = 0, ii = 0;
	while (currentvalue <= finalvalue)
	{
		while (i < v1.size() && v1[i] == currentvalue)
		{
			vout.push_back(i);
			i++;			
			if (ii < v2.size() && v2[ii] == currentvalue)
				ii++;
		}
		while (ii < v2.size() && v2[ii] == currentvalue)
		{
			vout.push_back(i - 1);
			ii++;
		}
		currentvalue++;
	}
	return vout;
}

void ExpandVector(vector<int> &vin, vector<int> vindex)
{
	int n = vindex.size();
	for (int i = 0; i < n; i++)
		vindex[i] = vin[vindex[i]];
	vin = vindex;
}

FMesh* Mesh2OMesh(ON_Mesh* p_mesh)
{

	//unsigned int vertexCount = p_mesh->VertexCount();
	//unsigned int faceCount = p_mesh->FaceCount();
	//FMesh* mesh = new FMesh;

	//std::vector<FMesh::VertexHandle> vhandle;
	//vhandle.reserve(vertexCount);

	//// generate vertices
	//for (unsigned int ii = 0; ii < vertexCount; ii++)
	//{
	//	ON_3dPoint p = p_mesh->m_V[ii];
	//	vhandle.push_back(mesh->add_vertex(FMesh::Point(p.x, p.y, p.z)));
	//}

	//// generate triangulated faces
	//for (unsigned int jj = 0; jj < faceCount; jj++)
	//{
	//	unsigned int ind0 = p_mesh->m_F[jj].vi[0];
	//	unsigned int ind1 = p_mesh->m_F[jj].vi[1];
	//	unsigned int ind2 = p_mesh->m_F[jj].vi[2];
	//	mesh->add_face(vhandle[ind0], vhandle[ind1], vhandle[ind2]);
	//}

	//vhandle.clear();
	//mesh->request_face_normals();
	//mesh->update_normals();

	const ON_MeshTopology *T_mesh = &(p_mesh->Topology());

	unsigned int vertexCount = T_mesh->TopVertexCount();
	unsigned int faceCount = T_mesh->TopFaceCount();

	FMesh* mesh = new FMesh;

	std::vector<FMesh::VertexHandle> vhandle;
	vhandle.reserve(vertexCount);

	// generate vertices
	for (unsigned int ii = 0; ii < vertexCount; ii++)
	{
		ON_3dPoint p = T_mesh->TopVertexPoint(ii);
		vhandle.push_back(mesh->add_vertex(FMesh::Point(p.x, p.y, p.z)));
	}

	// generate triangulated faces
	for (unsigned int jj = 0; jj < faceCount; jj++)
	{
		unsigned int ind0 = T_mesh->m_topv_map[T_mesh->m_mesh->m_F[jj].vi[0]];
		unsigned int ind1 = T_mesh->m_topv_map[T_mesh->m_mesh->m_F[jj].vi[1]];
		unsigned int ind2 = T_mesh->m_topv_map[T_mesh->m_mesh->m_F[jj].vi[2]];
		mesh->add_face(vhandle[ind0], vhandle[ind1], vhandle[ind2]);
	}

	vhandle.clear();
	mesh->request_face_normals();
	mesh->update_face_normals();

	//Added by ZR for architecture project
	/*OpenMesh::VPropHandleT<int> patch;
	mesh->add_property(patch);*/
	//initialize patch flag on vertices.
	FMesh::VertexIter v_it(mesh->vertices_begin()), v_end(mesh->vertices_end());
	for (v_it; v_it != v_end; ++v_it)
	{
		mesh->data(*v_it).patch = 0;	//for flood-fill check.
		mesh->data(*v_it).RANSAC = 0;	//for iterative RANSAC flag
		mesh->data(*v_it).TopoNode_idx = -1;	//Initialization. -1 means the node is not an TopoNode.
		mesh->data(*v_it).RANSAC_TYPE = 0;	//for final RANSAC result.
		//mesh->data(*v_it).iso = 0;	//for PCA evaluaton.
		//mesh->data(*v_it).Patch_idx = 0;	//0 for initialization. Count from 1.
	}

	//Added by ZR end
		


	return mesh;
}

std::vector<FMesh::VertexHandle> ExtractBnd(FMesh *meshin)
{
	FMesh::HalfedgeIter heIter;
	for (heIter = meshin->halfedges_begin(); heIter != meshin->halfedges_end();++heIter)
	{
		if (meshin->is_boundary(*heIter))
		{
			break; 
		}
	}
	FMesh::HalfedgeHandle heh, hehInit;
	hehInit = *heIter;
	heh = meshin->next_halfedge_handle(hehInit);
	std::vector<FMesh::VertexHandle> vhBoundary;
	while (heh!=hehInit)
	{
		vhBoundary.push_back(meshin->from_vertex_handle(heh));
		heh = meshin->next_halfedge_handle(heh);
	}
	return vhBoundary;

}


/*FMesh* FlipSurface(FMesh *meshin)
{
	FMesh* meshout = new FMesh; 
	FMesh::VertexIter inViter;
	for (inViter = meshin->vertices_begin(); inViter != meshin->vertices_end();++inViter)
	{
		meshout->add_vertex(meshin->point(*inViter));
	}
	FMesh::FaceIter inFiter;
	FMesh::ConstFaceVertexCWIter inFViter;
	std::vector<FMesh::VertexHandle> vh;
	for (inFiter = meshin->faces_begin();inFiter != meshin->faces_end();++inFiter)
	{
		for (inFViter = meshin->fv_cwiter(*inFiter); inFViter.is_valid();++inFViter)
		{
			vh.push_back(meshout->vertex_handle(inFViter->idx()));
		}
		meshout->add_face(vh);
		vh.clear();
	}
	return meshout;
}*/

void MeshProjection(FMesh* meshin, FMesh* des, ON_3dVector dir, ON_3dVector xaxis, int res)
{
	ON_3dVector yaxis = ON_CrossProduct(dir, xaxis);
	FMesh::VertexIter v0 = meshin->vertices_begin();
	ON_3dVector p = ON_3dVector(meshin->point(*v0).data());
	double xmin = p*xaxis, xmax = xmin, ymin = p*yaxis, ymax = ymin, x, y, z = p*dir;
	for (FMesh::VertexIter vi = meshin->vertices_begin(); vi != meshin->vertices_end(); ++vi)
	{
		p = ON_3dVector(meshin->point(*vi).data());
		x = p*xaxis;
		y = p*yaxis;
		if (x < xmin)
			xmin = x;
		if (x > xmax)
			xmax = x;
		if (y < ymin)
			ymin = y;
		if (y > ymax)
			ymax = y;
	}
	ON_3dPoint origin = xaxis * xmin + yaxis * ymin + dir * z;
	double width = xmax - xmin + 0.01, height = ymax - ymin +0.01;
	LaGenMatDouble Distance = LaGenMatDouble::ones(res + 1, res + 1);
	Distance *= 10; //initial distance to tooth
	double dx = width / res, dy = height / res, dis[3];
	for (FMesh::FaceIter fi = des->faces_begin(); fi != des->faces_end(); ++fi)
	{
		ON_3dVector nor = ON_3dVector(des->normal(*fi).data());
		if (nor*dir > 0)
			continue;
		bool is_inside = false;
		FMesh::ConstFaceVertexCWIter mFVIter = des->fv_cwiter(*fi);
		ON_3dPoint v[3];
		ON_2dPoint v2d[3];
		int vx[3], vy[3];
		for (int i = 0; i < 3; ++i)
		{
			v[i] = ON_3dPoint(des->point(*mFVIter).data());
			mFVIter++;
			v2d[i].x = (v[i] - origin)*xaxis;
			v2d[i].y = (v[i] - origin)*yaxis;
			vx[i] = round(v2d[i].x / dx);
			vy[i] = round(v2d[i].y / dy);
			if (vx[i] >= -1 && vx[i] <= res + 1 && vy[i] >= -1 && vy[i] <= res + 1)
				is_inside = true;
		}
		if (!is_inside)
			continue;
		int vx_max = vx[0], vx_min = vx[0], vy_max = vy[0], vy_min = vy[0];
		for (int i = 0; i < 3; i++)
		{
			dis[i] = (v[i] - origin)*dir;
			if (vx[i] > vx_max)
				vx_max = vx[i];
			if (vx[i] < vx_min)
				vx_min = vx[i];
			if (vy[i] > vy_max)
				vy_max = vy[i];
			if (vy[i] < vy_min)
				vy_min = vy[i];
		}
		double par[3];
		vx_max = vx_max > res ? res : vx_max;
		vx_min = vx_min < 0 ? 0 : vx_min;
		vy_max = vy_max > res ? res : vy_max;
		vy_min = vy_min < 0 ? 0 : vy_min;
		for (int i = vx_min; i <= vx_max; i++)
		{
			for (int j = vy_min; j <= vy_max; j++)
			{
				ON_2dPoint pp(dx*i, dy*j);
				if (TriangleInterp(v2d[0], v2d[1], v2d[2], pp, par))
				{
					double d = dis[0] * par[0] + dis[1] * par[1] + dis[2] * par[2];
					if (d < Distance(j, i))
						Distance(j, i) = d;
				}
			}
		}
	}
	for (FMesh::VertexIter vi = meshin->vertices_begin(); vi != meshin->vertices_end(); ++vi)
	{
		ON_3dPoint vp = ON_3dPoint(meshin->point(*vi).data());
		double vpx = (vp - origin)*xaxis, vpy = (vp - origin)*yaxis, vpz = (vp - origin)*dir;
		int x0 = floor(vpx / dx), y0 = floor(vpy / dy);
		double px = (vpx - dx*x0) / dx, py = (vpy - dy*y0) / dy;
		double interp_dist = (1 - py)*((1 - px)*Distance(y0, x0) + px*Distance(y0, x0 + 1)) + py*((1 - px)*Distance(y0 + 1, x0) + px*Distance(y0 + 1, x0 + 1));
		vpz = interp_dist - vpz;
		vp = vp + dir*vpz;
		meshin->set_point(*vi, FMesh::Point(vp.x, vp.y, vp.z));
	}
	
}

FMesh* DeepCopy(FMesh* meshin, bool flipnormal)
{
	/*FMesh *meshout = new FMesh;
	for (FMesh::FaceIter fi = meshin->faces_begin(); fi != meshin->faces_end(); ++fi)
	{
	if (!flipnormal) //CCW
	{
	FMesh::ConstFaceVertexCCWIter vfi = meshin->fv_ccwiter(*fi);
	FMesh::VertexHandle vhandle[3];
	for (int i = 0; i < 3; ++i)
	{
	vhandle[i] = meshout->add_vertex(meshin->point(*vfi));
	++vfi;
	}
	meshout->add_face(vhandle[0], vhandle[1], vhandle[2]);
	}
	else //CW
	{
	FMesh::ConstFaceVertexCWIter vfi = meshin->fv_cwiter(*fi);
	FMesh::VertexHandle vhandle[3];
	for (int i = 0; i < 3; ++i)
	{
	vhandle[i] = meshout->add_vertex(meshin->point(*vfi));
	++vfi;
	}
	meshout->add_face(vhandle[0], vhandle[1], vhandle[2]);
	}
	}
	meshout->request_face_normals();
	meshout->update_face_normals();
	return meshout;*/
	FMesh* meshout = new FMesh;
	FMesh::VertexIter inViter;
	for (inViter = meshin->vertices_begin(); inViter != meshin->vertices_end(); ++inViter)
	{
		meshout->add_vertex(meshin->point(*inViter));
	}
	if (flipnormal)
	{

		FMesh::FaceIter inFiter;
		FMesh::ConstFaceVertexCWIter inFViter;
		std::vector<FMesh::VertexHandle> vh;
		for (inFiter = meshin->faces_begin(); inFiter != meshin->faces_end(); ++inFiter)
		{
			for (inFViter = meshin->fv_cwiter(*inFiter); inFViter.is_valid(); ++inFViter)
			{
				vh.push_back(meshout->vertex_handle(inFViter->idx()));
			}
			meshout->add_face(vh);
			vh.clear();
		}
	}
	else
	{
		FMesh::FaceIter inFiter;
		FMesh::ConstFaceVertexCCWIter inFViter;
		std::vector<FMesh::VertexHandle> vh;
		for (inFiter = meshin->faces_begin(); inFiter != meshin->faces_end(); ++inFiter)
		{
			for (inFViter = meshin->fv_ccwiter(*inFiter); inFViter.is_valid(); ++inFViter)
			{
				vh.push_back(meshout->vertex_handle(inFViter->idx()));
			}
			meshout->add_face(vh);
			vh.clear();
		}

	}
	meshout->request_face_normals();
	meshout->update_face_normals();
	return meshout;
}


bool ExportOMesh(FILE* fout, FMesh* _mesh, bool isBinary /*= true*/)
{
	if (fout == 0)
		return false;

	// output based on openmesh
	if (isBinary)
	{
		// binary format reference: http://www.fabbers.com/tech/STL_Format#Sct_binary


		unsigned short attributes = 0;

		FMesh::ConstFaceIter        f_it(_mesh->faces_begin()),
			f_end(_mesh->faces_end());
		FMesh::ConstFaceVertexIter  fv_it;

		OpenMesh::Vec3d _p;
		for (; f_it != f_end; ++f_it)
		{
			fv_it = _mesh->cfv_iter(*f_it);

			const OpenMesh::Vec3d& _n = _mesh->normal(*f_it);
			float nvt[3];
			nvt[0] = _n[0];
			nvt[1] = _n[1];
			nvt[2] = _n[2];
			fwrite(nvt, sizeof (float), 3, fout);

			_p = _mesh->point(*fv_it);
			nvt[0] = _p[0];
			nvt[1] = _p[1];
			nvt[2] = _p[2];
			fwrite(nvt, sizeof (float), 3, fout);
			++fv_it;

			_p = _mesh->point(*fv_it);
			nvt[0] = _p[0];
			nvt[1] = _p[1];
			nvt[2] = _p[2];
			fwrite(nvt, sizeof (float), 3, fout);
			++fv_it;

			_p = _mesh->point(*fv_it);
			nvt[0] = _p[0];
			nvt[1] = _p[1];
			nvt[2] = _p[2];
			fwrite(nvt, sizeof (float), 3, fout);

			fwrite(&attributes, 1, sizeof(short), fout);

		}

	}
	else // in ASCII
	{

		FMesh::ConstFaceIter        f_it(_mesh->faces_begin()),
			f_end(_mesh->faces_end());
		FMesh::ConstFaceVertexIter  fv_it;

		OpenMesh::Vec3d _p1, _p2, _p3;
		for (; f_it != f_end; ++f_it)
		{
			fv_it = _mesh->cfv_iter(*f_it);

			const OpenMesh::Vec3d& _n = _mesh->normal(*f_it);

			_p1 = _mesh->point(*fv_it);
			++fv_it;
			_p2 = _mesh->point(*fv_it);
			++fv_it;
			_p3 = _mesh->point(*fv_it);

			fprintf(fout, "  facet normal %13e %13e %13e\r\n", _n[0], _n[1], _n[2]);
			fprintf(fout, "    outer loop\r\n");
			fprintf(fout, "      vertex  %13e %13e %13e\r\n", _p1[0], _p1[1], _p1[2]);
			fprintf(fout, "      vertex  %13e %13e %13e\r\n", _p2[0], _p2[1], _p2[2]);
			fprintf(fout, "      vertex  %13e %13e %13e\r\n", _p3[0], _p3[1], _p3[2]);
			fprintf(fout, "    endloop\r\n");
			fprintf(fout, "  endfacet\r\n");

		}

	}
	return true;

}


////////added by ZR for architecture project//////
void Tooth::CalRefVector(ON_3dVector vn, ON_3dVector axi )
{
	ON_3dVector tempvec, delta;
	delta = vn - axi;
	if (delta.Length() < 1e-9)
	{
		delta.Set(1e-8, 1e-8, 1e-8);
		axi -= delta;
	}	//tilt the axi away from the vector normal.
	tempvec = ON_CrossProduct(vn, axi);
	tempvec.Unitize();
	refvec = ON_CrossProduct(vn, tempvec);	
	refvec.Unitize();
}


void Tooth::CalRefVector()
{
	ON_3dVector tempvec,vn;
	ON_3dVector axi(0, 0, 1);	//Use global Z-axis to get the reference vector.
	ON_3dVector delta;
	vn = m_pMesh->m_N[0];
	delta = vn - axi;
	if (delta.Length() < 1e-9)
	{
		delta.Set(1e-8, 1e-8, 1e-8);
		axi -= delta;	//tilt the axi away from the vector normal.
	}	
	tempvec = ON_CrossProduct(vn, axi);
	tempvec.Unitize();
	refvec = ON_CrossProduct(vn, tempvec);
	refvec.Unitize();
}


void Tooth::AlignBestFitLine()
{
	//intersection line orientation alignment
	ON_3dVector tempdis1, tempdis2;
	int bestline1, bestline2;
	for (int nodenum = 0; nodenum < (tpNode.size() - 1); nodenum++)
	{
		bestline1 = tpNode[nodenum].best_fit_line_index;
		bestline2 = tpNode[nodenum + 1].best_fit_line_index;
		/*tempdis1 = tpNode[nodenum].intersectCurve[bestline1]->curve[0] - tpNode[nodenum + 1].intersectCurve[bestline2]->curve[0];
		tempdis2 = tpNode[nodenum].intersectCurve[bestline1]->curve[0] - *(tpNode[nodenum + 1].intersectCurve[bestline2]->curve.Last());
		if (tempdis1.Length() > tempdis2.Length() )
		{
			tpNode[nodenum + 1].intersectCurve[bestline2]->curve.Reverse();
		}*/
		tempdis1 = *(tpNode[nodenum].intersectCurve[bestline1]->curve.Last()) - tpNode[nodenum].intersectCurve[bestline1]->curve[0];
		tempdis2 = *(tpNode[nodenum + 1].intersectCurve[bestline2]->curve.Last()) - tpNode[nodenum + 1].intersectCurve[bestline2]->curve[0];
		if (ON_DotProduct(tempdis1, tempdis2) < 0)
		{
			tpNode[nodenum + 1].intersectCurve[bestline2]->curve.Reverse();
			tpNode[nodenum + 1].intersectCurve[bestline2]->curveplane.SetNormal(-(tpNode[nodenum + 1].intersectCurve[bestline2]->curveplane.GetNormal()));
			tpNode[nodenum + 1].best_plane_nor = tpNode[nodenum + 1].intersectCurve[bestline2]->curveplane.GetNormal();
		}
	}
	//update the end points of best_fit_line.
	for (int nodenum = 0; nodenum < tpNode.size(); nodenum++)
	{
		bestline1 = tpNode[nodenum].best_fit_line_index;
		tpNode[nodenum].best_fitted_line_points[0] = tpNode[nodenum].intersectCurve[bestline1]->curve[0];	//for single segment
		tpNode[nodenum].best_fitted_line_points[1] = *(tpNode[nodenum].intersectCurve[bestline1]->curve.Last());
	}



}

void Tooth::AlignBestFitLineSeg()
{
	//intersection line orientation alignment
	ON_3dVector tempdis1, tempdis2;
	int bestline1, bestline2;
	for (int nodenum = 0; nodenum < (tpNode.size() - 1); nodenum++)
	{
		bestline1 = tpNode[nodenum].best_fit_line_index;
		bestline2 = tpNode[nodenum + 1].best_fit_line_index;

		//Use the first segment as reference. Make sure all segments are aligned to this direction already.
		tempdis1 = *(tpNode[nodenum].intersectCurve[bestline1]->curves_seg_trim[0].Last()) - tpNode[nodenum].intersectCurve[bestline1]->curves_seg_trim[0][0];
		tempdis2 = *(tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_trim[0].Last()) - tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_trim[0][0];
		

		if (ON_DotProduct(tempdis1, tempdis2) < 0)
		{
			//update the single segment
			tpNode[nodenum + 1].intersectCurve[bestline2]->curve.Reverse();
			tpNode[nodenum + 1].intersectCurve[bestline2]->curveplane.SetNormal(-(tpNode[nodenum + 1].intersectCurve[bestline2]->curveplane.GetNormal()));
			tpNode[nodenum + 1].best_plane_nor = tpNode[nodenum + 1].intersectCurve[bestline2]->curveplane.GetNormal();

			//flip PCA
			ON_3dPoint pt = tpNode[nodenum + 1].intersectCurve[bestline2]->fitted_line[0];
			tpNode[nodenum + 1].intersectCurve[bestline2]->fitted_line[0] = tpNode[nodenum + 1].intersectCurve[bestline2]->fitted_line[1];
			tpNode[nodenum + 1].intersectCurve[bestline2]->fitted_line[1] = pt;

			//update multiple segments and face handlers. Need to reverse the whole vector. AND need to flip each segments.
			vector<ON_3dPointArray>::iterator itb = tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_trim.begin();
			vector<ON_3dPointArray>::iterator ite = tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_trim.end();
			std::reverse(itb, ite);
			vector<vector<FMesh::FaceHandle>>::iterator fh_itb = tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_fh_trim.begin();
			vector<vector<FMesh::FaceHandle>>::iterator fh_ite = tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_fh_trim.end();
			std::reverse(fh_itb, fh_ite);

			//flip each segments
			for (int i = 0; i < tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_trim.size(); i++)
			{
				tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_trim[i].Reverse();
				vector<FMesh::FaceHandle>::iterator fh_seg_itb = tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_fh_trim[i].begin();
				vector<FMesh::FaceHandle>::iterator fh_seg_ite = tpNode[nodenum + 1].intersectCurve[bestline2]->curves_seg_fh_trim[i].end();
				std::reverse(fh_seg_itb, fh_seg_ite);
			}
		}
	}
	//update the end points of best_fit_line.
	for (int nodenum = 0; nodenum < tpNode.size(); nodenum++)
	{
		bestline1 = tpNode[nodenum].best_fit_line_index;	//although called bestline1, it in fact updates all because this is a new loop.
		//tpNode[nodenum].best_fitted_line_points[0] = tpNode[nodenum].intersectCurve[bestline1]->curve[0];	//for single segment. These points are not used or display yet.
		//tpNode[nodenum].best_fitted_line_points[1] = *(tpNode[nodenum].intersectCurve[bestline1]->curve.Last());

		tpNode[nodenum].best_fitted_line_points[0] = tpNode[nodenum].intersectCurve[bestline1]->curves_seg_trim[0][0];
		int lastseg = tpNode[nodenum].intersectCurve[bestline1]->curves_seg_trim.size()-1;
		tpNode[nodenum].best_fitted_line_points[1] = *(tpNode[nodenum].intersectCurve[bestline1]->curves_seg_trim[lastseg].Last());

	}

}


//added on 20171111
double Planar::Distance(ON_3dVector p)
{
	ON_3dVector v = p - pOri;
	double d = ON_DotProduct(v, pNor);
	return d;
}

int EdgeIntersectPlane(ON_3dVector pa, ON_3dVector pb, Planar* p, ON_3dPoint& intpt)
{
	double d1, d2;
	d1 = p->Distance(pa);
	d2 = p->Distance(pb);
	if (d1*d2 > 0)
		return 0;	//no intersection
	else
	{
		if (d1 == 0 && d2 == 0)
		{
			intpt = pb;
			return 1;
		}
		else
		{
			intpt = pa + (pb - pa) * abs(d1) / (abs(d1) + abs(d2));
			return 1;
		}
	}
}

int PlaneIntersectMesh(FMesh* mesh, FMesh::VertexIter p0_it, Planar* p, ON_3dPointArray& intcurve)
{
	ON_3dVector nodeOri, nodeNor;	//nodeNor is actually plane normal.
	nodeOri = p->GetOrigin();
	nodeNor = p->GetNormal();

	OpenMesh::Vec3d p0(nodeOri.x, nodeOri.y, nodeOri.z);	//p0 is the given node.
	OpenMesh::Vec3d dis, pa, pb, ptemp;	//End points of an edge to check Plane/Edge intersection. From pa to pb.
	FMesh::VertexIter p0_v_it;	//store the iterator of p0.

	FMesh::VertexOHalfedgeIter p0_out_edge_it;	//outgoing halfedge starting from p0.
	FMesh::HalfedgeHandle p0_sur_edge_h;	//handle of edge surrounding p0. 
	FMesh::HalfedgeHandle temp_edge_h, oppo_edge_h; //for intersection propagation.
	vector<FMesh::HalfedgeHandle> initial_edge_h;	//initial intersection edges surrounding p0.
	FMesh::VertexHandle pa_v_h, pb_v_h;	//handle of end points.
	ON_3dVector pav, pbv;
	ON_3dPoint intpt;

	ON_3dVector tempv;	//for flipside check
	ON_3dVector nodeNormal;
	ptemp = mesh->normal(*p0_it);
	nodeNormal = ptemp.data();

	//int int_num=0;	//intersection numbers around p0
	ON_3dPointArray start_pts;	//initial intersection points next to p0. Up to 2 points if no duplication.
	start_pts.Destroy();
	start_pts.Reserve(4);
	bool initial_intersect = false;
	bool no_dup = true;	//duplicated points
	bool flipside = false;	//ensure all intersection lines have same orientation

	p0_out_edge_it = mesh->voh_iter(*p0_it);
	for (p0_out_edge_it; p0_out_edge_it.is_valid(); ++p0_out_edge_it)	//Circulate around the initial node.
	{
		if (!mesh->is_boundary(*p0_out_edge_it))	//If the edge is not on the boundary, then get its next_halfedge.
		{
			//get the surrounding edge
			p0_sur_edge_h = mesh->next_halfedge_handle(*p0_out_edge_it);
			pa_v_h = mesh->from_vertex_handle(p0_sur_edge_h);	//start point of the half edge.
			pb_v_h = mesh->to_vertex_handle(p0_sur_edge_h);	//end point of the half edge.
			pa = mesh->point(pa_v_h);
			pb = mesh->point(pb_v_h);
			pav = pa.data();
			pbv = pb.data();

			//check initial intersection points around p0
			if (EdgeIntersectPlane(pav, pbv, p, intpt))
			{
				if (!intpt)	//in case no intersection points. for bandit debug 20180627
					continue;

				if (!initial_intersect)	//for the first initial intersection points
				{
					start_pts.Append(intpt);
					initial_edge_h.push_back(p0_sur_edge_h);
					initial_intersect = true;
				}
				else //There are already intersection points. check if the points are duplicated
				{
					for (int i = 0; i < start_pts.Count(); i++)
					{
						if ((intpt - start_pts[i]).Length() < 1e-9)
						{
							no_dup = false;
						}
					}
					if (no_dup)
					{
						start_pts.Append(intpt);
						initial_edge_h.push_back(p0_sur_edge_h);
					}
				}
			}
		}
	}

	if (!initial_intersect)	//if no intersection
	{
		return 0;
	}
	else
	{		
		//expand from one side
		if (!start_pts[0])	//in case no intersection points. for bandit debug 20180627
			return 0;
		intcurve.Append(start_pts[0]);
		oppo_edge_h = mesh->opposite_halfedge_handle(initial_edge_h[0]);		
		while (!mesh->is_boundary(oppo_edge_h))
		{
			temp_edge_h = mesh->next_halfedge_handle(oppo_edge_h);
			while (temp_edge_h != oppo_edge_h)	//traverse the facet edges
			{
				pa_v_h = mesh->from_vertex_handle(temp_edge_h);	//start point of the half edge.
				pb_v_h = mesh->to_vertex_handle(temp_edge_h);	//end point of the half edge.
				pa = mesh->point(pa_v_h);
				pb = mesh->point(pb_v_h);
				pav = pa.data();
				pbv = pb.data();
				if (EdgeIntersectPlane(pav, pbv, p, intpt))
				{
					if (!intpt)	//in case no intersection points. for bandit debug 20180627
						continue;
					if (!intcurve)	//for debug 20180628. UNKNOWN bug. intcurve becomes NULL after .Append().
						return -1;

					intcurve.Append(intpt);
					oppo_edge_h = mesh->opposite_halfedge_handle(temp_edge_h);
					break;	//only look for one intersection
				}
				temp_edge_h = mesh->next_halfedge_handle(temp_edge_h);
			}
		}
		//check the line orientation. Doesn't work well at the stage. Evaluate globally later. UPDATE: check the plane normal orientation beforehand.
		tempv = start_pts[0] - nodeOri;	
		tempv = ON_CrossProduct(tempv, nodeNormal);
		tempv.Unitize();
		if (ON_DotProduct(tempv, nodeNor) < 0)	//Very important! Fix the orientation of intersection curve and the normal, so one can be represented by the other.
		{
			flipside = true;
		}/**/
		intcurve.Reverse();				
		intcurve.Append(nodeOri);	//add the original node to the list.

		//expand from another side if any
		if (start_pts.Count() > 1)	//expand from the other side
		{
			//if (!start_pts[1])	//in case no intersection points. for bandit debug 20180627
			//	return 0;
			
			intcurve.Append(start_pts[1]);
			oppo_edge_h = mesh->opposite_halfedge_handle(initial_edge_h[1]);
			while (!mesh->is_boundary(oppo_edge_h))
			{
				temp_edge_h = mesh->next_halfedge_handle(oppo_edge_h);
				while (temp_edge_h != oppo_edge_h)
				{
					pa_v_h = mesh->from_vertex_handle(temp_edge_h);	//start point of the half edge.
					pb_v_h = mesh->to_vertex_handle(temp_edge_h);	//end point of the half edge.
					pa = mesh->point(pa_v_h);
					pb = mesh->point(pb_v_h);
					pav = pa.data();
					pbv = pb.data();
					if (EdgeIntersectPlane(pav, pbv, p, intpt))
					{
						if (!intpt)	//in case no intersection points. for bandit debug 20180627
							continue;

						intcurve.Append(intpt);
						oppo_edge_h = mesh->opposite_halfedge_handle(temp_edge_h);
						break;	//only look for one intersection
					}
					temp_edge_h = mesh->next_halfedge_handle(temp_edge_h);
				}
			}
		}
		if (intcurve.Count()>2)	//ignore curves that has less than 2 points.
		{
			if (flipside)
			{
				intcurve.Reverse();
			}/**/
			return 1;
		}
		else
		{
			return 0;
		}
	}

	////for test
	//v_it = mesh->vertices_begin();
	//ptemp = mesh->point(*v_it);
	////p0_v_h = v_it.handle();
	//
	////Look for the corresponding node on the mesh according to the given point. And get its handle.
	//for (v_it = mesh->vertices_begin(); v_it != v_end; ++v_it)
	//{
	//	ptemp = mesh->point(*v_it);
	//	dis = p0 - ptemp;
	//	if (dis.length() < 1e-9)
	//	{
	//		p0_v_it = v_it;
	//		//p0_v_h = v_it.handle();
	//		break;
	//	}
	//}
}


int PlaneIntersectMeshThrough(FMesh* mesh, FMesh::VertexIter p0_it, Planar* p, vector<ON_3dPointArray>& intcurve_seg, vector<vector<FMesh::FaceHandle>>& curves_seg_fh)
{
	ON_3dVector nodeOri, nodeNor;	//nodeNor is actually plane normal.
	nodeOri = p->GetOrigin();
	nodeNor = p->GetNormal();

	OpenMesh::Vec3d p0(nodeOri.x, nodeOri.y, nodeOri.z);	//p0 is the given node.
	OpenMesh::Vec3d dis, pa, pb, ptemp;	//End points of an edge to check Plane/Edge intersection. From pa to pb.
	ON_3dVector pav, pbv;
	FMesh::VertexHandle pa_v_h, pb_v_h;	//handle of end points.
	ON_3dPoint intpt;

	FMesh::EdgeIter e_it=mesh->edges_begin();	//edge iterator
	FMesh::HalfedgeHandle he_h;
	vector<FMesh::HalfedgeHandle> int_bd_he_h;	//handle of intersection boundary edges
	FMesh::HalfedgeHandle temp_edge_h, oppo_edge_h; //for intersection propagation.
	//ON_3dPointArray int_bd_pts;	//intersection points on boundary edges
	vector<ON_3dPoint> int_bd_pts;	//intersection points on boundary edges
	ON_3dPointArray tempseg;	//temp segments
	ON_3dPoint temppt;
	vector<FMesh::FaceHandle> tempseg_fh;	//face handler of temp segments
	
	bool initial_intersect = false;

	//traverse all mesh boundary edges. Get the intersection points with the plane.
	for (e_it; e_it != mesh->edges_end(); ++e_it)
	{
		if (mesh->is_boundary(*e_it))
		{
			//get the nodes on the edge
			he_h = mesh->halfedge_handle(*e_it, 0);
			if (!mesh->is_boundary(he_h))	//make sure to get the "outside" halfedge
			{
				he_h = mesh->halfedge_handle(*e_it, 1);
			}
			pa_v_h = mesh->from_vertex_handle(he_h);
			pb_v_h = mesh->to_vertex_handle(he_h);
			pa = mesh->point(pa_v_h);
			pb = mesh->point(pb_v_h);
			pav = pa.data();
			pbv = pb.data();
			//check if the edge intersect with the plane.
			//Get the intersection points on the mesh boundaries
			if (EdgeIntersectPlane(pav, pbv, p, intpt))
			{
				int_bd_pts.push_back(intpt);
				int_bd_he_h.push_back(he_h);
				initial_intersect = true;
			}
		}
	}
	//Use the boundary intersection points to expand the intersection segments. "Consume" intersect points in the above list.
	if (!initial_intersect)	//if no intersection
	{
		return 0;
	}
	else
	{
		//"consume" the list of boundary intersection point. 
		//Pick the first in the list, expand the intersection curve "inside the mesh"until hit the other boundary.
		//Check the list again. Eliminate the points which is identical to the end point of the intersection segments.

		while (int_bd_pts.size())
		{			
			temppt = *(int_bd_pts.begin());
			oppo_edge_h = mesh->opposite_halfedge_handle(*(int_bd_he_h.begin()));
			int_bd_pts.erase(int_bd_pts.begin());
			int_bd_he_h.erase(int_bd_he_h.begin());

			tempseg.Empty();
			tempseg.Append(temppt);
			tempseg_fh.clear();

			while (!mesh->is_boundary(oppo_edge_h))
			{
				temp_edge_h = mesh->next_halfedge_handle(oppo_edge_h);
				while (temp_edge_h != oppo_edge_h)
				{
					pa_v_h = mesh->from_vertex_handle(temp_edge_h);	//start point of the half edge.
					pb_v_h = mesh->to_vertex_handle(temp_edge_h);	//end point of the half edge.
					pa = mesh->point(pa_v_h);
					pb = mesh->point(pb_v_h);
					pav = pa.data();
					pbv = pb.data();
					if (EdgeIntersectPlane(pav, pbv, p, intpt))
					{
						tempseg.Append(intpt);
						oppo_edge_h = mesh->opposite_halfedge_handle(temp_edge_h);
						tempseg_fh.push_back(mesh->face_handle(temp_edge_h));	//face handler list of intersection curves. The number is -1 than the intersection points, because the last one is on the boundary halfedge which don't have triangles!
						break;
					}
					temp_edge_h = mesh->next_halfedge_handle(temp_edge_h);
				}
			}
			//duplicate the last triangle, in order to make the size of list matches that of list of intersection points "tempseg".
			FMesh::FaceHandle fh = *(tempseg_fh.end()-1);
			tempseg_fh.push_back(fh);

			//delete duplicated points. E.g. the plane will cross the given sampling node.
			int ii = 0;
			while (ii< (tempseg.Count()-1) )
			{
				ON_3dPoint pp1, pp2;
				pp1 = tempseg[ii];
				pp2 = tempseg[ii + 1];
				if ((pp1 - pp2).Length() < 10e-6)
				{
					tempseg.Remove(ii+1);
					tempseg_fh.erase(tempseg_fh.begin() + ii + 1);	//face handler list of intersection curves
					continue;
				}
				ii++;
			}			

			intcurve_seg.push_back(tempseg);
			curves_seg_fh.push_back(tempseg_fh);	//face handler list of intersection curves

			//check the identical intersection points by checking the handler in the list. The terminate condition.
			int i = 0;
			for (i; i < int_bd_he_h.size(); i++)
			{
				if (oppo_edge_h.idx() == int_bd_he_h[i].idx())
				{
					break;
				}
			}
			int_bd_pts.erase(int_bd_pts.begin()+i);
			int_bd_he_h.erase(int_bd_he_h.begin()+i);
		}
		return 1;
	}


}




void TopoNode::GeneratrixHunt()	//hunt for the initial genetratrix
{
	int intersect_num = 0; //intersection number. "int_num" above. Redefine another one for easier understanding
	double er, lowest_er = 1e6;	//error
	int lowest_index;
	ON_3dPoint line_head, line_end;

	ON_3dPointArray tempCurve;
	tempCurve.Empty();
	//PCA fitting lines
	intersect_num = intersectCurve.size();
	for (int i = 0; i < intersect_num; i++)
	{
		if (intersectCurve[i]->is_intersect)
		{
			tempCurve.Empty();
			tempCurve = intersectCurve[i]->curve;
			Fitting3DPointsToLine(tempCurve, er, line_head, line_end);
			intersectCurve[i]->error = er;	//max error. can be mean error. check the function to make sure.
			intersectCurve[i]->fitted_line[0] = line_head;
			intersectCurve[i]->fitted_line[1] = line_end;/**/

			//find the best fitting line
			if (er < lowest_er)
			{
				lowest_er = er;
				lowest_index = i;
			}
		}
	}
	best_fit_line_index = lowest_index;
	best_fit_error = lowest_er;
	best_plane_nor = intersectCurve[lowest_index]->curveplane.GetNormal();
	huntting_done = true;
}


void TopoNode::GeneratrixHuntSeg(double RANSAC_plane_tol, bool planar_filter)	//hunt for the initial genetratrix
{
	double planar_tol = 0.05 * RANSAC_plane_tol;
	double planar_percent = 30.0 / 180.0;	//Criteria. Percentage of PCA directions whose error is smaller than planar_tol.
	int intersect_num = 0; //intersection number. "int_num" above. Redefine another one for easier understanding
	double er, lowest_er = 1e6;	//error
	int lowest_index;
	ON_3dPoint line_head, line_end;

	ON_3dPointArray tempCurve;
	tempCurve.Empty();
	//PCA fitting lines
	intersect_num = intersectCurve.size();
	int planar_num = 0;
	for (int i = 0; i < intersect_num; i++)
	{
		if (intersectCurve[i]->is_intersect_through)
		{
			tempCurve.Empty();
			int segsize = intersectCurve[i]->curves_seg_trim.size();
			for (int j = 0; j < segsize; j++)
			{
				tempCurve.Append(intersectCurve[i]->curves_seg_trim[j].Count(), intersectCurve[i]->curves_seg_trim[j].First());
			}
			//old 3D PCA without alignment version
			Fitting3DPointsToLine(tempCurve, er, line_head, line_end);	//Need to input a sorted tempCurve, otherwise the line_head and line_end are wrong. Because it doesn't check the bounding box, but only the first and last node. 
																		//20180427 Changed to use the first and second points to calculate the initial PCA. The PCA is finally updated during the segment sorting. 
			////2D PCA version
			//ON_3dVector pnor = intersectCurve[i]->curveplane.GetNormal();
			//Fitting3DPointsToLine2D(tempCurve, pnor, er, line_head, line_end, 0, 0);

			intersectCurve[i]->error = er;	//max error. can be mean error. check the function to make sure.
			intersectCurve[i]->fitted_line[0] = line_head;
			intersectCurve[i]->fitted_line[1] = line_end;
			intersectCurve[i]->is_PCA_done = true;
			/**/

			//find the best fitting line
			if (er < lowest_er)
			{
				lowest_er = er;
				lowest_index = i;
			}

			//evaluate planar/isotropic nodes
			if (er < planar_tol)
			{
				planar_num++;
			}
		}
	}

	//evaluate planar/isotropic nodes
	if (planar_filter)
	{
		double pp = planar_num / double(intersect_num);
		if ( pp > planar_percent && planar_num > 3 )	//Means the planar direction should be significant enough. To avoid 1/4 situation.
		{
			primType = 1;	//mark this node as planar/isotropic.
		}
	}

	best_fit_line_index = lowest_index;
	best_fit_error = lowest_er;
	best_plane_nor = intersectCurve[lowest_index]->curveplane.GetNormal();
	huntting_done = true;
}


void TopoNode::GeneratrixHunt(int fine_search_int_num)	//hunt for refine search iteration
{
	int intersect_num = 0; //intersection number. "int_num" above. Redefine another one for easier understanding
	double er, lowest_er = 1e6;	//error
	int lowest_index;
	ON_3dPoint line_head, line_end;

	ON_3dPointArray tempCurve;
	tempCurve.Empty();
	//PCA fitting lines
	intersect_num = intersectCurve.size();
	for (int i = (intersect_num - fine_search_int_num); i < intersect_num; i++)
	{
		if (intersectCurve[i]->is_intersect)
		{
			tempCurve.Empty();
			tempCurve = intersectCurve[i]->curve;
			Fitting3DPointsToLine(tempCurve, er, line_head, line_end);
			intersectCurve[i]->error = er;	//max error. can be mean error. check the function to make sure.
			intersectCurve[i]->fitted_line[0] = line_head;
			intersectCurve[i]->fitted_line[1] = line_end;/**/

			//find the best fitting line
			if (er < lowest_er)
			{
				lowest_er = er;
				lowest_index = i;
			}
		}
	}
	best_fit_line_index = lowest_index;
	best_fit_error = lowest_er;
	best_plane_nor = intersectCurve[lowest_index]->curveplane.GetNormal();
	huntting_done = true;
}


int TopoNode::GeneratrixHuntGolden(double t_interval, double t_error, ON_3dVector nodeRefVec, FMesh* mesh, FMesh::VertexIter p0_v_it)
{
	static const double lambda = 0.5*(sqrt(5) - 1);
	//static const double mu = 0.5*(3 - sqrt(5));	//1-lambda
	double aa = 0, bb = PI*(1 - (1/180.0)/10.0);	//initial interval
	double x1, x2;	//interval
	double err_x1, err_x2, err_aa, err_bb;	//error on the interval and end points.
	double err_temp = 1e6;
	ON_3dPoint line_head, line_end;
	ON_3dVector initial_vec = nodeRefVec, planeNor;

	Planar* int_plane = new Planar;
	//ON_3dPointArray tempCurve;
	//int ci = 0;	//curve index.

	//intersect and PCA evaluate for each direction
	//get initial [a,b].
	//Get initial a.
	planeNor = initial_vec;
	int_plane->SetOrigin(point);
	int_plane->SetNormal(planeNor);
	err_aa = IntersectAndEva(int_plane, mesh, p0_v_it);
	//Get initial b.
	planeNor = initial_vec;
	planeNor.Rotate(bb, normal);
	int_plane->SetNormal(planeNor);
	err_bb = IntersectAndEva(int_plane, mesh, p0_v_it);
	//Initial x1 and x2
	x1 = bb - lambda*(bb - aa);
	planeNor = initial_vec;
	planeNor.Rotate(x1, normal);
	int_plane->SetNormal(planeNor);
	err_x1 = IntersectAndEva(int_plane, mesh, p0_v_it);
	x2 = aa + lambda*(bb - aa);
	planeNor = initial_vec;
	planeNor.Rotate(x2, normal);
	int_plane->SetNormal(planeNor);
	err_x2 = IntersectAndEva(int_plane, mesh, p0_v_it);
	err_temp = min(err_x1, err_x2);
	while ( ((bb-aa) > t_interval) && (err_temp > t_error) )
	{
		//Should've checked if no intersect. This has been eliminated by selecting "internal vertices" which are not on the boundary facet.
		if (err_x1 > err_x2)
		{
			aa = x1;
			err_aa = err_x1;
			if (((bb - aa) < t_interval) || (err_temp < t_error))
				break;						
			x1 = x2;
			err_x1 = err_x2;
			x2 = aa + lambda*(bb - aa);
			planeNor = initial_vec;
			planeNor.Rotate(x2, normal);
			int_plane->SetNormal(planeNor);
			err_x2 = IntersectAndEva(int_plane, mesh, p0_v_it);
		}
		else
		{
			bb = x2;
			err_bb = err_x2;
			if (((bb - aa) < t_interval) || (err_temp < t_error))
				break;
			x2 = x1;
			err_x2 = err_x1;
			x1 = bb - lambda*(bb - aa);
			planeNor = initial_vec;
			planeNor.Rotate(x1, normal);
			int_plane->SetNormal(planeNor);
			err_x1 = IntersectAndEva(int_plane, mesh, p0_v_it);
		}
		err_temp = min(err_x1, err_x2);
	}
	best_fit_error = err_temp;
	best_fit_line_index = intersectCurve.size() - 1;	//
	if (ON_DotProduct( (intersectCurve[best_fit_line_index]->curveplane.GetNormal()), initial_vec) < 0)	//if has been rotate over 90 degree, then flip it, IN ORDER TO make sure the plane follows the same orientation for Rolling evaluation.
	{
		(intersectCurve[best_fit_line_index]->curveplane.GetNormal()) = -(intersectCurve[best_fit_line_index]->curveplane.GetNormal());
	}
	best_plane_nor = intersectCurve[best_fit_line_index]->curveplane.GetNormal();
	huntting_done = true;
	if (err_temp < t_error)
	{
		return 1;
	}
	else
	{
		return 0;
	}

	//IntersectCurve* int_curve = new IntersectCurve;
	//tempCurve.Empty();
	//intersectCurve.push_back(int_curve);
	//ci = intersectCurve.size() - 1;	//last element of intersect curve
	//intersectCurve[ci]->curveplane.SetOrigin(point);
	//intersectCurve[ci]->curveplane.SetNormal(planeNor);
	//if (PlaneIntersectMesh(mesh, p0_v_it, int_plane, tempCurve))
	//{
	//	intersectCurve[ci]->curve = tempCurve;
	//	intersectCurve[ci]->is_intersect = true;
	//	Fitting3DPointsToLine(tempCurve, err_temp, line_head, line_end);
	//	intersectCurve[ci]->error = err_temp;	//max error. can be mean error. check the function to make sure.
	//	intersectCurve[ci]->fitted_line[0] = line_head;
	//	intersectCurve[ci]->fitted_line[1] = line_end;
	//	err_aa = err_temp;
	//}
	////Get initial b.
	//planeNor = initial_vec;
	//planeNor.Rotate(bb, normal);

}


double TopoNode::IntersectAndEva(Planar* int_plane, FMesh* omesh, FMesh::VertexIter p0_v_it)
{		
	ON_3dPointArray tempCurve;
	int ci;	//index of current intersectCurve, i.e. the last one.
	ON_3dPoint line_head, line_end;
	double err_temp;	

	tempCurve.Empty();
	if (!huntting_done)	//check if the fitting process has been done. If so, return -2.
	{
		IntersectCurve* int_curve = new IntersectCurve;
		intersectCurve.push_back(int_curve);
		ci = intersectCurve.size() - 1;
		intersectCurve[ci]->curveplane.SetOrigin(int_plane->GetOrigin());
		intersectCurve[ci]->curveplane.SetNormal(int_plane->GetNormal());
		if (PlaneIntersectMesh(omesh, p0_v_it, int_plane, tempCurve))
		{
			intersectCurve[ci]->curve = tempCurve;
			intersectCurve[ci]->is_intersect = true;
			Fitting3DPointsToLine(tempCurve, err_temp, line_head, line_end);
			intersectCurve[ci]->error = err_temp;	//max error. can be mean error. check the function to make sure.
			intersectCurve[ci]->fitted_line[0] = line_head;
			intersectCurve[ci]->fitted_line[1] = line_end;
			return err_temp;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -2;	//If the best fit line has been found, return -2.
	}
}


void TopoNode::Roll(FMesh* omesh)
{
	if (huntting_done)
	{
		int num = NULL;
		FMesh::VertexIter v_it, v_end(omesh->vertices_end());
		ON_3dVector	pt, pt_vec;

		for (v_it = omesh->vertices_begin(); v_it != v_end; ++v_it)
		{
			pt = omesh->point(*v_it).data();
			pt_vec = pt - point;
			if (ON_DotProduct(pt_vec, best_plane_nor) < 0)
			{
				num += 1;
			}
		}
		rollover = num;
	}
}


void TopoNode::TrimSeg(double tol_seg)
{	
	int int_num = intersectCurve.size();

	for (int i = 0; i < int_num; i++)	//for each intersection direction
	{
		int ref_seg;
		bool ref_found;
		//get the segment which contains the node.
		ref_found = false;
		for (int seg_num = 0; seg_num < (intersectCurve[i]->curves_seg_raw).size(); seg_num++)	//for each segment on this direction.
		{
			for (int int_pt = 0; int_pt < (intersectCurve[i]->curves_seg_raw)[seg_num].Count(); int_pt++)	//for each intersection points on this segment.
			{
				ON_3dPoint temppt = intersectCurve[i]->curves_seg_raw[seg_num][int_pt];
				if ((temppt - point).Length() < 10e-9)	//got the TopoNode
				{
					ref_seg = seg_num;
					ref_found = true;
					break;
				}
			}
			if (ref_found)
				break;
		}
		//save the reference segment in the first segment position
		/*ON_3dPointArray refseg = intersectCurve[i]->curves_seg_raw[ref_seg];
		vector<FMesh::FaceHandle> refseg_fh = intersectCurve[i]->curves_seg_fh_raw[ref_seg];
		curves_seg_sort_temp.push_back(refseg);
		curves_seg_fh_sort_temp.push_back(refseg_fh);*/

		//copy the segments out, in order to preserve the raw segment data. delete the reference segments.
		vector<ON_3dPointArray> seg_raw_copy = intersectCurve[i]->curves_seg_raw;
		vector<vector<FMesh::FaceHandle>> seg_fh_raw_copy = intersectCurve[i]->curves_seg_fh_raw;
		/*seg_raw_copy.erase(seg_raw_copy.begin() + ref_seg);
		seg_fh_raw_copy.erase(seg_fh_raw_copy.begin() + ref_seg);*/

		//evaluate the distance from this segment and end points of other segments. Delete those with tolerance > tol.
		ON_3dVector ref_ori, tempa, tempb, proja, projb;	//distance from the end points of other segments to the reference segment.
		double dista, distb;
		ON_3dPoint ref_seg_a, ref_seg_b, temp_seg_a, temp_seg_b;	//end points of the segment.
		ref_seg_a = *(intersectCurve[i]->curves_seg_raw[ref_seg].First());
		ref_seg_b = *(intersectCurve[i]->curves_seg_raw[ref_seg].Last());
		ref_ori = ref_seg_b - ref_seg_a;
		ref_ori.Unitize();
		int seg_num = 0;
		while (seg_num < seg_raw_copy.size())
		{
			temp_seg_a = *(seg_raw_copy[seg_num].First());
			temp_seg_b = *(seg_raw_copy[seg_num].Last());
			tempa = temp_seg_a - ref_seg_a;
			tempb = temp_seg_b - ref_seg_a;
			proja = ON_DotProduct(tempa, ref_ori) * ref_ori;
			projb = ON_DotProduct(tempb, ref_ori) * ref_ori;
			dista = (tempa - proja).Length();
			distb = (tempb - projb).Length();
			if (((dista + distb) / 2) > tol_seg)	//if this segment is too far apart then delete it, then continue to the next segment.
			{
				seg_raw_copy.erase(seg_raw_copy.begin() + seg_num);
				seg_fh_raw_copy.erase(seg_fh_raw_copy.begin() + seg_num);
				continue;
			}
			//If the segment is align with the reference segment, then align with the ref_ori by flipping the orientation if necessary. 
			if (ON_DotProduct((temp_seg_b - temp_seg_a), ref_ori) < 0)
			{
				seg_raw_copy[seg_num].Reverse();
				std::reverse(seg_fh_raw_copy[seg_num].begin(), seg_fh_raw_copy[seg_num].end());
			}
			seg_num++;
		}
		//output the result to IntersectCurves
		(intersectCurve[i])->curves_seg_trim = seg_raw_copy;
		(intersectCurve[i])->curves_seg_fh_trim = seg_fh_raw_copy;
	}
	is_trimmed = true;
}


void TopoNode::SortSeg(int cn)
{
	if ((intersectCurve[cn]->is_PCA_done) )
	{
		ON_3dVector ref_ori;
		ON_3dPoint ref_a, ref_b;
		ref_a = intersectCurve[cn]->fitted_line[0];
		ref_b = intersectCurve[cn]->fitted_line[1];
		ref_ori = ref_b - ref_a;	//use the PCA as reference. CAUTION: need to check the direction of normal and PCA. Make sure they form the right-hand CS.
		ref_ori.Unitize();

		//IMPORTANT! Check if node normal, plane normal and PCA form a right-hand CS. If not, flip PCA and all segments.
		ON_3dVector align_ori, cnor = intersectCurve[cn]->curveplane.GetNormal();	//curve normal
		align_ori = ON_CrossProduct(normal, cnor);
		align_ori.Unitize();
		double d = ON_DotProduct(ref_ori, align_ori);
		if (d < 0)
		{
			//flip PCA
			intersectCurve[cn]->fitted_line[0] = ref_b;
			intersectCurve[cn]->fitted_line[1] = ref_a;
			ref_a = intersectCurve[cn]->fitted_line[0];
			ref_b = intersectCurve[cn]->fitted_line[1];
			ref_ori = ref_b - ref_a;
			ref_ori.Unitize();
			//flip all segments
			for (int i = 0; i < intersectCurve[cn]->curves_seg_trim.size(); i++)
			{
				intersectCurve[cn]->curves_seg_trim[i].Reverse();
				auto fh_seg_itb = intersectCurve[cn]->curves_seg_fh_trim[i].begin();
				auto fh_seg_ite = intersectCurve[cn]->curves_seg_fh_trim[i].end();
				std::reverse(fh_seg_itb, fh_seg_ite);
			}
		}

		//Sort segments if there is more than 1 segment. Intersection sort.
		if ((intersectCurve[cn]->curves_seg_trim.size()>1))
		{
			ON_3dPointArray seg_temp, seg_pre;
			vector<FMesh::FaceHandle> seg_fh_temp, seg_fh_pre;
			ON_3dVector v_i, v_j;
			
			int i, j, seg_num = intersectCurve[cn]->curves_seg_trim.size();
			for (i = 1; i < seg_num; i++)
			{
				//wrong version
				/*double di, dj;
				seg_temp = intersectCurve[cn]->curves_seg_trim[i];
				seg_fh_temp = intersectCurve[cn]->curves_seg_fh_trim[i];

				di = ON_DotProduct((*(seg_temp.First()) - ref_a), ref_ori);	//distance from i-th segment's First points to ref_ori.

				j = i - 1;
				seg_pre = intersectCurve[cn]->curves_seg_trim[j];
				dj = ON_DotProduct((*(seg_pre.First()) - ref_a), ref_ori);

				while ((j >= 0) && (dj > di))
				{
					seg_pre = intersectCurve[cn]->curves_seg_trim[j];
					dj = ON_DotProduct((*(seg_pre.First()) - ref_a), ref_ori);
					intersectCurve[cn]->curves_seg_trim[j + 1] = intersectCurve[cn]->curves_seg_trim[j];
					intersectCurve[cn]->curves_seg_fh_trim[j + 1] = intersectCurve[cn]->curves_seg_fh_trim[j];
					intersectCurve[cn]->curves_seg_trim[j] = seg_temp;
					intersectCurve[cn]->curves_seg_fh_trim[j] = seg_fh_temp;
					j--;
				}*/
	
				for (int j = i - 1; j >= 0; j--)
				{
					double di, dj;
					di = ON_DotProduct((*(intersectCurve[cn]->curves_seg_trim[j+1].First()) - ref_a), ref_ori);	//distance from i-th segment's First points to ref_ori.
					dj = ON_DotProduct((*(intersectCurve[cn]->curves_seg_trim[j].First()) - ref_a), ref_ori);
					if (dj < di)
						break;
					seg_temp = intersectCurve[cn]->curves_seg_trim[j+1];
					seg_fh_temp = intersectCurve[cn]->curves_seg_fh_trim[j+1];
					intersectCurve[cn]->curves_seg_trim[j + 1] = intersectCurve[cn]->curves_seg_trim[j];
					intersectCurve[cn]->curves_seg_fh_trim[j + 1] = intersectCurve[cn]->curves_seg_fh_trim[j];
					intersectCurve[cn]->curves_seg_trim[j] = seg_temp;
					intersectCurve[cn]->curves_seg_fh_trim[j] = seg_fh_temp;
				}
			}
		}

		//extend PCA's two end points to the end points of the final line segments
		double di, dj;
		di = ON_DotProduct((intersectCurve[cn]->curves_seg_trim[0][0] - ref_a), ref_ori);	//distance from original PCA head
		intersectCurve[cn]->fitted_line[0] = intersectCurve[cn]->fitted_line[0] + di*ref_ori;
		dj = ON_DotProduct((*(intersectCurve[cn]->curves_seg_trim[(intersectCurve[cn]->curves_seg_trim.size()-1)].Last()) - ref_b), ref_ori);
		intersectCurve[cn]->fitted_line[1] = intersectCurve[cn]->fitted_line[1] + dj*ref_ori;
	}
}


int	Tooth::Roll(int ni, int roll_type)
{
	m_oMesh;
	int ci = tpNode[ni].best_fit_line_index;
	vector<vector<FMesh::FaceHandle>> seg_fh = tpNode[ni].intersectCurve[ci]->curves_seg_fh_trim;	//vector<vector<FMesh::FaceHandle>>
	int rol_num=0;
	FMesh::FaceVertexIter tri;
	ON_3dPoint pt, pa, pb;
	ON_3dVector pv, pav, pbv;
	
	ON_3dPoint node;
	ON_3dPointArray node_list;
	vector<FMesh::VertexHandle> nh_list;	//handle list of openmesh nodes.	
	vector<FMesh::HalfedgeHandle> bdry_hf;	//half edge handler of boundary half edges that on the left side of the intersection line.
	FMesh::FaceHalfedgeIter he_tri;

	//Initialization. reset vertex color for calculation of other tpNodes. Comment for single node check.
	FMesh::VertexIter v_it, v_end(m_oMesh->vertices_end());
	for (v_it = m_oMesh->vertices_begin(); v_it != v_end; ++v_it)
	{
		m_oMesh->data(*v_it).patch = 0;
	}


	//paint all nodes of intersection triangle red
	for (int si = 0; si < seg_fh.size(); si++)	//all segments
	{
		for (int ti = 0; ti < seg_fh[si].size(); ti++)	//all triangles
		{			
			tri = m_oMesh->fv_iter(seg_fh[si][ti]);
			for (tri; tri.is_valid(); ++tri)
			{
				pt = m_oMesh->point(*tri).data();
				//check if the node is on the left side of intersection plane.
				pv = pt - tpNode[ni].point;
				if (ON_DotProduct(pv, tpNode[ni].best_plane_nor) > 0 && roll_type)	//If on the right side then ignore
					continue;

				//paint color and update the roll number
				m_oMesh->data(*tri).patch = 1;
				rol_num++;
				//Get the node handler list
				nh_list.push_back(*tri);
				//Get the node and put to a ON_3dPointArray
				node_list.Append(pt);
				
			}

			//get the half edge boundary on the left side
			he_tri = m_oMesh->fh_iter(seg_fh[si][ti]);
			for (he_tri; he_tri.is_valid(); ++he_tri)
			{
				auto vi_f = m_oMesh->from_vertex_handle(*he_tri);
				pa = m_oMesh->point(vi_f).data();
				pav = pa - tpNode[ni].point;
				auto vi_t = m_oMesh->to_vertex_handle(*he_tri);
				pb = m_oMesh->point(vi_t).data();
				pbv = pb - tpNode[ni].point;

				
				if ( (ON_DotProduct(pav, tpNode[ni].best_plane_nor) <0) && (ON_DotProduct(pbv, tpNode[ni].best_plane_nor) <0) )	//if on the left
				{
					bdry_hf.push_back(*he_tri);
				}
			}

		}
	}

	//The exact node may be ignore due to precision error. Paint it and add it manually.
	if (roll_type)
	{
		FMesh::VertexIter nodeiter = tpNode[ni].node_it;
		m_oMesh->data(*nodeiter).patch = 1;
		rol_num++;
		nh_list.push_back(*nodeiter);
		node_list.Append(pt);
	}

	//////flood-fill to paint the left-side of intersection points//////
	FMesh::HalfedgeHandle last_he, op_he, next_he;
	FMesh::VertexHandle op_vh;
	auto he_list_temp = bdry_hf;
	while (he_list_temp.size())
	{
		last_he = *(he_list_temp.end() - 1);
		he_list_temp.pop_back();
		if (m_oMesh->is_boundary(last_he))	//if on the boundary then ignore
			continue;
		op_he = m_oMesh->opposite_halfedge_handle(last_he);	//get the opposite he
		next_he = m_oMesh->next_halfedge_handle(op_he);	//next half edge
		op_vh = m_oMesh->to_vertex_handle(next_he);	//the opposite node
		int clr = m_oMesh->data(op_vh).patch;
		if (clr == 0)	//if white color, then paint and proceed.
		{
			m_oMesh->data(op_vh).patch = 1;
			rol_num++;
			//update list
			nh_list.push_back(op_vh);
			pt = m_oMesh->point(op_vh).data();
			node_list.Append(pt);

			he_list_temp.push_back(next_he);
			next_he = m_oMesh->next_halfedge_handle(next_he);
			he_list_temp.push_back(next_he);
		}
	}

	
	//output
	tpNode[ni].rollover = rol_num;
	tpNode[ni].roll_vh = nh_list;
	tpNode[ni].roll_node = node_list;
	is_rolled = 1; //flag for display check
	tpNode[ni].roll_heh = bdry_hf;	//for display check. Initial boundary half-edge
	return 1;
}


int	Tooth::FixPlanarRegion()
{
	int sz = tpNode.size();
	int planar_num = 0;
	std::vector<FMesh::VertexIter> remain_it;
	for (int i = 0; i < sz; i++)
	{
		if (tpNode[i].primType == 1)
		{
			planar_num++;
			remain_it.push_back(tpNode[i].node_it);
		}
	}
	if (planar_num == sz)	//all nodes are planar nodes. WIP select randomly a point, then force all nodes to align to it?
	{
		cout << "All nodes are isotropic nodes." << endl;
		//select the first node as reference, set this node as non-planar for the following process, then force all nodes to align to it.
		FMesh::VertexIter ref_node_it = tpNode[0].node_it;
		tpNode[0].primType = 0;

		//Use the node as reference to sort the planar nodes according to their distances from this reference node.
		//m_oMesh;
		std::vector<FMesh::Point> subsampled_points_openmesh;
		std::vector<FMesh::Normal> subsampled_normal_openmesh;	//No use here. Used in RANSAC.
		GetOMeshPointNNormal(m_oMesh, remain_it, subsampled_points_openmesh, subsampled_normal_openmesh);	//For Dijkstra.
		std::vector<int> dist_list;	//distance sequence list. Show the point list according to the distance to the start_point on the mesh.
		FMesh::Point start_point = m_oMesh->point(*ref_node_it);
		DijkstraAlgorithm(m_oMesh, subsampled_points_openmesh, start_point, dist_list);

		OpenMesh::Vec3d ptemp, p0;
		for (int i = 0; i < remain_it.size(); i++)
		{
			//Look for closest non-planar tpNode as reference node
			ptemp = subsampled_points_openmesh[i];
			double dist, min_dist = 10e9;
			int closest_idx;
			for (int j = 0; j < sz; j++)
			{
				if (tpNode[j].primType == 1)
					continue;
				p0 = m_oMesh->point(*(tpNode[j].node_it));
				dist = (ptemp - p0).length();
				if (dist < min_dist)
				{
					min_dist = dist;
					closest_idx = j;
				}
			}

			//Find the intersectioni direction which is best aligned to the reference node.
			////ON_3dVector ref_PCA = tpNode[closest_idx].best_fitted_line_points[1] - tpNode[closest_idx].best_fitted_line_points[0];	//WRONG. Haven't got this yet. This is obtained after global alignment.
			ON_3dVector ref_PCA = (tpNode[closest_idx].intersectCurve[tpNode[closest_idx].best_fit_line_index])->fitted_line[1] - (tpNode[closest_idx].intersectCurve[tpNode[closest_idx].best_fit_line_index])->fitted_line[0];
			ref_PCA.Unitize();
			ON_3dVector temp_PCA, dist_to_ref_PCA;
			double dist2, max_dist = 0;
			int closest_idx2;
			int intersect_sz = tpNode[i].intersectCurve.size();
			for (int intersect_idx = 0; intersect_idx < intersect_sz; intersect_idx++)
			{
				temp_PCA = (tpNode[i].intersectCurve[intersect_idx])->fitted_line[1] - (tpNode[i].intersectCurve[intersect_idx])->fitted_line[0];
				temp_PCA.Unitize();
				dist2 = ON_DotProduct(temp_PCA, ref_PCA);
				if (abs(dist2) > max_dist)	//when two PCA are aligned, the distance returned is MAXIMUM (-1 or 1).
				{
					max_dist = abs(dist2);
					closest_idx2 = intersect_idx;
				}
			}

			//Get back the tpNode index
			int tpNode_idx = m_oMesh->data(*remain_it[i]).TopoNode_idx;
			//Update the best_fit_line_index, best_fit_error, best_plane_nor, best_fitted_line_points, accordingly.
			tpNode[tpNode_idx].best_fit_line_index = closest_idx2;
			tpNode[tpNode_idx].best_fit_error = (tpNode[tpNode_idx].intersectCurve[closest_idx2])->error;
			tpNode[tpNode_idx].best_plane_nor = (tpNode[tpNode_idx].intersectCurve[closest_idx2])->curveplane.GetNormal();
			tpNode[tpNode_idx].SortSeg(tpNode[tpNode_idx].best_fit_line_index);

			//Set the node to non-isotropic node.
			tpNode[tpNode_idx].primType = 0;
		}

		return -1;
	}
	else if(planar_num == 0)
	{
		cout << "No isotropic node." << endl;
		return -1;
	}

	//Look for one non-planar node.
	FMesh::VertexIter ref_node_it;
	for (int i = 0; i < sz; i++)
	{
		if (tpNode[i].primType != 1)
		{
			ref_node_it = tpNode[i].node_it;
			break;
		}
	}
	//Use the node as reference to sort the planar nodes according to their distances from this reference node.
	//m_oMesh;
	std::vector<FMesh::Point> subsampled_points_openmesh;
	std::vector<FMesh::Normal> subsampled_normal_openmesh;	//No use here. Used in RANSAC.
	GetOMeshPointNNormal(m_oMesh, remain_it, subsampled_points_openmesh, subsampled_normal_openmesh);	//For Dijkstra.
	std::vector<int> dist_list;	//distance sequence list. Show the point list according to the distance to the start_point on the mesh.
	FMesh::Point start_point = m_oMesh->point(*ref_node_it);
	DijkstraAlgorithm(m_oMesh, subsampled_points_openmesh, start_point, dist_list);

	OpenMesh::Vec3d ptemp, p0;
	for (int i = 0; i < remain_it.size(); i++)
	{
		//Look for closest non-planar tpNode as reference node
		ptemp = subsampled_points_openmesh[i];
		double dist, min_dist = 10e9;
		int closest_idx;
		for (int j = 0; j < sz; j++)
		{
			if (tpNode[j].primType == 1)
				continue;
			p0 = m_oMesh->point(*(tpNode[j].node_it));
			dist = (ptemp - p0).length();
			if ( dist < min_dist)
			{
				min_dist = dist;
				closest_idx = j;
			}
		}

		//Find the intersectioni direction which is best aligned to the reference node.
		////ON_3dVector ref_PCA = tpNode[closest_idx].best_fitted_line_points[1] - tpNode[closest_idx].best_fitted_line_points[0];	//WRONG. Haven't got this yet. This is obtained after global alignment.
		ON_3dVector ref_PCA = (tpNode[closest_idx].intersectCurve[tpNode[closest_idx].best_fit_line_index])->fitted_line[1] - (tpNode[closest_idx].intersectCurve[tpNode[closest_idx].best_fit_line_index])->fitted_line[0];
		ref_PCA.Unitize();
		ON_3dVector temp_PCA, dist_to_ref_PCA;
		double dist2, max_dist = 0;
		int closest_idx2;
		int intersect_sz = tpNode[i].intersectCurve.size();
		for (int intersect_idx = 0; intersect_idx < intersect_sz; intersect_idx++)
		{
			temp_PCA = (tpNode[i].intersectCurve[intersect_idx])->fitted_line[1] - (tpNode[i].intersectCurve[intersect_idx])->fitted_line[0];
			temp_PCA.Unitize();
			dist2 = ON_DotProduct(temp_PCA, ref_PCA);
			if (abs(dist2) > max_dist)	//when two PCA are aligned, the distance returned is MAXIMUM (-1 or 1).
			{
				max_dist = abs(dist2);
				closest_idx2 = intersect_idx;
			}
		}

		//Get back the tpNode index
		int tpNode_idx = m_oMesh->data(*remain_it[i]).TopoNode_idx;
		//Update the best_fit_line_index, best_fit_error, best_plane_nor, best_fitted_line_points, accordingly.
		tpNode[tpNode_idx].best_fit_line_index = closest_idx2;
		tpNode[tpNode_idx].best_fit_error = (tpNode[tpNode_idx].intersectCurve[closest_idx2])->error;
		tpNode[tpNode_idx].best_plane_nor = (tpNode[tpNode_idx].intersectCurve[closest_idx2])->curveplane.GetNormal();
		tpNode[tpNode_idx].SortSeg(tpNode[tpNode_idx].best_fit_line_index);

		//Set the node to non-isotropic node.
		tpNode[tpNode_idx].primType = 0;		
	}

	return 0;
}

//////added by ZR end.//////