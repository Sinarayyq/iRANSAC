#include "Jig.h"


Jig::Jig(int n)
{
	m_iEntType = JIG;
	// progress bar
	progresswidget = new ProgressWidget();
	progresswidgetExport = new ProgressWidget();
	
}

//void Jig::CreateJig(Teeth* teethin, const vector<int> &selected)
//{
//	int n = selected.size();
//	i_selected = selected;
//	progresswidget->setRange(0, n);
//	progresswidget->setValue(0);
//	for (int i = 0; i < n; i++)
//	{
//		progresswidget->setValue(i+1);
//		JigBlock* JB = new JigBlock(teethin->upperorlower, teethin->GetTooth(selected[i]), BRACKET_HEIGHT + JIGOFFSET + 0.5*BRACKET_SLOT + JIGOVERHEIGHT, JIGWIDTH, 80);
//		m_blocks.push_back(JB);
//	}
//	ConnectBlocks_curvy(teethin->upperorlower);
//}

void Jig::CreateJig2(Teeth* teethin, const vector<int> &selected, double jigwidth, double occluheight, double res)
{
	resol = res;
	int n = selected.size();
	i_selected = selected;
	progresswidget->setRange(0, n);
	progresswidget->setValue(0);
	int upperorlower = teethin->upperorlower;
	double width;
	for (int i = 0; i < n; i++)
	{
		progresswidget->setValue(i + 1);
		double height;
		if (teethin->GetTooth(selected[i])->m_bracket)
		{
			height = (teethin->GetTooth(selected[i])->m_bracket->origin - teethin->GetTooth(selected[i])->GetBottomPnt(upperorlower)) * teethin->GetTooth(selected[i])->m_bracket->zaxis + 0.5*BRACKET_SLOT + JIGOVERHEIGHT + JIGOFFSET;
			width = jigwidth + 1; //thicken the wall
		}
		else
		{
			height = BRACKET_HEIGHT + JIGOFFSET + 0.5*BRACKET_SLOT + JIGOVERHEIGHT;
			width = jigwidth;
		}
		PntBlock *pb = new PntBlock(upperorlower, teethin->GetTooth(selected[i]), height, width, MAXDIS, resol, occluheight);
		MeshBlock *JB = new MeshBlock();
		JB->MeshReconstruct(pb);
		mblocks.push_back(JB);
		delete pb;
	}
	ConnectmBlocks_curvy(teethin->upperorlower);
}

Jig::~Jig()
{
	//for (int i = 0; i < m_blocks.size(); i++)
	//	delete m_blocks[i];
	//for (int i = 0; i < m_connectors.size(); i++)
	//	delete m_connectors[i];
	for (int i = 0; i < mblocks.size(); i++)
		delete mblocks[i];
	for (int i = 0; i < mconnectors.size(); i++)
		delete mconnectors[i];
	if (progresswidget != NULL)
		delete progresswidget;
	if (progresswidgetExport != NULL)
		delete progresswidgetExport;
}

//void Jig::ConnectBlocks(double upper, double lower, double left, double right)
//{
//	for (int i = 0; i < m_blocks.size() - 1; i++)
//	{
//		ON_3dPointArray leftbnd = m_blocks[i]->DigAHole(5, true, upper, lower, left, right);
//		ON_3dPointArray rightbnd = m_blocks[i + 1]->DigAHole(4, false, 1-lower, 1-upper, left, right);
//		JigFace *JF = new JigFace(rightbnd, leftbnd, 20);
//		m_connectors.push_back(JF);
//	}
//}

//void Jig::ConnectBlocks_curvy(int upperorlower, double upper_ini /*= 0.1*/, double lower_ini /*= 0.5*/, double width /*= 0.3*/)
//{
//	for (int i = 0; i < m_blocks.size() - 1; i++)
//	{
//		double upper = upper_ini, lower = lower_ini;
//		int facei = 0;
//		ON_3dPointArray leftp, rightp;
//		int xn, yn;
//		double left_0 = 1 - width, right_0 = 1, left_1 = 0, right_1 = width;
//		leftp = m_blocks[i]->GetFace(facei)->GetNodes(true, upper, lower, left_0, right_0, xn, yn);
//		rightp = m_blocks[i + 1]->GetFace(facei)->GetNodes(false, upper, lower, left_1, right_1, xn, yn);
//		xn--;
//		yn--;
//		double height = (leftp[1] - leftp[0]).Length();
//		double ratio[4] = { 4, 2, 2, 4 };
//		if ((leftp[1] - rightp[1]).Length() <= 1.5 * height) //not enough space to create connector
//		{
//			if (upperorlower == UPPER)
//			{
//				facei = 3;
//				ratio[0] = 3, ratio[1] = 1.5, ratio[2] = 1.5, ratio[3] = 3;
//			}
//			else
//			{
//				facei = 2;
//				ratio[0] = 1.5, ratio[1] = 3, ratio[2] = 3, ratio[3] = 1.5;
//			}
//			upper = 0.35, lower = 0.65;
//			left_0 = 0, right_0 = width;
//			leftp = m_blocks[i]->GetFace(facei)->GetNodes(true, upper, lower, left_0, right_0, yn, xn);
//			rightp = m_blocks[i + 1]->GetFace(facei)->GetNodes(false, upper, lower, left_1, right_1, yn, xn);
//			xn--;
//			yn--;
//			ON_3dPoint temp_pnt = leftp[3];
//			leftp.Insert(0, temp_pnt);
//			leftp.Remove(4);
//			temp_pnt = rightp[0];
//			rightp.Insert(4, temp_pnt);
//			rightp.Remove(0);
//			height = (leftp[1] - leftp[0]).Length();
//			
//		}
//		ON_3dVector leftnormal = m_blocks[i]->GetFace(facei)->GetNormal(0), rightnormal = m_blocks[i + 1]->GetFace(facei)->GetNormal(0);
//		ON_3dPointArray curves[4];
//		for (int j = 0; j < 4; j++)
//		{
//			ON_3dPointArray cv(4);
//			cv.Insert(0, leftp[j]);
//			cv.Insert(1, leftp[j] + leftnormal*height*ratio[j]);
//			cv.Insert(2, rightp[j] + rightnormal*height*ratio[j]);
//			cv.Insert(3, rightp[j]);
//			ON_BezierCurve BC(cv);
//			for (int k = 0; k < 20; k++)
//			{
//				curves[j].Insert(k, BC.PointAt((double)k / 19));
//			}
//		}
//		JigFace *JF1 = new JigFace(curves[0], curves[1], xn);
//		m_connectors.push_back(JF1);
//		JigFace *JF2 = new JigFace(curves[1], curves[2], yn);
//		m_connectors.push_back(JF2);
//		JigFace *JF3 = new JigFace(curves[2], curves[3], xn);
//		m_connectors.push_back(JF3);
//		JigFace *JF4 = new JigFace(curves[3], curves[0], yn);
//		m_connectors.push_back(JF4);
//		m_blocks[i]->DigAHole(facei, true, upper, lower, left_0, right_0);
//		m_blocks[i + 1]->DigAHole(facei, false, upper, lower, left_1, right_1);
//	}
//}

void Jig::ConnectmBlocks_curvy(int upperorlower, double upper /*= 0.3*/, double lower /*= 0.7*/, double width /*= 0.3*/)
{
	for (int i = 0; i < mblocks.size() - 1; i++)
	{
		if (i_selected[i + 1] - i_selected[i]>1)
			continue;
		ON_3dPoint leftcor = mblocks[i]->origin + mblocks[i]->xdir*mblocks[i]->xlength;
		ON_3dPoint rightcor = mblocks[i + 1]->origin;
		double height = mblocks[i]->ylength*width;
		ON_3dPointArray leftp, rightp;
		if ((leftcor - rightcor).Length() <= 1.5*height)
		{
			continue;
		}
		else
		{
			leftp.Insert(0, mblocks[i]->Coord(1 - width, upper, 0));
			leftp.Insert(1, mblocks[i]->Coord(1, upper , 0));
			leftp.Insert(2, mblocks[i]->Coord(1, lower, 0));
			leftp.Insert(3, mblocks[i]->Coord(1 - width, lower, 0));
			rightp.Insert(0, mblocks[i + 1]->Coord(width, upper, 0));
			rightp.Insert(1, mblocks[i + 1]->Coord(0, upper, 0));
			rightp.Insert(2, mblocks[i + 1]->Coord(0, lower, 0));
			rightp.Insert(3, mblocks[i + 1]->Coord(width, lower, 0));			
		}
		JigBlock *cb = new JigBlock(leftp, rightp, resol);
		mconnectors.push_back(cb);
	}
}

void Jig::Update()
{
	FT::FEntity::Update();
}

void Jig::DrawSmoothShade()
{
	if (m_isHide)  return;
	for (int i = 0; i < mblocks.size(); i++)
		mblocks[i]->DrawSmoothShade();
	for (int i = 0; i < mconnectors.size(); i++)
		mconnectors[i]->DrawSmoothShade();
}

void Jig::DrawMesh()
{
	for (int i = 0; i < mblocks.size(); i++)
		mblocks[i]->DrawMesh();
	for (int i = 0; i < mconnectors.size(); i++)
		mconnectors[i]->DrawMesh();
}

void Jig::InitDisplayList()
{
	for (int i = 0; i < mblocks.size(); i++)
		mblocks[i]->Update();
	for (int i = 0; i < mconnectors.size(); i++)
		mconnectors[i]->Update();
}



JigBlock::JigBlock(int upperorlower, Tooth *toothin, double height, double width, int density)
{
	m_tooth = toothin;
	ON_3dPoint	origin;
	ON_3dVector nor, xdir;
	int sign = -1;
	if (upperorlower == UPPER)
		sign = 1;
	if (m_tooth->m_bracket)
	{
		origin = m_tooth->m_bracket->origin;
		nor = m_tooth->m_bracket->xaxis, xdir = m_tooth->m_bracket->yaxis;
		origin = origin + nor * 2 - m_tooth->m_bracket->zaxis * sign *(height/2-0.5*BRACKET_SLOT-JIGOVERHEIGHT);
	}
	else
	{
		origin = m_tooth->GetBottomPnt(upperorlower) + m_tooth->longaxis*(height / 2 - JIGOFFSET)*sign;
		nor = -m_tooth->yaxis, xdir = m_tooth->xaxis;
		origin = origin + nor * 5;
	}	
	m_faces[0] = new JigFace(origin, nor, xdir, height, width, density); //front
	m_faces[1] = new JigFace;  //rear
	*m_faces[1] = *m_faces[0];
	m_faces[1]->RayProjection_f(-nor, MAXDIS, m_tooth, mindis2tooth);
	m_faces[0]->Translate(nor*(MINDIS-mindis2tooth));
	m_faces[2] = new JigFace(m_faces[0]->GetBoundary(1), m_faces[1]->GetBoundary(1), density);
	m_faces[3] = new JigFace(m_faces[0]->GetBoundary(3), m_faces[1]->GetBoundary(3), density);
	m_faces[4] = new JigFace(m_faces[0]->GetBoundary(0), m_faces[1]->GetBoundary(0), density);
	m_faces[5] = new JigFace(m_faces[0]->GetBoundary(2), m_faces[1]->GetBoundary(2), density);
}

JigBlock::JigBlock(ON_3dPointArray leftbnd, ON_3dPointArray rightbnd, double resol)
{
	ON_3dVector leftnormal = -ON_CrossProduct(leftbnd[1] - leftbnd[0], leftbnd[2] - leftbnd[1]);
	ON_3dVector rightnormal = ON_CrossProduct(rightbnd[1] - rightbnd[0], rightbnd[2] - rightbnd[1]);
	leftnormal.Unitize();
	rightnormal.Unitize();
	ON_3dPointArray curves[4];
	double ratio[4] = { 3.5, 2, 2, 3.5 };
	double width = (leftbnd[1] - leftbnd[0]).Length(), height = (leftbnd[2]-leftbnd[1]).Length();
	int xn = round(width / resol), yn = round(height / resol);
	for (int j = 0; j < 4; j++)
	{
		ON_3dPointArray cv(4);
		cv.Insert(0, leftbnd[j]);
		cv.Insert(1, leftbnd[j] + leftnormal*height*ratio[j]);
		cv.Insert(2, rightbnd[j] + rightnormal*height*ratio[j]);
		cv.Insert(3, rightbnd[j]);
		ON_BezierCurve BC(cv);
		for (int k = 0; k < 20; k++)
		{
			curves[j].Insert(k, BC.PointAt((double)k / 19));
		}
	}
	JigFace *JF1 = new JigFace(curves[0], curves[1], xn);
	m_faces[0] = JF1;
	JigFace *JF2 = new JigFace(curves[1], curves[2], yn);
	m_faces[1] = JF2;
	JigFace *JF3 = new JigFace(curves[2], curves[3], xn);
	m_faces[2] = JF3;
	JigFace *JF4 = new JigFace(curves[3], curves[0], yn);
	m_faces[3] = JF4;

	ON_3dPointArray leftl, leftr, rightl, rightr;
	for (int i = 0; i < yn; i++)
	{
		leftl.Insert(i, leftbnd[1] + (leftbnd[2] - leftbnd[1])*i / (yn - 1));
		leftr.Insert(i, leftbnd[0] + (leftbnd[3] - leftbnd[0])*i / (yn - 1));
		rightl.Insert(i, rightbnd[0] + (rightbnd[3] - rightbnd[0])*i / (yn - 1));
		rightr.Insert(i, rightbnd[1] + (rightbnd[2] - rightbnd[1])*i / (yn - 1));
	}
	JigFace *JF5 = new JigFace(leftl, leftr, xn);
	m_faces[4] = JF5;
	JigFace *JF6 = new JigFace(rightl, rightr, xn);
	m_faces[5] = JF6;
}

JigBlock::~JigBlock()
{
	for (int i = 0; i < 6; i++)
	{
		if (m_faces[i])
			delete m_faces[i];
	}
}

ON_3dPointArray JigBlock::DigAHole(int face_index, bool isCL, double upper, double lower, double left, double right)
{
	m_bDispList = false;
	ON_3dPointArray bnd = m_faces[face_index]->DigAHole(isCL, upper, lower, left, right);
	return bnd;
}

void JigBlock::Update()
{
	FT::FEntity::Update();
}

void JigBlock::DrawSmoothShade()
{
	for (int i = 0; i < 6; i++)
	{
		if (m_faces[i])
			m_faces[i]->DrawSmoothShade();
	}
}

void JigBlock::DrawMesh()
{
	for (int i = 0; i < 6; i++)
	{
		if (m_faces[i])
			m_faces[i]->DrawMesh();
	}
}

void JigBlock::InitDisplayList()
{
	m_bDispList = true;
	for (int i = 0; i < 6; i++)
	{
		if (m_faces[i])
			m_faces[i]->Update();
	}
}

JigTriangle::JigTriangle(int v1, int v2, int v3)
{
	nodei[0] = v1;
	nodei[1] = v2;
	nodei[2] = v3;
}

ON_3dPoint JigTriangle::GetVertex(int vi)
{
	return m_face->GetNode(nodei[vi]);
}

void JigTriangle::CalNormal(bool reversed)
{
	ON_3dVector v1 = GetVertex(1) - GetVertex(0), v2 = GetVertex(2) - GetVertex(1);
	normal = ON_CrossProduct(v1, v2);
	normal.Unitize();
	if (reversed)
	{
		normal = -normal;
		int tempi = nodei[0];
		nodei[0] = nodei[1];
		nodei[1] = tempi;
	}
}

bool JigTriangle::CheckTopology()
{
	for (int i = 0; i < 3; i++)
	{
		if (nodei[i] == -1)
			return false;
	}
	return true;
}

bool JigFace::RayTriangleIntersect(const ON_3dPoint &orig, const ON_3dVector &dir,
	const ON_3dPoint &v0, const ON_3dPoint &v1, const ON_3dPoint &v2,
	float &t, float &u, float &v)
{
	//implement by Lufeng
	ON_3dVector v0v1 = v1 - v0;
	ON_3dVector v0v2 = v2 - v0;
	ON_3dVector pvec = ON_CrossProduct(dir, v0v2);
	float det = ON_DotProduct(v0v1, pvec);

	// culling
	if (det < kEpsilon) return false;

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

bool JigFace::RayMeshIntersect(ON_3dPoint &orig, ON_3dVector &dir,
	const ON_MeshTopology* T_mesh)
{
	dir.Unitize();
	vector<float> inter_params;
	//const ON_MeshTopology *T_mesh = &(m_pMesh.Topology());
	int n = T_mesh->TopFaceCount();
	for (int i = 0; i < n; i++)
	{
		int ind0 = T_mesh->m_topv_map[T_mesh->m_mesh->m_F[i].vi[0]];
		int ind1 = T_mesh->m_topv_map[T_mesh->m_mesh->m_F[i].vi[1]];
		int ind2 = T_mesh->m_topv_map[T_mesh->m_mesh->m_F[i].vi[2]];
		ON_3dVector fnor = T_mesh->m_mesh->m_FN[i];
		ON_3dPoint v0 = T_mesh->TopVertexPoint(ind0);
		ON_3dPoint v1 = T_mesh->TopVertexPoint(ind1);
		ON_3dPoint v2 = T_mesh->TopVertexPoint(ind2);
		ON_3dVector v0v1 = v1 - v0;
		ON_3dVector v0v2 = v2 - v0;
		ON_3dVector nor = ON_CrossProduct(v0v1, v0v2);
		if (ON_DotProduct(fnor,nor)<0)
		{
			ON_3dPoint v1 = T_mesh->TopVertexPoint(ind2);
			ON_3dPoint v2 = T_mesh->TopVertexPoint(ind1);
		}
		float t, u, v;
		ON_3dPoint inter;
		if (RayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v))
		{
			inter_params.push_back(t);
		}
	}
	sort(inter_params.begin(), inter_params.end());

	int m = inter_params.size();
	if (m)
	{
		orig = orig + inter_params[0] * dir;
		return true;
	}
	else
		return false;
}

JigFace & JigFace::operator=(const JigFace &jf)
{
	this->nodelist = jf.nodelist;
	this->nodeflag = jf.nodeflag;
	this->trianglelist = jf.trianglelist;
	int n = trianglelist.size();
	for (int i = 0; i < n; i++)
		trianglelist[i].SetFace(this);
	this->nodeindex = jf.nodeindex;
	return *this;
}

JigFace::JigFace(ON_3dPoint ori, ON_3dVector nor, ON_3dVector xdir, double height, double width, int density)
{
	int density_row = density, density_col = density;
	if (height > width)
		density_col = density*height / width;
	else
		density_row = density*width / height;
	nodeindex = LaGenMatInt(density_col + 1, density_row + 1);
	nodeflag = LaGenMatInt::ones(density_col + 1, density_row + 1);
	nodelist.Reserve((density_col + 1)*(density_row + 1));
	ON_3dVector ydir = - ON_DECL::ON_CrossProduct(nor, xdir);
	ON_3dPoint node_ini = ori - xdir*(width / 2) - ydir*(height/2);
	double dh = height / density_col, dw = width / density_row;
	int nodei = 0;
	for (int i = 0; i < density_col + 1; i++) //row
	{
		ON_3dPoint node = node_ini + ydir*(dh*i);
		for (int j = 0; j < density_row + 1; j++) //column
		{
			nodelist.Insert(nodei, node + xdir*(dw*j));
			nodeindex(i, j) = nodei;
			nodei++;
		}
	}
	MeshTriangle();
}

JigFace::JigFace(ON_3dPointArray &leftbnd, ON_3dPointArray &rightbnd, int density)
{
	int yn = leftbnd.Count();
	nodeindex = LaGenMatInt(yn, density+1);
	nodelist.Reserve(yn*(density+1));
	nodeflag = LaGenMatInt::ones(yn, density + 1);
	int nodei = 0;
	for (int i = 0; i < yn; i++)
	{
		ON_3dVector xdir = rightbnd[i] - leftbnd[i];
		double dx = xdir.Length() / density;
		xdir.Unitize();
		for (int j = 0; j < density+1; j++)
		{
			nodeindex(i, j) = nodei;
			nodelist.Insert(nodei, leftbnd[i] + xdir*dx*j);
			nodei++;
		}
	}
	MeshTriangle();
}

void JigFace::MeshTriangle()
{
	int yn = nodeindex.size(0) - 1, xn = nodeindex.size(1) - 1;
	trianglelist.clear();
	trianglelist.reserve(2 * yn*xn);
	for (int i = 0; i < yn; i++)
	{
		for (int j = 0; j < xn; j++)
		{
			int v1 = nodeindex(i, j), v2 = nodeindex(i, j + 1), v3 = nodeindex(i + 1, j + 1), v4 = nodeindex(i + 1, j);
			int isjt1 = nodeflag(i, j) + nodeflag(i + 1, j + 1) + nodeflag(i, j + 1), isjt2 = nodeflag(i, j) + nodeflag(i + 1, j) + nodeflag(i + 1, j + 1);

			JigTriangle jt1(v1, v3, v2), jt2(v1, v4, v3);
			if (isjt1)
			{
				jt1.SetFace(this);
				jt1.CalNormal();
				trianglelist.push_back(jt1);
			}
			if (isjt2)
			{
				jt2.SetFace(this);
				jt2.CalNormal();
				trianglelist.push_back(jt2);
			}			
		}
	}
}

ON_3dPointArray JigFace::GetNodes(bool isCL, double upper, double lower, double left, double right, int &xnum, int &ynum)
{
	ON_3dPointArray p(4);
	int yn = nodeindex.size(0) - 1, xn = nodeindex.size(1) - 1;
	int u1 = floor(yn*upper), v1 = floor(xn*left), u2 = floor(yn*lower), v2 = floor(xn*right);
	xnum = u2 - u1 + 1;
	ynum = v2 - v1 + 1;
	if (isCL)
	{
		p.Insert(0, nodelist[nodeindex(u1, v1)]);
		p.Insert(1, nodelist[nodeindex(u1, v2)]);
		p.Insert(2, nodelist[nodeindex(u2, v2)]);
		p.Insert(3, nodelist[nodeindex(u2, v1)]);
	}
	else
	{
		p.Insert(0, nodelist[nodeindex(u1, v2)]);
		p.Insert(1, nodelist[nodeindex(u1, v1)]);
		p.Insert(2, nodelist[nodeindex(u2, v1)]);
		p.Insert(3, nodelist[nodeindex(u2, v2)]);
	}
	return p;
}

ON_3dPointArray JigFace::GetBoundary(int bi)
{
	int yn = nodeindex.size(0), xn = nodeindex.size(1);
	if (bi == 0)
	{
		ON_3dPointArray pa(yn);
		for (int i = 0; i < yn; i++)
		{
			if (nodeindex(yn - i - 1, 0) != -1)
				pa.Insert(i, GetNode(nodeindex(yn - i - 1, 0)));
		}			
		return pa;
	}
	if (bi == 1)
	{
		ON_3dPointArray pa(xn);
		for (int i = 0; i < xn; i++)
		{
			if (nodeindex(0, i)!=-1)
				pa.Insert(i, GetNode(nodeindex(0, i)));
		}
		return pa;
	}
	if (bi == 2)
	{
		ON_3dPointArray pa(yn);
		for (int i = 0; i < yn; i++)
		{
			if (nodeindex(i, xn - 1)!=-1)
				pa.Insert(i, GetNode(nodeindex(i, xn - 1)));
		}
			
		return pa;
	}
	if (bi == 3)
	{
		ON_3dPointArray pa(xn);
		for (int i = 0; i < xn; i++)
		{
			if (nodeindex(yn - 1, xn - i - 1)!=-1)
				pa.Insert(i, GetNode(nodeindex(yn - 1, xn - i - 1)));
		}
		return pa;
	}
}

void JigFace::RayProjection(ON_3dVector dir, const double maxdistance, Tooth *tooth_in)
{
	//implement by Lufeng

	// get tooth topology
	ON_Mesh* m_pMesh = tooth_in->GetMesh();
	const ON_MeshTopology *T_mesh = &(m_pMesh->Topology());

	int m = nodelist.Count();
	dir.Unitize();

	// tooth with bracket
	if (tooth_in->m_bracket) 
	{
		ON_Mesh* m_bMesh = tooth_in->m_bracket->GetMesh();
		const ON_MeshTopology* B_mesh = &(m_bMesh->Topology());
		ON_3dPointArray nodelistNew;
		for (int i = 0; i < m; i++)
		{
			// update node value in the nodelist
			ON_3dPoint orig = nodelist[i];
			// first intersect bracket, if no intersection, intersect tooth
			if (RayMeshIntersect(orig,dir,B_mesh))
			{
				nodelistNew.Insert(i, orig);
				//nodelist[i] = orig;
			}
			else if (RayMeshIntersect(orig, dir, T_mesh))
			{
				nodelistNew.Insert(i, orig);
				//nodelist[i] = orig;
			}
			else
			{
				ON_3dPoint pt(orig + maxdistance*dir);
				nodelistNew.Insert(i, pt);
				//nodelist[i] = pt;
			}
		}
		SetNodeList(nodelistNew);
	}
	// only tooth, no bracket (delete below if bracket must have)
	else
	{
		for (int i = 0; i < m; i++)
		{
			// update node value in the nodelist
			ON_3dPoint orig = nodelist[i];
			if (RayMeshIntersect(orig, dir, T_mesh))
			{
				//nodelist.Insert(i, orig);
				nodelist[i] = orig;
			}
			else
			{
				ON_3dPoint pt(orig + maxdistance*dir);
				//nodelist.Insert(i, pt);
				nodelist[i] = pt;
			}
		}
	}
	
	// calculate normal of the new generated jigface
	for (int i = 0; i < trianglelist.size(); i++)
	{
		trianglelist[i].CalNormal(true); //reverse normal
	}
}

//void JigFace::RayProjection_fast(ON_3dVector dir, const double maxdistance, Tooth *tooth_in)
//{
//	dir.Unitize();
//	ON_3dVector xdir = tooth_in->m_bracket->yaxis, nor = -dir;
//	ON_3dPoint ori = tooth_in->m_bracket->origin;
//	NodeTree tooth_tree(ori, nor, xdir), bracket_tree(ori, nor, xdir);
//	tooth_tree.SetMesh(tooth_in->GetMesh());
//	bracket_tree.SetMesh(tooth_in->m_bracket->GetMesh());
//	tooth_tree.NodeProjectToPlane();
//	bracket_tree.NodeProjectToPlane();
//	tooth_tree.ConstructKDTree();
//	bracket_tree.ConstructKDTree();
//	int n = nodelist.Count();
//	for (int i = 0; i < n; i++)
//	{
//		double d = maxdistance;
//		int bracket_topi[2];
//	}
//}

void JigFace::RayProjection_f(ON_3dVector dir, double maxdistance, Tooth *tooth_in, double& min_dis)
{
	Bracket *bracket_in = tooth_in->m_bracket;
	int nrow = nodeindex.rows(), ncol = nodeindex.cols();
	LaGenMatDouble Distance= LaGenMatDouble::ones(nrow,ncol);
	Distance *= maxdistance;
	min_dis = maxdistance;
	ON_3dPoint origin = nodelist[nodeindex(0, 0)];
	ON_3dVector xaxis = nodelist[nodeindex(0, ncol - 1)] - origin;
	ON_3dVector yaxis = nodelist[nodeindex(nrow - 1, 0)] - origin;
	double dx = xaxis.Length() / (ncol - 1), dy = yaxis.Length() / (nrow - 1), height=yaxis.Length(), dis[3];
	xaxis.Unitize();
	yaxis.Unitize();
	ON_Mesh *mesh_bracket=NULL, *mesh_tooth = tooth_in->GetMesh();
	if (bracket_in)
		mesh_bracket = bracket_in->GetMesh();
	ON_3dPoint v[3];
	ON_2dPoint v2d[3];
	int vx[3], vy[3];
	if (mesh_bracket)
	{
		for (int fi = 0; fi < mesh_bracket->FaceCount(); fi++)
		{
			if (mesh_bracket->m_FN[fi]*dir>=0)
				continue;
			const ON_MeshFace& f = mesh_bracket->m_F[fi];
			bool is_inside = false;
			for (int i = 0; i < 3; i++)
			{
				v[i] = mesh_bracket->Vertex(f.vi[i]);
				v2d[i].x = (v[i] - origin)*xaxis;
				v2d[i].y = (v[i] - origin)*yaxis;
				vx[i] = round(v2d[i].x / dx);
				vy[i] = round(v2d[i].y / dy);		
				if (vx[i] >= -1 && vx[i] <= ncol && vy[i] >= -1 && vy[i] <= nrow)
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
			vx_max = vx_max>ncol - 1 ? ncol - 1 : vx_max;
			vx_min = vx_min < 0 ? 0 : vx_min;
			vy_max = vy_max > nrow - 1 ? nrow - 1 : vy_max;
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
		//shorten the overhang
		for (int j = 0; j < nrow; j++)
		{
			for (int i = 0; i < ncol; i++)
			{
				if (Distance(j, i)>OVERHANGDIS && Distance(j, i) != maxdistance)
					Distance(j, i) = OVERHANGDIS;
			}
		}
		//shorten the overhang
		//leave a gap on the top region
		double d_min = maxdistance;
		int jmax = JIGOVERHEIGHT*nrow / height;
		for (int j = 0; j < jmax; j++)
		{
			for (int i = 0; i < ncol; i++)
				d_min = Distance(j, i) < d_min ? Distance(j, i) : d_min;
		}
		d_min -= 0.3; //leave a gap
		for (int j = 0; j < jmax; j++)
		{
			for (int i = 0; i < ncol; i++)
			{
				if (Distance(j, i) != maxdistance)
					Distance(j, i) = d_min;
			}
		}
		//leave a gap on the top region
	}	
	
	for (int fi = 0; fi < mesh_tooth->FaceCount(); fi++)
	{
		if (mesh_tooth->m_FN[fi]*dir>=0)
			continue;
		const ON_MeshFace& f = mesh_tooth->m_F[fi];
		bool is_inside = false;
		for (int i = 0; i < 3; i++)
		{
			v[i] = mesh_tooth->Vertex(f.vi[i]);
			v2d[i].x = (v[i] - origin)*xaxis;
			v2d[i].y = (v[i] - origin)*yaxis;
			vx[i] = round(v2d[i].x / dx);
			vy[i] = round(v2d[i].y / dy);
			if (vx[i] >= -1 && vx[i] <= ncol && vy[i] >= -1 && vy[i] <= nrow)
				is_inside = true;
			
			//if (vy[i] >= 0 || vy[i] < nrow)
			//	is_inside = true;
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
		vx_max = vx_max>ncol - 1 ? ncol - 1 : vx_max;
		vx_min = vx_min < 0 ? 0 : vx_min;
		vy_max = vy_max > nrow - 1 ? nrow - 1 : vy_max;
		vy_min = vy_min < 0 ? 0 : vy_min;
		for (int i = vx_min; i <= vx_max; i++)
		{
			for (int j = vy_min; j <= vy_max; j++)
			{
				ON_2dPoint pp(dx*i, dy*j);
				if (TriangleInterp(v2d[0], v2d[1], v2d[2], pp, par))
				{
					double d = dis[0] * par[0] + dis[1] * par[1] + dis[2] * par[2];
					if (d < min_dis)
						min_dis = d;
					if (d < Distance(j, i))
						Distance(j, i) = d;
				}
			}
		}
	}
	for (int i = 0; i < nrow; i++)
	{
		for (int j = 0; j < ncol; j++)
		{
			if (Distance(i, j) >= (maxdistance+min_dis-MINDIS))
				Distance(i, j) = maxdistance + min_dis - MINDIS;	//distance correction
			nodelist[nodeindex(i, j)] += dir*Distance(i, j);
		}
	}
	for (int i = 0; i < trianglelist.size(); i++)
	{
		trianglelist[i].CalNormal(true); 
	}
}

void JigFace::Translate(ON_3dVector v)
{
	nodelist.Translate(v);
	m_bDispList = false;
}

ON_3dPointArray JigFace::DigAHole(bool isCL, double upper, double lower, double left, double right)
{
	m_bDispList = false; //have to execute initdisplaylist() after mesh structure is changed.
	int yn = nodeindex.size(0)-1, xn = nodeindex.size(1)-1;
	int upper_i = floor(yn*upper), lower_i = floor(yn*lower), left_i = floor(xn*left), right_i = floor(xn*right);
	for (int i = upper_i ; i <= lower_i; i++)
	{
		for (int j = left_i ; j <= right_i; j++)
		{
			nodeflag(i, j) = 0;
		}
	}
	MeshTriangle();
	ON_3dPointArray bnd;
	int idx = 0;
	if (isCL)
	{
		for (int i = lower_i; i >= upper_i; i--)
			bnd.Insert(idx++, nodelist[nodeindex(i, left_i)]);
		for (int i = left_i + 1; i <= right_i; i++)
			bnd.Insert(idx++, nodelist[nodeindex(upper_i, i)]);
		for (int i = upper_i + 1; i <= lower_i; i++)
			bnd.Insert(idx++, nodelist[nodeindex(i, right_i)]);
		for (int i = right_i - 1; i >= left_i; i--)
			bnd.Insert(idx++, nodelist[nodeindex(lower_i, i)]);
	}
	else
	{
		for (int i = upper_i; i <= lower_i; i++)
			bnd.Insert(idx++, nodelist[nodeindex(i, left_i)]);
		for (int i = left_i + 1; i <= right_i; i++)
			bnd.Insert(idx++, nodelist[nodeindex(lower_i, i)]);
		for (int i = lower_i - 1; i >= upper_i; i--)
			bnd.Insert(idx++, nodelist[nodeindex(i, right_i)]);
		for (int i = right_i - 1; i >= left_i; i--)
			bnd.Insert(idx++, nodelist[nodeindex(upper_i, i)]);
	}
	return bnd;
}

void JigFace::Update()
{
	FT::FEntity::Update();
}

void JigFace::DrawSmoothShade()
{
	if (!nodelist)	return;
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	if (1 == true)	//trigger for highlight
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
	glCallList(m_uListSmooth);
	glPopMatrix();
	
}

void JigFace::DrawMesh()
{
	if (!nodelist)	return;
	if (m_iSelect == 0)
		glColor3f(m_red, m_green, m_blue);
	if (1 == true)	//trigger for highlight
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
	
	glPolygonOffset(1, 1);
	glEnable(GL_DEPTH_TEST);
	glPushMatrix();
	//glCallList(m_uListSmooth);
	glColor3f(0, 0, 1);
	glCallList(m_uListMesh);
	glPopMatrix();
	glPolygonOffset(0, 0);
	glDisable(GL_POLYGON_OFFSET_FILL);

}

void JigFace::InitDisplayList()
{
	if (NULL == nodelist)	return;
	m_bDispList = true;
	if (glIsList(tempList))
		glDeleteLists(tempList, 3);
	tempList = glGenLists(3);
	int fi;
	F3dPoint v[3];
	F3dVector n;
	int face_count = trianglelist.size();

	glNewList(tempList, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	

	/*glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);*/
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	glBegin(GL_TRIANGLES);
	for (int fi = 0; fi < face_count; fi++)
	{
		n = trianglelist[fi].GetNormal();
		v[0] = trianglelist[fi].GetVertex(0);
		v[1] = trianglelist[fi].GetVertex(1);
		v[2] = trianglelist[fi].GetVertex(2);
		/*v[1] = trianglelist[fi].GetVertex(2);
		v[2] = trianglelist[fi].GetVertex(1);*/

		glNormal3d(n.x, n.y, n.z);	//face normal
		for (int i = 0; i < 3; i++)
		{
			glVertex3d(v[i].x, v[i].y, v[i].z);
		}
	}
	glEnd();

	//glDisable(GL_CULL_FACE);

	glEndList();

	////drawflatshade list
	//m_uListFlat = tempList + 1;
	//glNewList(m_uListFlat, GL_COMPILE);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glCallList(tempList);
	//glEndList();

	//draw smooth shade list
//	F3fVector normal[4];
	m_uListSmooth = tempList + 1;
	glNewList(m_uListSmooth, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glCallList(tempList);
	glEndList();

	//draw mesh list
	m_uListMesh = tempList + 2;
	glNewList(m_uListMesh, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);         // Draw Mesh
	glCallList(tempList);
	glEndList();

	//draw point list
	/*int nsize = m_pMesh->m_V.Count();
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
	glEndList();*/

}



NodeTree::NodeTree(ON_3dPoint ori, ON_3dVector nor, ON_3dVector x)
{
	nodenum = 0;
	datapts = NULL;
	kdtree = NULL;
	origin = ori;
	normal = nor;
	x_axis = x;
	y_axis = ON_DECL::ON_CrossProduct(normal, x_axis);
}

NodeTree::~NodeTree()
{
	annDeallocPts(datapts);
	if (kdtree)
		delete kdtree;
}

void NodeTree::NodeProjectToPlane()
{
	const ON_MeshTopology *T_mesh = &(m_pMesh->Topology());
	int n = T_mesh->TopVertexCount();
	for (int i = 0; i < n; i++) //topvi
	{
		int vi = T_mesh->m_topv[i].m_vi[0];
		if (m_pMesh->m_N[vi] * normal>0)
		{
			nodetopindex.push_back(i);
			nodenum++;
		}
	}
	datapts = annAllocPts(nodenum, 2);
	for (int i = 0; i < nodenum; i++)
	{
		ON_3dPoint p = T_mesh->TopVertexPoint(nodetopindex[i]);
		datapts[i][0] = (p - origin)*x_axis; //xcoord
		datapts[i][1] = (p - origin)*y_axis; //ycoord
	}
}

void NodeTree::ConstructKDTree()
{
	if (!datapts)
		return;
	kdtree = new ANNkd_tree(datapts, nodenum, 2);
}

bool NodeTree::ANNSearch(ON_3dPoint querypnt, int k, int *topindex)
{
	ANNpoint q = annAllocPt(2);
	q[0] = (querypnt - origin)*x_axis;
	q[1] = (querypnt - origin)*y_axis;
	ANNidxArray nnIdx=new ANNidx[k];
	ANNdistArray dists=new ANNdist[k];
	if (kdtree)
	{
		kdtree->annkSearch(q, k, nnIdx, dists);
		for (int i = 0; i < k; i++)
			topindex[i] = nodetopindex[nnIdx[i]];
		delete[] nnIdx;
		delete[] dists;
		return true;
	}
	return false;
}

//bool Jig::ExportSTL(std::ostream &output)
//{
//	int blockNum = m_blocks.size();
//	int connectorNum = m_connectors.size();
//
//	if (!blockNum)
//	{
//		return false;
//	}
//	else
//	{
//		output << "solid Jig_Model\n";
//
//		for (int i = 0; i < blockNum; ++i)
//		{
//			m_blocks[i]->append_JigBlock(output);
//		}
//
//		for (int i = 0; i < connectorNum; ++i)
//		{
//			m_connectors[i]->append_JigFace(output);
//		}
//
//		output << "endsolid Jig_Model\n";
//	}
//}

//bool Jig::ExportSTL(QDataStream &output)
//{
//	int blockNum = m_blocks.size();
//	int connectorNum = m_connectors.size();
//
//	if (!blockNum)
//	{
//		return false;
//	}
//	else
//	{
//		output << "solid Jig_Model\r\n";
//
//		for (int i = 0; i < blockNum; ++i)
//		{
//			m_blocks[i]->append_JigBlock(output);
//		}
//
//		for (int i = 0; i < connectorNum; ++i)
//		{
//			m_connectors[i]->append_JigFace(output);
//		}
//
//		output << "endsolid Jig_Model\r\n";
//	}
//
//}

void JigFace::append_JigFace(std::ostream &output)
{
	int n = trianglelist.size();
	for (int i = 0; i < n; ++i)
	{
		ON_3dPoint v1 = trianglelist[i].GetVertex(0);
		ON_3dPoint v2 = trianglelist[i].GetVertex(1);
		ON_3dPoint v3 = trianglelist[i].GetVertex(2);
		ON_3dVector normal = trianglelist[i].GetNormal();
		std::stringstream stream;
		stream << v1.x << " " << v1.y << " " << v1.z;
		std::string vs1 = stream.str();
		stream.str("");
		stream << v2.x << " " << v2.y << " " << v2.z;
		std::string vs2 = stream.str();
		stream.str("");
		stream << v3.x << " " << v3.y << " " << v3.z;
		std::string vs3 = stream.str();

		output << "  facet normal "
			<< normal.x << " " << normal.y << " " << normal.z << " " << "\n";
		output << "    outer loop\n";
		output << "      vertex " << vs1 << "\n";
		output << "      vertex " << vs2 << "\n";
		output << "      vertex " << vs3 << "\n";
		output << "    endloop\n";
		output << "  endfacet\n";
	}
}

void JigFace::append_JigFace(QDataStream &output)
{
	int n = trianglelist.size();
	for (int i = 0; i < n; ++i)
	{
		ON_3dPoint v1 = trianglelist[i].GetVertex(0);
		ON_3dPoint v2 = trianglelist[i].GetVertex(1);
		ON_3dPoint v3 = trianglelist[i].GetVertex(2);
		ON_3dVector normal = trianglelist[i].GetNormal();
		/*QString vs1(QString::number(v1.x, 'f', 4) + " " + QString::number(v1.y, 'f', 4) + " " + QString::number(v1.z, 'f', 4));
		QString vs2(QString::number(v2.x, 'f', 4) + " " + QString::number(v2.y, 'f', 4) + " " + QString::number(v2.z, 'f', 4));
		QString vs3(QString::number(v3.x, 'f', 4) + " " + QString::number(v3.y, 'f', 4) + " " + QString::number(v3.z, 'f', 4));*/
		//std::stringstream stream;
		//stream << v1.x << " " << v1.y << " " << v1.z;
		//std::string vs1 = stream.str();
		//stream.str("");
		//stream << v2.x << " " << v2.y << " " << v2.z;
		//std::string vs2 = stream.str();
		//stream.str("");
		//stream << v3.x << " " << v3.y << " " << v3.z;
		//std::string vs3 = stream.str();

		output << "  facet normal "
			<< normal.x << " " << normal.y << " " << normal.z << " " << "\r\n";
		output << "    outer loop\r\n";
		output << "      vertex "
			<< (qint32)v1.x << " "
			<< (qint32)v1.y << " "
			<< (qint32)v1.z << " "
			<< "\r\n";
		output << "      vertex "
			<< (qint32)v2.x << " "
			<< (qint32)v2.y << " "
			<< (qint32)v2.z << " "
			<< "\r\n";
		output << "      vertex "
			<< (qint32)v3.x << " "
			<< (qint32)v3.y << " "
			<< (qint32)v3.z << " "
			<< "\r\n";
		output << "    endloop\r\n";
		output << "  endfacet\r\n";
	}
}

int JigBlock::GetFaceNum()
{
	int faceNum = 0;
	for (unsigned int i = 0; i < 6; i++)
	{
		faceNum += GetFace(i)->GetFaceNum();
	}

	return faceNum;
}

void JigBlock::append_JigBlock(std::ostream &output)
{
	for (int i = 0; i < 6; ++i)
	{
		m_faces[i]->append_JigFace(output);
	}
}

void JigBlock::append_JigBlock(QDataStream &output)
{
	for (int i = 0; i < 6; ++i)
	{
		m_faces[i]->append_JigFace(output);
	}
}

//bool Jig::ExportSTL(const char* filename)
//{
//	FILE *fp;
//	fp = fopen(filename, "w");
//	if (fp == 0)
//		return false;
//
//	int blockNum = m_blocks.size();
//	int connectorNum = m_connectors.size();
//
//	if (!blockNum)
//	{
//		return false;
//	}
//	else
//	{
//		int range = blockNum + connectorNum;
//		
//		progresswidgetExport->setRange(0, range);
//		progresswidgetExport->setValue(0);
//		fprintf(fp, "solid Jig\n");
//
//		for (int i = 0; i < blockNum; ++i)
//		{
//			//int curValue = progresswidget->value();
//			progresswidgetExport->setValue(i + 1);
//			m_blocks[i]->append_JigBlock(fp);
//		}
//
//		for (int i = 0; i < connectorNum; ++i)
//		{
//			//int curValue = progresswidget->value();
//			progresswidgetExport->setValue(blockNum + i + 1);
//			m_connectors[i]->append_JigFace(fp);
//		}
//
//		fprintf(fp, "endsolid \n");
//		fclose(fp);
//		return true;
//
//	}
//}

bool Jig::ExportSTL_binary(const char* filename, bool isBinary /*= true*/)
{
	FILE *fout;
	fout = fopen(filename, "wb");
	if (fout == 0)
		return false;

	// new data structure
	int blockNum = mblocks.size();
	int connectorNum = mconnectors.size();

	int range = blockNum + connectorNum;

	progresswidgetExport->setRange(0, range);
	progresswidgetExport->setValue(0);

	if (isBinary)
	{
		char header[128] = "solid                                                                                                ";

		unsigned long numFaces = 0;

		for (int i = 0; i < blockNum; ++i)
		{
			numFaces += mblocks[i]->GetFaceNum();
		}

		for (int i = 0; i < connectorNum; ++i)
		{
			numFaces += mconnectors[i]->GetFaceNum();
		}

		fwrite(header, 80, 1, fout);

		// write number of faces
		fwrite(&numFaces, 1, sizeof(int), fout);

		for (int i = 0; i < blockNum; ++i)
		{
			progresswidgetExport->setValue(i + 1);
			mblocks[i]->ExportSTL_binary(fout, isBinary);
		}

		for (int i = 0; i < connectorNum; ++i)
		{
			progresswidgetExport->setValue(blockNum + i + 1);
			mconnectors[i]->ExportSTL_binary(fout, isBinary);
		}

	}
	else // ASCII
	{
		fprintf(fout, "solid HKUST\n");

		for (int i = 0; i < blockNum; ++i)
		{
			progresswidgetExport->setValue(i + 1);
			mblocks[i]->ExportSTL_binary(fout, isBinary);
		}

		for (int i = 0; i < connectorNum; ++i)
		{
			progresswidgetExport->setValue(blockNum + i + 1);
			mconnectors[i]->ExportSTL_binary(fout, isBinary);
		}

		fprintf(fout, "endsolid HKUST\n");
	}

	fclose(fout);

	return true;
}

void JigBlock::append_JigBlock(FILE* fp)
{
	for (int i = 0; i < 6; ++i)
	{
		m_faces[i]->append_JigFace(fp);
	}
}

bool JigBlock::ExportSTL_binary(FILE* fout, bool isBinary /*= true*/)
{
	for (int i = 0; i < 6; ++i)
	{
		m_faces[i]->ExportSTL_binary(fout, isBinary);
	}

	return true;
}

void JigFace::append_JigFace(FILE* fp)
{
	char buf[255]; // create a buffer for fwrite to use
	int n = trianglelist.size();
	for (int i = 0; i < n; ++i)
	{
		
		ON_3dVector normal = trianglelist[i].GetNormal();
		sprintf(buf, "  facet normal %f %f %f\n", normal.x, normal.y, normal.z);
		fwrite(buf, 1, strlen(buf), fp);
		fprintf(fp, "    outer loop\n");
		for (int k = 0; k < 3; ++k)
		{
			ON_3dPoint v = trianglelist[i].GetVertex(k);
			sprintf(buf, "      vertex %f %f %f\n", v.x, v.y, v.z);
			fwrite(buf, 1, strlen(buf), fp);
		}
		fprintf(fp, "    endloop\n");
		fprintf(fp, "  endfacet\n");
		fflush(fp);
	}
}

bool JigFace::ExportSTL_binary(FILE* fout, bool isBinary /*= true*/)
{
	if (fout == 0)
		return false;

	if (isBinary)
	{
		// binary format reference: http://www.fabbers.com/tech/STL_Format#Sct_binary
		//char header[128] = "solid                                                                                                ";

		//unsigned long numFaces = trianglelist.size();

		//fwrite(header, 80, 1, fout);

		//// write number of faces
		//fwrite(&numFaces, 1, sizeof(int), fout);

		unsigned short attributes = 0;

		for (int i = 0; i < trianglelist.size(); ++i) {
			// For each triangle write the normal, the three coords and a short set to zero
			ON_3dVector n = trianglelist[i].GetNormal();
			float nvt[3];
			nvt[0] = n.x;
			nvt[1] = n.y;
			nvt[2] = n.z;
			fwrite(nvt, sizeof (float), 3, fout);
			for (int k = 0; k < 3; ++k)
			{
				ON_3dPoint v = trianglelist[i].GetVertex(k);
				nvt[0] = v.x;
				nvt[1] = v.y;
				nvt[2] = v.z;
				fwrite(nvt, sizeof (float), 3, fout);
			}

			fwrite(&attributes, 1, sizeof(short), fout);
		}


	}
	else // in ASCII
	{
		//fprintf(fout, "solid HKUST\n");

		for (int i = 0; i < trianglelist.size(); ++i)
		{
			ON_3dVector n = trianglelist[i].GetNormal();
			fprintf(fout, "  facet normal %13e %13e %13e\n", n.x, n.y, n.z);
			fprintf(fout, "    outer loop\n");
			for (int k = 0; k < 3; ++k)
			{
				ON_3dPoint v = trianglelist[i].GetVertex(k);
				fprintf(fout, "      vertex  %13e %13e %13e\n", v.x, v.y, v.z);
			}
			fprintf(fout, "    endloop\n");
			fprintf(fout, "  endfacet\n");
		}

		//fprintf(fout, "endsolid HKUST\n");
	}

	//fclose(fout);

	return true;
}

PntBlock::PntBlock(int upperorlower, Tooth *tooth_in, double height, double width, double depth, double resol, double occluheight)
{
	m_tooth = tooth_in;
	ON_3dPoint	centerpnt;
	ON_3dVector nor, xdir;
	int sign = -1;
	if (upperorlower == UPPER)
		sign = 1;
	if (m_tooth->m_bracket)
	{
		centerpnt = m_tooth->m_bracket->origin;
		nor = m_tooth->m_bracket->xaxis, xdir = m_tooth->m_bracket->yaxis;
		centerpnt = centerpnt + nor * 2 - m_tooth->m_bracket->zaxis * sign *(height / 2 - 0.5*BRACKET_SLOT - JIGOVERHEIGHT);
	}
	else
	{
		centerpnt = m_tooth->GetBottomPnt(upperorlower) + m_tooth->longaxis*(height / 2 - JIGOFFSET)*sign;
		nor = -m_tooth->yaxis, xdir = m_tooth->xaxis;
		centerpnt = centerpnt + nor * 6;
	}
	xaxis = xdir;
	zaxis = -nor;
	yaxis = ON_CrossProduct(zaxis, xaxis);

	origin = centerpnt - xaxis*(width / 2) - yaxis*(height / 2);
	samplewidth = resol;
	xn = round(width / samplewidth) + 1;
	yn = round(height / samplewidth) + 1;
	zn = round(depth / samplewidth) + 1;
	
	double min_dis;
	samplepnts.reserve(zn);
	for (int k = 0; k < zn; k++)
	{
		LaGenMatInt oneplane(yn, xn);
		samplepnts.push_back(oneplane);
	}
	ZProjection(min_dis);
	if (m_tooth->m_bracket)
		ZShifting(min_dis - MINDIS);
	else
		ZShifting(min_dis - 2.5);
	YProjection(JIGOFFSET + occluheight);
	MakeHole(m_tooth->m_bracket, upperorlower, 2.3);
	OffsetBnd();
}

void PntBlock::ZProjection(double & min_dis)
{
	Bracket *bracket_in = m_tooth->m_bracket;
	LaGenMatDouble Distance = LaGenMatDouble::ones(yn, xn);	
	ON_Mesh *mesh_bracket = NULL, *mesh_tooth = m_tooth->GetMesh();
	if (bracket_in)
		mesh_bracket = bracket_in->GetMesh();
	ON_3dPoint v[3];
	ON_2dPoint v2d[3];
	double dis[3];
	int vx[3], vy[3];
	ON_3dVector dir = zaxis;
	double maxdistance = samplewidth*(zn - 1), height = samplewidth*(yn-1);
	min_dis = maxdistance;
	Distance *= maxdistance;
	if (mesh_bracket)
	{
		for (int fi = 0; fi < mesh_bracket->FaceCount(); fi++)
		{
			if (mesh_bracket->m_FN[fi] * dir >= 0)
				continue;
			const ON_MeshFace& f = mesh_bracket->m_F[fi];
			bool is_inside = false;
			for (int i = 0; i < 3; i++)
			{
				v[i] = mesh_bracket->Vertex(f.vi[i]);
				v2d[i].x = (v[i] - origin)*xaxis;
				v2d[i].y = (v[i] - origin)*yaxis;
				vx[i] = round(v2d[i].x / samplewidth);
				vy[i] = round(v2d[i].y / samplewidth);
				if (vx[i] >= -1 && vx[i] <= xn && vy[i] >= -1 && vy[i] <= yn)
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
			vx_max = vx_max>xn - 1 ? xn - 1 : vx_max;
			vx_min = vx_min < 0 ? 0 : vx_min;
			vy_max = vy_max > yn - 1 ? yn - 1 : vy_max;
			vy_min = vy_min < 0 ? 0 : vy_min;
			for (int i = vx_min; i <= vx_max; i++)
			{
				for (int j = vy_min; j <= vy_max; j++)
				{
					ON_2dPoint pp(samplewidth*i, samplewidth*j);
					if (TriangleInterp(v2d[0], v2d[1], v2d[2], pp, par))
					{
						double d = dis[0] * par[0] + dis[1] * par[1] + dis[2] * par[2];
						if (d < Distance(j, i))
							Distance(j, i) = d;
					}
				}
			}
		}
		//shorten the overhang
		for (int j = 0; j < yn; j++)
		{
			for (int i = 0; i < xn; i++)
			{
				if (Distance(j, i) > OVERHANGDIS && Distance(j, i) != maxdistance)
					Distance(j, i) = OVERHANGDIS;
			}
		}
		//shorten the overhang
		//leave a gap on the top region
		double d_min = maxdistance;
		int jmax = JIGOVERHEIGHT*yn / height;
		for (int j = 0; j < jmax; j++)
		{
			for (int i = 0; i < xn; i++)
				d_min = Distance(j, i) < d_min ? Distance(j, i) : d_min;
		}
		d_min -= 0.1; //leave a gap
		for (int j = 0; j < jmax; j++)
		{
			for (int i = 0; i < xn; i++)
			{
				if (Distance(j, i) != maxdistance)
					Distance(j, i) = d_min;
			}
		}
		//leave a gap on the top region
	}
	for (int fi = 0; fi < mesh_tooth->FaceCount(); fi++)
	{
		if (mesh_tooth->m_FN[fi] * dir >= 0)
			continue;
		const ON_MeshFace& f = mesh_tooth->m_F[fi];
		bool is_inside = false;
		for (int i = 0; i < 3; i++)
		{
			v[i] = mesh_tooth->Vertex(f.vi[i]);
			v2d[i].x = (v[i] - origin)*xaxis;
			v2d[i].y = (v[i] - origin)*yaxis;
			vx[i] = round(v2d[i].x / samplewidth);
			vy[i] = round(v2d[i].y / samplewidth);
			if (vx[i] >= -1 && vx[i] <= xn && vy[i] >= -1 && vy[i] <= yn)
				is_inside = true;

			//if (vy[i] >= 0 || vy[i] < nrow)
			//	is_inside = true;
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
		vx_max = vx_max>xn - 1 ? xn - 1 : vx_max;
		vx_min = vx_min < 0 ? 0 : vx_min;
		vy_max = vy_max > yn - 1 ? yn - 1 : vy_max;
		vy_min = vy_min < 0 ? 0 : vy_min;
		for (int i = vx_min; i <= vx_max; i++)
		{
			for (int j = vy_min; j <= vy_max; j++)
			{
				ON_2dPoint pp(samplewidth*i, samplewidth*j);
				if (TriangleInterp(v2d[0], v2d[1], v2d[2], pp, par))
				{
					double d = dis[0] * par[0] + dis[1] * par[1] + dis[2] * par[2];
					if (d < min_dis)
						min_dis = d;
					if (d < Distance(j, i))
						Distance(j, i) = d;
				}
			}
		}
	}
	for (int i = 0; i < yn; i++)
	{
		for (int j = 0; j < xn; j++)
		{
			//if (Distance(i, j) >= (maxdistance + min_dis - MINDIS))
			//	Distance(i, j) = maxdistance + min_dis - MINDIS;	//distance correction
			for (int k = 0; k < zn; k++)
			{
				if (k*samplewidth <= Distance(i, j))
					samplepnts[k](i, j) = 1;
				else
					samplepnts[k](i, j) = 0;
			}
		}
	}
}

void PntBlock::YProjection(double maxdis)
{
	LaGenMatDouble Distance = LaGenMatDouble::ones(zn, xn);
	Distance *= maxdis;
	ON_Mesh *mesh_tooth = m_tooth->GetMesh();
	ON_3dVector dir = -yaxis;
	ON_3dPoint v[3];
	ON_2dPoint v2d[3];
	double dis[3];
	int vx[3], vy[3];
	ON_3dPoint morigin; //the left upper point of the bottom plane
	GetPnt(0, yn - 1, 0, morigin);
	for (int fi = 0; fi < mesh_tooth->FaceCount(); fi++)
	{
		if (mesh_tooth->m_FN[fi] * dir >= 0)
			continue;
		const ON_MeshFace& f = mesh_tooth->m_F[fi];
		bool is_inside = false;
		for (int i = 0; i < 3; i++)
		{
			v[i] = mesh_tooth->Vertex(f.vi[i]);
			v2d[i].x = (v[i] - morigin)*xaxis;
			v2d[i].y = (v[i] - morigin)*zaxis;
			vx[i] = round(v2d[i].x / samplewidth);
			vy[i] = round(v2d[i].y / samplewidth);
			if (vx[i] >= -1 && vx[i] <= xn && vy[i] >= -1 && vy[i] <= zn)
				is_inside = true;

			//if (vy[i] >= 0 || vy[i] < nrow)
			//	is_inside = true;
		}
		if (!is_inside)
			continue;
		int vx_max = vx[0], vx_min = vx[0], vy_max = vy[0], vy_min = vy[0];
		for (int i = 0; i < 3; i++)
		{
			dis[i] = (v[i] - morigin)*dir;
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
		vx_max = vx_max>xn - 1 ? xn - 1 : vx_max;
		vx_min = vx_min < 0 ? 0 : vx_min;
		vy_max = vy_max > zn - 1 ? zn - 1 : vy_max;
		vy_min = vy_min < 0 ? 0 : vy_min;
		for (int i = vx_min; i <= vx_max; i++)
		{
			for (int j = vy_min; j <= vy_max; j++)
			{
				ON_2dPoint pp(samplewidth*i, samplewidth*j);
				if (TriangleInterp(v2d[0], v2d[1], v2d[2], pp, par))
				{
					double d = dis[0] * par[0] + dis[1] * par[1] + dis[2] * par[2];
					//if (d < min_dis)
					//	min_dis = d;
					if (d < Distance(j, i))
						Distance(j, i) = d;
				}
			}
		}
	}
	for (int i = yn-1; i >= 0; i--)
	{
		for (int j = 0; j < xn; j++)
		{
			//if (Distance(i, j) >= (maxdistance + min_dis - MINDIS))
			//	Distance(i, j) = maxdistance + min_dis - MINDIS;	//distance correction
			for (int k = 0; k < zn; k++)
			{
				if ((yn-1-i)*samplewidth <= Distance(k, j))
					samplepnts[k](i, j) = 1;
			}
		}
	}
}

void PntBlock::ZShifting(double distance)
{
	int n = round(distance / samplewidth);
	origin = origin + zaxis*(samplewidth*n);
	if (n > 0) //towards tooth
	{
		samplepnts.erase(samplepnts.begin(), samplepnts.begin() + n - 1);
		samplepnts.insert(samplepnts.end(), n, samplepnts[samplepnts.size() - 1]);
	}
	if (n < 0)
	{
		samplepnts.insert(samplepnts.begin(), -n, samplepnts[0]);		
		samplepnts.erase(samplepnts.end() + n, samplepnts.end() - 1);
	}
}

void PntBlock::OffsetBnd()
{
	for (int i = 0; i < yn; i++)
	{
		for (int j = 0; j < xn; j++)
		{
			samplepnts[0](i, j) = 0;
			samplepnts[zn-1](i, j) = 0;
		}
	}
	for (int k = 0; k < zn; k++)
	{
		for (int j = 0; j < xn; j++)
		{
			samplepnts[k](0, j) = 0;
			samplepnts[k](yn-1, j) = 0;
		}
	}
	for (int k = 0; k < zn; k++)
	{
		for (int i = 0; i < yn; i++)
		{
			samplepnts[k](i, 0) = 0;
			samplepnts[k](i, xn-1) = 0;
		}
	}
//	origin = origin + zaxis*samplewidth + xaxis*samplewidth + yaxis*samplewidth;
}

void PntBlock::MakeHole(bool isbracket, int upperorlower, double zwidth)
{
	if (!isbracket)
		return;
	ON_3dPoint coord;
	int xm = floor(xn / 2), xs, zs, ys;
	if (upperorlower == UPPER)
	{
		ys = floor(yn / 3);
		for (zs = 0; zs < zn; ++zs)
		{
			if (!GetPnt(xm, 0, zs, coord))
				break;
		}
		int zend = round(zs + zwidth / samplewidth);

		for (xs = 0; xs < xn; ++xs)
		{
			if (!GetPnt(xs, 0, zs, coord))
				break;
		}
		int xend = xn - xs - 1;
		for (int i = xs; i <= xend; ++i)
		{
			for (int j = ys; j < yn; ++j)
			{
				for (int k = zs; k <= zend; ++k)
					SetPnt(i, j, k, false);
			}
		}
	}

}

bool PntBlock::GetPnt(int xi, int yi, int zi, ON_3dPoint &coord)
{
	if (xi >= xn || yi >= yn || zi >= zn)
		return false;
	coord = origin + xaxis*(xi*samplewidth) + yaxis*(yi*samplewidth) + zaxis*(zi*samplewidth);
	int pnt = samplepnts[zi](yi, xi);
	if (pnt == 1)
		return true;
	else
		return false;
}



void PntBlock::SetPnt(int xi, int yi, int zi, bool value)
{
	if (value)
		samplepnts[zi](yi, xi) = 1;
	else
		samplepnts[zi](yi, xi) = 0;
}

void PntBlock::Update()
{
	FT::FEntity::Update();
}

void PntBlock::DrawSmoothShade()
{
	glColor3f(m_red, m_green, m_blue);
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPushMatrix();
	glCallList(m_uListPoints);
	glPopMatrix();
}

void PntBlock::InitDisplayList()
{
	m_bDispList = true;
	if (glIsList(m_uListPoints))
		glDeleteLists(m_uListPoints, 1);
	m_uListPoints = glGenLists(1);
	glNewList(m_uListPoints, GL_COMPILE);
	glPointSize(2.0);
	glBegin(GL_POINTS);
	ON_3dPoint coord;
	for (int k = 0; k < zn; k++)
	{
		for (int j = 0; j < yn; j++)
		{
			for (int i = 0; i < xn; i++)
			{
				if (GetPnt(i, j, k, coord))
				{
					glVertex3d(coord.x, coord.y, coord.z);
				}
			}
		}
	}
	glEnd();
	glEndList();
}

MeshBlock::~MeshBlock()
{
	//if (hemesh)
	//	delete hemesh;
}

void MeshBlock::MeshReconstruct(PntBlock *pb)
{
	
	// implement by Lufeng
	ON_Mesh* mesh = new ON_Mesh;
	float minValue = 1.0; // threshold for generating iso-surface
	unsigned int ncellsX = pb->xn;
	unsigned int ncellsY = pb->yn;
	unsigned int ncellsZ = pb->zn;

	unsigned int numTriangles = int(0);

	//go through all the points
	for (unsigned int i = 0; i < ncellsX; i++)			//x axis
	for (unsigned int j = 0; j < ncellsY; j++)		//y axis
	for (unsigned int k = 0; k < ncellsZ; k++)	//z axis
	{
		//initialize vertices
		mp4Vector verts[8];

		//(step 3)

		//verts[0].x = i;		verts[0].y = j;		verts[0].z = k;
		//verts[1].x = i;		verts[1].y = j;		verts[1].z = k + 1;
		//verts[2].x = i + 1;	verts[2].y = j;		verts[2].z = k + 1;
		//verts[3].x = i + 1;	verts[3].y = j;		verts[3].z = k;
		//verts[4].x = i;		verts[4].y = j + 1;	verts[4].z = k;
		//verts[5].x = i;		verts[5].y = j + 1;	verts[5].z = k + 1;
		//verts[6].x = i + 1;	verts[6].y = j + 1;	verts[6].z = k + 1;
		//verts[7].x = i + 1;	verts[7].y = j + 1;	verts[7].z = k;

		ON_3dPoint coord;
		verts[0].val = pb->GetPnt(i,j,k,coord);
		verts[0].x = coord.x;		verts[0].y = coord.y;		verts[0].z = coord.z;
		verts[1].val = pb->GetPnt(i, j, k + 1,coord);
		verts[1].x = coord.x;		verts[1].y = coord.y;		verts[1].z = coord.z;
		verts[2].val = pb->GetPnt(i + 1, j, k + 1,coord);
		verts[2].x = coord.x;		verts[2].y = coord.y;		verts[2].z = coord.z;
		verts[3].val = pb->GetPnt(i + 1, j, k,coord);
		verts[3].x = coord.x;		verts[3].y = coord.y;		verts[3].z = coord.z;
		verts[4].val = pb->GetPnt(i, j + 1, k,coord);
		verts[4].x = coord.x;		verts[4].y = coord.y;		verts[4].z = coord.z;
		verts[5].val = pb->GetPnt(i, j + 1, k + 1,coord);
		verts[5].x = coord.x;		verts[5].y = coord.y;		verts[5].z = coord.z;
		verts[6].val = pb->GetPnt(i + 1, j + 1, k + 1,coord);
		verts[6].x = coord.x;		verts[6].y = coord.y;		verts[6].z = coord.z;
		verts[7].val = pb->GetPnt(i + 1, j + 1, k,coord);
		verts[7].x = coord.x;		verts[7].y = coord.y;		verts[7].z = coord.z;


		// for marching cube method, the iso-surface is found by locating those sign changing edge, 
		// to make it happen, we force the most outer boundary value equal to zero

		double outValue = 0; // used for changing the intersection pt of the iso-surface, due to the linear interpolation.

		//get the index
		int cubeIndex = int(0);
		for (int n = 0; n < 8; n++)
		{
			if (verts[n].val == 0.0)
				verts[n].val = outValue;

			/*(step 4)*/
			if (verts[n].val >= minValue)
				cubeIndex |= (1 << n);
		}

		//check if its completely inside or outside

		/*(step 5)*/
		if (!edgeTable[cubeIndex])
			continue;

		//get intersection vertices on edges and save into the array
		mpVector intVerts[12];

		/*(step 6)*/

		if (edgeTable[cubeIndex] & 1) intVerts[0] = LinearInterp(verts[0], verts[1], minValue);
		if (edgeTable[cubeIndex] & 2) intVerts[1] = LinearInterp(verts[1], verts[2], minValue);
		if (edgeTable[cubeIndex] & 4) intVerts[2] = LinearInterp(verts[2], verts[3], minValue);
		if (edgeTable[cubeIndex] & 8) intVerts[3] = LinearInterp(verts[3], verts[0], minValue);
		if (edgeTable[cubeIndex] & 16) intVerts[4] = LinearInterp(verts[4], verts[5], minValue);
		if (edgeTable[cubeIndex] & 32) intVerts[5] = LinearInterp(verts[5], verts[6], minValue);
		if (edgeTable[cubeIndex] & 64) intVerts[6] = LinearInterp(verts[6], verts[7], minValue);
		if (edgeTable[cubeIndex] & 128) intVerts[7] = LinearInterp(verts[7], verts[4], minValue);
		if (edgeTable[cubeIndex] & 256) intVerts[8] = LinearInterp(verts[0], verts[4], minValue);
		if (edgeTable[cubeIndex] & 512) intVerts[9] = LinearInterp(verts[1], verts[5], minValue);
		if (edgeTable[cubeIndex] & 1024) intVerts[10] = LinearInterp(verts[2], verts[6], minValue);
		if (edgeTable[cubeIndex] & 2048) intVerts[11] = LinearInterp(verts[3], verts[7], minValue);

		//now build the triangles using triTable
		for (int n = 0; triTable[cubeIndex][n] != -1; n += 3)
		{
			/*(step 7)*/
			//triangles[numTriangles].p[0] = intVerts[triTable[cubeIndex][n+2]];
			//triangles[numTriangles].p[1] = intVerts[triTable[cubeIndex][n+1]];
			//triangles[numTriangles].p[2] = intVerts[triTable[cubeIndex][n]];
			//TRIANGLE t;
			//t.p[0] = intVerts[triTable[cubeIndex][n + 2]];
			//t.p[1] = intVerts[triTable[cubeIndex][n + 1]];
			//t.p[2] = intVerts[triTable[cubeIndex][n]];

			mpVector p0, p1, p2;
			p0 = intVerts[triTable[cubeIndex][n + 2]];
			p1 = intVerts[triTable[cubeIndex][n + 1]];
			p2 = intVerts[triTable[cubeIndex][n]];
			mesh->m_V.Append(ON_3fPoint(p0.x, p0.y, p0.z));
			mesh->m_V.Append(ON_3fPoint(p1.x, p1.y, p1.z));
			mesh->m_V.Append(ON_3fPoint(p2.x, p2.y, p2.z));

			ON_MeshFace mf;
			int index[4];
			index[0] = numTriangles * 3;
			index[1] = numTriangles * 3 + 1;
			index[2] = numTriangles * 3 + 2;
			index[3] = index[2];
			for (int m = 0; m < 4; m++)
			{
				mf.vi[m] = index[m];
			}
			mesh->m_F.Append(mf);

			/*(step 8)*/
			//calculate normal
			//t.norm =((t.p[1] - t.p[0]).Cross(t.p[2] - t.p[1])).Normalize();

			//triangles.push_back(t);

			numTriangles++;
		}
	}	//END OF FOR LOOP



	if (mesh)
	{
		mesh->CombineIdenticalVertices(true, true);
		omesh = Mesh2OMesh(mesh);
		// Initialize smoother with input mesh
		OpenMesh::Smoother::JacobiLaplaceSmootherT<FMesh> smoother(*omesh);
		smoother.initialize(OpenMesh::Smoother::JacobiLaplaceSmootherT<FMesh>::Tangential_and_Normal, OpenMesh::Smoother::JacobiLaplaceSmootherT<FMesh>::C0);
		smoother.smooth(1);
		omesh->request_face_normals();
		omesh->update_face_normals();
	}

	xdir = pb->xaxis;
	ydir = pb->yaxis;
	zdir = pb->zaxis;
	xlength = (pb->xn - 3)*pb->samplewidth;
	ylength = (pb->yn - 3)*pb->samplewidth;
	zlength = (pb->zn - 3)*pb->samplewidth;
	origin = pb->origin + xdir*pb->samplewidth + ydir*pb->samplewidth + zdir*pb->samplewidth;
	//mesh->Destroy();
}

ON_3dPoint MeshBlock::Coord(double px, double py, double pz)
{
	return origin + xdir*px*xlength + ydir*py*ylength + zdir*pz*zlength;
}

int MeshBlock::GetFaceNum()
{
	FMesh::FaceIter f_it(omesh->faces_end());

	return f_it->idx() + 1;
}

bool MeshBlock::ExportSTL_binary(FILE* fout, bool isBinary)
{
	if (fout == 0)
		return false;

	// output based on openmesh
	if (isBinary)
	{
		// binary format reference: http://www.fabbers.com/tech/STL_Format#Sct_binary


		unsigned short attributes = 0;

		FMesh::ConstFaceIter        f_it(omesh->faces_begin()),
			f_end(omesh->faces_end());
		FMesh::ConstFaceVertexIter  fv_it;

		OpenMesh::Vec3d _p;
		for (; f_it != f_end; ++f_it)
		{
			fv_it = omesh->cfv_iter(*f_it);

			const OpenMesh::Vec3d& _n = omesh->normal(*f_it);
			float nvt[3];
			nvt[0] = _n[0];
			nvt[1] = _n[1];
			nvt[2] = _n[2];
			fwrite(nvt, sizeof (float), 3, fout);

			_p = omesh->point(*fv_it);
			nvt[0] = _p[0];
			nvt[1] = _p[1];
			nvt[2] = _p[2];
			fwrite(nvt, sizeof (float), 3, fout);
			++fv_it;

			_p = omesh->point(*fv_it);
			nvt[0] = _p[0];
			nvt[1] = _p[1];
			nvt[2] = _p[2];
			fwrite(nvt, sizeof (float), 3, fout);
			++fv_it;

			_p = omesh->point(*fv_it);
			nvt[0] = _p[0];
			nvt[1] = _p[1];
			nvt[2] = _p[2];
			fwrite(nvt, sizeof (float), 3, fout);

			fwrite(&attributes, 1, sizeof(short), fout);

		}

	}
	else // in ASCII
	{

		FMesh::ConstFaceIter        f_it(omesh->faces_begin()),
			f_end(omesh->faces_end());
		FMesh::ConstFaceVertexIter  fv_it;

		OpenMesh::Vec3d _p1, _p2, _p3;
		for (; f_it != f_end; ++f_it)
		{
			fv_it = omesh->cfv_iter(*f_it);

			const OpenMesh::Vec3d& _n = omesh->normal(*f_it);

			_p1 = omesh->point(*fv_it);
			++fv_it;
			_p2 = omesh->point(*fv_it);
			++fv_it;
			_p3 = omesh->point(*fv_it);

			fprintf(fout, "  facet normal %13e %13e %13e\n", _n[0], _n[1], _n[2]);
			fprintf(fout, "    outer loop\n");
			fprintf(fout, "      vertex  %13e %13e %13e\n", _p1[0], _p1[1], _p1[2]);
			fprintf(fout, "      vertex  %13e %13e %13e\n", _p2[0], _p2[1], _p2[2]);
			fprintf(fout, "      vertex  %13e %13e %13e\n", _p3[0], _p3[1], _p3[2]);
			fprintf(fout, "    endloop\n");
			fprintf(fout, "  endfacet\n");

		}

	}
	return true;
	
}

bool MeshBlock::ExportPLY(const char*filename)
{
	// adapt to openmesh structure
	FMesh::VertexIter        v_it, v_end(omesh->vertices_end());
	FMesh::FaceIter        f_it(omesh->faces_begin()),
		f_end(omesh->faces_end());
	FMesh::ConstFaceVertexCCWIter  fv_it;

	int numTris = v_end->idx() + 1;
	int numVertices = GetFaceNum();

	FILE* fout = fopen(filename, "wb");
	printf("Vertices counted: %d Triangles counted: %d \n", numVertices, numTris);
	PLYWriter::writeHeader(fout, numVertices, numTris);

	clock_t start = clock();
	size_t i;
	float vt[3];
	for (v_it = omesh->vertices_begin(); v_it != v_end; ++v_it)
	{
		FMesh::Point p = omesh->point(*v_it);
		vt[0] = p[0];
		vt[1] = p[1];
		vt[2] = p[2];
		PLYWriter::writeVertex(fout, vt);
	}

	int fc[3];
	for (; f_it != f_end; ++f_it)
	{
		fv_it = omesh->fv_ccwbegin(*f_it);
		fc[0] = fv_it->idx(); ++fv_it;
		fc[1] = fv_it->idx(); ++fv_it;
		fc[2] = fv_it->idx();
		PLYWriter::writeFace(fout, 3, fc);
	}

	clock_t finish = clock();
	printf("Time used: %f seconds.\n", (float)(finish - start) / (float)CLOCKS_PER_SEC);

	fclose(fout);

	return true;
}

void MeshBlock::Update()
{
	FT::FEntity::Update();
}

void MeshBlock::DrawSmoothShade()
{
	//glDisable(GL_LIGHTING);
	glColor3f(0.8, 0.2, 0.2);
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPushMatrix();
	glCallList(m_uListSmooth);
	glPopMatrix();
}

void MeshBlock::DrawMesh()
{
	glShadeModel(GL_SMOOTH);
	glColor3f(0.8, 0.2, 0.2);
	GLfloat specComp[4] = { 0.25, 0.25, 0.25, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = { 0.2, 0.2, 0.2, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = { 0.8, 0.8, 0.8, 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);

	glPolygonOffset(1, 1);
	glEnable(GL_DEPTH_TEST);
	glPushMatrix();
	glColor3f(0, 0, 1);
	//glCallList(m_uListSmooth);
	glCallList(m_uListMesh);
	glPopMatrix();
	glPolygonOffset(0, 0);
	glDisable(GL_POLYGON_OFFSET_FILL);
}

void MeshBlock::InitDisplayList()
{
	m_bDispList = true;
	if (glIsList(tempList))
		glDeleteLists(tempList, 3);
	tempList = glGenLists(3);

	glNewList(tempList, GL_COMPILE);
	glBegin(GL_TRIANGLES);

	//********* openmesh display********************
	FMesh::ConstFaceIter        f_it(omesh->faces_begin()),
		f_end(omesh->faces_end());
	FMesh::ConstFaceVertexIter  fv_it;

	OpenMesh::Vec3d _p;
	for (; f_it != f_end; ++f_it)
	{
		if (omesh->has_face_normals())
		{
			const OpenMesh::Vec3d& _n = omesh->normal(*f_it);
			glNormal3f(_n[0], _n[1], _n[2]);
		}
		fv_it = omesh->cfv_iter(*f_it);
		_p = omesh->point(*fv_it);
		glVertex3f(_p[0], _p[1], _p[2]);
		++fv_it;
		_p = omesh->point(*fv_it);
		glVertex3f(_p[0], _p[1], _p[2]);
		++fv_it;
		_p = omesh->point(*fv_it);
		glVertex3f(_p[0], _p[1], _p[2]);
	}




	glEnd();
	glEndList();

	// smooth shade list
	m_uListSmooth = tempList + 1;
	glNewList(m_uListSmooth, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glCallList(tempList);
	glEndList();

	// draw mesh list
	m_uListMesh = tempList + 2;
	glNewList(m_uListMesh, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);         // Draw Mesh
	glCallList(tempList);
	glEndList();
}

mpVector LinearInterp(mp4Vector p1, mp4Vector p2, float value)
{
	//p = (mpVector)p1 + ((mpVector)p2 - (mpVector)p1) / (p2.val - p1.val)*(value - p1.val);
	mpVector p;
	if (fabs(p1.val - p2.val) < 0.00001)
		return (mpVector)p1;
	if (fabs(value - p1.val) < 0.00001)
		return (mpVector)p1;
	if (fabs(value - p2.val) < 0.00001)
		return (mpVector)p2;

	float mu = (value - p1.val) / (p2.val - p1.val);
	p.x = p1.x + mu * (p2.x - p1.x);
	p.y = p1.y + mu * (p2.y - p1.y);
	p.z = p1.z + mu * (p2.z - p1.z);

	return p;
}
