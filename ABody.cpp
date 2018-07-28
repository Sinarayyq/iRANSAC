#include "ABody.h"

void ABody::Init()
{
	m_pMesh = NULL;
	m_pBrep = NULL;
	m_bMeshed = false;
}
ABody::ABody()
{
	Init();
}
ABody::~ABody()
{
	if (m_pMesh)
		delete m_pMesh;
	if (m_pBrep)
		delete m_pBrep;
}

void ABody::DrawPoints()
{
	FEntity::DrawPoints();
	if (!m_pMesh )	return;
	if(m_iSelect==0) 
		glColor3f(m_red, m_green, m_blue);
	GLfloat specComp[4] = {0.25, 0.25, 0.25, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = {0.2, 0.2, 0.2, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = {0.8, 0.8, 0.8, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);
	glCallList(m_uListPoints);	
}

void ABody::DrawSmoothShade()
{
	if (!m_pMesh )	return;
	if(m_iSelect==0) 
		glColor3f(m_red, m_green, m_blue);
	GLfloat specComp[4] = {0.25, 0.25, 0.25, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = {0.2, 0.2, 0.2, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = {0.8, 0.8, 0.8, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);
	glCallList(m_uListSmooth);
}

void ABody::DrawMesh()
{
	if (!m_pMesh )	return;
	if(m_iSelect==0) 
		glColor3f(m_red, m_green, m_blue);
	GLfloat specComp[4] = {0.25, 0.25, 0.25, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = {0.2, 0.2, 0.2, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = {0.8, 0.8, 0.8, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0,1.0);

	glEnable(GL_DEPTH_TEST);		
	glCallList(m_uListSmooth);
	glPolygonOffset(0.0f, 0.0f);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor3f(0.0f,0.0f,0.0f);
	glCallList(m_uListMesh);
}

void ABody::DrawFlatShade()
{
	if (!m_pMesh )	return;
	if(m_iSelect==0) 
		glColor3f(m_red, m_green, m_blue);
	GLfloat specComp[4] = {0.25, 0.25, 0.25, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specComp);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 30);
	GLfloat ambientParams[4] = {0.2, 0.2, 0.2, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientParams);
	GLfloat diffuseParams[4] = {0.8, 0.8, 0.8, 1};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseParams);
	glCallList(m_uListFlat);
}

void ABody::Update()
{
	FT::FEntity::Update();
	if (!m_bMeshed)
	{
		//put remesh code here
	}
	if (!m_bBox)
	{
		CalBoundBox();
	}
}

void ABody::CalBoundBox()
{
	if (!m_pMesh)	return;
	double minPnt[3], maxPnt[3];
	if (m_pMesh->GetBBox(minPnt, maxPnt))
	{
		m_entBox.SetMinPoint(F3dPoint(minPnt));
		m_entBox.SetMaxPoint(F3dPoint(maxPnt));
	}
}

void ABody::InitDisplayList()
{
	if ( NULL == m_pMesh )	return;
	m_bDispList = true;
	if (glIsList(m_uListFlat))
		glDeleteLists(m_uListFlat,5);
	GLuint tempList = glGenLists(5);
	int fi;
	F3dPoint v[4];
	F3fVector n;
	int face_count = m_pMesh->FaceCount(); 
	glNewList(tempList,GL_COMPILE);
	glBegin(GL_TRIANGLES);
	for (int fi=0; fi<face_count; fi++) 
	{
		const ON_MeshFace& f = m_pMesh->m_F[fi];
		v[0] = m_pMesh->m_V[f.vi[0]];
		v[1] = m_pMesh->m_V[f.vi[1]];
		v[2] = m_pMesh->m_V[f.vi[2]];
		n = m_pMesh->m_FN[fi];
		glNormal3d(n.x,n.y,n.z);	//face normal
		for (int i = 0; i < 3; i++ )
		{
			glVertex3d(v[i].x,v[i].y,v[i].z);
		}
	}
	glEnd();
	glEndList();

	//drawflatshade list
	m_uListFlat = tempList+1;
	glNewList(m_uListFlat, GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glCallList(tempList);
	glEndList();

	//draw smooth shade list
	F3fVector normal[4];
	m_uListSmooth = tempList+2;
	glNewList(m_uListSmooth,GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	ON_GL(*m_pMesh);
	glEndList();

	//draw mesh list
	m_uListMesh = tempList+3;
	glNewList(m_uListMesh,GL_COMPILE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);         // Draw Mesh
	glCallList(tempList);
	glEndList();

	//draw point list
	int nsize = m_pMesh->m_V.Count();
	m_uListPoints = tempList+4;
	glNewList(m_uListPoints,GL_COMPILE);
	glPointSize(3.0);
	glBegin(GL_POINTS);
	for (int i=0; i<nsize; i++) 
	{
		n = m_pMesh->m_N[i];
		glNormal3d(n.x,n.y,n.z);	//face normal
		glVertex3d(m_pMesh->m_V[i].x, m_pMesh->m_V[i].y, m_pMesh->m_V[i].z);
	}
	glEnd();
	glEndList();
}