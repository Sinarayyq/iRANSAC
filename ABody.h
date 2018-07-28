/*	
\brief	This file is only used to demonstrate how to create a display entity.For practice usage, please 
		use FBody instead. 
\author: Jacky
\date    2014-04
*/
#ifndef F_SOLID_H
#define F_SOLID_H

#include "../fkernel/FEntity.h"
#include "../fkernel/FKernelDefines.h"

using namespace FT;


class ABody: public FEntity
{
public:
	ABody();
	~ABody();
	/////////////////////////////// Accessing methods ///////////////////////////////////////////
	void			SetBRep(ON_Brep* pBrep){m_pBrep=pBrep;}
	ON_Brep*		GetBRep(){return m_pBrep;}
	void			SetMesh(ON_Mesh* pMesh){m_pMesh=pMesh;}
	ON_Mesh*		GetMesh(){return m_pMesh;}
	void			SetbMeshed(bool bval){m_bMeshed=bval;}
	bool			IsMeshed(){return m_bMeshed;}

	/////////////////////////////// Public methods ///////////////////////////////////////////
	void			Init();	
	void			Update();
	virtual void	DrawPoints();
	virtual void	DrawSmoothShade();
	virtual void	DrawFlatShade();
	virtual void	DrawMesh();
	virtual void	InitDisplayList();
	virtual void	CalBoundBox();

protected:
	ON_Brep*		m_pBrep;			///surface representation
	ON_Mesh*		m_pMesh;			///mesh representation
	bool			m_bMeshed;			///if false, m_pMesh needs to be updated
private:
	GLuint			m_uListFlat,m_uListSmooth,m_uListMesh, m_uListPoints;
};

#endif