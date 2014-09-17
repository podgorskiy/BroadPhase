#include "StdAfx.h"
#include <cimg/CImg.h>
#include "VisualizationWinow.h"

using namespace cimg_library;

VisualizationWinow::~VisualizationWinow()
{
	assert(m_disp);
	delete m_disp;
	m_disp = NULL;
}

void VisualizationWinow::Init(int width, int height, float size, const char* title)
{
	m_disp = new CImgDisplay(width, height, title, 0);
	m_worldHeight = size;
	m_size = size;
	m_worldOffset_x = 0;
	m_worldOffset_y = 0;
	m_canvas = new Canvas(m_disp->width(),m_disp->height(), 1, 3, 255);
}

void VisualizationWinow::BeginDraw()
{
	if( m_disp->is_resized() )
	{
		m_disp->resize();
		assert(m_canvas);
		delete m_canvas;
		m_canvas = new Canvas(m_disp->width(),m_disp->height(),1,3,255);
	}
	m_canvas->fill(0);
	m_aspect = static_cast<float>(m_disp->width()) / static_cast<float>(m_disp->height());
}

void VisualizationWinow::EndDraw()
{
	m_canvas->display(*m_disp);
	cimg::wait(10);
};

bool VisualizationWinow::GoingToClose() const
{
	return m_disp->is_closed() || m_disp->is_keyESC();
};

void VisualizationWinow::toScreen(float w_x, float w_y, int& s_x, int& s_y) const
{
	s_x = int( (  ( w_x - m_worldOffset_x ) / m_worldHeight / m_aspect	+ 0.5f ) * m_disp->width() );
	s_y = int( ( -( w_y - m_worldOffset_y ) / m_worldHeight				+ 0.5f ) * m_disp->height() );
};

void VisualizationWinow::toWorld(float& w_x, float& w_y, int s_x, int s_y) const
{
	w_x =   ( s_x / m_disp->width()  * 2.0f - 1.0f ) * m_aspect * m_worldHeight + m_worldOffset_x;
	w_y = - ( s_y / m_disp->height() * 2.0f - 1.0f ) * m_worldHeight + m_worldOffset_y;
};

void VisualizationWinow::DrawAxes() const
{
	int xh0,yh,xh1;
	toScreen( m_size/2.0f,0,xh0,yh);
	toScreen(-m_size/2.0f,0,xh1,yh);
	int xv,yv0,yv1;
	toScreen(0, m_size/2.0f,xv,yv0);
	toScreen(0,-m_size/2.0f,xv,yv1);
	m_canvas->draw_line(xh1, yh, xh0, yh, white);
	m_canvas->draw_line(xv, yv1, xv, yv0, white);
	char txt[32] = { 0 };
	for(int i=-5;i<=5;i++)
	{
		if(i==0)
			continue;
		cimg_snprintf(txt,sizeof(txt),"%g",(float)i*10.0f/m_size);
		const int
        xi = (int)(i/5.0f*m_size/2.0f),
        tx = 3,
		ty = 7;
		int x,y;
		toScreen(float(xi), 0.f, x, y);
		m_canvas->draw_point(x, y + 1, white, 1).draw_point(x, y - 1, white, 1);
		m_canvas->draw_text(x - tx, y + 2, txt, white, black, 1, 15);
		toScreen(0.f, float(xi), x,y);
		m_canvas->draw_point(x + 1, y, white, 1).draw_point(x - 1, y, white, 1);
		m_canvas->draw_text(x + 4, y - ty, txt, white, black, 1, 15);
	}
}

void VisualizationWinow::DrawLineScreenSpace(int x0, int y0, int x1, int y1, const unsigned char* color) const
{
	m_canvas->draw_line(x0, y0, x1, y1, color, 1.0f);
}

void VisualizationWinow::DrawLine(float wx0, float wy0, float wx1, float wy1, const unsigned char* color) const
{
	int x0,y0,x1,y1;
	toScreen(wx0, wy0, x0, y0);
	toScreen(wx1, wy1, x1, y1);
	DrawLineScreenSpace(x0, y0, x1, y1, color);
}


void VisualizationWinow::DrawTextL(std::string text, int s_x, int s_y, const unsigned char* color) const
{
	m_canvas->draw_text(s_x, s_y, text.c_str(), white, black, 1, 15, 1.0f, 15);
}
