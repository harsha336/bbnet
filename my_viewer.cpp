
# include "my_viewer.h"

# include <sig/sn_model.h>
# include <sig/sn_material.h>
# include <sig/sn_transform.h>

# include <sigogl/ws_run.h>

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }

MyViewer::MyViewer ( int x, int y, int w, int h, const char *l ) : WsViewer ( x, y, w, h, l )
{
	_root = WsViewer::rootg();

	_lines = new SnLines;
	_world = new SnLines;
	_points = new SnPoints;
	_spheres = new SnGroup;
	net_lines_ = new SnLines;

	view ( false, true, true ); // lines, world, spheres

	_root->add(_lines);
	_root->add(_points);
	_root->add(_world);
	_root->add(_spheres);
	_root->add(net_lines_);
  
	_sphereradius = 0.2f;
}

MyViewer::~MyViewer ()
{
}

enum Events {	EvInit,
				EvViewSpheres, EvViewVelVecs,
				EvRun, EvWind, EvBounce,
				EvSCollision, EvVCollision, EvTScale, EvCRestitut,
				EvView, EvExit
			};

void MyViewer::build_ui ()
{
	UiPanel *p, *sp, *paramsp;
	UiManager* uim = WsWindow::uim();
	bool detachparms = true;

	// Top Menu Bar:
	p = uim->add_panel ( "", UiPanel::HorizLeft );
	p->add ( new UiButton ( "Init", EvInit ) );
	if (!detachparms) p->add ( new UiButton ( "Parameters", sp=new UiPanel() ) ); paramsp=sp;
	p->add ( new UiButton ( "View", sp=new UiPanel() ) );
	{	UiPanel* p=sp;
		p->add ( _vvel = new UiCheckButton ( "Velocities", EvView, false ) ); p->top()->separate();
		p->add ( _vworld = new UiCheckButton ( "World", EvView, true ) ); 
		p->add ( _vsphere = new UiCheckButton ( "Spheres", EvView, true ) ); 
	}
	p->add ( new UiButton ( "Exit", EvExit ) );

	if (!detachparms) p = paramsp; // build parameters panel now
	else p = uim->add_panel ( "Parameters", UiPanel::Vertical, UiPanel::Left, 0, 30 );

	p->add ( _run = new UiCheckButton ( "Run", EvRun ) );
	p->add ( _wind = new UiCheckButton ( "Wind", EvWind ) ); 
	p->add ( _scol = new UiCheckButton ( "Spring-Based Collision", EvSCollision ) );
	p->add ( _vcol = new UiCheckButton ( "Vel. Reflection Collision", EvVCollision ) );
	p->add ( _bounce = new UiCheckButton ( "Bounce", EvBounce ) ); _bounce->value(true);

	p->add ( new UiElement("Wind Force:") ); p->top()->separate();
	p->add ( _windslider = new UiSlider("", EvWind) );
	_windslider->range(-5.0f,5.0f);

	p->add ( new UiElement("Time Scale:") ); 
	p->add ( _tscaleslider = new UiSlider("", EvTScale) );
	_tscaleslider->range(0.01f,4.0f); _tscaleslider->value(1.0f);

	p->add ( new UiElement("Coeff. of Restitution:") ); 
	p->add ( _crestslider = new UiSlider("", EvCRestitut) );
}

void MyViewer::view ( bool vel, bool world, bool spheres )
{
	_lines->visible ( vel );
	_world->visible ( world );
	_spheres->visible ( spheres );
	_points->visible ( !spheres );
	net_lines_->visible( true );
}

void MyViewer::build_scene ( BBNet& bbn, int nfaces )
{
	const GsArray<Particle>& p = bbn.particles();
	int i, psize=p.size();
	bbn_ = &bbn;
	_nfaces = nfaces;

	nres_ = bbn_->getNetRes();
	nlevel_ = bbn_->getNetLevel();

	_crestslider->value ( bbn_->coeffRestitution() );

	SnModel* sphere = new SnModel;
	sphere->model()->make_sphere ( GsPnt::null, _sphereradius, nfaces, true );

	SnGroup* g;
	SnTransform* t;
	SnMaterial* m;

	_spheres->init();
	_positions.size(0); // fast access to later update positions
	for ( i=0; i<psize; i++ )
	{	_spheres->add ( g=new SnGroup );
		g->separator(true); // transformations inside group do not affect outside group
		g->add ( t=new SnTransform );
		g->add ( m=new SnMaterial );
		g->add ( sphere );
		_positions.push() = &(t->get());
		t->get().translation ( p[i].x );
		m->material().diffuse = GsColor::darkblue;
    }

	_points->init();
	_points->point_size ( 3.0f );
	_points->color ( GsColor::blue );
	for ( i=0; i<psize; i++ )
	{	_points->push ( p[i].x );
	}

	_lines->init();
	_lines->color ( GsColor::blue );
	for ( i=0; i<psize; i++ )
	{
		_lines->push ( p[i].x, p[i].x+p[i].v );
	}

	net_lines_->init();
	net_lines_->color ( GsColor::black );
	int idx_next, idx_prev;
	for ( i = 0; i < nlevel_ - 1; i++ )
	{
		for( int j = 0;j < nres_;j++ )
		{
			if( j == 0 )
			{
				idx_next = (i+1)*nres_ + j + 1;
				idx_prev = (i+1)*nres_ + j + (nres_ - 1);
			}
			else if( j == nres_ - 1 )
			{
				idx_next = (i+1)*nres_;
				idx_prev = (i+1)*nres_ + j - 1;
			}
			else
			{
				idx_next = (i+1)*nres_ + j + 1;
				idx_prev = (i+1)*nres_ + j - 1;
			}
			net_lines_->push( p[i].x, p[idx_next].x );
			net_lines_->push( p[i].x, p[idx_prev].x );
		}
	}

	_world->init();
	_world->color ( GsColor::red );
      
	int r=3;
	float fr = (float) r;
	float fr2 = fr+fr;
	const GsBox& w = bbn.world();
	GsVec dx = GsVec::i*w.dx();
	GsVec dz = GsVec::k*w.dz();
	GsPnt a ( w.a.x*fr2, w.a.y, w.a.z*fr2 );
	for ( i=-r; i<=r; i++ )
	{	_world->push ( a, a+(dx*fr2) );
		a+=dz;
	}
	a.set ( w.a.x*fr2, w.a.y, w.a.z*fr2 );
	for ( i=-r; i<=r; i++ )
	{	_world->push ( a, a+(dz*fr2) );
		a+=dx;
	}
}

void MyViewer::update_scene ()
{
	const GsArray<Particle>& p = bbn_->particles();
	int i, psize=p.size();

	if ( _spheres->visible() )
	{	for ( i=0; i<psize; i++ )
		{	_positions[i]->setrans ( p[i].x );
		}
	}
	else
    {	for ( i=0; i<psize; i++ )
		{	_points->P[i] = p[i].x;
			_points->touch();
		}
	}

	if ( _lines->visible() )
	{	_lines->init();
		for ( i=0; i<psize; i++ )
		{	_lines->push ( p[i].x, p[i].x+p[i].v );
		}
	}
}

void MyViewer::run ()
{
	if ( !_run->value() ) return; // turning sim off

	double tfac = 1.0; //ui_timeslider->value();
	double t0, tprior, tnow;

	t0 = tprior = gs_time()*tfac;
	while ( _run->value() )
	{
		// draw current state:
		update_scene ();
		ws_check();

		// check time scale:
		if ( tfac!=_tscaleslider->value() )
		{	t0 /= tfac;
			tprior /= tfac;
			tfac=_tscaleslider->value();
			t0 *= tfac;
			tprior *= tfac;
		}

		// compute next step:
		tnow = gs_time () * tfac;
		bbn_->eulerStep ( float(tnow-tprior) );
		tprior = tnow;

		// display some info:
		message().setf ( "run: %5.2f s", float(tnow-t0) );
	}
}

int MyViewer::uievent ( int e )
{
	switch ( e )
	{	case EvInit:
		{	bbn_->init ( bbn_->get_cfg_file() );
			build_scene ( *bbn_, _nfaces );
		} break;

		case EvWind:
		{	float windmag = _windslider->value();
			if ( !_wind->value() ) windmag=0;
			bbn_->externalForce ( GsVec( windmag, 0, 0 ) );
		} break;

		case EvBounce:
		{	bbn_->bounce ( _bounce->value() );
		} break;

		case EvSCollision:
		{	bbn_->springCollisionPrevention ( _scol->value() );
		} break;

		case EvVCollision:
		{	bbn_->velocityCollisionPrevention ( _vcol->value() );
		} break;

		case EvCRestitut:
		{	bbn_->coeffRestitution ( _crestslider->value() );
		} break;

		case EvView:
		{	view ( _vvel->value(), _vworld->value(), _vsphere->value() );
		} break;

		case EvRun: run(); break;

		case EvExit: gs_exit();
	}
	return WsViewer::uievent(e);
}

int MyViewer::handle_scene_event ( const GsEvent &e )
{
	if ( e.button1 )
	{	// nothing here for now
	}

	return WsViewer::handle_scene_event ( e );
}
