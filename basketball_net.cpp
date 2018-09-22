
#include "basketball_net.h"
#include "sig/gs_string.h"

#include <sigogl/ui_dialogs.h>

#include <sig/gs_trace.h>

BBNet::BBNet()
{
	time_ = 0;
	bounce_ = true;
	collides_ = collidev_ = false;
	gravity_.set (0, -9.807f, 0 );
	restitution_ = 0.5f;
	colradius_ = 1.0f;
	colks_ = 1.0f;
	colkd_ = 1.0f;
}

BBNet::~BBNet()
{
}

void BBNet::init( const char* cfgfile )
{
	GsInput in;
	if( !cfgfile ) cfgfile = ui_select_file( "Enter Configuration File", 0, "*.txt" );
	if( !cfgfile ) gsout.fatal( "Configuration file needed!" );
	if( !in.open(cfgfile) ) gsout.fatal( "Could not open cfg file!" );
	lcfgfile_ = cfgfile;

	GsVars* params = new GsVars;
	in.commentchar( '#' );
	in.lowercase( false );
	while( true )
	{
		if( in.get() == GsInput::End ) break;
		GS_TRACE1 ( "Token: " << in.ltoken() );

		if(in.ltoken() == "parameters" )
		{
			in >> *params;
		}
		else if( in.ltoken() == "end" )
		{
			break;
		}
	}
	GS_TRACE1( "End Parsing" );

	init( params );
	delete params;
}

void BBNet::init( const GsVars* vars )
{
	int size = vars->geti("particles", 0);

	rim_radius_ = 8.0f;//vars->getf("radius", 0);
	if(rim_radius_ < 5.0f || rim_radius_ > 8.0f)
		rim_radius_ = 5;
	net_res_ = 10;//vars->geti("res", 0);
	if(net_res_ <= 0)
		net_res_ = 10;

	float mass1 = vars->getf("part.mass", 0);
	float mass2 = vars->getf("part.mass", 1);

	float rest1 = vars->getf("part.restit", 0); if(rest1 <= 0.1f) rest1 = 0.1f;
	float rest2 = vars->getf("part.restit", 1); if(rest2 >= 1.0f) rest2 = 1.0f;

	float vel1 = vars->getf("part.vel", 0);
	float vel2 = vars->getf("part.vel", 1);
	GsVec vel( vars->getf("part.dir", 0), vars->getf("part.dir", 1), vars->getf("part.dir", 2));
	GsString velrand = vars->gets("part.dir.rand");

	colks_ = vars->getf("col.ks");
	colkd_ = vars->getf("col.kd");

	restitution_ = vars->getf("restitution");

	GsVar* v;
	v = vars->get("worldbox");
	world_.set( GsVec(v->getf(0),v->getf(1),v->getf(2)), GsVec(v->getf(3),v->getf(4),v->getf(5)) );

	v = vars->get("initpos");
	GsPnt posa( GsVec(v->getf(0), v->getf(1), v->getf(2)) );
	GsPnt posb( GsVec(v->getf(3), v->getf(4), v->getf(5)) );

	particles_.size( LEVEL*net_res_ );
	maxmass_ = 0;
	time_ = 0;

	int i;
	GsQuat q;
	GsVec pos;
	float rr = rim_radius_;
	float ydiff;
	for( i = 0;i < LEVEL; i++ )
	{
		float theta = (2*M_PI) / net_res_;
		for( int j = 0;j < net_res_;j++ )
		{
			int idx = i*net_res_ + j;
			GS_TRACE1(" Current index is: "<<idx);
			pos.set( rr*cos(j*theta), (LEVEL-i)*1.0f, rr*sin(j*theta) );
			particles_[idx].init( gs_random(mass1, mass2), pos );
			particles_[idx].r = gs_random(rest1, rest2);
			if( particles_[idx].m > maxmass_ ) maxmass_ = particles_[i].m;
			vel.len( gs_random(vel1, vel2) );
			particles_[idx].v = vel;
			if( velrand == "random" )
			{
				GsQuat q;
				q.random();
				particles_[idx].v = q.apply( vel );
			}
		}
		rr -= 0.75f;
	}
}

void BBNet::getState()
{
	state_.size( particles_.size() * 6 );
	float *s = &state_[0];
	for( int i = 0, size = particles_.size(); i < size; i++)
	{
		Particle& p = particles_[i];
		*(s++) = p.x.x; *(s++) = p.x.y; *(s++) = p.x.z;
		*(s++) = p.v.x; *(s++) = p.v.y; *(s++) = p.x.z;
	}
}

void BBNet::setState()
{
	const float *s = &state_[0];
	for( int i = 0, size = particles_.size(); i < size; i++ )
	{
		Particle& p = particles_[i];
		p.x.x = *(s++); p.x.y = *(s++); p.x.z = *(s++);
		p.v.x = *(s++); p.v.y = *(s++); p.v.z = *(s++);
	}
}

void BBNet::computeDerivatives()
{
	clearForces();
	computeForces();

	derivatives_.size( particles_.size() * 6 );
	float *s = &derivatives_[0];

	for( int i = 0; i < particles_.size(); i++)
	{
		Particle& p = particles_[i];
		*(s++) = p.v.x;
		*(s++) = p.v.y;
		*(s++) = p.v.z;
		*(s++) = p.f.x/p.m;
		*(s++) = p.f.y/p.m;
		*(s++) = p.f.z/p.m;
	}
}

void BBNet::clearForces()
{
	for( int i = 0; i < particles_.size(); i++)
	{
		particles_[i].f = GsVec::null;
	}
}

void BBNet::computeForces()
{
	int i, j, size = particles_.size();

	for( int i = 0;i < size; i++ )
	{
		particles_[i].f += (particles_[i].m*gravity_) + extforce_;
	}

	if( collides_ )
	{
		float dist, r2 = colradius_ * 2.0f;
		GsVec l, dv, f;

		for( i = 0;i < size; i++ )
		{
			for( j = 0;j < size;j++ )
			{
				if( i == j ) continue;
				dist = ::dist(particles_[i].x, particles_[j].x);
				if( dist > 0.0001f && dist < r2)
				{
					l = particles_[i].x - particles_[j].x;
					dv = particles_[i].v - particles_[j].v;
					f = l / dist * -(colks_*(dist-r2) + dot(dv,l)*colkd_/dist); // spring behaviour
					particles_[i].f += f;

					/*GsVec n = l; n.normalize();
					 * f = particles_[i].f;
					 * particles_[i].f += -f*dot(n,f);*/
				}
			}
		}
	}

	if( 0 )
	{
		GsVec n = GsVec::j; // bottom plane normal pointing up
		GsVec v, nv;
		for( i = 0;i < size; i++ )
		{
			//we do something simple here with the "floor":
			if( particles_[i].x.y < world_.a.y && dot(particles_[i].v, n) < 0)
			{
				//particles_[i].v = reflect(n, particles_[i].v * resolution_ * particles_[i].r;
				GsVec f = particles_[i].f;
				particles_[i].f += f * -dot(f,n);
			}
		}

	}
}

inline GsVec reflect( const GsVec& n, const GsVec& v)
{
	GsVec nv, tv;
	nv = n*dot(v, n); //normal component
	tv = v - nv;
	return tv-nv;
}

void BBNet::eulerStep( float deltat )
{
	computeDerivatives();
	int i, stsize = derivatives_.size();

	for( i = 0;i < stsize; i++ ) derivatives_[i] *= deltat;
	getState();
	for( i = 0;i < stsize; i++ ) state_[i] += derivatives_[i];

	setState();

	time_ = deltat;

	if( collidev_ )
	{
		float dist, r2 = colradius_ * 2.0f;
		GsVec n, v;
		int i, j, psize = particles_.size();

		for( i = 0;i < psize; i++ )
		{
			for( j = 0;j < psize; j++ )
			{
				if( i == j ) continue;
				dist = ::dist(particles_[i].x, particles_[j].x);
				if(dist > 0.0001f && dist < r2)
				{
					n = particles_[i].x - particles_[j].x;
					n.normalize();
					v = particles_[i].v;
					v.normalize();
					if( dot(n, v) < 0 )
						particles_[i].v = reflect(n, particles_[i].v) * restitution_ * particles_[i].r;
				}
			}
		}

	}

	// to try: bounce against all planes!
	if( bounce_ )
	{
		GsVec n = GsVec::j; //bottom plane normal pointing up
		GsVec v, nv;
		for( i = 0;i < particles_.size(); i++ )
		{
			//we do something simple here with the "floor":
			if( particles_[i].x.y < world_.a.y && dot(particles_[i].v, n) < 0 )
				particles_[i].v = reflect( n, particles_[i].v ) * restitution_ * particles_[i].r; 
		}

	}
}
