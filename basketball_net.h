# pragma once

#include <sig/gs_array.h>
#include <sig/gs_box.h>
#include <sig/gs_quat.h>
#include <sig/gs_vars.h>
#include <math.h>

#include "particle.h"

class BBNet
{
	private:
		GsArray<Particle> particles_;
		GsArray<float> state_;
		GsArray<float> derivatives_;
		const int LEVEL = 10;
		float time_;
		float rim_radius_;
		int net_res_;
		GsVec extforce_;
		GsVec gravity_;
		GsBox world_;
		GsPnt initpos_;
		bool bounce_;
		bool collides_, collidev_;
		float colradius_;
		float colks_;
		float colkd_;
		float restitution_;
		float maxmass_;
		GsCharPt lcfgfile_;

	public:
		BBNet ();
		~BBNet ();

		const char* get_cfg_file () const { return lcfgfile_; }

		void init( const char* cfgfile );
		void init(const GsVars* vars );

		void bounce( bool b ) { bounce_ = b; }
		int getNetLevel() { return LEVEL; }
		int getNetRes() { return net_res_; }
		void springCollisionPrevention( bool sc ) { collides_ = sc; }
		void velocityCollisionPrevention( bool vc ) { collidev_ = vc; }
		void coeffRestitution( float r ) { restitution_ = r; }
		float coeffRestitution() { return restitution_; }

		void externalForce( const GsVec& f ) { extforce_ = f; }
		void collisionRadius( float r ) { colradius_ = r; }

		const GsArray<Particle>& particles() const { return particles_; }
		const GsBox& world() const { return world_; }

		void getState();
		void setState();
		void clearForces();
		void computeForces();
		void computeDerivatives();

		void eulerStep( float delta_t );
};
