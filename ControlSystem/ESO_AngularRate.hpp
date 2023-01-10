#pragma once

//角速度带模型ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "AC_Math.hpp"
#include "Filters_LP.hpp"
#include "Filters_BS.hpp"

//观测量延时

class ESO_AngularRate_Base
{
	public:
		virtual double get_T() const = 0;
		virtual double get_b() const = 0;
		virtual double get_u() const = 0;
	
		virtual void update_u( double u ) = 0;
		virtual double run( double v ) = 0;
	
		virtual double get_EsAngularRate() const = 0;
		virtual double get_EsDisturbance() const = 0;
		virtual double get_EsAngularAcceleration() const = 0;
		virtual double get_EsMainPower() const = 0;
};

class ESO_AngularRate : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;
		double z2;
		
		double last_err;

		double Hz;
		double h;
		
		Filter_Butter2_LP rate_filter;
		Filter_Butter2_LP acc_filter;
		Filter_Butter2_BS acc_BSfilter1;
		Filter_Butter2_BS acc_BSfilter2;
	
	public:	
		double beta1;
		double beta2;
	
		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
	
		inline void init( double T , double b , double beta1 , double beta2 , double Hz )
		{
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			this->beta1 = beta1;
			this->beta2 = beta2;
			rate_filter.set_cutoff_frequency( Hz, beta1 );
			acc_filter.set_cutoff_frequency( Hz, beta2 );
			acc_BSfilter1.set_cutoff_frequency( Hz, 55, 55.0 );
			acc_BSfilter2.set_cutoff_frequency( Hz, 121, 50.0 );
			
			this->z1 = this->z2 = this->z_inertia = 0;
			
			this->T = T;	this->invT = 1.0f / T;
			this->b = b;
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
			this->z1 += this->h * ( this->z_inertia + this->z2 );
		}
		
		double run( double v );
		
		inline double get_EsAngularRate() const
		{
			return this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z2;
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->z2 + this->z_inertia;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};

class ESO_AngularRateHeli : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;

		double last_v;
		double acc;
	
		double Hz;
		double h;
		
		Filter_Butter2_LP rate_filter;
	
	public:	
		double beta1;
	
		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
	
		inline void init( double T , double b , double beta1 , double Hz )
		{
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			this->beta1 = beta1;
			rate_filter.set_cutoff_frequency( Hz, beta1 );
			
			this->z1 = this->z_inertia = 0;
			
			this->T = T;	this->invT = 1.0f / T;
			this->b = b;
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
		}
		
		double run( double v );
		
		inline double get_EsAngularRate() const
		{
			return this->z_inertia + this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z1;
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->acc;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};