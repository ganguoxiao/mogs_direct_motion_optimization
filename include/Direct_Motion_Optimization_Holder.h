//      Direct_Motion_Optimization_Holder.h
//      Copyright (C) 2014 lengagne (lengagne@gmail.com)
//      Copyright (C) 2014 piot
// 
//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.
// 
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
// 
//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//      This program was developped in the following labs:
// 	From 2014 : Université Blaise Pascal, ISPR, MACCS, Clermont-Ferrand, France

#ifndef	__DIRECT_MOTION_OPTIMIZATION_HOLDER__
#define __DIRECT_MOTION_OPTIMIZATION_HOLDER__

#include "config_DIRECTOPTIMIZATION.h"
#include "Optimization_Holder.h"
#include "Optimization_Options.h"
#include "Dynamics_Integrate.h"

#include "fadiff.h"

class Direct_Motion_Optimization_Holder:public Optimization_Holder
{
      public:
	// Constructor
	Direct_Motion_Optimization_Holder();
	// Destructor
	~Direct_Motion_Optimization_Holder()
	{}
	
	// Virtual fonctions to implement
	/**	Determine if the size of the robot is constant or is a result of the problem	*/
	bool known_size(int i);	
	
	void read_problem_specific (tinyxml2::XMLElement * root);

	/**	test the xml and return false in case of problem (regarding the DTD) */
	bool testXml (const std::string xml_name);

	void print_type ();

	/**	Initialize the optimization */
	void initialize ();

	/**	Get the bounds on the limits and on the constraints	*/
	void get_bounds_info (double *x_l, double *x_u, double *g_l, double *g_u);

	/**	Get the initial point	*/
	void get_starting_point (double *x);

	/**	Compute the objective value	*/
	double eval_f (bool new_x, const double *x);

	/**	Compute the derivative of the objective value	*/
	void eval_grad_f (bool new_x, const double *x, double *grad_f);

	/**	Compute the constraints*/
	void eval_g (bool new_x, const double *x, double *g);

	/**	Get the dependencies of the jacobian	*/
	void get_dependencies (int *iRow, int *jCol);

	/**	Return the value of the jacobian (regarding the defined structure	*/
	void eval_grad_g (bool new_x, const double *x,
				  double *values) ;

	/**	Returns the final results of the optimization	*/
	void this_is_final_results (const double *x, int status);
	
	
	void get_q_dq(int num_robot,
			std::vector<double> param,
			double time,
			Eigen::Matrix< double, Eigen::Dynamic, 1> &Q,
			bool get_dq,
			Eigen::Matrix< double, Eigen::Dynamic, 1> &DQ);
	
	/**	return if we need to do a pre-optim*/
	bool do_pre_optimization();
	
	/**	Perform the pre optimization*/
	void pre_optimize();
	
	double get_duration(std::vector<double> param);
	
#ifdef MoGS_Motion_Capture_FOUND
	bool do_you_need_Motion_Capture();
#endif
	
#ifdef MoGS_Contact_FOUND
	bool do_you_need_contact();
#endif

	// non virtual functions          
	void get_result_param(const std::string result_name, std::vector<double> &param); 
	void update_time_result_q_result(const double *x);
	int it(int time, int robot, int param, int nbdof);
	int git(int time, int robot, int param, int nbdof);
	int pit(int robot, int nbdof);
	
	protected:
	
	// parameters
	bool constraint_on_q_;
	bool constraint_on_dq_;
	double coeff_dq_max_;
	bool optim_motion_duration_;
	bool constraint_on_torques_;
	std::string criteria_;
	std::string parallelization_;
	double integration_step_;
	// motion
	std::vector<double> init_posture_;
	std::vector<double> final_posture_;
	double motion_duration_;

	// iterators
	std::vector<std::vector<std::vector<std::vector<int> > > > it_, git_;
	
	// others
	double nb_step_;
	int total_nb_dofs_;
	
	// dynamics
	std::vector<RigidBodyDynamics::Dynamics<double> > Dyn_;
	Dynamics_Integrate<double> *dyn_integrate_;
	std::vector<Eigen::Matrix <double,Eigen::Dynamic,1> > q_, dq_, ddq_, torque_;
	
// 	std::vector<RigidBodyDynamics::Dynamics<F<double> > > FDyn_;
// 	Dynamics_Integrate<F<double> > *Fdyn_integrate_;
	std::vector<Eigen::Matrix <F<double>,Eigen::Dynamic,1> > Fq_, Fdq_, Fddq_, Ftorque_;
	
	// stock results
	std::vector<double> time_result_;
	std::vector < Eigen::Matrix <  double, Eigen::Dynamic, 1 >, Eigen::aligned_allocator < Eigen::Matrix < double,  Eigen::Dynamic, 1 > > > q_result_;
	  
	/* inherited variables
	
	protected: 
	  
	///////////////////////////  From Optimization_Holder
	  
	int nb_param_;                                              // number of parameters
	int nb_ctr_;                                                // number of constraints
	int nb_jac_non_null_;                                       // number of non null values in the jacobian

	bool known_size_ ; 
	Optimization_Options Optim_properties_;

	///////////////////////////  From Problem_Holder
	
	tinyxml2::XMLElement * root_;
	tinyxml2::XMLDocument doc_;
	std::string xml_problem_filename_;
	std::string soluce_name_;
	Environment* env_;
	int nb_robots_;                                             // number of robots
	std::vector<std::string> Robot_name_;                       // name of the robot
	std::vector<std::string> Robot_xml_;                        // xml of the robot
	std::string Environment_name_;                              // xml of the robot 
	std::string Environment_xml_;                               // xml of the robot
	std::string Problem_type_;
	std::vector<RigidBodyDynamics::Robot<double>*>Robots_;      // list of the used robots
	std::vector<int> nb_dofs_;                                  // number of dof of the robots
	std::string mem_result_name_;
	bool save_result_;
	
	*/

};

template <typename TT>
void integrate_bidon(std::vector< Eigen::Matrix< TT, Eigen::Dynamic, 1> > &q_,
			std::vector< Eigen::Matrix< TT, Eigen::Dynamic, 1> > &dq_,
			std::vector< Eigen::Matrix< TT, Eigen::Dynamic, 1> > &ddq_,
			std::vector< Eigen::Matrix< TT, Eigen::Dynamic, 1> > &torque_,
			double eps)
{
	int r, n;
	for (r=0;r<q_.size();++r) {
		for (n=0;n<q_[r].size();++n) { // parce que neufneuf !
			q_[r](n) += dq_[r](n) * eps;
			dq_[r](n) += ddq_[r](n) * eps;
		}
	}
}

#endif