//      Direct_Motion_Optimization_Holder.cpp
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

#include <iostream>
#include <iomanip>
#include "Direct_Motion_Optimization_Holder.h"


/**
	                   how informations are stored in IPOPT vector x
	|-------------------------------------------------------------------------------|
	|                   t1                  |                  t2                   |
	|-------------------------------------------------------------------------------|
	|       robot1      |       robot2      |       robot1      |       robot2      |   etc
	|-------------------------------------------------------------------------------|
	|  q, dq, ddq, tor  |  q, dq, ddq, tor  |  q, dq, ddq, tor  |  q, dq, ddq, tor  | (vectors)
	|-------------------------------------------------------------------------------|
	
	             how informations are stored in IPOPT vector g (constraintes)
	|-------------------------------------------------------------------------------|
	|                   t1                  |                  t2                   |
	|-------------------------------------------------------------------------------|
	|       robot1      |       robot2      |       robot1      |       robot2      |   etc
	|-------------------------------------------------------------------------------|
	|     q,     dq     |     q,     dq     |     q,     dq     |     q,     dq     | (vectors)
	|-------------------------------------------------------------------------------|
*/

Direct_Motion_Optimization_Holder::Direct_Motion_Optimization_Holder()
{
	env_ = new Environment ();
	// set default value to MultiContact motion generation
	Problem_type_ = "DirectMotionOptimization";
// 	Motion_properties_ = new Optim_Motion_Options ( );
// 	Motion_properties_->set_criteria_type(MotionCaptureFitting);
	reading_init ();
// 	min_MC_delta_ = -1;
	known_size_ = false;	// FIXME not very relevant with regard to MoGS_Project.cpp at line 1230
// 	pre_optim_ = false;
}

bool Direct_Motion_Optimization_Holder::known_size(int i) // this parameter i is useless
{
	return known_size_;
}

void Direct_Motion_Optimization_Holder::read_problem_specific (tinyxml2::XMLElement * root)
{
	double tmp = 0.;
  
	// retrieve solver name and its options
	Optim_properties_.read_problem(root);
	Optim_properties_.check_problem();
	
	// retrieve parameters informations
	tinyxml2::XMLElement * ElParameters = root->FirstChildElement ("parameters");
	tinyxml2::XMLElement * ElConstraint_on_q = ElParameters->FirstChildElement ("constraint_on_q");
	constraint_on_q_ = string_to_bool("constraint_on_q", char_to_string(ElConstraint_on_q->GetText()));
	tinyxml2::XMLElement * ElConstraint_on_dq = ElParameters->FirstChildElement ("constraint_on_dq");
	constraint_on_dq_ = string_to_bool("constraint_on_dq", char_to_string(ElConstraint_on_dq->GetText()));
	tinyxml2::XMLElement * ElCoeff_dq_max = ElParameters->FirstChildElement ("coeff_dq_max");
	coeff_dq_max_ = string_to_double(char_to_string(ElConstraint_on_q->GetText()));
	tinyxml2::XMLElement * ElOptim_m_duration = ElParameters->FirstChildElement ("optim_motion_duration");
	optim_motion_duration_ = string_to_bool("optim_motion_duration", char_to_string(ElOptim_m_duration->GetText()));
	tinyxml2::XMLElement * ElConstraint_on_torques = ElParameters->FirstChildElement ("constraint_on_torques");
	constraint_on_torques_ = string_to_bool("constraint_on_torques", char_to_string(ElConstraint_on_torques->GetText()));
	tinyxml2::XMLElement * ElCriteria = ElParameters->FirstChildElement ("criteria");
	criteria_ = char_to_string(ElCriteria->GetText());
	std::cout << "criteria_ = " << criteria_ << std::endl;
	tinyxml2::XMLElement * ElParallelization = ElParameters->FirstChildElement ("parallelization");
	parallelization_ = char_to_string(ElParallelization->GetText());
	tinyxml2::XMLElement * ElIntegration_step = ElParameters->FirstChildElement ("integration_step");
	integration_step_ = string_to_double(char_to_string(ElIntegration_step->GetText()));
	std::cout << "integration_step_ = " << integration_step_ << std::endl;
	
	// retrieve motion informations
	tinyxml2::XMLElement * ElMotion = root->FirstChildElement ("motion");
	tinyxml2::XMLElement * ElInit_posture = ElMotion->FirstChildElement ("init_posture");
	if (ElInit_posture)
	{
		
		std::istringstream iss (char_to_string (ElInit_posture->GetText ()), std::ios_base::in);
		while (iss) {
			iss >> tmp;
			init_posture_.push_back(tmp);
		}
		init_posture_.pop_back();
		
	}
	tinyxml2::XMLElement * ElFinal_posture = ElMotion->FirstChildElement ("final_posture");
	if (ElFinal_posture)
	{
		std::istringstream iss (char_to_string (ElFinal_posture->GetText ()), std::ios_base::in);
		while (iss) {
			iss >> tmp;
			final_posture_.push_back(tmp);
		}
		final_posture_.pop_back();
	}
	velocity_init_zero_ = false;
	tinyxml2::XMLElement * ElInit_Velocity = ElMotion->FirstChildElement ("init_velocity");
	if (ElInit_Velocity)
	{
		std::string str_tmp = char_to_string(ElInit_Velocity->GetText());
		if (str_tmp.compare("none") == 0)
		{} // do nothing
		else if (str_tmp.compare("zero") == 0)
			velocity_init_zero_ = true;
		else
		{
			std::istringstream iss (str_tmp, std::ios_base::in);
			while (iss) {
				iss >> tmp;
				init_velocity_.push_back(tmp);
			}
			init_velocity_.pop_back();
		}
	}
	velocity_final_zero_ = false;
	tinyxml2::XMLElement * ElFinal_Velocity = ElMotion->FirstChildElement ("final_velocity");
	if (ElFinal_Velocity)
	{
		std::string str_tmp = char_to_string(ElFinal_Velocity->GetText());
		if (str_tmp.compare("none") == 0)
		{} // do nothing
		else if (str_tmp.compare("zero") == 0)
			velocity_final_zero_ = true;
		else
		{
			std::istringstream iss (str_tmp, std::ios_base::in);
			while (iss) {
				iss >> tmp;
				final_velocity_.push_back(tmp);
			}
			final_velocity_.pop_back();
		}
	}
	tinyxml2::XMLElement * ElConstraint_cm = ElMotion->FirstChildElement ("cyclic_motion");
	cyclic_motion_ = string_to_bool("cyclic_motion", char_to_string(ElConstraint_cm->GetText()));
	tinyxml2::XMLElement * ElMotion_duration = ElMotion->FirstChildElement ("motion_duration");
	motion_duration_ = string_to_double(char_to_string(ElMotion_duration->GetText()));
}

bool Direct_Motion_Optimization_Holder::testXml (const std::string xml_name)
{
	std::string cmd = "xmllint --dtdvalid " + (std::string) DIRECTOPTIMIZATION_DTD_REPOSITORY + "/direct_optim.dtd --noout " + xml_name;
	int retCode = system (cmd.c_str ());
	if (retCode != 0)
	{
		std::cerr << "Error when execute " << cmd <<" (to check the XML file)" << std::endl;
		return false;
	}
	return true;
}

void Direct_Motion_Optimization_Holder::print_type ()
{
	std::cout << "type: DirectMotionOptimization" << std::endl;
}

void Direct_Motion_Optimization_Holder::initialize ()
{
	problem_init(); 
	
	env_->setXml(Environment_xml_);	
	env_->loadXml();
	
	std::cout << "init_posture_.size() : " << init_posture_.size() << std::endl;
	std::cout << "final_posture_.size() : " << final_posture_.size() << std::endl;
	if (velocity_init_zero_)
		std::cout << "velocity_init_zero_ : " << velocity_init_zero_ << std::endl;
	else 
	{
		std::cout << "init_velocity_ : ";
		for (int i=0; i<init_velocity_.size();++i)
			std::cout << init_velocity_[i] << " ";
		std::cout << std::endl;
	}
	if (velocity_final_zero_) 
		std::cout << "velocity_final_zero_ : " << velocity_final_zero_ << std::endl;
	else
	{
		std::cout << "final_velocity_ : ";
		for (int i=0; i<final_velocity_.size();++i)
			std::cout << final_velocity_[i] << " ";
		std::cout << std::endl;
	}
	
	total_nb_dofs_ = 0;
	for (int i = 0; i < nb_dofs_.size(); ++i) 
		total_nb_dofs_ += nb_dofs_[i];
	
	std::cout << "total_nb_dofs_ = " << total_nb_dofs_ << std::endl;
	
	if (total_nb_dofs_ != init_posture_.size() || total_nb_dofs_ != final_posture_.size()) 
	{
		printf("\033[%sm","31"); // print in red
		std::cout << "Error when loading initial or final posture in xml problem file : wrong arity" << std::endl;
		printf("\033[%sm","0");
		exit(-1);
	}
	
	if ((init_velocity_.size() > 0 && init_velocity_.size() != total_nb_dofs_) || (final_velocity_.size() > 0 && final_velocity_.size() != total_nb_dofs_))
	{
		printf("\033[%sm","31"); // print in red
		std::cout << "Error when loading initial or final velocity in xml problem file : wrong arity" << std::endl;
		printf("\033[%sm","0"); 
		exit(-1);
	  
	}
	
	nb_step_ = ceil(motion_duration_ / integration_step_) + 1;
	std::cout << "nb_step_ = " << nb_step_ << std::endl;
	
	// set nb_param_
	nb_param_ = nb_step_ * total_nb_dofs_ * 4; // 4 for q, dq, ddq, torque
	
	// set nb_ctr_
	nb_ctr_ = total_nb_dofs_ * (nb_step_-1) * 2; // 2 for q, dq 
	// maybe we will add ddq later
	
	std::cout << "nb_param_ = " << nb_param_<<std::endl;
	std::cout << "nb_ctr_ = " << nb_ctr_<<std::endl;
	
	// initialize nb_jac_non_null_
	nb_jac_non_null_ = 0;
	for (int s=0; s<nb_step_-1; ++s) for (int r=0; r<nb_robots_; ++r) for (int p=0;p<2;++p) for (int k=0;k<nb_dofs_[r];++k)
	{
		nb_jac_non_null_ += 1 + 4 * nb_dofs_[r];
	}
	
	std::cout << "nb_jac_non_null_ = " << nb_jac_non_null_ << std::endl;
	
	// print some informations
	std::cout << "cyclic_motion_ = " << cyclic_motion_ << std::endl;
	// FIXME DO cyclic motion by adding constraints and modify all IPOPT functions....
	
	
	// create Dyn_ and dyn_integrate for double
	Dyn_.resize(nb_robots_);
	// FIXME add in xml file init pos and init rot
	Eigen::Matrix< double, 3, 1> init_pos, init_rot;
	for (int r=0;r<nb_robots_;++r)
	{
		Dyn_[r].SetRobotXml(Robot_xml_[r]);
		init_pos[0] = init_posture_[pit(r,0)];
		init_pos[1] = init_posture_[pit(r,1)];
		init_pos[2] = init_posture_[pit(r,2)];
		init_rot[0] = init_posture_[pit(r,3)];
		init_rot[1] = init_posture_[pit(r,4)];
		init_rot[2] = init_posture_[pit(r,5)];
		if (!Dyn_[r].is_free_floating_base())
			Dyn_[r].set_root_transformation( init_pos, init_rot);
	}
	dyn_integrate_ = new Dynamics_Integrate<double>(nb_robots_, *env_, Dyn_);

	// create Dyn_ and dyn_integrate for F<double>
	FDyn_.resize(nb_robots_);
	Eigen::Matrix<F<double>, 3, 1> Finit_pos, Finit_rot;
	for (int r=0;r<nb_robots_;++r)
	{
		FDyn_[r].SetRobotXml(Robot_xml_[r]);
		Finit_pos[0] = init_posture_[pit(r,0)];
		Finit_pos[1] = init_posture_[pit(r,1)];
		Finit_pos[2] = init_posture_[pit(r,2)];
		Finit_rot[0] = init_posture_[pit(r,3)];
		Finit_rot[1] = init_posture_[pit(r,4)];
		Finit_rot[2] = init_posture_[pit(r,5)];
		if (!FDyn_[r].is_free_floating_base())
			FDyn_[r].set_root_transformation( init_pos, init_rot);
	}
	Fdyn_integrate_ = new Dynamics_Integrate<F<double>>(nb_robots_, *env_, FDyn_);
	
	// initialize vectors for integrate calculus
	q_.resize(nb_robots_);
	dq_.resize(nb_robots_);
	ddq_.resize(nb_robots_);
	torque_.resize(nb_robots_);
	for (int r=0;r<nb_robots_;++r) 
	{
		q_[r].resize(nb_dofs_[r]);                                   
		dq_[r].resize(nb_dofs_[r]);
		ddq_[r].resize(nb_dofs_[r]);
		torque_[r].resize(nb_dofs_[r]);
	}
	
	Fq_.resize(nb_robots_);
	Fdq_.resize(nb_robots_);
	Fddq_.resize(nb_robots_);
	Ftorque_.resize(nb_robots_);
	for (int r=0;r<nb_robots_;++r) 
	{
		Fq_[r].resize(nb_dofs_[r]);                                   
		Fdq_[r].resize(nb_dofs_[r]);
		Fddq_[r].resize(nb_dofs_[r]);
		Ftorque_[r].resize(nb_dofs_[r]);
	}
	
	// initialize iterators
	it_.resize(nb_step_);
	git_.resize(nb_step_);
	for (int s=0; s<nb_step_; ++s) {
		it_[s].resize(nb_robots_);
		git_[s].resize(nb_robots_);
		for ( int r=0;r<nb_robots_;++r) {
			it_[s][r].resize(4);
			git_[s][r].resize(2);
			it_[s][r][0].resize(nb_dofs_[r]);
			it_[s][r][1].resize(nb_dofs_[r]);
			it_[s][r][2].resize(nb_dofs_[r]);
			it_[s][r][3].resize(nb_dofs_[r]);
			git_[s][r][0].resize(nb_dofs_[r]);
			git_[s][r][1].resize(nb_dofs_[r]);
			for (int n=0; n<nb_dofs_[r]; ++n) {
				it_[s][r][0][n] = it(s,r,0,n);
				it_[s][r][1][n] = it(s,r,1,n);
				it_[s][r][2][n] = it(s,r,2,n);
				it_[s][r][3][n] = it(s,r,3,n);
				git_[s][r][0][n] = git(s,r,0,n);
				git_[s][r][1][n] = git(s,r,1,n);
			}
		}
	}
	pit_.resize(nb_robots_);
	for (int r=0; r<nb_robots_; ++r) {
		pit_[r].resize(nb_dofs_[r]);
		for ( int n=0;n<nb_dofs_[r];++n) {
			pit_[r][n] = pit(r,n);
		}
	}
	
	/** Check many parameters compatibility */
	
	// check compatibility : cyclic motion with init final posture an velocity
	if (cyclic_motion_)
	{
		for (int i=0; i<init_posture_.size();++i)
		{
			if (init_posture_[i] !=  final_posture_[i])
			{
				printf("\033[%sm","31"); // print in red
				std::cout << "With cyclic motion, initial posture must be equal with final posture" << std::endl;
				printf("\033[%sm","0"); // reinitialize color
				exit(-1);
			}
			if (init_velocity_.size() > 0 && init_velocity_[i] !=  final_velocity_[i])
			{
				printf("\033[%sm","31"); // print in red
				std::cout << "With cyclic motion, initial velocity must be equal with final velocity (or none or zero)" << std::endl;
				printf("\033[%sm","0"); // reinitialize color
				exit(-1);
			}
		}
	}
	
}

int Direct_Motion_Optimization_Holder::it(int time, int robot, int param, int nbdof)
{
	// iterator for IPOPT vectors x (x_l, x_u)
	// param : 0 => q; 1 => dq; 2 => ddq; 3 => torque
	int tot_rob = 0;
	for (int i = 0; i < robot; ++i)
		tot_rob += nb_dofs_[i] * 4;
	return time * (4 * total_nb_dofs_) + robot * (tot_rob) + param * (nb_dofs_[robot]) + nbdof;
}

int Direct_Motion_Optimization_Holder::git(int time, int robot, int param, int nbdof)
{
	// iterator for IPOPT vectors g
	// param : 0 => q; 1 => dq;
	int tot_rob = 0;
	for (int i = 0; i < robot; ++i)
		tot_rob += nb_dofs_[i] * 2;
	return time * (2 * total_nb_dofs_) + robot * (tot_rob) + param * (nb_dofs_[robot]) + nbdof;
}

int Direct_Motion_Optimization_Holder::pit(int robot, int nbdof)
{
	// iterator for init_posture_ and final_posture_ 
	int tot_rob = 0;
	for (int i=0; i < robot; ++i)
		tot_rob += nb_dofs_[i];
	return tot_rob + nbdof;
}

void Direct_Motion_Optimization_Holder::get_bounds_info (double *x_l, double *x_u, double *g_l, double *g_u)
{
	int r,s,n,i;
	
	// x.size() = nb_param_
	std::vector<double> p_l, p_u, v_u, t_u; 
	for (r=0;r<nb_robots_;++r) {                 
		Robots_[r]->getPositionLimit(p_l, p_u);
		Robots_[r]->getVelocityLimit(v_u);
		Robots_[r]->getTorqueLimit(t_u);
		/** check parameters with robot bounds */
		for (i=0; i<nb_dofs_[r];++i){
			if (p_l[i] > init_posture_[pit_[r][i]] || p_u[i] < init_posture_[pit_[r][i]]) {
				printf("\033[%sm","31"); // print in red
				std::cout << "Robot " << r << ", initial posture " << i << ", does not fit with robot's bound" << std::endl;
				printf("\033[%sm","0"); // reinitialize color
				exit(-1);
			}
			if (p_l[i] > final_posture_[pit_[r][i]] || p_u[i] < final_posture_[pit_[r][i]]) {
				printf("\033[%sm","31"); // print in red
				std::cout << "Robot " << r << ", final posture " << i << ", does not fit with robot's bound" << std::endl;
				printf("\033[%sm","0"); // reinitialize color
				exit(-1);
			}
			if (init_velocity_.size() > 0 && (-v_u[i] > init_velocity_[pit_[r][i]] || v_u[i] < init_velocity_[pit_[r][i]])) {
				printf("\033[%sm","31"); // print in red
				std::cout << "Robot " << r << ", initial velocity " << i << ", does not fit with robot's bound" << std::endl;
				printf("\033[%sm","0"); // reinitialize color
				exit(-1);
			}
			if (init_velocity_.size() > 0 && (-v_u[i] > init_velocity_[pit_[r][i]] || v_u[i] < init_velocity_[pit_[r][i]])) {
				printf("\033[%sm","31"); // print in red
				std::cout << "Robot " << r << ", final velocity " << i << ", does not fit with robot's bound" << std::endl;
				printf("\033[%sm","0"); // reinitialize color
				exit(-1);
			}
		}
		/** end check */
		for (s=0; s<nb_step_; ++s) for (n=0; n<nb_dofs_[r]; ++n) 
		{   
			x_l[it_[s][r][0][n]] = p_l[n];  // set bounds for q
			x_u[it_[s][r][0][n]] = p_u[n];    
			x_l[it_[s][r][1][n]] = -v_u[n]; // set bounds for dq       
			x_u[it_[s][r][1][n]] = v_u[n];
			x_l[it_[s][r][2][n]] = -1e20;   // set bounds for ddq
			x_u[it_[s][r][2][n]] = 1e20;
			
			if (Robots_[r]->is_robot_floating_base() && n < 6 ) 
			{
				x_l[it_[s][r][3][n]] = 0;       // set bounds for torque
				x_u[it_[s][r][3][n]] = 0;  
			}
			else 
			{
				x_l[it_[s][r][3][n]] = -t_u[n]; // set bounds for torque
				x_u[it_[s][r][3][n]] = t_u[n];
			}
		}
	}
	// fixe first and last step for q and dq
	for (r=0;r<nb_robots_;++r) for (n=0; n<nb_dofs_[r]; ++n) 
	{
		// q
		x_l[it_[0][r][0][n]] = init_posture_[pit_[r][n]];  // set bounds for q init
		x_u[it_[0][r][0][n]] = init_posture_[pit_[r][n]];    
		x_l[it_[nb_step_-1][r][0][n]] = final_posture_[pit_[r][n]];  // set bounds for q last
		x_u[it_[nb_step_-1][r][0][n]] = final_posture_[pit_[r][n]];    
		
		// dq
		if (init_velocity_.size() > 0) // velocity : specified
		{
			x_l[it_[0][r][1][n]] = init_velocity_[pit_[r][n]];  // set bounds for dq init
			x_u[it_[0][r][1][n]] = init_velocity_[pit_[r][n]];    
			x_l[it_[nb_step_-1][r][1][n]] = final_velocity_[pit_[r][n]];  // set bounds for dq last
			x_u[it_[nb_step_-1][r][1][n]] = final_velocity_[pit_[r][n]];    
		}
		else // velocity : zero
		{
			if (velocity_init_zero_) {
				x_l[it_[0][r][1][n]] = 0;  // set bounds for dq init
				x_u[it_[0][r][1][n]] = 0;    
			}
			if (velocity_final_zero_) {
				x_l[it_[nb_step_-1][r][1][n]] = 0;  // set bounds for dq last
				x_u[it_[nb_step_-1][r][1][n]] = 0;    
			}
		} // velocity none
	}
	
	// g.size() = nb_ctr_
	for (i=0;i<nb_ctr_;++i) 
	{
		g_l[i] = 0.;
		g_u[i] = 0.;
	}
}

void Direct_Motion_Optimization_Holder::get_starting_point (double *x)
{
	int r, s, n;
	for (s=0; s<nb_step_; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) {
		x[it_[s][r][0][n]] = init_posture_[pit_[r][n]] + (1.*s)*(final_posture_[pit_[r][n]] - init_posture_[pit_[r][n]])/(nb_step_-1); // set initial value for position with linear interpolation
		x[it_[s][r][1][n]] = 0.; // set initial value for velocity at 0
		x[it_[s][r][2][n]] = 0.; // set initial value for accel at 0
		x[it_[s][r][3][n]] = 0.; // set initial value for torque at 0
	}
}

double Direct_Motion_Optimization_Holder::eval_f (bool new_x, const double *x)
{
	if (criteria_.compare("none") == 0) {
		return 0.;
	}
	else if (criteria_.compare("energy") == 0) {
		int s, r, n;
		double res = 0., buf;
		for (s=0; s<nb_step_; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) {    
			buf = x[it_[s][r][3][n]];
			res += buf * buf;
		}
		return res;
	}
	else if (criteria_.compare("jerk") == 0) {
		std::cout << "Jerk is not implemented yet" << std::endl;
		exit(EXIT_FAILURE);
	}
	else if (criteria_.compare("fitting") == 0) {
		std::cout << "Fitting is not implemented yet" << std::endl;
		exit(EXIT_FAILURE);
	}
	return 0.;
}

void Direct_Motion_Optimization_Holder::eval_grad_f (bool new_x, const double *x, double *grad_f)
{
	for (int i=0; i<nb_param_; ++i)
		grad_f[i] = 0.;	
	
	if (criteria_.compare("none") == 0) {
		// do nothing
	}
	else if (criteria_.compare("energy") == 0) {
		int s, r, n;
		for (s=0; s<nb_step_; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n)
			grad_f[it_[s][r][3][n]] = 2 * x[it_[s][r][3][n]];
	}
	else if (criteria_.compare("jerk") == 0) {
		std::cout << "Jerk is not implemented yet" << std::endl;
		exit(EXIT_FAILURE);
	}
	else if (criteria_.compare("fitting") == 0) {
		std::cout << "Fitting is not implemented yet" << std::endl;
		exit(EXIT_FAILURE);
	}
}

void Direct_Motion_Optimization_Holder::eval_g (bool new_x, const double *x, double *g)
{
	//pour tous les intervalles de temps Ti
	//recupere q,dq, (ddq, torque) => q_debut_i
	// calcule (q_eps_i,dq_eps_i) = integration<double>( q_debut_i, dq_debut_i, ..., eps)
	// la contrainte : q_eps_i - q_debut_i+1 = 0
	// la contrainte : dq_eps_i - dq_debut_i+1 = 0
	
	int s, r, n;
	int cpt_g = 0;
	for (s=0; s<nb_step_ - 1; ++s) {          // Caution -1     // for all steps
		for (r=0; r<nb_robots_; ++r) {                      // for all robots
			for (n=0; n<nb_dofs_[r]; ++n) {             // filling vectors
				q_[r][n] = x[it_[s][r][0][n]];
				dq_[r][n] = x[it_[s][r][1][n]];
				ddq_[r][n] = x[it_[s][r][2][n]];
				torque_[r][n] = x[it_[s][r][3][n]];
			}
		}
		
		dyn_integrate_->integrate(q_, dq_, ddq_, torque_, integration_step_); 
// 		integrate_bidon<double>(q_, dq_, ddq_, torque_, integration_step_);
		for (r=0; r<nb_robots_; ++r) {
			for (n=0; n<nb_dofs_[r]; ++n) {             // set constraints
				g[cpt_g++] = q_[r][n] - x[it_[s+1][r][0][n]]; 
				g[cpt_g++] = dq_[r][n] - x[it_[s+1][r][1][n]];
			}
		}
	}
}

void Direct_Motion_Optimization_Holder::get_dependencies (int *iRow, int *jCol)
{
	// In iRow, set ctr_ index, in jCol set param_ index
	int s, r, p, n, k, cpt = 0;
	int cpt_g = 0;
	for (s=0; s<nb_step_-1; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) for (p=0;p<2;++p)
	{
		iRow[cpt] = cpt_g; 
		jCol[cpt] = it_[s+1][r][p][n];
		cpt++;
		for (int m=0; m<nb_dofs_[r]; ++m) 
			for(k=0;k<4;++k) 
			{
				iRow[cpt] = cpt_g; 
				jCol[cpt] = it_[s][r][k][m];
				cpt++;
			}
		cpt_g++;
	}
}

void Direct_Motion_Optimization_Holder::eval_grad_g (bool new_x, const double *x, double *values)
{
	//pour tous tes intervalles de temps Ti
	//recupere q,dq, (ddq, torque) => q_debut_i
	// calcule (q_eps_i,dq_eps_i) = integration<F<double> >( q_debut_i, dq_debut_i, ..., eps)
	// la contrainte : q_eps_i - q_debut_i+1 = 0
	// la contrainte : dq_eps_i - dq_debut_i+1 = 0      
	
	int s, r, p, n, i;
	int cpt_diff, cpt = 0;
	int cpt_diff_max;
	F<double> contrainte_q, contrainte_dq;
	for (s=0; s<nb_step_ - 1; ++s) 
	{        
		cpt_diff = 0;
		for (r=0; r<nb_robots_; ++r) {   
			for (n=0; n<nb_dofs_[r]; ++n) {
				q_[r][n] = x[it_[s][r][0][n]];
				Fq_[r][n] = x[it_[s][r][0][n]];
				Fq_[r][n].diff(cpt_diff++, total_nb_dofs_ * 4);
				Fdq_[r][n] = x[it_[s][r][1][n]];
				Fdq_[r][n].diff(cpt_diff++, total_nb_dofs_ * 4);
				Fddq_[r][n] = x[it_[s][r][2][n]];
				Fddq_[r][n].diff(cpt_diff++, total_nb_dofs_ * 4);
				Ftorque_[r][n] = x[it_[s][r][3][n]];
				Ftorque_[r][n].diff(cpt_diff++, total_nb_dofs_ * 4);
			}
		}
		cpt_diff_max = cpt_diff;
		Fdyn_integrate_->integrate(Fq_, Fdq_, Fddq_, Ftorque_, integration_step_); 
// 		integrate_bidon<F<double> >(Fq_, Fdq_, Fddq_, Ftorque_, integration_step_);
		for (r=0; r<nb_robots_; ++r) {
			for (n=0; n<nb_dofs_[r]; ++n)
			{
				// constraint on q
				values[cpt++] = -1;
				contrainte_q = Fq_[r][n];
				for (i=0; i<cpt_diff_max; ++i)
					values[cpt++] = contrainte_q.d(i);
				// constraint on dq
				values[cpt++] = -1;
				contrainte_dq = Fdq_[r][n];
				for (i=0; i<cpt_diff_max; ++i)
					values[cpt++] = contrainte_dq.d(i);
			}
		}
	}
}

void Direct_Motion_Optimization_Holder::this_is_final_results (const double *x, int status)
{
	std::cout << " optim finished" << std::endl;

	update_time_result_q_result(x);
	
	tinyxml2::XMLElement * results = root_->FirstChildElement ("results");
	assert (results);
	
	tinyxml2::XMLNode * result = doc_.NewElement ("result");	
	
	tinyxml2::XMLNode * Elname = doc_.NewElement ("result_name");
	tinyxml2::XMLText * name= doc_.NewText ( soluce_name_.c_str() );
	Elname->InsertEndChild (name);
	result->InsertEndChild (Elname);		
	
	tinyxml2::XMLNode * Eldate = doc_.NewElement ("date");
	time_t rawtime;
	time (&rawtime);
	std::string sdate = ctime (&rawtime);
	tinyxml2::XMLText * date = doc_.NewText ( sdate.c_str() );
	Eldate->InsertEndChild (date);
	result->InsertEndChild (Eldate);
	
	std::cout.precision(10);
	std::ostringstream X,oss;
// 	for (int i=0;i<nb_param_;i++)
// 		X << std::setprecision(20)<< x[i]<<" ";
// 	std::string s = X.str();
	
	tinyxml2::XMLNode * Elnbparam = doc_.NewElement ("nbparam");	
	oss << nb_param_;
	std::string s1 = oss.str();
	tinyxml2::XMLText * nbparam = doc_.NewText ( s1.c_str());
	Elnbparam->InsertEndChild (nbparam);
	result->InsertEndChild (Elnbparam);	
	
// 	tinyxml2::XMLNode * Elparam = doc_.NewElement ("param");	
// 	tinyxml2::XMLText * param = doc_.NewText ( s.c_str() );
// 	Elparam->InsertEndChild (param);
// 	result->InsertEndChild (Elparam);
	
	for(int s=0;s<nb_step_;s++)
	{
		
		tinyxml2::XMLElement * Elvalue_q = doc_.NewElement ("q");
		Elvalue_q->SetAttribute("time",integration_step_ * s);
		
		for (int k=0; k < nb_robots_; k++)
		{
			std::ostringstream oss_q;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_q << q_result_[s](j)<< " ";
				}

			tinyxml2::XMLElement *xml_q = doc_.NewElement ("Qrobot");
			xml_q->SetAttribute("name",(Robots_[k]->getRobotName()).c_str());
			tinyxml2::XMLText *xml_text = doc_.NewText ( oss_q.str().c_str());
			xml_q->InsertEndChild (xml_text);
			Elvalue_q->InsertEndChild (xml_q);
		}  
		
		result->InsertEndChild (Elvalue_q);  
	}

	for(int s=0;s<nb_step_;s++)
	{		
		tinyxml2::XMLElement * Elvalue_dq = doc_.NewElement ("dq");
		Elvalue_dq->SetAttribute("time",integration_step_ * s);

		for (int k=0; k < nb_robots_; k++)
		{
			std::ostringstream oss_dq;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_dq << x[it_[s][k][1][j]] << " ";
				}

			tinyxml2::XMLElement *xml_dq = doc_.NewElement ("Qrobot");
			xml_dq->SetAttribute("name",(Robots_[k]->getRobotName()).c_str());
			tinyxml2::XMLText *xml_text_dq = doc_.NewText ( oss_dq.str().c_str());
			xml_dq->InsertEndChild (xml_text_dq);
			Elvalue_dq->InsertEndChild (xml_dq);
		}
		
		result->InsertEndChild (Elvalue_dq);  
	}
	
	for(int s=0;s<nb_step_;s++)
	{
		tinyxml2::XMLElement * Elvalue_ddq = doc_.NewElement ("ddq");
		Elvalue_ddq->SetAttribute("time",integration_step_ * s);

		for (int k=0; k < nb_robots_; k++)
		{
			std::ostringstream oss_ddq;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_ddq << x[it_[s][k][2][j]] << " ";
				}

			tinyxml2::XMLElement *xml_ddq = doc_.NewElement ("Qrobot");
			xml_ddq->SetAttribute("name",(Robots_[k]->getRobotName()).c_str());
			tinyxml2::XMLText *xml_text_ddq = doc_.NewText ( oss_ddq.str().c_str());
			xml_ddq->InsertEndChild (xml_text_ddq);
			Elvalue_ddq->InsertEndChild (xml_ddq);
		}    
		
		result->InsertEndChild (Elvalue_ddq); 
	}
	
	for(int s=0;s<nb_step_;s++)
	{		
		tinyxml2::XMLElement * Elvalue_tau = doc_.NewElement ("tau");
		Elvalue_tau->SetAttribute("time",integration_step_ * s);

		for (int k=0; k < nb_robots_; k++)
		{
			std::ostringstream oss_tau;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_tau <<  x[it_[s][k][3][j]] << " ";
				}
			
			tinyxml2::XMLElement *xml_tau = doc_.NewElement ("Qrobot");
			xml_tau->SetAttribute("name",(Robots_[k]->getRobotName()).c_str());
			tinyxml2::XMLText *xml_text_tau = doc_.NewText ( oss_tau.str().c_str());
			xml_tau->InsertEndChild (xml_text_tau);
			Elvalue_tau->InsertEndChild (xml_tau);
		}    		
		
		result->InsertEndChild (Elvalue_tau);  
	}
	
	results->InsertEndChild (result);
	doc_.SaveFile (xml_problem_filename_.c_str() );
	std::cout << " status = "<< status << std::endl;  
}

void Direct_Motion_Optimization_Holder::get_q_dq(int num_robot, std::vector<double> param, double time,
		Eigen::Matrix< double, Eigen::Dynamic, 1> &Q, bool get_dq,
		Eigen::Matrix< double, Eigen::Dynamic, 1> &DQ)
{
	if (num_robot != 0)
	{
		std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__<<std::endl;
		std::cerr<<"For the moment, we can consider only one robot"<<std::endl;
		exit(-1);
	}
	Q.resize(nb_dofs_[num_robot]);
	DQ.resize(nb_dofs_[num_robot]);
	
	int nb = time_result_.size();
	int cpt = 0 ;
	for (int i = 0;i<nb;i++)
	{
		if (time < time_result_[i])
			break;
		cpt = i;
	}
	
	if (cpt <nb-1)
		for(int i=0;i<nb_dofs_[0];i++)
			Q[i] = ( q_result_[cpt+1](i) - q_result_[cpt](i)) / (time_result_[cpt+1] - time_result_[cpt]) * (time - time_result_[cpt]) +  q_result_[cpt](i);
	else
		for(int i=0;i<nb_dofs_[0];i++)
			Q[i] = q_result_[cpt](i);
	
	if (get_dq)
	{
		if (cpt <nb-1)
			for(int i=0;i<nb_dofs_[0];i++)
				DQ[i] = ( q_result_[cpt+1](i) - q_result_[cpt](i)) / (time_result_[cpt+1] - time_result_[cpt]);
		else
			for(int i=0;i<nb_dofs_[0];i++)
				DQ[i] = 0;
	}
	std::cout<<"Q = "<< Q <<std::endl;
}

bool Direct_Motion_Optimization_Holder::do_pre_optimization()
{
	return false; 
}

void Direct_Motion_Optimization_Holder::pre_optimize()
{
	// do nothing. By the way this function won't be called
}

// argument param is useless
double Direct_Motion_Optimization_Holder::get_duration(std::vector<double> param)
{
	std::cout << "Problem_Simulation get_duration" << std::endl;
	return (time_result_[ time_result_.size() -1]);
}

bool Direct_Motion_Optimization_Holder::do_you_need_Motion_Capture()
{
	// do nothing
	return false;
}

void Direct_Motion_Optimization_Holder::get_result_param(const std::string result_name, std::vector<double> &param)
{
	assert (root_);	
	tinyxml2::XMLElement * results = root_->FirstChildElement ("results");
	assert (results);	
	tinyxml2::XMLElement * result = results->FirstChildElement ("result");
	for (result; result; result = result->NextSiblingElement ("result"))
	{
		tinyxml2::XMLElement * Elname = result->FirstChildElement ("result_name");
		if ( result_name ==  char_to_string(Elname->GetText()))
		{
			tinyxml2::XMLElement * Elnb = result->FirstChildElement ("nbparam");
			int nb = string_to_int(Elnb->GetText());
			tinyxml2::XMLElement * Elparam = result->FirstChildElement ("param");
                        
                        std::cerr<< __FILE__<< __LINE__<<" Call of get_result_param "<<std::endl; // FIXIT
			std::string sparam = char_to_string( Elparam->GetText()); // FIXIT
			std::istringstream Data (sparam, std::ios_base::in);
			double tval;
			for (int i = 0; i < nb; i++)
			{
				Data >> tval;
				param.push_back(tval);
			}
			assert( param.size() == nb );
			return;
		}
	}	
	std::cerr<< __FILE__<< __LINE__<<" Error when try to read the param"<<std::endl;
	exit(-1);
}

void Direct_Motion_Optimization_Holder::update_time_result_q_result(const double *x)
{
	// for the moment we store only for one robot
	int s,r,n,i,buf;
	double t = 0;
	q_result_.resize(nb_step_);
	time_result_.resize(nb_step_);
	for (s=0; s<nb_step_; ++s) {
		q_result_[s].resize(total_nb_dofs_);
		int cpt = 0;
// 		for (r=0; r<nb_robots_; ++r) {
		for (r=0; r<1; ++r) {
			for (n=0; n<nb_dofs_[r]; ++n) 
				q_result_[s](cpt++) = x[it_[s][r][0][n]]; 
		}
		time_result_[s] = t;
		t += integration_step_;
	}
}

extern "C" Direct_Motion_Optimization_Holder* create() 
{
    return new Direct_Motion_Optimization_Holder;
}

extern "C" void destroy(Direct_Motion_Optimization_Holder* p) 
{
    delete p;
}
