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
#include <fstream>
#include <iomanip>
#include <ctime>
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
  
	/** retrieve solver name and its options */
	Optim_properties_.read_problem(root);
	Optim_properties_.check_problem();
	
	/** retrieve parameters informations */
	tinyxml2::XMLElement * ElParameters = root->FirstChildElement ("parameters");
	// get constraint on q
	tinyxml2::XMLElement * ElConstraint_on_q = ElParameters->FirstChildElement ("constraint_on_q");
	constraint_on_q_ = string_to_bool("constraint_on_q", char_to_string(ElConstraint_on_q->GetText()));
	// get constraint on dq
	tinyxml2::XMLElement * ElConstraint_on_dq = ElParameters->FirstChildElement ("constraint_on_dq");
	constraint_on_dq_ = string_to_bool("constraint_on_dq", char_to_string(ElConstraint_on_dq->GetText()));
	// get coeff dq max
	tinyxml2::XMLElement * ElCoeff_dq_max = ElParameters->FirstChildElement ("coeff_dq_max");
	coeff_dq_max_ = string_to_double(char_to_string(ElConstraint_on_q->GetText()));
	// get optim motion duration
	tinyxml2::XMLElement * ElOptim_m_duration = ElParameters->FirstChildElement ("optim_motion_duration");
	optim_motion_duration_ = string_to_bool("optim_motion_duration", char_to_string(ElOptim_m_duration->GetText()));
	// get constraint on torques
	tinyxml2::XMLElement * ElConstraint_on_torques = ElParameters->FirstChildElement ("constraint_on_torques");
	constraint_on_torques_ = string_to_bool("constraint_on_torques", char_to_string(ElConstraint_on_torques->GetText()));
	// get criteria
	tinyxml2::XMLElement * ElCriteria = ElParameters->FirstChildElement ("criteria");
	std::string crit = char_to_string(ElCriteria->GetText());
	if (crit.compare("none") == 0)
		criteria_ = NoCriteria;
	if (crit.compare("fitting") == 0)
		criteria_ = MotionCaptureFitting;
	if (crit.compare("energy") == 0)
		criteria_ = Energy;
	if (crit.compare("torquesquare") == 0)
		criteria_ = TorqueSquare;
	if (crit.compare("jerk") == 0)
		criteria_ = Jerk;
	std::cout << "criteria_ = " << crit << std::endl;
	// get parallelization
	tinyxml2::XMLElement * ElParallelization = ElParameters->FirstChildElement ("parallelization");
	parallelization_ = char_to_string(ElParallelization->GetText());
	// get integration step
	tinyxml2::XMLElement * ElIntegration_step = ElParameters->FirstChildElement ("integration_step");
	integration_step_ = string_to_double(char_to_string(ElIntegration_step->GetText()));
	std::cout << "integration_step_ = " << integration_step_ << std::endl;
	
	/** retrieve motion informations */
	tinyxml2::XMLElement * ElMotion = root->FirstChildElement ("motion");
	// get initial posture
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
	// get final posture
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
	// get constraint with time
	tinyxml2::XMLElement *ElConstraint;
	std::string buf_str;
	for (ElConstraint = ElMotion->FirstChildElement ("constraint"); ElConstraint; ElConstraint = ElConstraint->NextSiblingElement("constraint"))
	{
		// get constraint time
		buf_str = ElConstraint->Attribute("time");
		constraint_time_.push_back(string_to_double(buf_str));
		// get constraint robot name
		buf_str = ElConstraint->Attribute("robot_name");
		constraint_name_.push_back(buf_str);
		// get constraint robot body
		buf_str = ElConstraint->Attribute("robot_body");
		constraint_body_.push_back(buf_str);
		// get constraint type
		buf_str = ElConstraint->Attribute("type");
		constraint_type_.push_back(buf_str);
		// get constraint option
		tinyxml2::XMLElement *ElConstraint_option = ElConstraint->FirstChildElement ("option");
		std::istringstream iss (char_to_string(ElConstraint_option->GetText()),std::ios_base::in);
		std::vector<double> v_tmp_o;
		while (iss) {
			iss >> tmp;
			v_tmp_o.push_back(tmp);
		}
		v_tmp_o.pop_back();
		constraint_option_.push_back(v_tmp_o);
		// get constraint value
		tinyxml2::XMLElement *ElConstraint_value = ElConstraint->FirstChildElement ("value");
		std::istringstream iss2 (char_to_string(ElConstraint_value->GetText()),std::ios_base::in);
		std::vector<double> v_tmp_v;
		while (iss2) {
			iss2 >> tmp;
			v_tmp_v.push_back(tmp);
		}
		v_tmp_v.pop_back();
		constraint_value_.push_back(v_tmp_v);
	}
	// get initial velocity
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
	// get final velocity
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
	//get cyclic motion
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

	for (int i=0; i<constraint_name_.size(); ++i)
	{
		if (((int)((constraint_time_[i]/integration_step_)*10))%10 != 0)
		{
			constraint_time_[i] = ceil(constraint_time_[i]/integration_step_);
			printf("\033[%sm","34"); // print in blue
			std::cout << "Time's constraint n° " << i << " has been modified to fit the motion. New value  : " << constraint_time_[i] <<  std::endl;
			printf("\033[%sm","0"); 
		}
		else 
			constraint_time_[i] = constraint_time_[i]/integration_step_;
		bool name_error = true;
		for (int r=0; r<nb_robots_; ++r)
		{
			if (Robots_[r]->getRobotName().compare(constraint_name_[i]) == 0)
			{
				constraint_robot_number_.push_back(r);
				name_error = false;
				bool body_error = true;
				int body_id = Robots_[r]->GetBodyId(constraint_body_[i]); 
				if (body_id != std::numeric_limits <unsigned int >::max()) {
					constraint_body_number_.push_back(body_id);
					body_error = false;
				}
				if (body_error)
				{
					printf("\033[%sm","31"); // print in red
					std::cout << "The body \"" << constraint_body_[i] << "\" of robot \"" << constraint_name_[i] << "\" does not exist" << std::endl;
					printf("\033[%sm","0"); 
					exit(-1);  
				}
				break;
			}
		}
		if (name_error)
		{
			printf("\033[%sm","31"); // print in red
			std::cout << "The robot \"" << constraint_name_[i] << "\" does not exist" << std::endl;
			printf("\033[%sm","0"); 
			exit(-1);  
		}
		if (constraint_option_[i].size() != 3 || constraint_value_[i].size() != 3)
		{
			printf("\033[%sm","31"); // print in red
			std::cout << "Error when loading constraint's option or value for robot " << constraint_name_[i] << " in xml problem file : wrong arity (must be 3 for X Y Z in world frame)" << std::endl;
			printf("\033[%sm","0"); 
			exit(-1);
		}
		if (constraint_time_[i] >= motion_duration_ / integration_step_ || constraint_time_[i] == 0)
		{
			printf("\033[%sm","31"); // print in red
			std::cout << "Error when loading constraint's time for robot " << constraint_name_[i] << " in xml problem file : time must be between the beginning and the end of the motion" << std::endl;
			printf("\033[%sm","0"); 
			exit(-1);
		}
	}
	
	nb_step_ = ceil(motion_duration_ / integration_step_) + 1;
	if (motion_duration_ - ((nb_step_-1) * integration_step_) != 0) 
	{
		motion_duration_ = (nb_step_-1) * integration_step_;
		printf("\033[%sm","34"); // print in blue
		std::cout << "Motion duration has been modified to fit the problem, new value : " << motion_duration_ << std::endl;
		printf("\033[%sm","0");
	}
	std::cout << "nb_step_ = " << nb_step_ << std::endl;

	if (criteria_ == Jerk && nb_step_ < 5)
	{
		printf("\033[%sm","31"); // print in red
		std::cout << "Jerk criteria required at least 5 steps (for calculus)" << std::endl;
		printf("\033[%sm","0"); 
		exit(-1);
	}
	
	std::cout << "cyclic_motion_ = " << cyclic_motion_ << std::endl;
	
	// set nb_param_
	nb_param_ = nb_step_ * total_nb_dofs_ * 4; // 4 for q, dq, ddq, torque
	
	// set nb_ctr_
	nb_ctr_ = total_nb_dofs_ * (nb_step_-1) * 3; // 2 for q, dq, ddq
	
	// ctr_ for constraint position
	nb_ctr_ += constraint_name_.size() * 3 * 2; // for X Y Z, wanted position and null velocity for each
	
	if (cyclic_motion_)
		nb_ctr_ += total_nb_dofs_ * 4; // 4 for q, dq, ddq, torque
	
	std::cout << "nb_param_ = " << nb_param_<<std::endl;
	std::cout << "nb_ctr_ = " << nb_ctr_<<std::endl;
	
	// initialize nb_jac_non_null_
	nb_jac_non_null_ = 0;
	for (int s=0; s<nb_step_-1; ++s) for (int r=0; r<nb_robots_; ++r) for (int p=0;p<3;++p) for (int k=0;k<nb_dofs_[r];++k)
	{
		nb_jac_non_null_ += 1 + 4 * nb_dofs_[r];
	}
	
	for (int i=0; i<constraint_name_.size(); ++i)
		nb_jac_non_null_ += nb_dofs_[constraint_robot_number_[i]] * 9; // 3 for position, 6 for velocity
	
	if (cyclic_motion_)
		nb_jac_non_null_ += total_nb_dofs_ * 4 * 2;
		
	std::cout << "nb_jac_non_null_ = " << nb_jac_non_null_ << std::endl;
	
	// create Dyn_ and dyn_integrate for double
	Dyn_.resize(nb_robots_);
	for (int r=0;r<nb_robots_;++r)
		Dyn_[r].SetRobot(Robots_[r]);

	dyn_integrate_ = new Dynamics_Integrate<double>(nb_robots_, *env_, Dyn_);

	// create Dyn_ and dyn_integrate for F<double>
	FDyn_.resize(nb_robots_);
	for (int r=0;r<nb_robots_;++r)
		FDyn_[r].SetRobot(Robots_[r]);
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
			if (constraint_on_q_) {                               // set bounds for q
				x_l[it_[s][r][0][n]] = p_l[n];     
				x_u[it_[s][r][0][n]] = p_u[n];    
			}
			else {
				x_l[it_[s][r][0][n]] = -1e20;
				x_u[it_[s][r][0][n]] = 1e20;
			}
			if (constraint_on_dq_) {                              // set bounds for dq
				x_l[it_[s][r][1][n]] = -v_u[n];           
				x_u[it_[s][r][1][n]] = v_u[n];
			}
			else {
				x_l[it_[s][r][1][n]] = -1e20;
				x_u[it_[s][r][1][n]] = 1e20;
			}
			x_l[it_[s][r][2][n]] = -1e20;                          // set bounds for ddq
			x_u[it_[s][r][2][n]] = 1e20;
			
			if (Robots_[r]->is_robot_floating_base() && n < 6 ) {  // set bounds for torque
				x_l[it_[s][r][3][n]] = 0;          
				x_u[it_[s][r][3][n]] = 0;  
			}
			else if (constraint_on_torques_) {
				x_l[it_[s][r][3][n]] = -t_u[n];     
				x_u[it_[s][r][3][n]] = t_u[n];
			}
			else {
				x_l[it_[s][r][3][n]] = -1e20;
				x_u[it_[s][r][3][n]] = 1e20;
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
	int s, r, n;
	double res = 0., buf;
	switch (criteria_) 
	{
		case NoCriteria:
			break;
		case Energy:
			std::cout << "Energy is not implemented yet" << std::endl;
			exit(EXIT_FAILURE);
			break;
		case TorqueSquare:   // minimize torque square
			for (s=0; s<nb_step_; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) {    
				buf = x[it_[s][r][3][n]];
				res += buf * buf;
			}
			res *= integration_step_;
			return res;
		case Jerk:            // minimize difference of acceleration one step to another
			for (s=0; s<nb_step_-1; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) {
				buf = x[it_[s][r][2][n]] - x[it_[s+1][r][2][n]];
				res += buf * buf;
			}
			res *= integration_step_;
			return res;
		case MotionCaptureFitting:
			std::cout << "Fitting is not implemented yet" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
	return 0.;
}

void Direct_Motion_Optimization_Holder::eval_grad_f (bool new_x, const double *x, double *grad_f)
{
	for (int i=0; i<nb_param_; ++i)
		grad_f[i] = 0.;	
	
	int s, r, n;
	switch (criteria_) 
	{
		case NoCriteria:
			break;
		case Energy:
			std::cout << "Energy is not implemented yet" << std::endl;
			exit(EXIT_FAILURE);
			break;
		case TorqueSquare:    // minimize torque square
			for (s=0; s<nb_step_; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n)
				grad_f[it_[s][r][3][n]] = 2 * x[it_[s][r][3][n]] * integration_step_;
			break;
		case Jerk:            // minimize difference of acceleration one step to another
			for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) {
				grad_f[it_[0][r][2][n]] = (2*x[it_[0][r][2][n]] - 2*x[it_[1][r][2][n]]) * integration_step_;
				grad_f[it_[nb_step_-1][r][2][n]] = (2*x[it_[nb_step_-1][r][2][n]] - 2*x[it_[nb_step_-2][r][2][n]]) * integration_step_;
			}
			for (s=1; s<nb_step_ - 1; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) {
				grad_f[it_[s][r][2][n]] = (-2*x[it_[s-1][r][2][n]] + 4*x[it_[s][r][2][n]] - 2*x[it_[s+1][r][2][n]]) * integration_step_;
			}
			break;
		case MotionCaptureFitting:
			std::cout << "Fitting is not implemented yet" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
}

void Direct_Motion_Optimization_Holder::eval_g (bool new_x, const double *x, double *g)
{
	int s, r, n, i, j;
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
				g[cpt_g++] = ddq_[r][n] - x[it_[s+1][r][2][n]];
			}
		}
	}
	
	for (i=0; i<constraint_name_.size(); ++i)
	{
		// constraint for position
		Eigen::Matrix<double,Eigen::Dynamic,1> q_tmp;
		Eigen::Matrix<double,Eigen::Dynamic,1> dq_tmp;
		q_tmp.resize(nb_dofs_[constraint_robot_number_[i]]);
		dq_tmp.resize(nb_dofs_[constraint_robot_number_[i]]);
		for (j=0; j<nb_dofs_[constraint_robot_number_[i]]; ++j)
		{
			q_tmp(j) = x[it_[constraint_time_[i]][constraint_robot_number_[i]][0][j]];  // get position
			dq_tmp(j) = x[it_[constraint_time_[i]][constraint_robot_number_[i]][1][j]]; // get velocity
		}
		
		Eigen::Matrix<double,3,1> option (constraint_option_[i][0],constraint_option_[i][1],constraint_option_[i][2]);
		
		Eigen::Matrix<double,3,1> xyz_world = Dyn_[constraint_robot_number_[i]].CalcBodyToBaseCoordinates(q_tmp, constraint_body_number_[i], option, true);
		
		g[cpt_g++] = xyz_world(0) - constraint_value_[i][0];
		g[cpt_g++] = xyz_world(1) - constraint_value_[i][1];
		g[cpt_g++] = xyz_world(2) - constraint_value_[i][2];
		
		// constraint for null velocity
		Eigen::Matrix <double, 3, 1 > vel_world = Dyn_[constraint_robot_number_[i]].CalcPointVelocity (q_tmp, dq_tmp, constraint_body_number_[i],option, true);
		
		g[cpt_g++] = vel_world(0);
		g[cpt_g++] = vel_world(1);
		g[cpt_g++] = vel_world(2);
		
	}
	
	if (cyclic_motion_)
	{
		for (r=0; r<nb_robots_; ++r) for (int p=0; p<4; ++p) for (n=0; n<nb_dofs_[r]; ++n)
			g[cpt_g++] = x[it_[0][r][p][n]] - x[it_[nb_step_-1][r][p][n]];
	}
	
// 	affiche_torque(x);
}

void Direct_Motion_Optimization_Holder::get_dependencies (int *iRow, int *jCol)
{
	// In iRow, set ctr_ index, in jCol set param_ index
	int s, r, p, n, k, cpt = 0, i, j;
	int cpt_g = 0;
	for (s=0; s<nb_step_-1; ++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n) for (p=0;p<3;++p)
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
	
	for (i=0; i<constraint_name_.size(); ++i) 
	{
		for (j=0;j<3;++j)   // x y z for position
		{
			for (int m=0; m<nb_dofs_[constraint_robot_number_[i]]; ++m) 
			{
				iRow[cpt] = cpt_g; 
				jCol[cpt] = it_[constraint_time_[i]][constraint_robot_number_[i]][0][m];
				cpt++;
			}
			cpt_g++;
		}
		for (j=0;j<3;++j)   
		{
			for (int m=0; m<nb_dofs_[constraint_robot_number_[i]]; ++m) // x y z for velocity 
			{
				iRow[cpt] = cpt_g; 
				jCol[cpt] = it_[constraint_time_[i]][constraint_robot_number_[i]][0][m];
				cpt++;
			}
			for (int m=0; m<nb_dofs_[constraint_robot_number_[i]]; ++m) // dx, dy, dz for velocity
			{
				iRow[cpt] = cpt_g; 
				jCol[cpt] = it_[constraint_time_[i]][constraint_robot_number_[i]][1][m];
				cpt++;
			}
			cpt_g++;
		}
	}
	
	if (cyclic_motion_)
	{
		for (r=0; r<nb_robots_; ++r) for (int p=0; p<4; ++p) for (n=0; n<nb_dofs_[r]; ++n)
		{
			iRow[cpt] = cpt_g;
			jCol[cpt] = it_[0][r][p][n];
			cpt++;
			iRow[cpt] = cpt_g;
			jCol[cpt] = it_[nb_step_-1][r][p][n];
			cpt++;
			cpt_g++;
		}
	}
}

void Direct_Motion_Optimization_Holder::eval_grad_g (bool new_x, const double *x, double *values)
{
	int s, r, p, n, i, j, k;
	int cpt_diff, cpt = 0;
	int cpt_diff_max;
	F<double> contrainte_q, contrainte_dq, contrainte_ddq;
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
				// constraint on ddq
				values[cpt++] = -1;
				contrainte_ddq = Fddq_[r][n];
				for (i=0; i<cpt_diff_max; ++i)
					values[cpt++] = contrainte_ddq.d(i);
			}
		}
	}
	
	for (i=0; i<constraint_name_.size(); ++i)
	{
		Eigen::Matrix<F<double>,Eigen::Dynamic,1> Fq_tmp;
		Fq_tmp.resize(nb_dofs_[constraint_robot_number_[i]]);
		Eigen::Matrix<F<double>,Eigen::Dynamic,1> Fdq_tmp;
		Fdq_tmp.resize(nb_dofs_[constraint_robot_number_[i]]);
		int cpt_fd = 0;
		for (j=0; j<nb_dofs_[constraint_robot_number_[i]]; ++j) {
			Fq_tmp(j) = x[it_[constraint_time_[i]][constraint_robot_number_[i]][0][j]];
			Fq_tmp(j).diff(cpt_fd++,2*nb_dofs_[constraint_robot_number_[i]]);
		}
		for (j=0; j<nb_dofs_[constraint_robot_number_[i]]; ++j) {
			Fdq_tmp(j) = x[it_[constraint_time_[i]][constraint_robot_number_[i]][1][j]];
			Fdq_tmp(j).diff(cpt_fd++,2*nb_dofs_[constraint_robot_number_[i]]);
		}
		
		Eigen::Matrix<F<double>,3,1> Foption ((F<double>)constraint_option_[i][0],(F<double>)constraint_option_[i][1],(F<double>)constraint_option_[i][2]);
		
		Eigen::Matrix<F<double>,3,1> Fxyz_world = FDyn_[constraint_robot_number_[i]].CalcBodyToBaseCoordinates(Fq_tmp, constraint_body_number_[i], Foption, true);
		
		Eigen::Matrix<F<double>,3,1> Fdxyz_world = FDyn_[constraint_robot_number_[i]].CalcPointVelocity(Fq_tmp, Fdq_tmp, constraint_body_number_[i], Foption, true);
		
		for (int j=0; j<3; j++)
			for (k=0; k<nb_dofs_[constraint_robot_number_[i]]; ++k)
				values[cpt++] = Fxyz_world(j).d(k);

		for (int j=0; j<3; j++)
		{
			for (k=0; k<nb_dofs_[constraint_robot_number_[i]]; ++k)
				values[cpt++] = Fdxyz_world(j).d(k);
			for (k=0; k<nb_dofs_[constraint_robot_number_[i]]; ++k)
				values[cpt++] = Fdxyz_world(j).d(k+nb_dofs_[constraint_robot_number_[i]]);
		}	
	}
	
	if (cyclic_motion_)
	{
		for (r=0; r<nb_robots_; ++r) for (int p=0; p<4; ++p) for (n=0; n<nb_dofs_[r]; ++n)
		{
			values[cpt++] = 1;
			values[cpt++] = -1;
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
	for (int i=0;i<nb_param_;i++)
		X << std::setprecision(20)<< x[i]<<" ";
	std::string s = X.str();
	
// 	tinyxml2::XMLNode * Elnbparam = doc_.NewElement ("nbparam");	
// 	oss << nb_param_;
// 	std::string s1 = oss.str();
// 	tinyxml2::XMLText * nbparam = doc_.NewText ( s1.c_str());
// 	Elnbparam->InsertEndChild (nbparam);
// 	result->InsertEndChild (Elnbparam);	
	
// 	tinyxml2::XMLNode * Elparam = doc_.NewElement ("param");	
// 	tinyxml2::XMLText * param = doc_.NewText ( s.c_str() );
// 	Elparam->InsertEndChild (param);
// 	result->InsertEndChild (Elparam);

	tinyxml2::XMLElement * Elparam = doc_.NewElement ("param");	
	Elparam->SetAttribute("nb_param",nb_param_);
	tinyxml2::XMLText * param = doc_.NewText ( s.c_str() );
	Elparam->InsertEndChild (param);
	result->InsertEndChild (Elparam);
	
	for(int s=0;s<nb_step_;s++)
	{
		
		tinyxml2::XMLElement * Elvalue = doc_.NewElement ("log");
		Elvalue->SetAttribute("time",integration_step_ * s);
		for (int k=0; k < nb_robots_; k++)
		{
			std::ostringstream oss_q;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_q << x[it_[s][k][0][j]] << " ";
				}

			tinyxml2::XMLElement *xml_q = doc_.NewElement ("q");
			xml_q->SetAttribute("robot",(Robots_[k]->getRobotName()).c_str());
			tinyxml2::XMLText *xml_text = doc_.NewText ( oss_q.str().c_str());
			xml_q->InsertEndChild (xml_text);
			Elvalue->InsertEndChild (xml_q);

			std::ostringstream oss_dq;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_dq << x[it_[s][k][1][j]] << " ";
				}

			tinyxml2::XMLElement *xml_dq = doc_.NewElement ("dq");
			xml_dq->SetAttribute("robot",(Robots_[k]->getRobotName()).c_str());
			xml_text = doc_.NewText ( oss_dq.str().c_str());
			xml_dq->InsertEndChild (xml_text);
			Elvalue->InsertEndChild (xml_dq);			

			std::ostringstream oss_ddq;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_ddq << x[it_[s][k][2][j]] << " ";
				}

			tinyxml2::XMLElement *xml_ddq = doc_.NewElement ("ddq");
			xml_ddq->SetAttribute("robot",(Robots_[k]->getRobotName()).c_str());
			xml_text = doc_.NewText ( oss_ddq.str().c_str());
			xml_ddq->InsertEndChild (xml_text);
			Elvalue->InsertEndChild (xml_ddq);
		
			std::ostringstream oss_tau;
			if ( k ==0)	// FIXME for the moment only one robot
				for (int j=0;j<nb_dofs_[k];j++)
				{
					oss_tau << x[it_[s][k][3][j]] << " ";
				}

			tinyxml2::XMLElement *xml_tau = doc_.NewElement ("tau");
			xml_tau->SetAttribute("robot",(Robots_[k]->getRobotName()).c_str());
			xml_text = doc_.NewText ( oss_tau.str().c_str());
			xml_tau->InsertEndChild (xml_text);
			Elvalue->InsertEndChild (xml_tau);
		}  
		result->InsertEndChild (Elvalue);  

	}
	
	results->InsertEndChild (result);
	doc_.SaveFile (xml_problem_filename_.c_str() );
	std::cout << " status = "<< status << std::endl;  
	
// 	std::cout << "max torque = "  << max_torque_ << std::endl;
	
	for (int i=0; i<constraint_name_.size(); ++i)
	{
		Eigen::Matrix<double,Eigen::Dynamic,1> q_tmp;
		q_tmp.resize(nb_dofs_[constraint_robot_number_[i]]);
		for (int j=0; j<nb_dofs_[constraint_robot_number_[i]]; ++j)
			q_tmp(j) = x[it_[constraint_time_[i]][constraint_robot_number_[i]][0][j]];
		
		Eigen::Matrix<double,3,1> option (constraint_option_[i][0],constraint_option_[i][1],constraint_option_[i][2]);
		
		Eigen::Matrix<double,3,1> xyz_world = Dyn_[constraint_robot_number_[i]].CalcBodyToBaseCoordinates(q_tmp, constraint_body_number_[i], option, true);
		
		printf("\033[%sm","31"); // print in red
		std::cout << "actual pos = " << xyz_world(0) << " " << xyz_world(1) << " "  << xyz_world(2) << std::endl;
		printf("\033[%sm","0");
	}
	
	export_to_csv(x);
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
			// FIXME
			tinyxml2::XMLElement * Elnb = result->FirstChildElement ("nbparam");
			int nb = string_to_int(Elnb->GetText());
			tinyxml2::XMLElement * Elparam = result->FirstChildElement ("param");
			Elparam->SetAttribute("nb_param",nb_param_);
                        
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

void Direct_Motion_Optimization_Holder::export_to_csv(const double *x)
{
	// FIXME only one robot
	int s,r,p,n,i,j;
	std::ostringstream strs;
	time_t uxtime = time(0);
	int unix_time = uxtime;
	strs << unix_time;
	std::string crit = "\t";
	std::vector<std::vector<double> > tmp_jerk;
	std::vector<std::vector<double> > tmp_torque;
	tmp_jerk.resize(nb_dofs_[0]);
	tmp_torque.resize(nb_dofs_[0]);
	double buf;
	for (i=0; i<nb_dofs_[0]; ++i)
	{
		tmp_jerk[i].resize(nb_step_);
		tmp_torque[i].resize(nb_step_);
	}
	// for torque calculus
	for (i=0;i<nb_dofs_[0]; ++i)   // set first row
		tmp_torque[i][0] = x[it_[0][0][3][i]] * x[it_[0][0][3][i]] * integration_step_;
	for (s=1;s<nb_step_; ++s)      // set all other rows
		for (i=0;i<nb_dofs_[0]; ++i)
			tmp_torque[i][s] = tmp_torque[i][s-1] + x[it_[s][0][3][i]] * x[it_[s][0][3][i]] * integration_step_;
	// for jerk calculus
	for (i=0;i<nb_dofs_[0]; ++i)   // set first row
		tmp_jerk[i][0] = 0.;
	for (s=1;s<nb_step_; ++s)      // set all other rows
		for (i=0;i<nb_dofs_[0]; ++i)
		{
			buf = x[it_[s-1][0][2][i]] - x[it_[s][0][2][i]];
			tmp_jerk[i][s] = tmp_jerk[i][s-1] + buf * buf * integration_step_;
		}
	switch(criteria_)
	{
		case NoCriteria: crit = "None"; break;
		case Energy: crit = "Energy"; break;
		case TorqueSquare: crit = "TorqueSquare"; break;
		case Jerk: crit = "Jerk"; break;
		case MotionCaptureFitting: crit = "Fitting"; break;
	}
	std::string filename = strs.str() +"_save_robot-"+ Robots_[0]->getRobotName() +"_criteria-"+ crit +".csv";
	std::ofstream myfile;
	myfile.open(filename);
	std::string sep = " "; // separator for csv file
	// writing in file
	for (p=0; p<4;++p) // for all parameters
	{
		for (s=0; s<nb_step_; ++s) // for all steps
		{
			std::ostringstream str1;
			str1 << (integration_step_*s);
			myfile << str1.str();
			myfile << sep;
			for (n=0;n<nb_dofs_[0]; ++n)
			{
				std::ostringstream str2;
				str2 << x[it_[s][0][p][n]]; 
				myfile << str2.str();
				myfile << sep;
			}
			if (p == 2) for (n=0;n<nb_dofs_[0]; ++n)
			{
				std::ostringstream str3;
				str3 << tmp_jerk[n][s]; 
				myfile << str3.str();
				myfile << sep;
			}
			if (p == 3) for (n=0;n<nb_dofs_[0]; ++n)
			{
				std::ostringstream str4;
				str4 << tmp_torque[n][s]; 
				myfile << str4.str();
				myfile << sep;
			}
			myfile << "\n";
		}
		myfile << "\n";
	}	
	myfile.close();
  
}

void Direct_Motion_Optimization_Holder::affiche_torque(const double *x)
{
	int s, r, n;
	max_torque_ = 0;
	for (s=0; s< nb_step_;++s) for (r=0; r<nb_robots_; ++r) for (n=0; n<nb_dofs_[r]; ++n)
		if (fabs(max_torque_) < fabs(x[it_[s][r][3][n]]))
			max_torque_ = x[it_[s][r][3][n]];
}


extern "C" Direct_Motion_Optimization_Holder* create() 
{
    return new Direct_Motion_Optimization_Holder;
}

extern "C" void destroy(Direct_Motion_Optimization_Holder* p) 
{
    delete p;
}
