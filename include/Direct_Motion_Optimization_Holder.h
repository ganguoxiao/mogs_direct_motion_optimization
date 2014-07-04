// //      Direct_Motion_Optimization_Holder.h
// //      Copyright (C) 2014 lengagne (lengagne@gmail.com)
// //      Copyright (C) 2014 piot
// // 
// //      This program is free software: you can redistribute it and/or modify
// //      it under the terms of the GNU General Public License as published by
// //      the Free Software Foundation, either version 3 of the License, or
// //      (at your option) any later version.
// // 
// //      This program is distributed in the hope that it will be useful,
// //      but WITHOUT ANY WARRANTY; without even the implied warranty of
// //      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// //      GNU General Public License for more details.
// // 
// //      You should have received a copy of the GNU General Public License
// //      along with this program.  If not, see <http://www.gnu.org/licenses/>.
// //
// //      This program was developped in the following labs:
// // 	From 2014 : Université Blaise Pascal, ISPR, MACCS, Clermont-Ferrand, France
// 
// #ifndef	__DIRECT_MOTION_OPTIMIZATION_HOLDER__
// #define __DIRECT_MOTION_OPTIMIZATION_HOLDER__
// 
// #include "Optimization_Holder.h"
// 
// class Direct_Motion_Optimization_Holder:public Motion_Optimization_Holder
// {
//       public:
// 
// 	
// 	/**	Determine if the size of the robot is constant or is a result of the problem	*/
// 	bool known_size(int i);	
// 	
// 	void read_problem_specific (tinyxml2::XMLElement * root) = 0;
// 
// 	/**	test the xml and return false in case of problem (regarding the DTD)
//  	 */
// 	bool testXml (const std::string xml_name) = 0;
// 
// 	void print_type () = 0;
// 
// 	/**	Initialize the optimization */
// 	void initialize () = 0;
// 
// 	/**	Get the bounds on the limits and on the constraints	*/
// 	void get_bounds_info (double *x_l, double *x_u, double *g_l, double *g_u)  = 0;
// 
// 	/**	Get the initial point	*/
// 	void get_starting_point (double *x) = 0;
// 
// 	/**	Compute the objective value	*/
// 	double eval_f (bool new_x, const double *x) = 0;
// 
// 	/**	Compute the derivative of the objective value	*/
// 	void eval_grad_f (bool new_x, const double *x, double *grad_f) = 0;
// 
// 	/**	Compute the constraints*/
// 	void eval_g (bool new_x, const double *x, double *g) = 0;
// 
// 	/**	Get the dependencies of the jacobian	*/
// 	void get_dependencies (int *iRow, int *jCol) = 0;
// 
// 	/**	Return the value of the jacobian (regarding the defined structure	*/
// 	void eval_grad_g (bool new_x, const double *x,
// 				  double *values)  = 0;
// 
// 	/**	Returns the final results of the optimization	*/
// 	void this_is_final_results (const double *x, int status) = 0;
// 	
// 	
// 	void get_q_dq(	int num_robot,
// 				std::vector<double> param,
// 				double time,
// 				Eigen::Matrix< double, Eigen::Dynamic, 1> &Q,
// 				bool get_dq,
// 				Eigen::Matrix< double, Eigen::Dynamic, 1> &DQ) = 0;
// 	
// 	/**	return if we need to do a pre-optim*/
// 	bool do_pre_optimization() = 0;
// 	
// 	/**	Perform the pre optimization*/
// 	void pre_optimize() = 0;
// 
// 	///// non virtual functions     
// 	void get_result_param(	const std::string result_name, 
// 					std::vector<double> & param) = 0; /*Array of vectors - for several robots*/
// 	
// 	/**	Returns the number of constraints	*/
// 	int get_nb_constraints(){return nb_ctr_;}
// 
// 	/**	Returns the number of non-null value in the Jacobian*/
// 	int get_nb_jacobian_non_null(){return nb_jac_non_null_;}
// 
// 	/**	Returns the number of parameters	*/
// 	int get_nb_param(){return nb_param_;}
// 	
// 	/**	Copy the optimization properties	*/
// 	void set_optim_properties( const Optimization_Options & in){Optim_properties_ = in;}
// 
// 	/**	Set the solver 	*/
// 	void set_solver(MoGS_Solver ** solver, std::string soluce_name);
// 
// 
//       protected:
// 	int nb_param_;		// number of parameters
// 	int nb_ctr_;		// number of constraints
// 	int nb_jac_non_null_;	// number of non null values in the jacobian
// 
// 	bool known_size_ ;   
// };
// 
// #endif