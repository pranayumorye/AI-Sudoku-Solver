#include"BTSolver.hpp"

using namespace std;

// =====================================================================
// Constructors
// =====================================================================

BTSolver::BTSolver ( SudokuBoard input, Trail* _trail,  string val_sh, string var_sh, string cc )
: sudokuGrid( input.get_p(), input.get_q(), input.get_board() ), network( input )
{
	valHeuristics = val_sh;
	varHeuristics = var_sh;
	cChecks =  cc;

	trail = _trail;
}
// =====================================================================

// Consistency Checks
// =====================================================================

// Basic consistency check, no propagation done
bool BTSolver::assignmentsCheck ( void )
{
	for ( Constraint c : network.getConstraints() )
		if ( ! c.isConsistent() )
			return false;

	return true;
}

// =================================================================
// Arc Consistency
// =================================================================
bool BTSolver::arcConsistency ( void )
{
    vector<Variable*> toAssign;
    vector<Constraint*> RMC = network.getModifiedConstraints();
    for (int i = 0; i < RMC.size(); ++i)
    {
        vector<Variable*> LV = RMC[i]->vars;
        for (int j = 0; j < LV.size(); ++j)
        {
            if(LV[j]->isAssigned())
            {
                vector<Variable*> Neighbors = network.getNeighborsOfVariable(LV[j]);
                int assignedValue = LV[j]->getAssignment();
                for (int k = 0; k < Neighbors.size(); ++k)
                {
                    Domain D = Neighbors[k]->getDomain();
                    if(D.contains(assignedValue))
                    {
                        if (D.size() == 1)
                            return false;
                        if (D.size() == 2)
                            toAssign.push_back(Neighbors[k]);
                        trail->push(Neighbors[k]);
                        Neighbors[k]->removeValueFromDomain(assignedValue);
                    }
                }
            }
        }
    }
    if (!toAssign.empty())
    {
        for (int i = 0; i < toAssign.size(); ++i)
        {
            Domain D = toAssign[i]->getDomain();
            vector<int> assign = D.getValues();
            trail->push(toAssign[i]);
            toAssign[i]->assignValue(assign[0]);
        }
        return arcConsistency();
    }
    return network.isConsistent();
}

/**
 * Part 1 TODO: Implement the Forward Checking Heuristic
 *
 * This function will do both Constraint Propagation and check
 * the consistency of the network
 *
 * (1) If a variable is assigned then eliminate that value from
 *     the square's neighbors.
 *
 * Note: remember to trail.push variables before you change their domain
 * Return: a pair of a map and a bool. The map contains the pointers to all MODIFIED variables, mapped to their MODIFIED domain. 
 * 		   The bool is true if assignment is consistent, false otherwise.
 */
pair<map<Variable*,Domain>,bool> BTSolver::forwardChecking ( void )
{
	ConstraintNetwork::VariableSet vars = getNetwork().getVariables();
	bool flag = true;
	map<Variable*, Domain> ans;

	for(auto var = vars.begin(); var != vars.end(); var++)
	{
		if(!(*var)->isAssigned()) continue;

		ConstraintNetwork::VariableSet nbrs = getNetwork().getNeighborsOfVariable(*var);
		for(auto nbr = nbrs.begin(); nbr != nbrs.end(); nbr++)
		{
			Domain::ValueSet nbrdomain = (*nbr)->getValues();
			if(find(nbrdomain.begin(),nbrdomain.end(),(*var)->getAssignment()) != nbrdomain.end())
			{
				trail->push(*nbr);
				(*nbr)->removeValueFromDomain((*var)->getAssignment());
				Domain tempdomain((*nbr)->getDomain());
				ans.erase(*nbr);
				ans.insert(std::make_pair((*nbr),tempdomain));
				if((*nbr)->size() == 0){
					flag = false;
				}
			}
				
		}
	}

	return make_pair(ans, flag);
}

/**
 * Part 2 TODO: Implement both of Norvig's Heuristics
 *
 * This function will do both Constraint Propagation and check
 * the consistency of the network
 *
 * (1) If a variable is assigned then eliminate that value from
 *     the square's neighbors.
 *
 * (2) If a constraint has only one possible place for a value
 *     then put the value there.
 *
 * Note: remember to trail.push variables before you change their domain
 * Return: a pair of a map and a bool. The map contains the pointers to all variables that were assigned during 
 *         the whole NorvigCheck propagation, and mapped to the values that they were assigned.
 *         The bool is true if assignment is consistent, false otherwise.
 */
pair<map<Variable*,int>,bool> BTSolver::norvigCheck ( void )
{
	pair<map<Variable*,int>, bool> ans;
	ans.second = forwardChecking().second;
	vector<Constraint*> modConstraints = network.getModifiedConstraints();
	int N = sudokuGrid.get_q()*sudokuGrid.get_p();
	
	for(int i=0; i<modConstraints.size(); i++) {
		vector<Variable*> variables_ = modConstraints[i]->vars;
		for(int j=0; j<variables_.size(); j++) {
			int row = variables_[j]->row(); int col = variables_[j]->col(); int block = variables_[j]->block();
			vector<Variable*> varNeighbors = network.getNeighborsOfVariable(variables_[j]);
			varNeighbors.push_back(variables_[j]);
			int count[N+1]; 

			for(int a=0; a<=N; a++) {
				count[a] = 0;
			}

			for(int k=0; k<varNeighbors.size(); k++) {
				if(varNeighbors[k]->row() == row) {
					vector<int> v = varNeighbors[k]->getDomain().getValues();
					for(int l=0; l<v.size(); l++) {
						count[v[l]]++;
					}
				}
			}

			for(int k=1; k<=N; k++) {
				if(count[k] == 1) {
					for(int m=0; m<varNeighbors.size(); m++) {
						if(!varNeighbors[m]->isAssigned() &&  varNeighbors[m]->row() == row) {
							if(varNeighbors[m]->getDomain().contains(k)) {
								trail->push(varNeighbors[m]);
								varNeighbors[m]->assignValue(k);
								ans.first[varNeighbors[m]] = k;
								ans.second = forwardChecking().second;
							}
						}
					}
				}
			}
			
			for(int a=0; a<=N; a++) {
				count[a] = 0;
			}

			for(int k=0; k<varNeighbors.size(); k++) {
				if(varNeighbors[k]->col() == col) {
					vector<int> v = varNeighbors[k]->getDomain().getValues();
					for(int m=0; m<v.size(); m++) {
						count[v[m]]++;
					}
				}
			}

			for(int k=1;k<=N;k++) {
				if(count[k] == 1) {
					for(int m=0; m<varNeighbors.size(); m++) {
						if(!varNeighbors[m]->isAssigned() && varNeighbors[m]->col() == col) {
							if(varNeighbors[m]->getDomain().contains(k)) {
								trail->push(varNeighbors[m]);
								varNeighbors[m]->assignValue(k);
								ans.first[varNeighbors[m]] = k;
								ans.second = forwardChecking().second;
							}
						}
					}
				}
			} 

			for(int a=0; a<=N; a++) {
				count[a] = 0;
			}

			for(int k=0; k<varNeighbors.size(); k++) {
				if(varNeighbors[k]->block() == block) {
					vector<int> v = varNeighbors[k]->getDomain().getValues();
					for(int m=0;m<v.size();m++) {
						count[v[m]]++;
					}
				}
			}

			for(int k=1; k<=N; k++) {
				if(count[k] == 1) {
					for(int m=0; m<varNeighbors.size(); m++) {
						if(!varNeighbors[m]->isAssigned() && varNeighbors[m]->block() == block) {
							if(varNeighbors[m]->getDomain().contains(k)) {
								trail->push(varNeighbors[m]);
								varNeighbors[m]->assignValue(k);
								ans.first[varNeighbors[m]] = k;
								ans.second = forwardChecking().second;
							}
						}
					}
				}
			}

		}
	}

	return ans;
}

/**
 * Optional TODO: Implement your own advanced Constraint Propagation
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
bool BTSolver::getTournCC ( void )
{
	return false;
}

// =====================================================================
// Variable Selectors
// =====================================================================

// Basic variable selector, returns first unassigned variable
Variable* BTSolver::getfirstUnassignedVariable ( void )
{
	for ( Variable* v : network.getVariables() )
		if ( !(v->isAssigned()) )
			return v;

	// Everything is assigned
	return nullptr;
}

/**
 * Part 1 TODO: Implement the Minimum Remaining Value Heuristic
 *
 * Return: The unassigned variable with the smallest domain
 */
Variable* BTSolver::getMRV ( void )
{
	//cout<<"Inside MRV\n";
	ConstraintNetwork::VariableSet vars = getNetwork().getVariables();
	int minval = 2147483647;

	Variable *ans = NULL;

	for(auto it = vars.begin(); it != vars.end() ; it++)
	{
		if((*it)->size()<minval && !(*it)->isAssigned())
		{
			minval = (*it)->size();
			ans = *it;
		}
	}
	//cout<<"Row:"<<ans->row()<<" "<<"Col:"<<ans->col()<<endl;
	//cout<<"Finish MRV\n";
	return ans;
}

/**
 * Part 2 TODO: Implement the Minimum Remaining Value Heuristic
 *                with Degree Heuristic as a Tie Breaker
 *
 * Return: The unassigned variable with the smallest domain and affecting the most unassigned neighbors.
 * 		   If there are multiple variables that have the same smallest domain with the same number 
 * 		   of unassigned neighbors, add them to the vector of Variables.
 *         If there is only one variable, return the vector of size 1 containing that variable.
 */
vector<Variable*> BTSolver::MRVwithTieBreaker ( void )
{

	Variable* ans = getMRV();
	vector<Variable*> mad;
	int minSize = ans->size();
	int degree = 0;
	vector<Variable*> neighbours = getNetwork().getNeighborsOfVariable(ans);
	
	for(auto it = neighbours.begin(); it != neighbours.end(); it++){
		if(!(*it)->isAssigned()){
			degree++;
		}
	} 
	
	vector<Variable*> vars = getNetwork().getVariables();
	for(auto it = vars.begin(); it != vars.end(); it++){

		if(!(*it)->isAssigned() && (*it)->size() == minSize){

			int count = 0;
			vector<Variable*> tempNeighbors = getNetwork().getNeighborsOfVariable(*it);
			
			for(auto v = tempNeighbors.begin(); v != tempNeighbors.end(); v++){
				if(!(*v)->isAssigned())
					count++;
			}
			
			if(count>degree){
				degree = count;
				mad.clear();
			}
			
			if(count == degree){
			mad.push_back(*it);
			}
		}
	}
return mad;	
}
/**
 * Optional TODO: Implement your own advanced Variable Heuristic
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
Variable* BTSolver::getTournVar ( void )
{
	return nullptr;
}

// =====================================================================
// Value Selectors
// =====================================================================

// Default Value Ordering
vector<int> BTSolver::getValuesInOrder ( Variable* v )
{
	vector<int> values = v->getDomain().getValues();
	sort( values.begin(), values.end() );
	return values;
}

/**
 * Part 1 TODO: Implement the Least Constraining Value Heuristic
 *
 * The Least constraining value is the one that will knock the least
 * values out of it's neighbors domain.
 *
 * Return: A list of v's domain sorted by the LCV heuristic
 *         The LCV is first and the MCV is last
 */

bool compare(const pair<int, int> &a, const pair<int, int> &b)
{
	if(a.second == b.second)
		return a.first<b.first;
	return a.second<b.second;
}

vector<int> BTSolver::getValuesLCVOrder ( Variable* v )
{
	ConstraintNetwork::VariableSet neighbors = getNetwork().getNeighborsOfVariable(v);
	Domain::ValueSet currdomain = v->getValues();
	vector<pair<int,int> > counts;
	for(auto it = currdomain.begin(); it != currdomain.end(); it++)
	{
		counts.push_back(make_pair((*it), 0));
	}

	for(auto currdomval = counts.begin(); currdomval != counts.end(); currdomval++)
	{
		for(auto neighbor = neighbors.begin(); neighbor != neighbors.end(); neighbor++)
		{
			Domain::ValueSet currneighbordomain = (*neighbor)->getValues();
			if(std::find(currneighbordomain.begin(), currneighbordomain.end(),(*currdomval).first) != currneighbordomain.end())
			{
					(*currdomval).second++;
			}
		}
		
	}
	sort(counts.begin(),counts.end(), compare);

	vector<int> ans;
	for(auto it = counts.begin(); it != counts.end(); it++)
	{
		ans.push_back((*it).first);
	}
    return ans;
}

/**
 * Optional TODO: Implement your own advanced Value Heuristic
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
vector<int> BTSolver::getTournVal ( Variable* v )
{
	return vector<int>();
}

// =====================================================================
// Engine Functions
// =====================================================================

int BTSolver::solve ( float time_left)
{
	if (time_left <= 60.0)
		return -1;
	double elapsed_time = 0.0;
    clock_t begin_clock = clock();

	if ( hasSolution )
		return 0;

	// Variable Selection
	Variable* v = selectNextVariable();

	if ( v == nullptr )
	{
		for ( Variable* var : network.getVariables() )
		{
			// If all variables haven't been assigned
			if ( ! ( var->isAssigned() ) )
			{
				return 0;
			}
		}

		// Success
		hasSolution = true;
		return 0;
	}

	// Attempt to assign a value
	for ( int i : getNextValues( v ) )
	{
		// Store place in trail and push variable's state on trail
		trail->placeTrailMarker();
		trail->push( v );

		// Assign the value
		v->assignValue( i );

		// Propagate constraints, check consistency, recurse
		if ( checkConsistency() ) {
			clock_t end_clock = clock();
			elapsed_time += (float)(end_clock - begin_clock)/ CLOCKS_PER_SEC;
			double new_start_time = time_left - elapsed_time;
			int check_status = solve(new_start_time);
			if(check_status == -1) {
			    return -1;
			}
			
		}

		// If this assignment succeeded, return
		if ( hasSolution )
			return 0;

		// Otherwise backtrack
		trail->undo();
	}
	return 0;
}

bool BTSolver::checkConsistency ( void )
{
	if ( cChecks == "forwardChecking" )
		return forwardChecking().second;

	if ( cChecks == "norvigCheck" )
		return norvigCheck().second;

	if ( cChecks == "tournCC" )
		return getTournCC();

	return assignmentsCheck();
}

Variable* BTSolver::selectNextVariable ( void )
{
	if ( varHeuristics == "MinimumRemainingValue" )
		return getMRV();

	if ( varHeuristics == "MRVwithTieBreaker" )
		return MRVwithTieBreaker()[0];

	if ( varHeuristics == "tournVar" )
		return getTournVar();

	return getfirstUnassignedVariable();
}

vector<int> BTSolver::getNextValues ( Variable* v )
{
	if ( valHeuristics == "LeastConstrainingValue" )
		return getValuesLCVOrder( v );

	if ( valHeuristics == "tournVal" )
		return getTournVal( v );

	return getValuesInOrder( v );
}

bool BTSolver::haveSolution ( void )
{
	return hasSolution;
}

SudokuBoard BTSolver::getSolution ( void )
{
	return network.toSudokuBoard ( sudokuGrid.get_p(), sudokuGrid.get_q() );
}

ConstraintNetwork BTSolver::getNetwork ( void )
{
	return network;
}
