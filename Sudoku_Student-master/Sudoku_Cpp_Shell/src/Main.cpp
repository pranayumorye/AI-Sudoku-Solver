#include "BTSolver.hpp"
#include "SudokuBoard.hpp"
#include "Trail.hpp"

#include <iostream>
#include <ctime>
#include <cmath>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <time.h>
#include <fstream>

using namespace std;

/**
 * Main driver file, which is responsible for interfacing with the
 * command line and properly starting the backtrack solver.
 */

int main ( int argc, char *argv[] )
{
	// Set random seed
	srand( time ( NULL ) );

	// Important Variables
	string file   = "";
	string var_sh = "";
	string val_sh = "";
	string cc     = "";

	for ( int i = 1; i < argc; ++i )
	{
		string token = argv[i];

		if ( token == "MRV" )
			var_sh = "MinimumRemainingValue";

		else if ( token == "MAD" )
			var_sh = "MRVwithTieBreaker";

		else if ( token == "LCV" )
			val_sh = "LeastConstrainingValue";

		else if ( token == "FC" )
			cc = "forwardChecking";

		else if ( token == "NOR" )
			cc = "norvigCheck";

		else if ( token == "TOURN" )
		{
			 var_sh = "tournVar";
			 val_sh = "tournVal";
			 cc     = "tournCC";
		}

		else
			file = token;
	}

	Trail trail;

	ofstream filewriter;
	filewriter.open("timetaken.txt", std::ios_base::app);

	if ( file == "" )
	{
		//iterating over all the M values
		for(int i=81; i<=81; i++)
		{
			double sum = 0;
			//averaging out 10 values for each M
			for(int j=8; j<9; j++)
			{
			SudokuBoard board( 7, 7, 18 );
			cout << board.toString() << endl;

			BTSolver solver = BTSolver( board, &trail, val_sh, var_sh, cc );

			//time measurement

			clock_t t = clock();

			if (cc == "forwardChecking" or cc == "norvigCheck" or cc == "tournCC")
				solver.checkConsistency();
			solver.solve(600.0);

			if ( solver.haveSolution() )
			{
				cout << solver.getSolution().toString() << endl;
				cout << "Trail Pushes: " << trail.getPushCount() << endl;
				trail.clear();
				cout << "Backtracks: "  << trail.getUndoCount() << endl;
			}

			else
			{
				cout << "Failed to find a solution" << endl;
			}
			t = clock() - t;
			double time_taken = double(t)/CLOCKS_PER_SEC;
			sum += time_taken;
		}
		
		filewriter<<sum<<endl;
		}
		return 0;
	}

	filewriter.close();

	struct stat path_stat;
	stat ( file.c_str(), &path_stat );
	bool folder = S_ISDIR ( path_stat.st_mode );

	if ( folder )
	{
		DIR *dir;
		if ( ( dir = opendir ( file.c_str() ) ) == NULL )
		{
			cout << "[ERROR] Failed to open directory." << endl;
			return 0;
		}

		struct dirent *ent;

		int numSolutions = 0;
		while ( ( ent = readdir (dir) ) != NULL )
		{
			if ( ent->d_name[0] == '.' )
				continue;

			cout << "Running board: " << ent->d_name << endl;

			string individualFile = file + "/" + ent->d_name;


			SudokuBoard board( individualFile );

			BTSolver solver = BTSolver( board, &trail, val_sh, var_sh, cc );
			if (cc == "forwardChecking" or cc == "norvigCheck" or cc == "tournCC")
	            solver.checkConsistency();
			solver.solve(600.0);

			if ( solver.haveSolution() )
				numSolutions++;

			trail.clear();
		}

		cout << "Solutions Found: " << numSolutions << endl;
		cout << "Trail Pushes: " << trail.getPushCount() << endl;
		cout << "Backtracks: "  << trail.getUndoCount() << endl;
		closedir (dir);

		return 0;
	}

	SudokuBoard board( file );
	cout << board.toString() << endl;

	//time measurement

	BTSolver solver = BTSolver( board, &trail, val_sh, var_sh, cc );
	if (cc == "forwardChecking" or cc == "norvigCheck" or cc == "tournCC")
					solver.checkConsistency();
	solver.solve(600.0);

	if ( solver.haveSolution() )
	{
		cout << solver.getSolution().toString() << endl;
		cout << "Trail Pushes: " << trail.getPushCount() << endl;
		cout << "Backtracks: "  << trail.getUndoCount() << endl;
	}
	else
	{
		cout << "Failed to find a solution" << endl;
	}

	return 0;
}
