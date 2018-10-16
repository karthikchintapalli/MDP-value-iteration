#include <iostream>
#include <cstdlib>
#include <cmath>
using namespace std;
#define inf 999.0

float team_number = 43.0;
float gammaval = 1.0;
float delta_bound = (1.0/20.0) * team_number;
//float delta_bound = 0.01;

class MDP {
	public:
		struct state {
			int row, column;
			float utility;
			bool status;
			int type; // normal = 0, goal = 1, start = 4, border = 3, sink = 2 
		};
		struct state s[4][4];

		MDP();
		float transition ( state state1, state state2, int a );
		float reward ( state state1, int a );
		void value_iteration();
		int same (state s1, state s2);
};

int MDP::same (state s1, state s2) {
	if ( s1.row == s2.row && s1.column == s2.column ) return 1;
	return 0;
}

float MDP::transition ( state s1, state s2, int a ) {
	if ( s2.status == 0 ) return 0;
	float prob = 0.0;
	switch (a) {
		case 1:
				if (same(s1, s2)){
					if(s1.column == 0) prob += 0.8;
					else if (s[s1.row][s1.column - 1].status == 0 ) prob += 0.8;	
					
					if(s1.row == 0) prob += 0.1;
					else if(s[s1.row - 1][s1.column].status == 0) prob += 0.1;

					if(s1.row == 3) prob += 0.1;
					else if(s[s1.row + 1][s1.column].status == 0) prob += 0.1;

					return prob;
				}
				else if ( s1.column - s2.column == 1 && s2.row == s1.row ) return 0.8;
				else if ( s2.column == s1.column && abs(s2.row - s1.row) == 1 ) return 0.1;
				else return 0;
			break;
		case 2:
				if (same(s1, s2)){
					if(s1.column == 3) prob += 0.8;
					else if (s[s1.row][s1.column + 1].status == 0 ) prob += 0.8;	
					
					if(s1.row == 0) prob += 0.1;
					else if(s[s1.row - 1][s1.column].status == 0) prob += 0.1;

					if(s1.row == 3) prob += 0.1;
					else if(s[s1.row + 1][s1.column].status == 0) prob += 0.1;

					return prob;
				}
				else if ( s2.column - s1.column == 1 && s2.row == s1.row ) return 0.8;
				else if ( s2.column == s1.column && abs(s2.row - s1.row) == 1 ) return 0.1;
				else return 0;
			break;
		case 3:
				if (same(s1, s2)){
					if(s1.row == 0) prob += 0.8;
					else if (s[s1.row - 1][s1.column].status == 0 ) prob += 0.8;	
					
					if(s1.column == 0) prob += 0.1;
					else if(s[s1.row][s1.column - 1].status == 0) prob += 0.1;

					if(s1.column == 3) prob += 0.1;
					else if(s[s1.row][s1.column + 1].status == 0) prob += 0.1;

					return prob;
				}
				else if ( s1.row - s2.row == 1 && s2.column == s1.column ) return 0.8;
				else if ( s2.row == s1.row && abs(s2.column - s1.column) == 1 ) return 0.1;
				else return 0;
			break;
		case 4:
				if (same(s1, s2)){
					if(s1.row == 3) prob += 0.8;
					else if (s[s1.row + 1][s1.column].status == 0 ) prob += 0.8;	
					
					if(s1.column == 0) prob += 0.1;
					else if(s[s1.row][s1.column - 1].status == 0) prob += 0.1;

					if(s1.column == 3) prob += 0.1;
					else if(s[s1.row][s1.column + 1].status == 0) prob += 0.1;

					return prob;
				}
				else if ( s2.row - s1.row == 1 && s2.column == s1.column ) return 0.8;
				else if ( s2.row == s1.row && abs(s2.column - s1.column) == 1 ) return 0.1;
				else return 0;
			break;
	}
	return 0;
}

float MDP::reward ( state s1, int a ) {
	if (s1.type != 1 && s1.type != 2)
		return (-1.0/20.0) * team_number;
	return 0;
}
void MDP::value_iteration(){
	float prev_utilities[4][4] = {0};
	for ( int i = 0; i < 4; i++ ) 
		for (int j = 0; j < 4; j++ ) 
			prev_utilities[i][j] = s[i][j].utility;
	float delta = inf;
	float time_step = 0;
	cout << " Utilities for time: " << time_step << endl;
	cout << "Delta: " << delta << endl; 
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << s[i][j].utility << "\t" ;
		}
		cout << endl;
	}
	//int ctr = 0;
	int policyiter = 0;
	while(delta > delta_bound || policyiter == 1){
		if (policyiter == 1)
		{
			cout << endl << "Policy iteration";
			policyiter = 2;
		}
		delta = 0;
		for ( int i = 0; i < 4; i++ ){ 
			for (int j = 0; j < 4; j++ ) {

				if( s[i][j].status == 1 && (s[i][j].type == 0 || s[i][j].type == 4)) {
					float max_utility = -100.0;
					for (int a = 1; a <= 4; a++) {

						float value = reward ( s[i][j], a );

						for (int k = 0; k < 4; k++ ) {
							for (int l = 0; l < 4; l++ ) {
								//cout << "Transition: (" << i << "," << j << ") to (" << k << "," << l << ") - action " << a << " -> " << transition ( s[i][j] , s[k][l], a ) << endl; 
								value += transition (s[i][j], s[k][l], a) * prev_utilities[k][l];
							}
						}
						cout << "(" << i << ", " << j << ", " << a << ") - " << value + 2.15 << " -2.15 = " << value << endl;

						if (max_utility < value) max_utility = value;

					}

					s[i][j].utility = max_utility;
					float delta_local = fabs (max_utility - prev_utilities[i][j]);
					if(delta < delta_local) delta = delta_local;
				}
			}
		}
		time_step ++;
		// output new utilities 
		cout << " Utilities for time: " << time_step << endl;
		cout << " Delta: " << delta << endl;
		cout << "Delta Bound : " << delta_bound << endl;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << s[i][j].utility << "\t" ;
			}
			cout << endl;
		}
		cout << endl;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << fabs(s[i][j].utility - prev_utilities[i][j])<< "\t" ;
			}
			cout << endl;
		}
		for ( int i = 0; i < 4; i++ ) 
		for (int j = 0; j < 4; j++ ) 
			prev_utilities[i][j] = s[i][j].utility;

		if(delta <= delta_bound && policyiter == 0) policyiter = 1;
	}
		
	


}

MDP::MDP() {

	for ( int i = 0; i < 4; i++ ) {
		for (int j = 0; j < 4; j++ ) {
			s[i][j].row = i;
			s[i][j].column = j;
			s[i][j].status = 1;
			s[i][j].utility = 0;
			s[i][j].type = 0;
		}
	}

	s[0][0].status = 0;
	s[0][0].type = 3;
	s[0][1].status = 0;
	s[0][1].type = 3;
	s[0][3].status = 0;
	s[0][3].type = 3;
	s[2][2].status = 0;
	s[2][2].type = 3;

	s[0][2].utility = team_number;
	s[0][2].type = 1;
	s[2][1].utility = -1 * team_number;
	s[2][1].type = 2;

	s[3][0].type = 4;
	
	for ( int i = 0; i < 4; i++ ) {
		for (int j = 0; j < 4; j++ ) {
			cout << "\nState (" << i << "," << j << ")\n";
			cout << "row = " << s[i][j].row << " column = " << s[i][j].column << endl;
			cout << "utility = " << s[i][j].utility << " type = " << s[i][j].type << endl;

		}
	}

	value_iteration();
}

int main()
{
	MDP problem1;
	return 0;

}

/*0,0 	0,1 	0,2 	0,3
1,0		1,1 	1,2		1,3
2,0		2,1		2,2		2,3
3,0		3,1		3,2		3,3
*/

