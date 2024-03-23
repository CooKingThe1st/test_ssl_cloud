#ifndef GENE_UTILS   /* Include guard */
#define GENE_UTILS

#include "GeneticConfig.h"
// #include "GeneticEnvi.h"
#include <iostream>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <assert.h>
#include <math.h>       /* fabs */
#include <utility>      // std::pair, std::make_pair

#include <iostream>     // std::cout
#include <algorithm>    // std::sort

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

using namespace std;

// include cross-over and mutation operators
// include fitness function

double MAX_BOUND_FITNESS = 100000;

struct Genome{
	std::vector<double > adn;

	Genome(){		adn.clear();	}


	Genome(vector<double > assign_adn){
		adn.clear();
		for (auto i : assign_adn)
			adn.push_back(i);
	}

	Genome(vector<double> lower_bound, vector<double> upper_bound){
		adn.clear();
		assert(lower_bound.size() == upper_bound.size());
		for (std::vector<int>::size_type jid = 0; jid < upper_bound.size(); jid++){
			assert(lower_bound[jid] <= upper_bound[jid]);
		    double f = (double)rand() / RAND_MAX;
		    adn.push_back(lower_bound[jid] + f * (upper_bound[jid] - lower_bound[jid]) );

		    // cout << adn.back() << '\n';
		}
	}

	int random_indice(){ return int( rand() % (adn.size())); }

	double operator-(const Genome& another) {
		double diff = 0;
		for (std::vector<int>::size_type jid = 0; jid < adn.size(); jid++)
			diff += fabs(adn[jid] - another.adn[jid]);
		return diff;
	}

	friend ostream& operator<<(ostream& os, const Genome& dt)
	{
		for (auto i : dt.adn)
		    os << i << ' ';
	    return os;
	}

	pair<Genome, Genome> cross_swap(const Genome& another) {
		Genome mutated_1(adn);
		Genome mutated_2(another.adn);
		int mask_xover = rand() % ( (1 << (adn.size())) -2) + 1;
		for (int i = 0; i < adn.size(); i++){
			if (CHECK_BIT(mask_xover, i)) continue;
			std::swap( mutated_1.adn[i], mutated_2.adn[i] );
		}
		return make_pair(mutated_1, mutated_2);
	}


	pair<Genome, Genome> cross_fuse(const Genome& another, double fuse_rate = 0.3){
		Genome mutated_1(adn);
		Genome mutated_2(another.adn);

	    double f = (double)rand() / RAND_MAX;
	    fuse_rate = 0.05 + f * (0.3 - 0.05);

		for (std::vector<int>::size_type jid = 0; jid < adn.size(); jid++){
			// int coin_flip = rand() % 2;
			// if (coin_flip == 0) continue;
			double x = mutated_1.adn[jid], y = mutated_2.adn[jid];
			mutated_1.adn[jid] += fuse_rate *(y-x);
			mutated_2.adn[jid] += fuse_rate *(x-y);
		}
		return make_pair(mutated_1, mutated_2);
	}

	Genome mutate_add(){
		Genome mutated(adn);
		for (std::vector<int>::size_type jid = 0; jid < adn.size(); jid++){
			int mode = rand() % 10;
			mode -= 5;

		    double substance = (double)rand() / RAND_MAX;
		    substance = 0.02 + substance * (0.3 - 0.02);

			mutated.adn[jid] += mutated.adn[jid] * substance * mode;
		}
		return mutated;
	}

	Genome mutate_fuse(double fuse_rate = 0.3){
		Genome mutated(adn);

		int num_repeated = rand() % (adn.size()) + 2;

		for (int i = 0; i < num_repeated; i++){
			// int coin_flip = rand() % 2;
			// if (coin_flip == 0) continue;
			int id_1 = random_indice();
			int id_2 = random_indice();

			double y = mutated.adn[id_2], x = mutated.adn[id_1];
			mutated.adn[id_2] += fuse_rate *(x-y);
			mutated.adn[id_2] += fuse_rate *(y-x);
		}
		return mutated;
	}
};

double fitness_function(Genome gene);

struct Village{
	int popu_size;
	int num_generation = 0;

	vector<pair< double, Genome >> cell;
	std::vector<double> l_b, u_b;

	Village(){
		popu_size = 0;
		num_generation = 0;
		cell.clear();
		l_b.clear();
		u_b.clear();
	}

	Village(int given_size, vector<double> lower_bound, vector<double> upper_bound){
		cell.clear();
		popu_size = given_size;
		l_b = lower_bound;
		u_b = upper_bound;
		num_generation = 0;

		for (int i = 0; i < popu_size; i++){
			cell.push_back(make_pair(-MAX_BOUND_FITNESS, Genome(lower_bound, upper_bound))  );
		}

		fitness_get();
		fitness_sort();
	}



	void fitness_get(){
		for (int i = 0; i < popu_size; i++)
			if (cell[i].first - 1 < -MAX_BOUND_FITNESS)
				cell[i].first = fitness_function(cell[i].second);
	}

	void fitness_sort(){// descending order, the stronger is placed closer to 0
		sort(cell.begin(), cell.end(), [](pair<double, Genome> l_g, pair<double, Genome> r_g) {
    		return l_g.first > r_g.first;
		});
	}

	void genome_gen(){
		vector<pair< double, Genome > > next_gen;

		vector<pair< double, Genome > > temp;

		fitness_sort();
		while (cell.size() > popu_size) cell.pop_back();

		// selection 

		int Xover_popu = int(popu_size * GA_Xover_rate);
		int tourbin_size = int(GA_tourbin_size_rate * popu_size);

		for (int i = 0; i < Xover_popu; i++){
			// get the tourbin

			pair<double, Genome > best = make_pair(-MAX_BOUND_FITNESS-1, Genome());

			for (int j = 0; j < tourbin_size; j++){
				int r_id = rand() % popu_size;
				if (cell[r_id].first > best.first)
					best = cell[r_id];
			}

			temp.push_back(best);
		}

		// perform the Xover
		while (temp.size() > 1){
			auto parent_1 = temp.back(); temp.pop_back();
			auto parent_2 = temp.back(); temp.pop_back();

			pair<Genome, Genome> off_spring = parent_1.second.cross_swap(parent_2.second);

			int choice = rand() % 2;

			if (choice == 0){ off_spring = parent_1.second.cross_fuse(parent_2.second); }
			next_gen.push_back(make_pair(-MAX_BOUND_FITNESS, off_spring.first));
			next_gen.push_back(make_pair(-MAX_BOUND_FITNESS, off_spring.second));
		}		

		// cout << " XOVER GEN \n";
		// for (auto i : next_gen)
		// 	cout << i.second << " with cost " << i.first << '\n';

		int num_fill = popu_size - next_gen.size();
		// fill from the original
		for (int i = 0; i < popu_size - num_fill; i++)
			next_gen.push_back(cell[i]);

		// mutate operator
		for (int i = 0; i < popu_size; i++){
			bool is_mutated = ((double) rand() / (RAND_MAX)) <= GA_mutate_rate;
			if (is_mutated == false) continue;

			Genome mutated = next_gen[i].second.mutate_add();
			next_gen[i] = make_pair(-MAX_BOUND_FITNESS, mutated);
		}

		cell = next_gen;

		fitness_get();
		fitness_sort();

		// cout << " NEW GEN \n";
		// for (auto i : cell)
		// 	cout << i.second << " with cost " << i.first << '\n';

		num_generation += 1;
	}

	double diversity_get(){
		double diversity_point = 0;
		for (int i = 0; i < popu_size; i++)
			for (int j = i + 1; j < popu_size; j++)
				diversity_point += ( cell[i].second - cell[j].second );
		return diversity_point / popu_size;
	}

	pair<double, Genome> fitness_best_pair(){ return cell[0]; }

	double fitness_best_gen(){ return cell[0].first; }

	double last_gen_diversity = -1; int diversity_counter = 0;
	double last_gen_fitness = -1; int fitness_counter = 0;

	bool terminate_ok(){

		if (num_generation > GA_max_gen) return true;

		double this_gen_diversity = diversity_get();
		// if  ( fabs(this_gen_diversity - last_gen_diversity) * 50 < fabs(last_gen_diversity)) 
		if  ( fabs(this_gen_diversity - last_gen_diversity) < 1) 
			diversity_counter+=1;
		else diversity_counter = 0;
		last_gen_diversity = this_gen_diversity;
			cout << " THIS GEN DIVERST " << this_gen_diversity << '\n';

		double this_gen_fitness = fitness_best_gen();
		// if  ( fabs(this_gen_fitness - last_gen_fitness) * 50 < fabs(last_gen_fitness)) 
		if  ( fabs(this_gen_fitness - last_gen_fitness) < 1) 
			fitness_counter+=1;
		else fitness_counter = 0;
		last_gen_fitness = this_gen_fitness;

		if (diversity_counter * 10 > GA_max_gen) return true;
		if (fitness_counter * 10 > GA_max_gen) return true;

		return false;
	}
};

Genome background_color;

double fitness_function(Genome gene){
	return 255*3 - (gene - background_color);
}


// void GA_init(int given_size, vector<double> l_b, vector<double> u_b){
// 	oneshot = Village(given_size, l_b, u_b);
// }

// void GA_check_terminate(){

// }


void RGB_test(int num_gen_run){
	srand(time(0));
	vector <double> color_ub{255, 255, 255};
	vector <double> color_lb{0, 0, 0};

	// background_color = Genome(color_lb, color_ub);
	background_color = Genome(std::vector<double>{125, 125, 125});
		std::cout << "BACK COLOR " << background_color << '\n';
	Village oneshot(GA_popu_size, color_lb, color_ub);

	for (int i = 0; i < num_gen_run; i++){
		if (oneshot.terminate_ok()) break;
		std::cout << "Gen " << oneshot.num_generation << " has best " << oneshot.fitness_best_gen() << " with " << oneshot.cell[0].second << '\n';
		oneshot.genome_gen();
	}

}

#endif 