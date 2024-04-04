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

double MAX_BOUND_FITNESS = 10000000000000;

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

	// each adn from genome A has a chance of swapping with that in genome B
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


	// each adn from genome A get a fraction of that in genome B, (5%->30%), multi_rate = 1->2
	pair<Genome, Genome> cross_fuse(const Genome& another){
		Genome mutated_1(adn);
		Genome mutated_2(another.adn);

		for (std::vector<int>::size_type jid = 0; jid < adn.size(); jid++){
			// int coin_flip = rand() % 2;
			// if (coin_flip == 0) continue;
		    double fuse_rate = (double)rand() / RAND_MAX;
		    fuse_rate = 0.05 + fuse_rate * (0.3 - 0.05);
		    fuse_rate *=  ( (rand() % 2 ) + 1);

			double x = mutated_1.adn[jid], y = mutated_2.adn[jid];
			mutated_1.adn[jid] += fuse_rate *(y-x);
			mutated_2.adn[jid] += fuse_rate *(x-y);
		}
		return make_pair(mutated_1, mutated_2);
	}

	// each adn get or lose a fraction of itself (2% -> 30%, multi_rate = -3..3)
	Genome mutate_add(){
		Genome mutated(adn);

		for (std::vector<int>::size_type jid = 0; jid < adn.size(); jid++){
			int mode =  ( (rand() % 7 ) -3);
		    double substance = (double)rand() / RAND_MAX;
		    substance = 0.02 + substance * (0.3 - 0.02);

			mutated.adn[jid] += mutated.adn[jid] * substance * mode;
		}
		return mutated;
	}

	// each adn from genome A get a fraction of another A's adn, (5%->30%), multi_rate = 1->2
	Genome mutate_fuse(){
		Genome mutated(adn);

		int num_repeated = rand() % (adn.size()) + 2;

		for (int i = 0; i < num_repeated; i++){
			// int coin_flip = rand() % 2;
			// if (coin_flip == 0) continue;
			int id_1 = random_indice();
			int id_2 = random_indice();

		    double fuse_rate = (double)rand() / RAND_MAX;
		    fuse_rate = 0.05 + fuse_rate * (0.3 - 0.05);
		    fuse_rate *=  ( (rand() % 2 ) + 1);

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
	double current_diversity = 10000000;

	vector<pair< double, Genome >> cell;

	Village(){
		popu_size = 0;
		num_generation = 0;
		cell.clear();
	}

	Village(int given_size, vector<double> lower_bound, vector<double> upper_bound){
		cell.clear();
		popu_size = given_size;
		num_generation = 0;

		for (int i = 0; i < popu_size; i++){
			cell.push_back(make_pair(-MAX_BOUND_FITNESS, Genome(lower_bound, upper_bound))  );
		}

		fitness_get();
		fitness_sort();
	}

	Village(vector<pair< double, Genome >> genomes){
		popu_size = genomes.size();
		num_generation = 0;
		cell = genomes;
		fitness_get();
		fitness_sort();
	}

	int set_popu(int new_size){ popu_size = new_size; }

	void fitness_get(){
		for (std::vector<int>::size_type i = 0; i < cell.size(); i++)
			if (cell[i].first - 1 < -MAX_BOUND_FITNESS)
				cell[i].first = fitness_function(cell[i].second);
	}

	void fitness_sort(){// descending order, the stronger is placed closer to 0
		sort(cell.begin(), cell.end(), [](pair<double, Genome> l_g, pair<double, Genome> r_g) {
    		return l_g.first > r_g.first;
		});
		while (cell.size() > popu_size) cell.pop_back();
	}

	void auto_fill(){
		while (cell.size() < popu_size){
			Genome mutated = cell[ rand() % cell.size() ].second.mutate_add();
			cell.push_back(make_pair(-MAX_BOUND_FITNESS, mutated) );
		}
		fitness_get();
	}

	void genome_gen(){
		if (cell.size() < popu_size) auto_fill();

		vector<pair< double, Genome > > next_gen;

		vector<pair< double, Genome > > temp;

		fitness_sort();

		// selection 

		int Xover_popu = int(popu_size * GA_Xover_rate);
		int tourbin_size = int(GA_tourbin_size_rate * popu_size) + 1;

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
		std::random_shuffle(temp.begin(), temp.end());

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

			int choice = rand() % 2;
			Genome mutated = next_gen[i].second.mutate_add();
			if (choice == 0) { 
				for (std::vector<int>::size_type jid = 0; jid < mutated.adn.size(); jid++)
					mutated.adn[jid] = next_gen [  rand() % popu_size ].second.adn[jid];
			}

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
		current_diversity = diversity_point / popu_size;
		return current_diversity;
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

struct Town{
	int numberVillage;
	int initVillages;
	int num_generation;
	int total_pop;
	double town_diversity;

	vector<Village> village;


	vector<double> ori_lower_bound;
	vector<double> ori_upper_bound;


	Town(int init_village, int POP, vector<double> lower_bound, vector<double> upper_bound){
		village.clear();
		total_pop = POP;
		num_generation = 0;
		numberVillage = init_village;
		initVillages = init_village;

		town_diversity = 1000000;
		ori_lower_bound = lower_bound;
		ori_upper_bound = upper_bound;

		int init_village_POP = max(3, int(ceil(total_pop / numberVillage)));
		if (init_village_POP * numberVillage < total_pop) init_village_POP ++;
		for (int i = 0; i < numberVillage; i++){
			village.push_back(  Village(init_village_POP, lower_bound, upper_bound)  );
		}
	}

	// Town(ReplayPack ??){
	// }

	void end_of_an_era(){
		numberVillage = initVillages;
		num_generation = 0;
		town_diversity = 1000000;

		pair< double, Genome > the_last_survivor = village[0].cell[0];
		village.clear();

		int init_village_POP = max(3, int(ceil(total_pop / numberVillage)));
		if (init_village_POP * numberVillage < total_pop) init_village_POP ++;
		for (int i = 0; i < numberVillage; i++){
			village.push_back(  Village(init_village_POP - 1, ori_lower_bound, ori_upper_bound)  );
			village[i].cell.push_back(the_last_survivor);
		}
	}

	void town_gen(){

		num_generation += 1;

		if (numberVillage > 1) {

			numberVillage--;
			int new_village_size = max(3, int(ceil(total_pop / numberVillage)));
			if (new_village_size * numberVillage < total_pop) new_village_size++;
			// int new_town_size = new_village_size * numberVillage;

			// vector<pair< double, Genome >> flatten;

			// cout << "current vil " << numberVillage << " new_village_size " << new_village_size << " new_town_size " << new_town_size << " popu_size " << total_pop << '\n';

			for (std::vector<int>::size_type jid = 0; jid + 1 < village.size(); jid++){
				// village[jid].genome_gen();
				// flatten.insert(flatten.end(), village[jid].cell.begin(), village[jid].cell.end());
					// cout << " current vil popu " << village[jid].cell.size() << '\n';
				village[jid].set_popu(new_village_size);
				while(village[jid].cell.size() > new_village_size){
					village[jid+1].cell.push_back(village[jid].cell.back());
					village[jid].cell.pop_back();
				}
				while (village[jid].cell.size() < new_village_size && village[jid+1].cell.size() > 0){
					village[jid].cell.push_back(village[jid+1].cell.back());
					village[jid+1].cell.pop_back();
				}
					// cout << " current vil popu " << village[jid].cell.size() << '\n';
				village[jid].genome_gen();

				// cout << " gen_town " << village[jid].cell.size() << ' ';
			}

			// while (flatten.size() < new_town_size )
			// 	flatten.push_back( flatten[ rand() % flatten.size() ] );

			village.pop_back();

			// for (int i = 0; i < numberVillage; i++){
			// 	vector<pair< double, Genome >> new_resident;
			// 	new_resident.insert(new_resident.end(), flatten.begin() + i * new_village_size, flatten.begin() + (i+1) * new_village_size );

			// 	village[i] = Village(new_resident);
			// }
		}
		else{
			village[0].genome_gen();
		}
	}

	double town_diversity_get(){
		town_diversity = -MAX_BOUND_FITNESS;
		for (auto i : village)
			town_diversity = max(town_diversity, i.diversity_get());
		return town_diversity;
	}


	double town_fitness_best_gen(){ 
		double best_fitness = -10000;
		for (auto i : village)
			best_fitness = max(best_fitness, i.fitness_best_gen());
		return best_fitness;
	}

	double last_gen_diversity = -1; int diversity_counter = 0;
	double last_gen_fitness = -1; int fitness_counter = 0;

	bool terminate_ok(){
		if (numberVillage > 1) return false;
			// cout << num_generation << ' ' << GA_max_gen << '\n';
		if (num_generation > GA_max_gen) return true;

		double town_diversity = town_diversity_get();
		// if  ( fabs(town_diversity - last_gen_diversity) * 50 < fabs(last_gen_diversity)) 
		if  ( fabs(town_diversity - last_gen_diversity) < 1) 
			diversity_counter+=1;
		else diversity_counter = 0;
		last_gen_diversity = town_diversity;
			// cout << " THIS TOWN GEN DIVERST " << town_diversity << '\n';

		double this_gen_fitness = town_fitness_best_gen();
		// if  ( fabs(this_gen_fitness - last_gen_fitness) * 50 < fabs(last_gen_fitness)) 
		if  ( fabs(this_gen_fitness - last_gen_fitness) < 1)
			fitness_counter+=1;
		else fitness_counter = 0;
		last_gen_fitness = this_gen_fitness;

			// cout << " THIS TOWN COUNTER " << (diversity_counter * 10 > GA_max_gen) << ' ' << (fitness_counter * 10 > GA_max_gen) << '\n';

		if (diversity_counter * 10 > GA_max_gen) return true;
		if (fitness_counter * 10 > GA_max_gen) return true;

		return false;
	}


};

Genome background_color;
int eval_count = 0;
double MAX_BOUND_VALUE = 1000000;

double fitness_function(Genome gene){
	eval_count += 1;
	return MAX_BOUND_VALUE*3 - (gene - background_color);
}


// void GA_init(int given_size, vector<double> l_b, vector<double> u_b){
// 	oneshot = Village(given_size, l_b, u_b);
// }

// void GA_check_terminate(){

// }


void RGB_test(int num_village, int num_gen_run, int num_era){
	srand(time(0));
	vector <double> color_ub{MAX_BOUND_VALUE, MAX_BOUND_VALUE, MAX_BOUND_VALUE};
	vector <double> color_lb{0, 0, 0};

	background_color = Genome(color_lb, color_ub);
	// background_color = Genome(std::vector<double>{125, 125, 125});
		std::cout << "BACK COLOR " << background_color << '\n';
	Town oneshot(num_village, GA_popu_size, color_lb, color_ub);

	while (num_era > 0){
		num_era--;
		for (int i = 0; i < num_gen_run; i++){
			if (oneshot.terminate_ok()) break;
			// cout << " COUNTER " << oneshot.diversity_counter + oneshot.fitness_counter  << '\n';
			// if (oneshot.diversity_counter + oneshot.fitness_counter > 0) std::cout << "Gen " << oneshot.num_generation << " has best " << oneshot.town_fitness_best_gen() << " with Div " << oneshot.town_diversity_get() << '\n';
			oneshot.town_gen();
		}
		cout << " NEW ERA " <<  oneshot.num_generation << " has best " << oneshot.town_fitness_best_gen() << " with Div " << oneshot.town_diversity_get() << '\n';
		cout << " PACK LEADER " << oneshot.village[0].cell[0].second << '\n';
		oneshot.end_of_an_era();
	}
	cout << "EVAL COUNT " << eval_count << '\n';
}

#endif 